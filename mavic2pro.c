/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: Drone controller with stabilization, manual control,
 * and filtered object recognition. Filters by specific object NAME (using id_name property)
 * and prints the associated name string and position once per cooldown period.
 */

#include <math.h>
#include <stdio.h>    // For printf
#include <stdlib.h>   // For EXIT_SUCCESS
#include <string.h>   // For strcmp 
#include <stdbool.h>  // For bool type

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/camera_recognition_object.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define PRINT_COOLDOWN_S 1.0 // Minimum time (in seconds) between detailed recognition prints

// --- TARGET DEFINITIONS ---
// Define the names (Webots Node Names) to search for
const char *TARGET_NAMES[] = {
    "fire extinguisher",
    "medicine bottle(1)",
    "Telephone"
};
const int NUM_TARGETS = sizeof(TARGET_NAMES) / sizeof(TARGET_NAMES[0]);


// Function to check if the detected object's name matches one of the targets
const char *get_target_name(const char *detected_name) {
    if (detected_name == NULL) return NULL; // Safety check
    
    for (int i = 0; i < NUM_TARGETS; i++) {
        // Use strcmp to compare the object's ID Name (node name) string
        if (strcmp(detected_name, TARGET_NAMES[i]) == 0) 
        {
            return TARGET_NAMES[i]; // Return the matched name
        }
    }
    return NULL; // No match found
}

// --- MAIN FUNCTION ---

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices (Initialization code Omitted for brevity)
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  wb_camera_recognition_enable(camera, timestep); 
  
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");

  WbDeviceTag motors[4];
  char *names[4] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};
  for (int i = 0; i < 4; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], 1.0);
  }

  // Display messages
  printf("Start the drone...\n");
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0) break;
  }
  printf("NODE NAME INSPECTOR MODE: Filtering by object node names only (id_name).\n");
  printf("Targets: 'fire extinguisher', 'medicine bottle(1)', 'Telephone'.\n");

  // Constants (for flight control)
  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;
  double target_altitude = 1.0;

  // Variable to track last detailed print time (for spam prevention)
  static double last_print_time = 0.0;

  // Main loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();

    // Flight control logic (same as previous)
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;

    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP: pitch_disturbance = -2.0; break;
        case WB_KEYBOARD_DOWN: pitch_disturbance = 2.0; break;
        case WB_KEYBOARD_RIGHT: yaw_disturbance = -1.3; break;
        case WB_KEYBOARD_LEFT: yaw_disturbance = 1.3; break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT): roll_disturbance = -1.0; break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT): roll_disturbance = 1.0; break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_altitude += 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_altitude -= 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = wb_keyboard_get_key();
    }
    
    // --- NODE NAME-FILTERED OBJECT RECOGNITION LOGIC (Spam Prevention) ---

    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    int filtered_count = 0;
    bool new_object_detected = false; 

    if (number_of_objects > 0) {
        
        for (int i = 0; i < number_of_objects; ++i) {
            
            // **CRITICAL CHANGE**: Using objects[i].id_name for the Node Name lookup
            const char *node_name = objects[i].id_name;
            
            // Check if the node name matches a target
            const char *target_name_string = get_target_name(node_name);
            
            if (target_name_string != NULL) {
                filtered_count++;
                new_object_detected = true; 
                
                // ACTION: ONLY PRINT IF ENOUGH TIME HAS PASSED SINCE LAST PRINT
                if (time - last_print_time >= PRINT_COOLDOWN_S) {
                    
                    double x = objects[i].position[0];
                    double y = objects[i].position[1];
                    double z = objects[i].position[2];
                    
                    printf("[Time: %.3f s] 🎯 **OBJECT FOUND**:\n", time);
                    printf("  - Identified Name: **%s**\n", target_name_string);
                    printf("  - Relative Position (X, Y, Z): [%.2f, %.2f, %.2f] m\n", x, y, z);
                    
                    // Crucially, update the last print time to start the cooldown
                    last_print_time = time;
                    
                    // Exit the loop after finding and printing the first object to enforce the cooldown
                    break;
                }
            }
        }
    }
    
    // Debug output: Print overall count (filtered) less frequently
    if ((int)(time * 1000) % 640 == 0) {
       // Only print the count if no new detailed message was printed this cycle
       if (!new_object_detected || time - last_print_time > 0.01) { 
           printf("[Time: %.3f s] Targets in view (Total Matched): %d\n", time, filtered_count);
       }
    }
    // --- END NODE NAME-FILTERED OBJECT RECOGNITION LOGIC ---


    // Compute and actuate motor inputs. (Omitted for brevity)
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;

    wb_motor_set_velocity(motors[0], front_left_motor_input);
    wb_motor_set_velocity(motors[1], -front_right_motor_input);
    wb_motor_set_velocity(motors[2], -rear_left_motor_input);
    wb_motor_set_velocity(motors[3], rear_right_motor_input);
  }

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}