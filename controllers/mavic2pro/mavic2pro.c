/*
 * Object Inspector Controller
 * Translates Python/Java getter methods to C Struct access
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h> // <--- CRITICAL INCLUDE
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <webots/gyro.h>

// Macro to limit values
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // 1. Setup Camera & Recognition
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  wb_camera_recognition_enable(camera, timestep); // Turn on the "Brain"

  // 2. Setup Flight Sensors (So you can fly to the object)
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);

  // 3. Setup Gimbal & Motors
  WbDeviceTag cam_pitch = wb_robot_get_device("camera pitch");
  WbDeviceTag motors[4];
  char *names[4] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};
  for (int i = 0; i < 4; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], 1.0);
  }

  // Constants
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;
  const double k_vert_p = 3.0;
  double target_alt = 1.0;

  printf("INSPECTOR MODE: Fly close to an object to see its ID and Position.\n");

  while (wb_robot_step(timestep) != -1) {
    double time = wb_robot_get_time();

    // --- YOUR REQUESTED LOGIC START ---
    
    // 1. Get the number of objects
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    
    // 2. Get the array (list) of objects
    // In C, this is a pointer to the first element of the array
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

    // 3. Loop through them (Simulating your "firstObject" logic for ALL objects)
    // We only print every 20 steps (approx 0.6 seconds) so the console is readable
    if ((int)(time * 1000) % 640 == 0) {
        printf("\n--- STATUS REPORT ---\n");
        printf("Objects Visible: %d\n", number_of_objects);
        
        for (int i = 0; i < number_of_objects; ++i) {
            // C Equivalent of: id = object.get_id()
            int id = objects[i].id;
            
            // C Equivalent of: model = object.get_model()
            char *model = objects[i].model;
            
            // C Equivalent of: position = object.get_position()
            // Note: position is relative to the CAMERA, not the world (GPS)
            double x = objects[i].position[0];
            double y = objects[i].position[1];
            double z = objects[i].position[2];

            printf("Object [%d]: Model='%s' | Relative Pos: [%.2f, %.2f, %.2f]\n", 
                   id, model, x, y, z);
        }
    }
    // --- YOUR REQUESTED LOGIC END ---

    // Flight Controls (Manual)
    wb_motor_set_position(cam_pitch, -0.4); // Look down 25 degrees
    
    // Simple stabilization so you can fly around
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double *gps_val = wb_gps_get_values(gps);
    const double *gyro_val = wb_gyro_get_values(gyro);
    
    double p_dist = 0, r_dist = 0, y_dist = 0;
    int key = wb_keyboard_get_key();
    while (key > 0) {
        switch (key) {
            case WB_KEYBOARD_UP: p_dist = -2.0; break;
            case WB_KEYBOARD_DOWN: p_dist = 2.0; break;
            case WB_KEYBOARD_RIGHT: y_dist = -1.0; break;
            case WB_KEYBOARD_LEFT: y_dist = 1.0; break;
            case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP): target_alt += 0.05; break;
            case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN): target_alt -= 0.05; break;
        }
        key = wb_keyboard_get_key();
    }

    double v_in = k_vert_p * pow(CLAMP(target_alt - gps_val[2] + 0.6, -1.0, 1.0), 3.0);
    double r_in = k_roll_p * CLAMP(rpy[0], -1.0, 1.0) + gyro_val[0] + r_dist;
    double p_in = k_pitch_p * CLAMP(rpy[1], -1.0, 1.0) + gyro_val[1] + p_dist;
    
    double m1 = 68.5 + v_in - r_in + p_in - y_dist;
    double m2 = 68.5 + v_in + r_in + p_in + y_dist;
    double m3 = 68.5 + v_in - r_in - p_in + y_dist;
    double m4 = 68.5 + v_in + r_in - p_in - y_dist;

    wb_motor_set_velocity(motors[0], m1);
    wb_motor_set_velocity(motors[1], -m2);
    wb_motor_set_velocity(motors[2], -m3);
    wb_motor_set_velocity(motors[3], m4);
  }

  wb_robot_cleanup();
  return 0;
}