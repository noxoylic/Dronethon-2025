#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>

// --- TUNING STATION ---
#define FORWARD_SIGN 1.0 
#define TARGET_HEIGHT 1.3 
// If the drone oscillates (bounces up and down), LOWER k_vert_i
// If the drone still sinks, INCREASE k_vert_i
const double k_vert_p = 5.0;  // Immediate reaction
const double k_vert_i = 0.5;  // "Memory" reaction (Fixes sinking)
const double k_roll_p = 20.0;
const double k_pitch_p = 20.0;

// --- HELPERS ---
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

bool contains(const char *haystack, const char *needle) {
    if (!haystack || !needle) return false;
    char *h = strdup(haystack); char *n = strdup(needle);
    for(int i = 0; h[i]; i++) h[i] = tolower(h[i]);
    for(int i = 0; n[i]; i++) n[i] = tolower(n[i]);
    bool res = (strstr(h, n) != NULL);
    free(h); free(n);
    return res;
}

int main(int argc, char **argv) {
    wb_robot_init();
    int timestep = (int)wb_robot_get_basic_time_step();

    // 1. Devices
    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, timestep);
    wb_camera_recognition_enable(camera, timestep);

    WbDeviceTag imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, timestep);
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, timestep);
    WbDeviceTag led = wb_robot_get_device("front left led");

    WbDeviceTag ds_left = wb_robot_get_device("ds_left");
    WbDeviceTag ds_right = wb_robot_get_device("ds_right");
    if(ds_left) wb_distance_sensor_enable(ds_left, timestep);
    if(ds_right) wb_distance_sensor_enable(ds_right, timestep);

    WbDeviceTag cam_pitch = wb_robot_get_device("camera pitch");
    WbDeviceTag motors[4];
    char *m_names[] = {"front left propeller", "front right propeller", "rear left propeller", "rear right propeller"};
    for (int i=0; i<4; i++) {
        motors[i] = wb_robot_get_device(m_names[i]);
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 1.0);
    }

    // 2. Control State
    double integral_alt_error = 0.0; // The "Memory" of being too low
    double actual_pitch_cmd = 0.0;
    
    // Flags
    bool found_ext = false, found_phone = false, found_med = false;

    printf("DIAGNOSTIC FLIGHT MODE. Watch Console for object names.\n");

    while (wb_robot_step(timestep) != -1) {
        double time = wb_robot_get_time();

        // --- SENSORS ---
        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
        const double *gps_val = wb_gps_get_values(gps);
        const double *gyro_val = wb_gyro_get_values(gyro);
        
        // --- VISION DIAGNOSTICS ---
        wb_motor_set_position(cam_pitch, -0.9); // Look Down Steeply (approx 50 degrees)
        
        int n_obj = wb_camera_recognition_get_number_of_objects(camera);
        const WbCameraRecognitionObject *objs = wb_camera_recognition_get_objects(camera);
        
        for(int i=0; i<n_obj; i++) {
            char *m = objs[i].model;

            // DIAGNOSTIC PRINT: Print everything so we know what the camera sees
            // Only print once every second to avoid spamming
            if ((int)(time * 10) % 20 == 0) { 
                printf("DEBUG: I see an object named: '%s' at distance %.2fm\n", 
                        m, sqrt(pow(objs[i].position[0],2) + pow(objs[i].position[2],2)));
            }

            if (!found_ext && (contains(m, "extinguisher") || contains(m, "fire"))) {
                found_ext = true;
                printf("!!! SUCCESS: FOUND EXTINGUISHER at %.2f, %.2f !!!\n", gps_val[0], gps_val[1]);
                wb_led_set(led, 1);
            }
            if (!found_phone && (contains(m, "phone") || contains(m, "mobile"))) {
                found_phone = true;
                printf("!!! SUCCESS: FOUND PHONE at %.2f, %.2f !!!\n", gps_val[0], gps_val[1]);
            }
            if (!found_med && (contains(m, "medicine") || contains(m, "bottle"))) {
                found_med = true;
                printf("!!! SUCCESS: FOUND MEDICINE at %.2f, %.2f !!!\n", gps_val[0], gps_val[1]);
            }
        }

        // --- ALTITUDE CONTROL (PI Controller) ---
        double alt_error = TARGET_HEIGHT - gps_val[2];
        
        // INTEGRAL TERM (The Fix): Accumulate error over time
        // We CLAMP the integral so it doesn't get too crazy (Windup protection)
        integral_alt_error += alt_error * (timestep / 1000.0);
        integral_alt_error = CLAMP(integral_alt_error, -15.0, 15.0);

        // P + I Logic
        double v_input = (k_vert_p * alt_error) + (k_vert_i * integral_alt_error);

        // Tilt Compensation (Boost power if tilted)
        double angle_correction = cos(CLAMP(rpy[0], -0.5, 0.5)) * cos(CLAMP(rpy[1], -0.5, 0.5));
        double thrust = (68.5 + v_input) / angle_correction;

        // --- NAVIGATION ---
        double dl = ds_left ? wb_distance_sensor_get_value(ds_left) : 1000;
        double dr = ds_right ? wb_distance_sensor_get_value(ds_right) : 1000;
        bool wall_ahead = (dl < 600 || dr < 600);

        double target_pitch_now = 0.0;
        
        double yaw_disturbance = 0.0;
        
        // Wait until we are 1m up before moving
        if (gps_val[2] > 1.0) {
            if (wall_ahead) {
                target_pitch_now = 0.0; // Stop
                yaw_disturbance = 0.8; // Turn
                integral_alt_error += 0.05; // Cheat: Add extra lift while hovering to be safe
            } else {
                target_pitch_now = -0.5 * FORWARD_SIGN; 
            }
        }

        // Smooth Ramp
        if (actual_pitch_cmd < target_pitch_now) actual_pitch_cmd += 0.01;
        if (actual_pitch_cmd > target_pitch_now) actual_pitch_cmd -= 0.01;

        // --- MOTOR MIXING ---
        double roll_input = k_roll_p * CLAMP(rpy[0], -1.0, 1.0) + gyro_val[0];
        double pitch_input = k_pitch_p * CLAMP(rpy[1], -1.0, 1.0) + gyro_val[1] + actual_pitch_cmd;
        double yaw_input = yaw_disturbance; 

        double m1 = thrust - roll_input + pitch_input - yaw_input;
        double m2 = thrust + roll_input + pitch_input + yaw_input;
        double m3 = thrust - roll_input - pitch_input + yaw_input;
        double m4 = thrust + roll_input - pitch_input - yaw_input;

        wb_motor_set_velocity(motors[0],  m1);
        wb_motor_set_velocity(motors[1], -m2);
        wb_motor_set_velocity(motors[2], -m3);
        wb_motor_set_velocity(motors[3],  m4);
    }

    wb_robot_cleanup();
    return 0;
}