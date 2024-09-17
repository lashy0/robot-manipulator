#ifndef ARM_ROBOT_H
#define ARM_ROBOT_H

#include "servo_pca9685.h"

#define PWM_FREQUENCY               50
#define SERVO_MAX_ANGLE             180
#define SERVO_MIN_PULSE_WIDTH_US    500
#define SERVO_MAX_PULSE_WIDTH_US    2500

typedef struct {
    servo_t base_servo;
    servo_t shoulder_servo;
    servo_t elbow_servo;
    servo_t wrist_rot_servo;
    servo_t wrist_ver_servo;
    servo_t gripper_servo;
} arm_robot_t;

/**
 * @brief Initializes the robot arm by setting up all six servos
 */
void arm_robot_init(arm_robot_t *robot, pca9685_t pca9685);

/**
 * @brief Moves the robot arm to its home (default) position
 * 
 * @param[in] robot Pointer to structure representing the robot arm
 * 
 * @return ESP_OK is Success, or ESP_FAIL
 */
esp_err_t arm_robot_home_state(arm_robot_t *robot);

/**
 * @brief Moves the robot arm to the specified angles for all servos
 * 
 * @param[in] robot Pointer to structure representing the robot arm
 * @param[in] base_angle Target angle for the base servo
 * @param[in] shoulder_angle Target angle for the shoulder servo
 * @param[in] elbow_angle Target angle for the elbow servo
 * @param[in] wrist_rot_angle Target angle for the wrist rotation servo
 * @param[in] wrist_ver_angle Target angle for the wrist vertical servo
 * @param[in] gripper_angle Target angle for the gripper servo
 * @param[in] delay Delay between each step of movement, in milliseconds
 * 
 * @return ESP_OK is Success, or ESP_FAIL
 */
esp_err_t arm_robot_movement(arm_robot_t *robot, float base_angle, float shoulder_angle, float elbow_angle,
                            float wrist_rot_angle, float wrist_ver_angle, float gripper_angle, int delay);

#endif