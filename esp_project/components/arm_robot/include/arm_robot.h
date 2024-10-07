#ifndef ARM_ROBOT_H
#define ARM_ROBOT_H

#include "servo_pca9685.h"

#define PWM_FREQUENCY  CONFIG_PWM_FREQUENCY

// Servo base
#define SERVO_BASE_START_ANGLE  CONFIG_SERVO_BASE_START_ANGLE
#define SERVO_BASE_NUMBER_PWM   CONFIG_SERVO_BASE_NUMBER_PWM
#define SERVO_BASE_MAX_ANGLE    CONFIG_SERVO_BASE_MAX_ANGLE
#define SERVO_BASE_MIN_PULSE_US CONFIG_SERVO_BASE_MIN_PULSE_US
#define SERVO_BASE_MAX_PULSE_US CONFIG_SERVO_BASE_MAX_PULSE_US

// Servo shoulder
#define SERVO_SHOULDER_START_ANGLE  CONFIG_SERVO_SHOULDER_START_ANGLE
#define SERVO_SHOULDER_NUMBER_PWM   CONFIG_SERVO_SHOULDER_NUMBER_PWM
#define SERVO_SHOULDER_MAX_ANGLE    CONFIG_SERVO_SHOULDER_MAX_ANGLE
#define SERVO_SHOULDER_MIN_PULSE_US CONFIG_SERVO_SHOULDER_MIN_PULSE_US
#define SERVO_SHOULDER_MAX_PULSE_US CONFIG_SERVO_SHOULDER_MAX_PULSE_US

// Servo elbow
#define SERVO_ELBOW_START_ANGLE  CONFIG_SERVO_ELBOW_START_ANGLE
#define SERVO_ELBOW_NUMBER_PWM   CONFIG_SERVO_ELBOW_NUMBER_PWM
#define SERVO_ELBOW_MAX_ANGLE    CONFIG_SERVO_ELBOW_MAX_ANGLE
#define SERVO_ELBOW_MIN_PULSE_US CONFIG_SERVO_ELBOW_MIN_PULSE_US
#define SERVO_ELBOW_MAX_PULSE_US CONFIG_SERVO_ELBOW_MAX_PULSE_US

// Servo wrist
#define SERVO_WRIST_START_ANGLE  CONFIG_SERVO_WRIST_START_ANGLE
#define SERVO_WRIST_NUMBER_PWM   CONFIG_SERVO_WRIST_NUMBER_PWM
#define SERVO_WRIST_MAX_ANGLE    CONFIG_SERVO_WRIST_MAX_ANGLE
#define SERVO_WRIST_MIN_PULSE_US CONFIG_SERVO_WRIST_MIN_PULSE_US
#define SERVO_WRIST_MAX_PULSE_US CONFIG_SERVO_WRIST_MAX_PULSE_US

// Servo wrist rotational
#define SERVO_WRIST_ROT_START_ANGLE  CONFIG_SERVO_WRIST_ROT_START_ANGLE
#define SERVO_WRIST_ROT_NUMBER_PWM   CONFIG_SERVO_WRIST_ROT_NUMBER_PWM
#define SERVO_WRIST_ROT_MAX_ANGLE    CONFIG_SERVO_WRIST_ROT_MAX_ANGLE
#define SERVO_WRIST_ROT_MIN_PULSE_US CONFIG_SERVO_WRIST_ROT_MIN_PULSE_US
#define SERVO_WRIST_ROT_MAX_PULSE_US CONFIG_SERVO_WRIST_ROT_MAX_PULSE_US

// Servo gripper
#define SERVO_GRIPPER_START_ANGLE  CONFIG_SERVO_GRIPPER_START_ANGLE
#define SERVO_GRIPPER_NUMBER_PWM   CONFIG_SERVO_GRIPPER_NUMBER_PWM
#define SERVO_GRIPPER_MAX_ANGLE    CONFIG_SERVO_GRIPPER_MAX_ANGLE
#define SERVO_GRIPPER_MIN_PULSE_US CONFIG_SERVO_GRIPPER_MIN_PULSE_US
#define SERVO_GRIPPER_MAX_PULSE_US CONFIG_SERVO_GRIPPER_MAX_PULSE_US

typedef enum {
    GRIPPER_OPEN,
    GRIPPER_CLOSE
} gripper_state_t;

typedef struct {
    servo_t gripper_servo;
    gripper_state_t state;
} gripper_t;

typedef struct {
    servo_t wrist_rot_servo;
    gripper_t gripper;
} arm_t;

typedef struct {
    servo_t base_servo;
    servo_t shoulder_servo;
    servo_t elbow_servo;
    servo_t wrist_servo;
} manipulator_t;

typedef struct {
    manipulator_t manipulator;
    arm_t arm;
} arm_robot_t;

/**
 * @brief Initializes the robot arm by setting up all six servos
 */
void arm_robot_init(arm_robot_t *robot, pca9685_t *pca9685);

/**
 * @brief Moves the robot arm to its home (default) position
 * 
 * @param[in] robot Pointer to structure representing the robot arm
 * 
 * @return ESP_OK is Success, or ESP_FAIL
 */
esp_err_t arm_robot_home_state(arm_robot_t *robot);

esp_err_t arm_robot_move_servo_to_angle(arm_robot_t *robot, uint8_t channel, float angle);

esp_err_t arm_robot_move_manipulator_to_angles(arm_robot_t *robot, float base_angle, float shoulder_angle, float elbow_angle, float wrist_angle);

#endif