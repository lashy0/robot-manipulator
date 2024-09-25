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

// TODO: target angle имеется в структуре серва
// TODO: посмотреть что лучше отслеживать всю систему с помщью is_moving или каждый серв отслеживать отдельно?
// TODO: сделать настраиваемый в процессе?? задержку и шаг угла
typedef struct {
    servo_t base_servo;
    servo_t shoulder_servo;
    servo_t elbow_servo;
    servo_t wrist_servo;
    servo_t wrist_rot_servo;
    servo_t gripper_servo;

    int delay;

    // ???
    float base_target_angle;
    float shoulder_target_angle;
    float elbow_target_angle;
    float wrist_target_angle;
    float wrist_rot_target_angle;
    float gripper_target_angle;

    bool is_moving; // флаг выполнения движения
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
 * @param[in] wrist_angle Target angle for the wrist servo
 * @param[in] wrist_rot_angle Target angle for the wrist rotation servo
 * @param[in] gripper_angle Target angle for the gripper servo
 * @param[in] delay Delay between each step of movement, in milliseconds
 * 
 * @return ESP_OK is Success, or ESP_FAIL
 */
void arm_robot_movement(arm_robot_t *robot);

void arm_robot_movement_timer(arm_robot_t *robot);

#endif