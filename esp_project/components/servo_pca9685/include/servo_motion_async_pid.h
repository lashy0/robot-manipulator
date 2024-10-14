#ifndef SERVO_MOTION_ASYNC_H
#define SERVO_MOTION_ASYNC_H

#include "servo_pca9685.h"
#include "esp_timer.h"
#include "pid.h"

/**
 * @brief The state structure for asynchronous motion of the servo
 */
typedef struct {
    servo_t *servo;                  /**< Pointer to the servo being controlled */
    float target_angle;              /**< The target angle in degress */
    int step_delay;                  /**< The delay between steps in milliseconds */
    bool is_moving;                  /**< Indicates if the servo is currently moving */
    esp_timer_handle_t timer_handle; /**< Timer handle for managing servo movement asynchronously */
    pid_controller_t pid;            /**< PID controller */
} async_motion_pid_t;

/**
 * @brief Initialize the asynchronous PID-controlled motion structure for the servo
 * 
 * @param[in] motion Pointer to the structure to initialize
 * @param[in] servo Pointer to the structure servo
 * @param[in] step_delay The delay between steps in milliseconds
 * @param[in] kp Proportional coefficient for PID controller
 * @param[in] ki Integral coefficient for PID controller
 * @param[in] kd Derivative coefficient for PID controller
 */
void servo_motion_async_pid_init(async_motion_pid_t *motion, servo_t *servo, int step_delay, float kp, float ki, float kd);

/**
 * @brief Sets a new target angle
 * 
 * @param[in] motion Pointer to the structure controlling the servo
 * @param[in] target_angle The new target angle in degress
 */
void servo_motion_pid_set_target_angle(async_motion_pid_t *motion, float target_angle);

/**
 * @brief Start smooth asynchronous movement of the servo to the target angle using PID control
 * 
 * @param[in] motion Pointer to the structure controlling the servo
 * @param[in] target_angle The desired target angle in degrees
 */
void servo_smooth_move_async_pid(async_motion_pid_t *motion, float target_angle);

#endif