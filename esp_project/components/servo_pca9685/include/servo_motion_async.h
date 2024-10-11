#ifndef SERVO_MOTION_ASYNC_H
#define SERVO_MOTION_ASYNC_H

#include "servo_pca9685.h"
#include "esp_timer.h"

/**
 * @brief The state structure for asynchronous motion of the servo
 */
typedef struct {
    servo_t *servo;                  /**< Pointer to the servo being controlled */
    float target_angle;              /**< The target angle in degress */
    float step_size;                 /**< The size of each step in degress */
    int step_delay;                  /**< The delay between steps in milliseconds */
    bool is_moving;                  /**< Indicates if the servo is currently moving */
    esp_timer_handle_t timer_handle; /**< Timer handle for managing servo movement asynchronously */
} async_motion_t;

/**
 * @brief Initialize the asynchronous motion structur for the servo
 * 
 * @param[in] motion Pointer to the structure to initialize
 * @param[in] servo Pointer to the structure servo
 * @param[in] step_size The size of each step in degress
 * @param[in] step_delay The delay between steps in milliseconds
 */
void servo_motion_async_init(async_motion_t *motion, servo_t *servo, float step_size, int step_delay);

/**
 * @brief Sets a new target angle
 * 
 * @param[in] motion Pointer to the structure controlling the servo
 * @param[in] target_angle The new target angle in degress
 */
void servo_motion_set_target_angle(async_motion_t *motion, float target_angle);

/**
 * @brief Starts smooth asynchronous movement servo to the target angle
 * 
 * @param[in] motion Pointer to the structure controlling the servo
 * @param[in] target_angle The desired target angle in degrees
 */
void servo_smooth_move_async(async_motion_t *motion, float target_angle);

#endif