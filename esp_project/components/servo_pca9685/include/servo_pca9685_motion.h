#ifndef SERVO_PCA9685_MOTION_H
#define SERVO_PCA9685_MOTION_H

#include "servo_pca9685.h"

/**
 * @brief Structure for controlling smooth motion of a servo motor using PCA9685
 */
typedef struct {
    servo_t *servo; /**< Pointer to the servo being controlled */
    float target_angle; /**< The target angle in degress */
    float step_size; /**< The size of each step in degress */
    int step_delay; /**< The delay between steps in milliseconds */
} servo_pca9685_motion_t;

/**
 * @brief Initialize the servo motion control structure
 * 
 * @param[in] motion Pointer to structure to initialize
 * @param[in] servo Pointer to structure which holds configuration details for the servo
 * @param[in] step_size The size of each step in degress for smooth movement
 * @param[in] step_delay The delay in milliseconds between each step.
 */
void servo_pca9685_motion_init(servo_pca9685_motion_t *motion, servo_t *servo, float step_size, int step_delay);

/**
 * @brief Set the target angle for the servo
 * 
 * @param[in] motion Pointer to structure that holds movement control details
 * @param[in] target_angle The desired target angle in degress
 */
void servo_pca9685_set_target_angle(servo_pca9685_motion_t *motion, float target_angle);

/**
 * @brief Smooth move the servo to the target angle
 * 
 * @param[in] motion Pointer to structure that holds movement control details
 * @param[in] pwm_freq The PWM frequency in Hz for controlling the servo
 * 
 * @return ESP_OK on success
 * @return ESP_FAIL if failed movement
 */
esp_err_t servo_pca9685_motion_smooth_move(servo_pca9685_motion_t *motion, float pwm_freq);

#endif