#ifndef SERVO_PCA9685_H
#define SERVO_PCA9685_H

#include "pca9685.h"

/**
 * @brief Configuration structure for the servo motor
 */
typedef struct {
    uint8_t channel;          /**< The PCA9685 channel used to control to servo */
    uint16_t min_pulse_width; /**< The minimum pulse width in microseconds */
    uint16_t max_pulse_width; /**< The maximum pulse width in microseconds */
    float min_angle;          /**< The minimum angle in degress */
    float max_angle;          /**< The maximum angle in degress */
} servo_config_t;

/**
 * @brief Structure representing the state of a servo motor
 */
typedef struct {
    pca9685_t pca9685;        /**< Pointer to the PCA9685 controller used to control the servo */
    uint16_t min_pulse_width; /**< The minimum pulse width in microseconds */
    uint16_t max_pulse_width; /**< The maximum pulse width in microseconds */
    float min_angle;          /**< The minimum angle in degress */
    float max_angle;          /**< The maximum angle in degress */
    uint8_t channel;          /**< The PCA9685 channel used to control th servo */
    float current_angle;      /**< The current angle in degress of the servo */
} servo_t;

/**
 * @brief Initialize the servo motor using PCA9685
 * 
 * @param[in] servo Pointer to the structure which holds configuration details for the servo
 * @param[in] pca9685 Pointer to the PCA9685 structure used to control the servo
 * @param[in] config Pointer to the structure which holds the servo configuration parameters
 */
void servo_pca9685_init(servo_t *servo, pca9685_t *pca9685, const servo_config_t *config);

/**
 * @brief Set the servo motor to a specified angle using PCA9685
 * 
 * @param[in] servo Pointer to the structure which holds configuration details for the servo
 * @param[in] angle The desired angle to set the servo to (in degrees)
 * @param[in] pwm_freq The PWM frequency in Hz
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if angle is out of bounds
 * @return ESP_FAIL if failed to set PWM value
 */
esp_err_t servo_pca9685_set_angle(servo_t *servo, float angle, float pwm_freq);

/**
 * @brief Get the current angle of the servo motor using PCA9685
 * 
 * @param[in] servo Pointer to the structure which holds configuration details for the servo
 * @param[out] angle Pointer to variable where the current angle will be stored
 * @param[out] pwm_freq The PWM frequency in Hz
 * 
 * @return ESP_OK on success
 * @return ESP_FAIL if failed to read PWM value
 */
esp_err_t servo_pca9685_get_angle(servo_t *servo,  float *angle, float pwm_freq);

#endif