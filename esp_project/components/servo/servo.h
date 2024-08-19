#ifndef SERVO_H
#define SERVO_H

#include "pca9685.h"

#define SERVO_MIN_PULSE_WIDTH   500
#define SERVO_MAX_PULSE_WIDTH   2500

typedef struct {
    pca9685_t pca9685_handle;
    uint8_t channel;
    float max_angle;
} servo_t;

/**
 * @brief Initialize a servo motor
 * 
 * This function initializes a servo motor connected to the PCA9685 PWM driver by associating 
 * the servo with a specific PWM channel and defining the maximum angle the servo can rotate to.
 * 
 * @param[in] servo Pointer to the servo_t structure to be initialized
 * @param[in] handle The PCA9685 handle used to control the servo
 * @param[in] channel The PWM channel on which the servo is connected (0-15)
 * @param[in] max_angle The maximum angle of the servo rotation (in degrees)
 * @return esp_err_t ESP_OK on successful initialization, or an error code
 */
esp_err_t servo_init(servo_t *servo, pca9685_t handle, uint8_t channel, float max_angle);

/**
 * @brief Set the angle of the servo motor
 * 
 * This function sets the desired angle of a servo motor by calculating the corresponding 
 * pulse width and sending it to the PCA9685 PWM driver. The angle must be within the 
 * range of 0 to max_angle specified during initialization.
 * 
 * @param[in] servo Pointer to the initialized servo_t structure
 * @param[in] angle The desired angle to set the servo to, in degrees
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t servo_set_angle(servo_t *servo, float angle);

#endif