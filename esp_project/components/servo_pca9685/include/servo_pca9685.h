#ifndef SERVO_H
#define SERVO_H

#include "pca9685.h"
#include "esp_timer.h"

typedef struct {
    uint8_t channel;
    uint16_t min_pulse_width;
    uint16_t max_pulse_width;
    float max_angle;
    float step;
    int delay;
} servo_config_t;

typedef struct {
    pca9685_t pca9685;
    uint16_t min_pulse_width;
    uint16_t max_pulse_width;
    float max_angle;
    uint8_t channel;
    bool is_busy; // флаг того что серво уже выполняет выставление угла
    float target_angle; // целевой угол для плавного движения
    float step;
    int delay;
    float current_angle;
    esp_timer_handle_t move_timer;
} servo_t;

void servo_pca9685_init(servo_t *servo, pca9685_t *pca9685, const servo_config_t *config);

/**
 * @brief Set the servo motor to a specified angle using PCA9685
 * 
 * @param[in] servo Pointer to the servo_t structure which holds configuration details for the servo
 * @param[in] angle The desired angle to set the servo to (in degrees)
 * @param[in] pwm_freq The PWM frequency in Hz
 * 
 * @return ESP_OK if successful, or ESP_FAIL
 */
esp_err_t servo_pca9685_set_angle(servo_t *servo, float angle, float pwm_freq);

/**
 * @brief Get the current angle of the servo motor using PCA9685
 * 
 * @param[in] servo Pointer to the servo_t structure which holds configuration details for the servo
 * @param[out] angle Pointer to variable where the current angle will be stored
 * @param[out] pwm_freq The PWM frequency in Hz
 * 
 * @return ESP_OK if successful, or ESP_FAIL
 */
esp_err_t servo_pca9685_get_angle(servo_t *servo,  float *angle, float pwm_freq);

#endif