#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_pca9685.h"

static const char *TAG = "servo_pca9685";

esp_err_t servo_pca9685_set_angle(servo_t *servo, float angle, float pwm_freq)
{
    if (angle < 0.0f || angle > servo->max_angle) {
        ESP_LOGE(TAG, "Invalid angle");
        return ESP_FAIL;
    }

    esp_err_t ret;

    // Converting angle to pulse duration
    float pulse_width = (angle / servo->max_angle) * (servo->max_pulse_width - servo->min_pulse_width) + servo->min_pulse_width;
    uint16_t off_time = (uint16_t)((pulse_width * 4095) / (1000000 / pwm_freq));
    uint16_t on_time = 0;

    ret = pca9685_set_pwm(&servo->pca9685, servo->channel, on_time, off_time);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t servo_pca9685_get_angle(servo_t *servo,  float *angle, float pwm_freq)
{
    esp_err_t ret;
    uint16_t on_time;
    uint16_t off_time;

    ret = pca9685_get_pwm(&servo->pca9685, servo->channel, &on_time, &off_time);
    if (ret != ESP_OK) {
        return ret;
    }

    float pulse_width = ((off_time - on_time) * (1000000 / pwm_freq)) / 4095;

    // Converting pulse duration to angle
    pulse_width = (pulse_width - servo->min_pulse_width);
    pulse_width = pulse_width < 0.0f ? 0.0f : pulse_width;
    *angle = (pulse_width * servo->max_angle) / (servo->max_pulse_width - servo->min_pulse_width);

    return ESP_OK;
}

esp_err_t servo_pca9685_move_smooth(servo_t *servo, float angle, float pwm_freq, float step, int delay)
{
    if (angle < 0.0f || angle > servo->max_angle) {
        ESP_LOGE(TAG, "Invalid angle");
        return ESP_FAIL;
    }

    float current_angle;
    esp_err_t ret;

    ret = servo_pca9685_get_angle(servo, &current_angle, pwm_freq);
    if (ret != ESP_OK) {
        return ret;
    }

    float increment = (angle > current_angle) ? step : -step;

    while ((increment > 0 && current_angle < angle) || (increment < 0 && current_angle > angle)) {
        current_angle += increment;

        if ((increment > 0 && current_angle > angle) || (increment < 0 && current_angle < angle)) {
            current_angle = angle;
        }

        ret = servo_pca9685_set_angle(servo, current_angle, pwm_freq);
        if (ret != ESP_OK) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(delay));
    }

    return ESP_OK;
}