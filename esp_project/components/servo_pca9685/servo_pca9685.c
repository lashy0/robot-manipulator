#include "esp_log.h"
#include "esp_err.h"
#include "servo_pca9685.h"

static const char *TAG = "servo_pca9685";

esp_err_t servo_pca9685_set_angle(servo_t *servo, float angle, float pwm_freq)
{
    if (angle < 0.0f || angle > servo->max_angle) {
        ESP_LOGE(TAG, "");
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