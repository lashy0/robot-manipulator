#include "esp_log.h"
#include "esp_err.h"
#include "servo_pca9685.h"

static const char *TAG = "servo_pca9685";

void servo_pca9685_init(servo_t *servo, pca9685_t *pca9685, const servo_config_t *config)
{
    if (servo == NULL || pca9685 == NULL || config == NULL) {
        ESP_LOGE(TAG, "Invalid argument: one or more input pointers NULL");
        return;
    }

    if (config->min_angle >= config->max_angle) {
        ESP_LOGE(
            TAG, "Invalid configuration: min_angle %.2f must be less than max_angle %.2f",
            config->min_angle, config->max_angle
        );
        return;
    }

    if (config->min_pulse_width >= config->max_pulse_width) {
        ESP_LOGE(
            TAG, "Invalid configuration: min_pulse_width %d must be less than max_pulse_width %d",
            config->min_pulse_width, config->max_pulse_width
        );
        return;
    }

    servo->pca9685 = *pca9685;
    servo->channel = config->channel;
    servo->min_pulse_width = config->min_pulse_width;
    servo->max_pulse_width = config->max_pulse_width;
    servo->min_angle = config->min_angle;
    servo->max_angle = config->max_angle;
    servo->current_angle = 0.0f;
}

esp_err_t servo_pca9685_set_angle(servo_t *servo, float angle, float pwm_freq)
{
    if (angle < 0.0f || angle > servo->max_angle) {
        ESP_LOGE(TAG, "Invalid angle: %.2f (min: %.2f, max: %.2f)", angle, servo->min_angle, servo->max_angle);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Converting angle to pulse duration
    float pulse_width = ((angle - servo->min_angle) / (servo->max_angle - servo->min_angle)) * (servo->max_pulse_width - servo->min_pulse_width) + servo->min_pulse_width;
    uint16_t off_time = (uint16_t)((pulse_width * 4096) / (1000000 / pwm_freq));
    uint16_t on_time = 0;

    ESP_LOGI(TAG, "Setting servo angle to %.2f degrees (pulse width: %.2f us, on_time: %d, off_time: %d)", angle, pulse_width, on_time, off_time);

    ret = pca9685_set_pwm(&servo->pca9685, servo->channel, on_time, off_time);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    servo->current_angle = angle;

    return ESP_OK;
}

esp_err_t servo_pca9685_get_angle(servo_t *servo,  float *angle, float pwm_freq)
{
    esp_err_t ret;
    uint16_t on_time;
    uint16_t off_time;

    ret = pca9685_get_pwm(&servo->pca9685, servo->channel, &on_time, &off_time);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    float pulse_width = ((off_time - on_time) * (1000000 / pwm_freq)) / 4096;

    // Converting pulse duration to angle
    float pulse_width_temp = (pulse_width - servo->min_pulse_width);
    pulse_width_temp = pulse_width_temp < 0.0f ? 0.0f : pulse_width_temp;
    *angle = (pulse_width_temp * servo->max_angle) / (servo->max_pulse_width - servo->min_pulse_width);

    ESP_LOGI(TAG, "Get servo angle to %.2f degrees (pulse width: %.2f us, on_time: %d, off_time: %d)", *angle, pulse_width, on_time, off_time);

    return ESP_OK;
}