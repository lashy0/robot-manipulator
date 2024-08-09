#include "servo_control.h"
#include "esp_log.h"

static const char *TAG = "servo";

esp_err_t servo_init(servo_t *servo, pca9685_handle_t pca9685_handle, uint8_t channel)
{
    servo->pca9685_handle = pca9685_handle;
    servo->channel = channel;

    ESP_LOGI(TAG, "Servo initialized on channel %d", channel);
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_t *servo, float angle)
{
    if (!servo || angle < 0.0 || angle > SERVO_MAX_ANGLE) {
        ESP_LOGE(TAG, "Invalid argument");
        return ESP_ERR_INVALID_ARG;
    }

    float pulse_width = SERVO_MIN_PULSE_WIDTH + (angle / SERVO_MAX_ANGLE) * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH);

    uint16_t on_time = 0;
    uint16_t off_time = (uint16_t)(pulse_width * 4096 / 20000); // 20000 микросекунд - период при 50 Гц

    esp_err_t ret = pca9685_set_pwm(servo->pca9685_handle, servo->channel, on_time, off_time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM for channel %d", servo->channel);
        return ret;
    }

    ESP_LOGI(TAG, "Servo on channel %d set to angle %.2f", servo->channel, angle);
    return ESP_OK;
}