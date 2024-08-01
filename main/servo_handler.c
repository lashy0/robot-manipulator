#include "servo_handler.h"
#include "pca9685.h"
#include "acs712.h"
#include <math.h>

static const char *TAG = "servo";

uint16_t angle_to_pwm(float angle)
{
    return (uint16_t)(SERVOMIN + (angle / 180.0) * SERVOMAX);
}

void set_servo_position(uint8_t channel, float angle)
{
    uint16_t pwm = angle_to_pwm(angle);
    esp_err_t ret = pca9685_set_pwm(channel, 0, pwm);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo position");
    }
}

static float current_angle = 20.0;

void rotate_servo(uint8_t channel, float angle_delta)
{
    float new_angle = current_angle + angle_delta;

    if (new_angle < MIN_ANGLE) new_angle = MIN_ANGLE;
    if (new_angle > MAX_ANGLE) new_angle = MAX_ANGLE;

    float step = (new_angle > current_angle) ? 1.0 : -1.0;

    while (fabs(current_angle - new_angle) > 0.1) {
        float current = acs712_read_current();
        ESP_LOGI(TAG, "Current: %.2f A", current);

        if (current > MAX_CURRENT) {
            ESP_LOGW(TAG, "Current limit exceeded! Stopping servo movement.");
            return;
        }

        current_angle += step;
        if ((step > 0 && current_angle > new_angle) || (step < 0 && current_angle < new_angle)) {
            current_angle = new_angle;
        }

        set_servo_position(channel, current_angle);
    }

    ESP_LOGI(TAG, "Rotation completed. New angle %.2f", current_angle);
}