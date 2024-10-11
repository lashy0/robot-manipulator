#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_pca9685_motion.h"

static const char *TAG = "servo_pca9685_motion";

void servo_pca9685_motion_init(servo_pca9685_motion_t *motion, servo_t *servo, float step_size, int step_delay)
{
    if (motion == NULL || servo == NULL) {
        ESP_LOGE(TAG, "Invalid argument: motion or servo is NULL");
        return;
    }

    if (step_size <= 0) {
        ESP_LOGE(TAG, "Invalid step size: must be greater than zero");
        return;
    }

    if (step_delay <= 0) {
        ESP_LOGE(TAG, "Invalid step delay: must be greater than zero");
        return;
    }

    motion->servo = servo;
    motion->target_angle = servo->current_angle;
    motion->step_size = step_size;
    motion->step_delay = step_delay;
}

void servo_pca9685_set_target_angle(servo_pca9685_motion_t *motion, float target_angle)
{
    if (target_angle < motion->servo->min_angle || target_angle > motion->servo->max_angle) {
        ESP_LOGW(
            TAG, "Invalid target angle: %.2f (min: %.2f, max: %.2f)",
            target_angle, motion->servo->min_angle, motion->servo->max_angle
        );
        if (target_angle < motion->servo->min_angle) {
            motion->target_angle = motion->servo->min_angle;
        } else if (target_angle > motion->servo->max_angle) {
            motion->target_angle = motion->servo->max_angle;
        }
    }

    motion->target_angle = target_angle;
}

esp_err_t servo_pca9685_motion_smooth_move(servo_pca9685_motion_t *motion, float pwm_freq)
{
    esp_err_t ret;
    float angle;
    float direction;

    while (motion->servo->current_angle != motion->target_angle) {
        direction = (motion->target_angle > motion->servo->current_angle) ? 1.0f : -1.0f;
        angle = motion->servo->current_angle + direction * motion->step_size;

        if ((direction > 0 && angle > motion->target_angle) ||
            (direction < 0 && angle < motion->target_angle)) {
            angle = motion->target_angle;
        }

        ret = servo_pca9685_set_angle(motion->servo, angle, pwm_freq);
        if (ret != ESP_OK) {
            return ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(motion->step_delay));
    }

    return ESP_OK;
}