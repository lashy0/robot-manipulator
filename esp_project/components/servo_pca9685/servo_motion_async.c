#include <string>
#include "esp_log.h"
#include "esp_err.h"
#include "servo_motion_async.h"

static const char *TAG = "servo_motion_async";

void servo_motion_async_init(async_motion_t *motion, servo_t *servo, float step_size, int step_delay)
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
    motion->step_delay = step_delay;
    motion->step_size = step_size;
    motion->is_moving = false;
    motion->timer_handle = NULL;
}

void servo_motion_set_target_angle(async_motion_t *motion, float target_angle)
{
    servo_t *servo = motion->servo;

    if (target_angle < servo->min_angle || target_angle > servo->max_angle) {
        ESP_LOGW(
            TAG, "Invalid target angle: %.2f (min: %.2f, max: %.2f)",
            target_angle, servo->min_angle, servo->max_angle
        );
        if (target_angle < servo->min_angle) {
            motion->target_angle = servo->min_angle;
        } else if (target_angle > servo->max_angle) {
            motion->target_angle = servo->max_angle;
        }
    }
    else {
        motion->target_angle = target_angle;
    }

    if (motion->is_moving) {
        ESP_LOGI(TAG, "Target angle update to %.2f while moving", target_angle);
    }
}

static void smooth_move_async_callback(void *arg)
{
    async_motion_t *motion = (async_motion_t*) arg;
    esp_err_t ret;

    servo_t *servo = motion->servo;

    float direction = (motion->target_angle > servo->current_angle) ? 1.0f : -1.0f;
    float angle = servo->current_angle + direction * motion->step_size;

    if ((direction > 0 && angle >= motion->target_angle) ||
        (direction < 0 && angle <= motion->target_angle)) {
        angle = motion->target_angle;
    }

    ret = servo_pca9685_set_angle(servo, angle, servo->pca9685->pwm_freq);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set servo angle");
        esp_timer_stop(motion->timer_handle);
        esp_timer_delete(motion->timer_handle);
        motion->timer_handle = NULL;
        motion->is_moving = false;
        return;
    }

    if (servo->current_angle == motion->target_angle) {
        ESP_LOGI(TAG, "Target angle reache. Stop moving");
        esp_timer_stop(motion->timer_handle);
        esp_timer_delete(motion->timer_handle);
        motion->timer_handle = NULL;
        motion->is_moving = false;
        return;
    }
}

void servo_smooth_move_async(async_motion_t *motion, float target_angle)
{
    if (motion->is_moving) {
        servo_motion_set_target_angle(motion, target_angle);
        return;
    }

    esp_err_t ret;

    motion->target_angle = target_angle;
    motion->is_moving = true;

    char timer_name[32];
    snprintg(timer_name, sizeof(timer_name), "servo_motion_timer_%d", motion->servo->channel);

    esp_timer_create_args_t timer_args = {
        .callback = &smooth_move_async_callback,
        .arg = motion,
        .name = timer_name
    };

    ret = esp_timer_create(&timer_args, &motion->timer_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create move timer for servo on channel %d", motion->servo->channel);
        motion->is_moving = false;
        return;
    }

    ret = esp_timer_start_periodic(motion->timer_handle, motion->step_delay * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start move timer for servo on channel %d", motion->servo->channel);
        esp_timer_delete(motion->timer_handle);
        motion->timer_handle = NULL;
        motion->is_moving = false;
    }
}