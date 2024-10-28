#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "servo_motion_async_pid.h"

static const char *TAG = "servo_motion_async_pid";

void servo_motion_async_pid_init(async_motion_pid_t *motion, servo_t *servo, int step_delay, float kp, float ki, float kd, float max_speed)
{
    if (motion == NULL || servo == NULL) {
        ESP_LOGE(TAG, "Invalid argument: motion or servo is NULL");
        return;
    }

    if (step_delay <= 0) {
        ESP_LOGE(TAG, "Invalid step delay: must be greater than zero");
        return;
    }

    motion->servo = servo;
    motion->target_angle = servo->current_angle;
    motion->step_delay = step_delay;
    motion->is_moving = false;
    motion->max_speed = max_speed;
    motion->timer_handle = NULL;
    motion->ramp_up_coeff = 0.1f;

    pid_controller_init(&(motion->pid), kp, ki, kd);
}

void servo_motion_pid_set_target_angle(async_motion_pid_t *motion, float target_angle)
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

    motion->pid.integral = 0.0f;
    motion->pid.prev_err = 0.0f;
    motion->ramp_up_coeff = 0.1f;
}

static void smooth_move_async_pid_callback(void *arg)
{
    async_motion_pid_t *motion = (async_motion_pid_t*) arg;
    esp_err_t ret;

    servo_t *servo = motion->servo;

    float delay_time = (float)motion->step_delay / 1000.0f;
    float pid_out = pid_calculate(&(motion->pid), motion->target_angle, servo->current_angle, delay_time);

    // Плавный старт движения ramp-up
    if (motion->ramp_up_coeff < 1.0f) {
        motion->ramp_up_coeff += 0.05f;
    }

    pid_out *= motion->ramp_up_coeff;

    if (pid_out > motion->max_speed) {
        pid_out = motion->max_speed;
    }
    else if (pid_out < -motion->max_speed) {
        pid_out = -motion->max_speed;
    }

    float angle = servo->current_angle + pid_out;
    
    // TODO: нужна ли проверка значения из ПИД?
    if ((angle >= motion->target_angle && pid_out > 0) || (angle <= motion->target_angle && pid_out < 0)) {
        angle = motion->target_angle;
    }

    ESP_LOGI(TAG, "Servo smooth move PWM: %d", motion->servo->channel);
    ESP_LOGI(TAG, "pid_out: %.2f, delay_time: %.2f, angle: %.2f", pid_out, delay_time, angle);

    ret = servo_pca9685_set_angle(servo, angle, servo->pca9685.pwm_freq);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set servo angle");
        esp_timer_stop(motion->timer_handle);
        esp_timer_delete(motion->timer_handle);
        motion->timer_handle = NULL;
        motion->is_moving = false;
        return;
    }

    // TODO: на основе моделировнаия проверить значение tol
    float angle_diff = servo->current_angle - motion->target_angle;
    ESP_LOGI(TAG, "Angle diff: %.2f, Current Angle: %.2f, Target Angle: %.2f", angle_diff, servo->current_angle, motion->target_angle);
    float tol = 0.1f;
    if ((angle_diff <= tol) && (angle_diff >= -tol)) {
        ESP_LOGI(TAG, "Target angle reache. Stop moving");
        esp_timer_stop(motion->timer_handle);
        esp_timer_delete(motion->timer_handle);
        motion->timer_handle = NULL;
        motion->is_moving = false;
        return;
    }
}

void servo_smooth_move_async_pid(async_motion_pid_t *motion, float target_angle)
{
    servo_motion_pid_set_target_angle(motion, target_angle);

    if (motion->is_moving) {
        return;
    }

    esp_err_t ret;

    motion->is_moving = true;

    char timer_name[32];
    snprintf(timer_name, sizeof(timer_name), "servo_motion_pid_timer_%d", motion->servo->channel);

    esp_timer_create_args_t timer_args = {
        .callback = &smooth_move_async_pid_callback,
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

void servo_smooth_move_pid_stop(async_motion_pid_t *motion)
{
    if (motion->is_moving) {
        esp_timer_stop(motion->timer_handle);
        esp_timer_delete(motion->timer_handle);
        motion->timer_handle = NULL;
        motion->is_moving = false;
        ESP_LOGI(TAG, "Motion stopped for servo on channel %d", motion->servo->channel);
    }
}