#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "servo_pca9685.h"

static const char *TAG = "servo_pca9685";

static esp_timer_handle_t move_timer;

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

    // ESP_LOGI(TAG, "pulse: %.2f", pulse_width);
    // ESP_LOGI(TAG, "on: %d, off: %d", on_time, off_time);

    return ESP_OK;
}

static void move_timer_callback(void *arg)
{
    servo_t *servo = (servo_t *)arg;
    float current_angle;
    esp_err_t ret;

    ret = servo_pca9685_get_angle(servo, &current_angle, 50);
    if (ret != ESP_OK) {
        esp_timer_stop(move_timer);
        servo->is_busy = false;
        return;
    }

    float increment = (servo->target_angle > current_angle) ? servo->step : -servo->step;

    current_angle += increment;

    // Проверка достигли целевого угла
    if ((increment > 0 && current_angle > servo->target_angle) || (increment < 0 && current_angle < servo->target_angle)) {
        current_angle = servo->target_angle;
        servo_pca9685_set_angle(servo, current_angle, 50);
        ESP_LOGI(TAG, "Moving to angle: %.2f; Target angle: %.2f", current_angle, servo->target_angle);
        esp_timer_stop(move_timer);
        servo->is_busy = false;
        return;
    }

    // Изменение угла
    // servo->current_angle += increment;
    ret = servo_pca9685_set_angle(servo, current_angle, 50);
    if (ret != ESP_OK) {
        esp_timer_stop(move_timer);
        servo->is_busy = false;
        return;
    }

    ESP_LOGI(TAG, "Moving to angle: %.2f; Target_angle: %.2f", current_angle, servo->target_angle);
}

void servo_pca9685_move_smoth_timer(servo_t *servo)
{
    // Прверка занят ли серва
    if (servo->is_busy) {
        ESP_LOGI(TAG, "Servo [pwm %d] is busy. Target angle update.", servo->channel);
        return;
    }

    servo->is_busy = true;

    // float current_angle;

    // esp_err_t ret = servo_pca9685_get_angle(servo, &current_angle, 50);
    // if (ret != ESP_OK) {
    //     servo->is_busy = false;
    //     return;
    // }

    // servo->current_angle = current_angle;

    // Настройка таймера
    const esp_timer_create_args_t timer_args = {
        .callback = &move_timer_callback,
        .arg = (void *)servo,
        .name = "move_servo"
    };

    if (esp_timer_create(&timer_args, &move_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create move timer");
        servo->is_busy = false;
        return;
    }

    if (esp_timer_start_periodic(move_timer, servo->delay) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start move timer");
        esp_timer_delete(move_timer);
        servo->is_busy = false;
        return;
    }
}

void servo_pca9685_move_smooth(servo_t *servo, float pwm_freq)
{
    // TODO: добавить информации о текущих значениях при ошибке
    if (servo->target_angle < 0.0f || servo->target_angle > servo->max_angle) {
        ESP_LOGE(TAG, "Invalid angle");
        return;
        // return ESP_FAIL;
    }

    // Прверка занят ли серва
    if (servo->is_busy) {
        ESP_LOGI(TAG, "Servo [pwm %d] is busy. Target angle update.", servo->channel);
        return;
        // return ESP_OK;
    }

    // Установка флага занятости серва
    servo->is_busy = true;
    // ESP_LOGI(TAG, "Servo is busy: %d", servo->is_busy);

    float current_angle;
    esp_err_t ret;

    ret = servo_pca9685_get_angle(servo, &current_angle, pwm_freq);
    if (ret != ESP_OK) {
        servo->is_busy = false; // сброс флага занятости
        // return ret;
        return;
    }

    // + мб менять при работе серва и текущию скорость?
    // теперь может меняться в процессе работы серва
    float increment = (servo->target_angle > current_angle) ? servo->step : -servo->step;

    while ((increment > 0 && current_angle < servo->target_angle) || (increment < 0 && current_angle > servo->target_angle)) {
        current_angle += increment;

        // проверка, если угол текущий вышел за целевой
        if ((increment > 0 && current_angle > servo->target_angle) || (increment < 0 && current_angle < servo->target_angle)) {
            current_angle = servo->target_angle;
        }

        ret = servo_pca9685_set_angle(servo, current_angle, pwm_freq);
        if (ret != ESP_OK) {
            servo->is_busy = false; // сброс флага занятости
            // return ret;
            return;
        }

        // обновление инкремента, если целевой угол изменился
        increment = (servo->target_angle > current_angle) ? servo->step : -servo->step;

        vTaskDelay(pdMS_TO_TICKS(servo->delay));
    }

    servo->is_busy = false; // сброс флага занятости по завершению
    // return ESP_OK;
}