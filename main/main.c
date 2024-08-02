#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "i2c_handler.h"
#include "pca9685.h"
#include "acs712.h"
#include "servo_handler.h"

static const char *TAG = "main";

esp_err_t set_pwm_for_current(uint8_t channel, float desired_current) {
    // Implement your logic to convert desired_current to PWM value
    // For simplicity, let's assume a direct mapping, which you may need to calibrate
    uint16_t on_time = 0;
    uint16_t off_time = (uint16_t)((desired_current / 5.0) * 4095);

    return pca9685_set_pwm(channel, on_time, off_time);
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(pca9685_init());
    ESP_ERROR_CHECK(acs712_init());

    esp_err_t ret;

    #if 0
    ret = pca9685_set_pwm_freq(40);
    if (ret != ESP_OK) {
        printf("Failed to set PWM frequency\n");
    }
    #endif

    float current_freq;
    ret = pca9685_get_pwm_freq(&current_freq);
    if (ret == ESP_OK) {
    printf("Current PWM frequency: %.2f Hz\n", current_freq);
    } else {
        printf("Failed to get current PWM frequency\n");
    }

    uint8_t channel = 10;
    float target_angle = 45.0;
    float ang;
    uint16_t pwm;
    float current;

    while (true) {
        current = acs712_read_current();
        ang = get_servo_position(channel);
        pwm = angle_to_pwm(ang);
        move_servo_smooth(channel, 90.0, 20);
        printf("Current: %.2f A\n", current);
        printf("Current angle channel %d: %.2f, %d\n", channel, ang, pwm);
        vTaskDelay(pdMS_TO_TICKS(3000));
        target_angle = (target_angle == 45.0) ? 0.0 : 45.0;
    }

    #if 0
    while (true) {
        float current = acs712_read_current();

        float desired_current = 2.0;  // For example, 2A
        if (set_pwm_for_current(0, desired_current) != ESP_OK) {
            printf("Failed to set PWM for current\n");
        }

        printf("Current: %.2f A\n", current);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    #endif
}
