#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"

#include "acs712.h"

#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

void app_main(void) {
    esp_err_t ret;

    // Initialize ACS712 5A
    acs712_t acs712 = {
        .adc_channel = ACS712_ADC_CHANNEL,
        .sensitivity = ACS712_SENSITIVITY
    };

    ret = acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN);
    if (ret != ESP_OK) {
        return;
    }
    printf("ACS712 5A initialize\n");

    int raw;
    for (int i = 0; i < 10; i++) {
        ret = acs712_read_raw(&acs712, &raw);
        if (ret != ESP_OK) {
            return;
        }
        printf("Raw: %d\n", raw);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    int voltage;
    for (int i = 0; i < 10; i++) {
        ret = acs712_read_voltage(&acs712, &voltage);
        if (ret != ESP_OK) {
            return;
        }
        printf("Voltage: %dmV\tCalibrate Voltage: %dmV\n", voltage, acs712.calibrate_voltage);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    float current;
    for (int i = 0; i < 10; i++) {
        ret = acs712_read_current(&acs712, &current);
        if (ret != ESP_OK) {
            return;
        }
        printf("Current: %2f A\n", current);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    for (int i = 0; i < 100; i++) {
        acs712_read_data(&acs712);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    ret = acs712_deinit(&acs712);
    if (ret != ESP_OK) {
        return;
    }
}