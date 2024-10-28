#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "acs712_filtered.h"

static const char *TAG = "acs712_filtered";

#define SAMPLES_COUNT 5
#define ALPHA 0.3

#define USE_EMA_FILTER

esp_err_t acs712_read_filtered_raw(acs712_t *acs712, int *data)
{
    esp_err_t ret;
    int raw = 0;

#ifdef USE_EMA_FILTER

    static float ema_value = 0;

    ret = acs712_read_raw(acs712, &raw);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    if (ema_value == 0) {
        ema_value = raw;
    } else {
        ema_value = (ALPHA * raw) + ((1 - ALPHA) * ema_value);
    }

    *data = (int)ema_value;

#else

    int sum = 0;

    for (int i = 0; i < SAMPLES_COUNT; i++) {
        ret = acs712_read_raw(acs712, &raw);
        if (ret != ESP_OK) {
            return ESP_FAIL;
        }
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    *data = sum / SAMPLES_COUNT;

#endif

    return ESP_OK;
}

esp_err_t acs712_read_filtered_voltage(acs712_t *acs712, int *data)
{
    esp_err_t ret;
    int raw;
    
    ret = acs712_read_filtered_raw(acs712, &raw);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = adc_cali_raw_to_voltage(acs712->cali_handle, raw, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t acs712_read_filtered_current(acs712_t *acs712, float *data)
{
     if (acs712->sensitivity == 0) {
        ESP_LOGE(TAG, "Sensitivity is zero, cannot calculate current");
        return ESP_FAIL;
    }

    esp_err_t ret;
    int voltage;

    ret = acs712_read_filtered_voltage(acs712, &voltage);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    *data = (float)(voltage - acs712->calibrate_voltage) / acs712->sensitivity;

    ESP_LOGI(TAG, "Voltage: %d mV\tCalibrate Voltage %d mV\tCurrent: %2f A\n", voltage, acs712->calibrate_voltage, *data);

    return ESP_OK;
}

esp_err_t acs712_calibrate_filtered_voltage(acs712_t *acs712, int samples)
{
    esp_err_t ret;
    int raw;
    int voltage;
    int sum = 0;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_filtered_raw(acs712, &raw) == ESP_OK) {
            sum += raw;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else {
            return ESP_FAIL;
        }
    }

    ret = adc_cali_raw_to_voltage(acs712->cali_handle, sum / samples, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    acs712->calibrate_voltage = voltage;

    return ESP_OK;
}