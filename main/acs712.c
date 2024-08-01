#include "acs712.h"

#define TAG "acs712"

static adc_oneshot_unit_handle_t adc_handle;

esp_err_t acs712_init(void)
{
    esp_err_t ret;

    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };

    ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit");
        return ret;
    }

    // Configure ADC width
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ACS712_ADC_ATTEN,
    };

    ret = adc_oneshot_config_channel(adc_handle, ACS712_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel");
        return ret;
    }

    // Test read to ensure ADC is configured correctly
    int adc_value;
    ret = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &adc_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from ADC channel");
        return ret;
    }

    ESP_LOGI(TAG, "ACS712 initialized successfully");
    return ESP_OK;
}

float acs712_read_current() {
    int adc_value;
    esp_err_t ret = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &adc_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from ADC channel");
        return 0.0;  // Return 0 on error
    }
    float voltage = (adc_value * 3.3) / 4096.0; // Assuming 3.3V ADC reference
    float current = (voltage - 2.5) / 0.185;   // For ACS712-05B, adjust as needed for other models
    return current;
}