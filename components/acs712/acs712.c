#include "acs712.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "acs712"

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;
static int zero_current_voltage = 2500; // Default zero current voltage in mV

void delay_us(uint32_t us) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        // Busy-wait loop
    }
}

esp_err_t init_adc_calibration() {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ACS712_ADC_ATTEN,
        .bitwidth = ACS712_ADC_BITWIDTH,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

    return ret;
}


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
        .bitwidth = ACS712_ADC_BITWIDTH,
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

    ret = init_adc_calibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC calibration initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ACS712 initialized successfully");
    return ESP_OK;
}

void acs712_calibrate_zero_current()
{
    if (adc_handle == NULL || adc_cali_handle == NULL) {
        ESP_LOGE(TAG, "ADC or Calibration handle not initialized");
        return;
    }

    int adc_reading = 0;
    int sum = 0;
    int samples = NUM_SAMPLES;
    for (int i = 0; i < samples; ++i) {
        adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &adc_reading);
        sum += adc_reading;
        delay_us(10000);  // Short delay between samples
    }
    adc_reading = sum / samples;
    adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &zero_current_voltage);
    printf("Calibrated zero current voltage: %d mV\n", zero_current_voltage);
}

int read_filtered_adc()
{
    int adc_reading = 0;
    int sum = 0;
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &adc_reading);
        sum += adc_reading;
        delay_us(10000); // Short delay between samples
    }
    return sum / NUM_SAMPLES;
}

float acs712_read_current(void) {
    int adc_value = read_filtered_adc();
    // esp_err_t ret = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &adc_value);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to read from ADC channel");
    //     return 0.0;  // Return 0 on error
    // }
    int voltage;
    adc_cali_raw_to_voltage(adc_cali_handle, adc_value, &voltage);

    float current = ((float)voltage - zero_current_voltage) / ACS712_SENSITIVITY;

    return current;
}

float acs712_read_voltage(void) {
    int adc_value = read_filtered_adc();
    // esp_err_t ret = adc_oneshot_read(adc_handle, ACS712_ADC_CHANNEL, &adc_value);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to read from ADC channel");
    //     return 0.0;  // Return 0 on error
    // }
    int voltage;
    esp_err_t ret = adc_cali_raw_to_voltage(adc_cali_handle, adc_value, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC value to voltage");
        return 0.0;  // Return 0 on error
    }

    return voltage / 1000.0; // Convert mV to V
}