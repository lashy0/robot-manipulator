#include "acs712.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "acs712";

static bool acs712_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void acs712_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

esp_err_t acs712_init(acs712_t *acs, adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, float sensitivity)
{
    esp_err_t ret = ESP_FAIL;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };

    ret = adc_oneshot_new_unit(&init_config, &acs->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit");
        return ret;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = atten,
    };

    ret = adc_oneshot_config_channel(acs->adc_handle, channel, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel");
        return ret;
    }

    acs->calibrated = acs712_calibration_init(unit, channel, atten, &acs->cali_handle);
    acs->adc_channel = channel;
    acs->sensitivity = sensitivity;

    return ESP_OK;
}

void acs712_deinit(acs712_t *acs)
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(acs->adc_handle));
    if (acs->calibrated) {
        acs712_calibration_deinit(acs->cali_handle);
    }
}

esp_err_t acs712_read_raw(acs712_t *acs, int *data)
{
    esp_err_t ret = ESP_FAIL;
    
    ret = adc_oneshot_read(acs->adc_handle, acs->adc_channel, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from ADC channel");
        return ret;
    }

    return ESP_OK;
}

esp_err_t acs712_read_filtered_raw(acs712_t *acs, int *data)
{
    esp_err_t ret = ESP_FAIL;
    int sum = 0;
    int raw;

    for (int i = 0; i < 10; i++) {
        ret = acs712_read_raw(acs, &raw);
        if (ret != ESP_OK) {
            return ret;
        }
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    *data = sum / 10;

    return ESP_OK;
}

esp_err_t acs712_read_voltage(acs712_t *acs, int *data)
{
    esp_err_t ret = ESP_FAIL;
    int raw;

    ret = acs712_read_raw(acs, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    int voltage;
    ret = adc_cali_raw_to_voltage(acs->cali_handle, raw, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage");
        return ret;
    }

    *data = voltage;

    return ESP_OK;
}

esp_err_t acs712_read_current(acs712_t *acs, float *data, float offset_voltage)
{
    esp_err_t ret = ESP_FAIL;
    int voltage;

    ret = acs712_read_voltage(acs, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read voltage");
        return ret;
    }

    *data = ((float)voltage - offset_voltage) / acs->sensitivity;

    return ESP_OK;
}

float acs712_calibrate(acs712_t *acs) {
    int raw;
    int sum = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_raw(acs, &raw) == ESP_OK) {
            sum += raw;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int average = sum / samples;
    int voltage;
    if (adc_cali_raw_to_voltage(acs->cali_handle, average, &voltage) == ESP_OK) {
        return voltage;
    }

    return 0;
}

void acs712_print_info(acs712_t *acs, float offset_voltage)
{
    int raw;
    int voltage;
    float current;

    acs712_read_raw(acs, &raw);
    acs712_read_voltage(acs, &voltage);
    current = ((float)voltage - offset_voltage) / acs->sensitivity;

    printf("Raw: %d\tVoltage: %d mV\tCurrent: %2f A\n", raw, voltage, current);
}