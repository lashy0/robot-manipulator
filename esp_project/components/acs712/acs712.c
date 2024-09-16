#include "esp_log.h"
#include "esp_err.h"
#include "acs712.h"

static const char *TAG = "acs712";

//
#define FILTER_SIZE 20

int average_filter(int data) {
    static int samples[FILTER_SIZE] = {0};
    static int sum = 0;
    static int index = 0;
    static int count = 0;

    sum -= samples[index];
    samples[index] = data;
    sum += data;

    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) {
        count++;
    }

    return sum / count;
}

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

esp_err_t acs712_calibrate_voltage(acs712_t *acs712, int *data)
{
    esp_err_t ret;
    int raw;
    int voltage;
    int samples = 100;
    int sum = 0;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_raw(acs712, &raw) == ESP_OK) {
            sum += raw;
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

    *data = voltage;

    return ESP_OK;
}

esp_err_t acs712_init(acs712_t *acs712, adc_unit_t unit, adc_atten_t atten)
{
    esp_err_t ret;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };

    ret = adc_oneshot_new_unit(&init_config, &acs712->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = atten,
    };

    ret = adc_oneshot_config_channel(acs712->adc_handle, acs712->adc_channel, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    acs712->calibrated = acs712_calibration_init(unit, acs712->adc_channel, atten, &acs712->cali_handle);

    ESP_LOGI(TAG, "Start calibrate voltage...");
    int calibrate_voltage;
    ret = acs712_calibrate_voltage(acs712, &calibrate_voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate voltage");
        return ESP_FAIL;
    }
    acs712->calibrate_voltage = calibrate_voltage;

    return ESP_OK;
}

esp_err_t acs712_deinit(acs712_t *acs712)
{
    // TODO: добавить проверку
    esp_err_t ret;

    ret = adc_oneshot_del_unit(acs712->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete ADC unit: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    if (acs712->calibrated) {
        acs712_calibration_deinit(acs712->cali_handle);
        acs712->calibrated = false;
    }

    // ???
    acs712->adc_handle = NULL;
    acs712->cali_handle = NULL;
    acs712->adc_channel = -1;
    acs712->sensitivity = 0.0;
    acs712->calibrate_voltage = 0;

    return ESP_OK;
}

esp_err_t acs712_read_raw(acs712_t *acs712, int *data)
{
    esp_err_t ret;
    int raw;

    ret = adc_oneshot_read(acs712->adc_handle, acs712->adc_channel, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from ADC channel: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    *data = average_filter(raw);

    return ESP_OK;
}

esp_err_t acs712_read_voltage(acs712_t *acs712, int *data)
{
    esp_err_t ret;
    int raw;
    int voltage;

    ret = acs712_read_raw(acs712, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = adc_cali_raw_to_voltage(acs712->cali_handle, raw, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    *data = voltage;

    return ESP_OK;
}

esp_err_t acs712_read_current(acs712_t *acs712, float *data)
{
    esp_err_t ret;
    int voltage;

    ret = acs712_read_voltage(acs712, &voltage);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    *data = (float)(voltage - acs712->calibrate_voltage) / acs712->sensitivity;

    return ESP_OK;
}

void acs712_read_data(acs712_t *acs712)
{
    esp_err_t ret;
    int raw;
    int voltage;
    float current;

    acs712_read_raw(acs712, &raw);
    
    ret = adc_cali_raw_to_voltage(acs712->cali_handle, raw, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
    }

    current = (float)(voltage - acs712->calibrate_voltage) / acs712->sensitivity;

    printf("Raw: %d\tVoltage: %d mV\tCalibrate Voltage %d mV\tCurrent: %2f A\n", raw, voltage, acs712->calibrate_voltage, current);
}