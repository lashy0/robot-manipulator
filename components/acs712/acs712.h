#ifndef ACS712_H
#define ACS712_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t adc_channel;
    adc_cali_handle_t cali_handle;
    bool calibrated;
    float sensitivity;
} acs712_t;


esp_err_t acs712_init(acs712_t *acs, adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, float sensitivity);
void acs712_deinit(acs712_t *acs);
esp_err_t acs712_read_raw(acs712_t *acs, int *data);
esp_err_t acs712_read_filtered_raw(acs712_t *acs, int *data);
esp_err_t acs712_read_voltage(acs712_t *acs, int *data);
esp_err_t acs712_read_current(acs712_t *acs, float *data);

#endif