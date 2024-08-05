#ifndef ACS712_H
#define ACS712_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ACS712_ADC_CHANNEL              ADC_CHANNEL_2
#define ACS712_ADC_ATTEN                ADC_ATTEN_DB_12
#define ACS712_ADC_BITWIDTH             ADC_BITWIDTH_12

// Constants for ACS712 5A version
#define ACS712_SENSITIVITY      185  // Sensitivity in mV/A for 5A version
#define NUM_SAMPLES             100

esp_err_t acs712_init(void);

void acs712_calibrate_zero_current();

int read_filtered_adc();

float acs712_read_current(void);

float acs712_read_voltage(void);

#endif