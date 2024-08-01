#ifndef ACS712_H
#define ACS712_H

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define ACS712_ADC_CHANNEL              ADC_CHANNEL_0
#define ACS712_ADC_ATTEN                ADC_ATTEN_DB_0

esp_err_t acs712_init(void);

float acs712_read_current(void);

#endif