#ifndef ACS712_H
#define ACS712_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/**
 * Structure for ACS712 current sensor control
 */
typedef struct {
    adc_oneshot_unit_handle_t adc_handle;   /**< ADC handle used for reading raw data */
    adc_channel_t adc_channel;              /**< ADC channel assigned to the sensor */
    adc_cali_handle_t cali_handle;          /**< ADC calibration handle */
    bool calibrated;                        /**< Flag indicating if the sensor has been calibrated */
    float sensitivity;                      /**< Sensitivity in mV/A based on the specific ACS712 version */
    int calibrate_voltage;                  /**< Calibrate voltage in mV */
} acs712_t;

/**
 * @brief Initialize the ACS712 current sensor
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[in] unit The ADC unit
 * @param[in] atten The ADC attenuation level
 * 
 * @return ESP_OK Success, or ESP_FAIL if the initialization or configuration fails at any step
 */
esp_err_t acs712_init(acs712_t *acs712, adc_unit_t unit, adc_atten_t atten);

/**
 * @brief Deinitialize the ACS712 current sensor
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t acs712_deinit(acs712_t *acs712);

/**
 * @brief Read the raw ADC data from the ACS712 current sensor
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the raw ADC value
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t acs712_read_raw(acs712_t *acs712, int *data);

/**
 * @brief Read the voltage output from the ACS712 sensor structure
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the voltage value in millivolts (mV)
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t acs712_read_voltage(acs712_t *acs712, int *data);

/**
 * @brief Read the current in amperes from the ACS712 sensor
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the calculated current in amperes (A)
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t acs712_read_current(acs712_t *acs712, float *data);

// Нужна ли?
void acs712_read_data(acs712_t *acs712);

#endif