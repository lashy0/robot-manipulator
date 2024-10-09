#ifndef ACS712_FILTERED_H
#define ACS712_FILTERED_H

#include "acs712.h"
#include "esp_err.h"

/**
 * @brief Read the filtered raw ADC value from the ACS712 sensor.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[in] data Pointer to store the filtered raw value
 * 
 * @return ESP_OK on success
 * @return ESP_FAIL if the reading fail
 */
esp_err_t acs712_read_filtered_raw(acs712_t *acs712, int *data);

/**
 * @brief Read the filtered voltage from the ACS712 sensor.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[in] data Pointer to store the filtered voltage value in millivolts (mV)
 * 
 * @return ESP_OK on success
 * @return ESP_FAIL if the reading or conversion fails
 */
esp_err_t acs712_read_filtered_voltage(acs712_t *acs712, int *data);

/**
 * @brief Read the filtered current from the ACS712 sensor.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[in] data Pointer to store the calculated filtered current in amperes (A)
 * 
 * @return ESP_OK on success
 * @return ESP_FAIL if the calculation fails or sensitivity is zero
 */
esp_err_t acs712_read_filtered_current(acs712_t *acs712, float *data);

/**
 * @brief Calibrate the filtered zero-point voltage of the ACS712 sensor.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[in] samples Number of samples to average for calibration.
 * 
 * @return ESP_OK on success
 * @return ESP_FAIL if the reading or conversion fails
 */
esp_err_t acs712_calibrate_filtered_voltage(acs712_t *acs712, int samples);

#endif