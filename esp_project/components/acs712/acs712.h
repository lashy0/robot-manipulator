#ifndef ACS712_H
#define ACS712_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;       /*!< ADC handle used for reading raw data */
    adc_channel_t adc_channel;                  /*!< ADC channel assigned to the sensor */
    adc_cali_handle_t cali_handle;              /*!< ADC calibration handle */
    bool calibrated;                            /*!< Flag indicating if the sensor has been calibrated */
    float sensitivity;                          /*!< Sensitivity in mV/A based on the specific ACS712 version */
} acs712_t;

// ОСТАВИТЬ
/**
 * @brief Initialize the ACS712 current sensor
 * 
 * This function initialize the ACS712 current sensor by setting up the ADC channel,
 * calibration handle, and sensitivity.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[in] unit The ADC unit
 * @param[in] attent The ADC attenuation level
 * @return esp_err_t ESP_OK on successful, or an error code
 */
esp_err_t acs712_init(acs712_t *acs712, adc_unit_t unit, adc_atten_t attent);

// ОСТАВИТЬ
/**
 * @brief Deinitialize the ACS712 current sensor
 * 
 * This function deinitialize the ACS712 sensor, releasing any resources
 * allocated during initialization.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 */
void acs712_deinit(acs712_t *acs712);

// ДОБАВИТЬ фильтр в нее, надо ли получать не отфильтрованное значение?
/**
 * @brief Read the raw ADC data from the ACS712 sensor
 * 
 * This function reads the raw ADC value from the ASC712 sensor without
 * appilying any calibration or conversion to voltage.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the raw ADC value
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t acs712_read_raw(acs712_t *acs712, int *data);

// ОСТАВИТЬ
/**
 * @brief Read the voltage output from the ACS712 sensor
 * 
 * This function reads the voltage output from the ACS712 sensor in millivolts,
 * using calibration if available. The voltage correspond to the current sensed by
 * the ACS712.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to stote the voltage value in millivolts (mV)
 * @return esp_err_t ESP_OK on success, or an error code 
 */
esp_err_t acs712_read_voltage(acs712_t *acs712, int *data);

// ПРОВЕРИТЬ, что получаемое напяжение в ходе программы не меняется
/**
 * @brief Calibrate the ACS712 sensor by reading the offset voltage
 * 
 * This function reads multiple samples from the ACS712 sensor to calculate 
 * an average raw ADC value, then converts it to a voltage value in millivolts (mV). 
 * The result is the offset voltage, which can be used to calculate the current more accurately.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the calculated offset voltage in millivolts (mV)
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t acs712_calibrate_voltage(acs712_t *acs712, int *data);

// ДОБАВИТЬ abs для получаемого силы тока
/**
 * @brief Read the current in amperes from the ACS712 sensor
 * 
 * This function reads the voltage output from the ACS712 sensor and calculates the 
 * corresponding current in amperes (A) using the offset voltage and the sensor sensitivity.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the calculated current in amperes (A)
 * @param[in] offset_voltage The offset voltage in millivolts (mV) that was calibrated
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t acs712_read_current(acs712_t *acs712, float *data, int offset_voltage);

// ОСТАВИТЬ
/**
 * @brief Convert raw ADC data to voltage
 * 
 * This function converts the given raw ADC value from the ACS712 sensor to 
 * a voltage value in millivolts (mV).
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the converted voltage value in millivolts (mV)
 * @param[in] raw The raw ADC value
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t acs712_raw_to_voltage(acs712_t *acs712, int *data, int raw);

// ОСТАВИТЬ
/**
 * @brief Convert raw ADC data to current in amperes
 * 
 * This function converts the given raw ADC value from the ACS712 sensor to 
 * a current value in amperes (A), using the offset voltage and the sensor sensitivity.
 * 
 * @param[in] acs712 Pointer to the ACS712 sensor structure
 * @param[out] data Pointer to store the calculated current in amperes (A)
 * @param[in] offset_voltage The offset voltage in millivolts (mV) that was calibrated
 * @param[in] raw The raw ADC value
 * @return esp_err_t ESP_OK on success, or an error code
 */
esp_err_t acs712_raw_to_current(acs712_t *acs712, float *data, float offset_voltage, int raw);

void acs712_print_data(acs712_t *acs712, int offset_voltage);

#endif