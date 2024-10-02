#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c_master.h"

// Не уверен на счет данных значений
#define MIN_PWM_FREQUENCY           24
#define MAX_PWM_FREQUENCY           1526

#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01
#define PCA9685_LED0_ON_L           0x06
#define PCA9685_LED0_ON_H           0x07
#define PCA9685_LED0_OFF_L          0x08
#define PCA9685_LED0_OFF_H          0x09
#define PCA9685_PRESCALE            0xFE

#define MODE1_ALLCAL                0x01
#define MODE1_SUB3                  0x02
#define MODE1_SUB2                  0x04
#define MODE1_SUB1                  0x08
#define MODE1_SLEEP                 0x10
#define MODE1_AI                    0x20
#define MODE1_EXTCLK                0x40
#define MODE1_RESTART               0x80

#define PCA9685_PRESCALE_MIN        3
#define PCA9685_PRESCALE_MAX        255
#define FREQUENCY_OSCILLATOR        25000000

/**
 * @brief Config structure for PCA9685 driver
 */
typedef struct {
    uint8_t i2c_address;                    /**< I2C address of the PCA9685 device */
    i2c_master_bus_handle_t bus_handle;     /**< I2C bus handle */
    uint32_t scl_speed;                     /**< I2C clock speed */
} pca9685_config_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint32_t oscillator_freq;
    bool is_initialized;
} pca9685_t;

/**
 * @brief Write a byte of data to a PCA9685 register
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * @param[in] reg Register address to write
 * @param[in] data Data bytes to write
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_write(pca9685_t *pca9685, uint8_t reg, uint8_t data);

/**
 * @brief Read a byte from PCA9685 register
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * @param[in] reg Register address to write
 * @param[out] data Pointer to store the data
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_read(pca9685_t *pca9685, uint8_t reg, uint8_t *data);

/**
 * @brief Initialize the PCA9685 device
 * 
 * @param[in] config Pointer configuration structure for PCA9685
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * 
 * @return ESP_OK Success, or ESP_FAIL 
 */
esp_err_t pca9685_init(pca9685_config_t *config, pca9685_t *pca9685);

/**
 * @brief Deinitialize the PCA9685 driver
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * 
 * @return ESP_OK Success, 
 */
esp_err_t pca9685_deinit(pca9685_t *pca9685);

/**
 * @brief Reset the PCA9685 device
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * 
 * @return ESP_OK Success, or ESP_FAIL 
 */
esp_err_t pca9685_reset(pca9685_t *pca9685);

/**
 * @brief Sleep the PCA9685 device
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_sleep(pca9685_t *pca9685);

/**
 * @brief Wake the PCA9685 device
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_wake(pca9685_t *pca9685);

/**
 * @brief Get the PWM value for a channel on the PCA9685
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * @param[in] channel The channel number (0 to 15) to set the PWM
 * @param[out] on_time Pointer to store the retrieved on_time value (0 to 4095)
 * @param[out] off_time Pointer to store the retrived off_time value (0 to 4095)
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_get_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t *on_time, uint16_t *off_time);

/**
 * @brief Set the PWM value for a channel on the PCA9685
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * @param[in] channel The channel number (0 to 15) to set the PWM
 * @param[in] on_time The time (0 to 4095) when the PWM signal shoukd turn on
 * @param[in] off_time The time (0 to 4095) when the PWM signal shoukd turn off
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_set_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t on_time, uint16_t off_time);

/**
 * @brief Set the PWM frequency on the PCA9685
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * @param[in] freq Desired PWM frequence in Hz
 * 
 * @return ESP_OK Success, or ESP_FAIL
 */
esp_err_t pca9685_set_pwm_freq(pca9685_t *pca9685, float freq);

/**
 * @brief Set frequency oscillator on the PCA9685 structure
 * 
 * @param[in] pca9685 Pointer to the PCA9685 structure
 * @param[in] freq Value frequenct oscillator
 */
void pca9685_set_osc_freq(pca9685_t *pca9685, uint32_t freq);

#endif