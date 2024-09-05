#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c_master.h"

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

#define PCA9685_PRESCALE_MIN    3
#define PCA9685_PRESCALE_MAX    255
#define FREQUENCY_OSCILLATOR    25000000

/**
 * @brief Config structure for PCA9685 driver
 */
typedef struct {
    uint8_t i2c_address;                    /*!< I2C address of the PCA9685 device */
    i2c_master_bus_handle_t bus_handle;     /*!< I2C bus handle */
    uint32_t scl_speed;                     /*!< I2C clock speed */
} pca9685_config_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint32_t oscillator_freq;
    bool is_initialized;
} pca9685_t;

/**
 * @brief Write a byte of data to a PCA9685 refister
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @param[in] reg Register address to write 
 * @param[in] data Data bytes to write
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_write(pca9685_t *handle, uint8_t reg, uint8_t data);

/**
 * @brief Read a byte of data from a PCA9685 register
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @param[in] reg Register address to write
 * @param[out] data Pointer to store the read data
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_read(pca9685_t *handle, uint8_t reg, uint8_t *data);

/**
 * @brief Initialize the PCA9685 device
 * 
 * @param[in] config Configuration structure for PCA9685
 * @param[in] handle Handle to the PCA9685 device
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_init(pca9685_config_t *config, pca9685_t *handle);

/**
 * @brief Set the PWM values for a channel on the PCA9685
 * 
 * This function sets the on_time and off_time for a given channel on the PCA9685.
 * Each channel has two 12-bit values for the PWM siganl:
 * 
 * - The on_time defines when the signal should go high within the 4096 step PWM cycle.
 * 
 * - The off_time defines when the signal should go low within the same cycle.
 * 
 * The PCA9685 allows control of up to 16 independent channel.
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @param[in] channel The channel number (0 to 15) to set the PWM
 * @param[in] on_time The time (0 t0 4095) when the PWM signal should turn on
 * @param[in] off_time The time (0 to 4095) when the PWM signal should torn off
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_set_pwm(pca9685_t *handle, uint8_t channel, uint16_t on_time, uint16_t off_time);

/**
 * @brief Get the PWM values for a channel on the PCA9685
 * 
 * This function reads the on_time and off_time for a given channel on the PCA9685.
 * It retrieves the 12-bit on_time and off_time values from the corresponding PWM
 * registers for the channel.
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @param[in] channel The channel number (0 to 15) to set the PWM
 * @param[out] on_time Pointer to store the retrieved on_time value (0 to 4095)
 * @param[out] off_time Pointer to store the retrieved off_time value (0 to 4095)
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_get_pwm(pca9685_t *handle, uint8_t channel, uint16_t *on_time, uint16_t *off_time);

/**
 * @brief Set the PWM frequency on the PCA9685
 * 
 * This function configures the PWM frequency by calculating and setting the
 * appropriate prescaler value in the PCA9685 register. The device is put into
 * sleep mode temporarily to change the prescaler and then restarted.
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @param[in] freq Desired PWM frequency in Hz
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_set_pwm_freq(pca9685_t *handle, float freq);

esp_err_t pca9685_set_ext_clk(pca9685_t *handle, uint8_t prescale);

/**
 * @brief Reset the PCA9685 device
 * 
 * This function sends the RESTART command ti the PCA9685 by writing to the MODE1
 * register, which resets the device and restarts the PWM controller.
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_reset(pca9685_t *handle);

/**
 * @brief Put thr PCA9685 device into sleep mode
 * 
 * This function reads thr MODE1 register of the PCA9685, sets the SLEEP bit, and
 * writes it back to the MODE1 register to put the device into sleep mode.
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_sleep(pca9685_t *handle);

/**
 * @brief Wake the PCA9685 device from sleep mode
 * 
 * This function reads mode the MODE1 register of the PCA9685, clears the SLEEP bit,
 * and writes it back to the MODE1 register to wake the device from sleep mode.
 * 
 * @param[in] handle Handle to the PCA9685 device
 * @return esp_err_t ESP_OK on success, or an error code other
 */
esp_err_t pca9685_wake(pca9685_t *handle);

/**
 * Something is wrong with the implementation
 */
// void pca9685_deinit(pca9685_t *handle);

void pca9685_set_osc_freq(pca9685_t *handle, uint32_t freq);

void pca9685_get_osc_freq(pca9685_t *handle, uint32_t *freq);

esp_err_t pca9685_get_prescale(pca9685_t *handle, uint8_t *data);

#endif