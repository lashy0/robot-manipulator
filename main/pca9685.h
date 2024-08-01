#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c_master.h"
#include "esp_log.h"

#define PCA9685_ADDR            0x40
#define PWM_DEFAULT_FREQ                50
#define FREQUENCY_OSCILLATOR    25000000

#define PCA9685_PRESCALE_MIN    3
#define PCA9685_PRESCALE_MAX    255

// Register addres
#define PCA9685_MODE1           0x00
#define PCA9685_PRESCALE        0xFE
#define PCA9685_LED0_ON_L       0x06

// MODE1 bits
#define MODE1_ALLCAL            0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3              0x02  /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2              0x04  /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1              0x08  /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP             0x10  /**< Low power mode. Oscillator off */
#define MODE1_AI                0x20  /**< Auto-Increment enabled */
#define MODE1_EXTCLK            0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART           0x80  /**< Restart enabled */

esp_err_t pca9685_init(void);

esp_err_t pca9685_write(uint8_t reg, uint8_t data);

esp_err_t pca9685_read(uint8_t reg, uint8_t *data);

esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);

esp_err_t pca9685_get_pwm(uint8_t channel, uint16_t *on, uint16_t *off);

esp_err_t pca9685_set_pwm_freq(uint16_t freq_hz);

#endif