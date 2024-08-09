#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c_master.h"

#define PCA9685_I2C_TIMEOUT_MS      1000

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

typedef struct {
    uint8_t i2c_address;
    i2c_master_bus_handle_t bus_handle;
    uint32_t scl_speed;
} pca9685_config_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint8_t i2c_address;
} pca9685_t;

typedef pca9685_t* pca9685_handle_t;

esp_err_t pca9685_write(pca9685_handle_t handle, uint8_t reg, uint8_t data);
esp_err_t pca9685_read(pca9685_handle_t handle, uint8_t reg, uint8_t *data);
esp_err_t pca9685_init(pca9685_config_t *pca9685_config, pca9685_handle_t *pca9685_handle);
esp_err_t pca9685_set_pwm(pca9685_handle_t handle, uint8_t channel, uint16_t on_time, uint16_t off_time);
esp_err_t pca9685_get_pwm(pca9685_handle_t handle, uint8_t channel, uint16_t *on_time, uint16_t *off_time);
esp_err_t pca9685_set_pwm_freq(pca9685_handle_t handle, float freq);
esp_err_t pca9685_reset(pca9685_handle_t handle);
esp_err_t pca9685_sleep(pca9685_handle_t handle);
esp_err_t pca9685_wake(pca9685_handle_t handle);

#endif