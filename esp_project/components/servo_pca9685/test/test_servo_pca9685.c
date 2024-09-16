#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"

#include "i2c.h"
#include "pca9685.h"
#include "servo_pca9685.h"

#define PCA9685_I2C_ADDR           0x40

#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

void app_main(void) {
    esp_err_t ret;

    // Initialize I2C
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
    };

    i2c_bus_t i2c_bus = {
        .is_initialized = false,
    };

    i2c_master_init(&i2c_bus, &i2c_master_config);

    // Initialize PCA9685
    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };
    
    // ???
    pca9685_t pca9685 = {
        .is_initialized = false,
    };

    ret = pca9685_init(&pca9685_config, &pca9685);
    if (ret != ESP_OK) {
        return;
    }
    printf("PCA9685 initialize\n");

    servo_t servo10 = {
        .pca9685 = pca9685,
        .min_pulse_width = 500,
        .max_pulse_width = 2500,
        .max_angle = 180,
        .channel = 10
    };

    ret = pca9685_deinit(&pca9685);
    if (ret != ESP_OK) {
        return;
    }
    printf("PCA9685 deinitialize\n");
}