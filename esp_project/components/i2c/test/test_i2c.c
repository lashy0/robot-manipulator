#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"

#include "i2c.h"
#include "i2c_utils.h"

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

    ret = i2c_master_init(&i2c_bus, &i2c_master_config);
    if (ret != ESP_OK) {
        return;
    }
    printf("I2C bus initialize\n");

    i2c_master_scan(&i2c_bus, 50);

    ret = i2c_master_deinit(&i2c_bus);
    if (ret != ESP_OK) {
        return;
    }
    printf("I2C bus deinitialize\n");
    
}