#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include "driver/i2c_master.h"
#include "esp_log.h"

// Default I2C values
#define I2C_MASTER_SCL_IO           GPIO_NUM_7
#define I2C_MASTER_SDA_IO           GPIO_NUM_6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

struct i2c_config_esp32
{
    int scl_io;
    int sda_io;
    i2c_port_t port;
    uint32_t freq_hz;
    uint32_t timeout_ms;
};

esp_err_t i2c_master_init(void);

i2c_master_bus_handle_t i2c_get_bus_handle(void);

bool i2c_check_device_connection(uint8_t device_addr);

void i2c_scan_device(void);

#endif