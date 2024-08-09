#ifndef I2C_TOOLS_H
#define I2C_TOOLS_H

#include "driver/i2c_master.h"

typedef struct {
    i2c_master_bus_handle_t handle;
    i2c_port_t port;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
    uint32_t clk_speed;
} i2c_bus_t;

esp_err_t i2c_master_init(i2c_bus_t *bus);
void i2c_master_deinit(i2c_bus_t *bus);
esp_err_t i2c_master_scan(i2c_bus_t *bus);
bool i2c_is_device_connect(i2c_bus_t *bus, uint8_t addr);

#endif