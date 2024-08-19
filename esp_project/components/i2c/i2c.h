#ifndef I2C_H
#define I2C_H

#include "driver/i2c_master.h"

/**
 * @brief I2C bus configuration structure
 */
typedef struct {
    i2c_port_t port;            /*!< I2C port number */
    gpio_num_t sda_io_num;      /*!< GPIO number for SDA line */
    gpio_num_t scl_io_num;      /*!< GPIO number for SCL line */
} i2c_config_bus_t;

/**
 * @brief I2C bus runtime structure
 */
typedef struct {
    i2c_master_bus_handle_t handle;     /*!< Handle for the initialized I2C bus */
    bool is_initialized;                /*!< Flag indicating if the bus is initialized */
} i2c_bus_t;

/**
 * @brief Initialize the I2C master bus
 * 
 * Initializes an I2C bus with the given configuration.
 * 
 * @param[in] bus Pointer to the I2C bus structure
 * @param[in] config Pointer to the I2C configuration structure
 * @return esp_err_t ESP_OK if successful, or an error code if initialization fails
 */
esp_err_t i2c_master_init(i2c_bus_t *bus, const i2c_config_bus_t *config);

/**
 * @brief Deinitialize the I2C master bus
 * 
 * Cleans up resources used by the I2C bus.
 * 
 * @param[in] bus Pointer to I2C bus structure
 */
void i2c_master_deinit(i2c_bus_t *bus);

#endif