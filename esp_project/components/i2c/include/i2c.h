#ifndef I2C_H
#define I2C_H

#include "driver/i2c_master.h"

/**
 * @brief I2C bus configuration structure
 */
typedef struct {
    i2c_port_t port;            /**< I2C port number */
    gpio_num_t sda_io_num;      /**< GPIO number for SDA line */
    gpio_num_t scl_io_num;      /**< GPIO number for SCL line */
} i2c_config_bus_t;

/**
 * @brief I2C bus runtime structure
 */
typedef struct {
    i2c_master_bus_handle_t handle;     /**< Handle for the initialized I2C bus */
    bool is_initialized;                /**< Flag indicating if the bus is initialized */
} i2c_bus_t;

#define I2C_BUS_DEFAULT() { \
    .handle = NULL, \
    .is_initialized = false \
}

/**
 * @brief Initialize the I2C master bus
 * 
 * @param[in] bus Pointer to the I2C bus structure
 * @param[in] config Pointer to the I2C configuration structure
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if I2C bus initialized
 * @return ESP_ERR_INVALID_ARG if config is NULL
 * @return ESP_FAIL if failed initialized
 */
esp_err_t i2c_master_init(i2c_bus_t *bus, const i2c_config_bus_t *config);

/**
 * @brief Deinitialize the I2C master bus
 * 
 * @param[in] bus Pointer to I2C bus structure
 * 
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if I2C bus not initialize
 * @return ESP_FAIL if faled deinitialization
 */
esp_err_t i2c_master_deinit(i2c_bus_t *bus);

#endif