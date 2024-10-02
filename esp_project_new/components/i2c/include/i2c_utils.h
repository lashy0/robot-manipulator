#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include "i2c.h"

/**
 * @brief Scan the I2C bus for devices
 *
 * This function scans all 128 possible addresses on the I2C bus and prints
 * the addresses of any devices that respond.
 *
 * @param[in] bus Pointer to the I2C bus structure
 * @param[in] timeout_ms Timeout in milliseconds for each device probe
 */
void i2c_master_scan(i2c_bus_t *bus, uint32_t timeout_ms);

/**
 * @brief Check if an I2C device is connected at a specific address
 *
 * Probes the I2C bus to check if a device responds at the given address.
 * The probe operation is configurable with a custom timeout.
 *
 * @param[in] bus Pointer to the I2C bus structure
 * @param[in] addr The I2C address to probe
 * @param[in] timeout_ms Timeout in milliseconds for the probe
 * @return true if the device is found, false if not
 */
bool i2c_is_device_connected(i2c_bus_t *bus, uint8_t addr, uint32_t timeout_ms);

#endif
