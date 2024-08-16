#include "i2c_utils.h"
#include "esp_log.h"

static const char *TAG = "i2c_utils";

esp_err_t i2c_master_scan(i2c_bus_t *bus, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t found_devices[128];
    int found_count = 0;

    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (int i = 0; i < 128; i++) {
        ret = i2c_master_probe(bus->handle, i, timeout_ms);
        if (ret == ESP_OK) {
            found_devices[found_count++] = i;
        }
    }

    if (found_count > 0) {
        printf("Found %d device(s) at addresses: ", found_count);
        for (int i = 0; i < found_count; i++) {
            printf("0x%02x ", found_devices[i]);
        }
        printf("\n");
    }
    else {
        printf("No I2C devices found.\n");
    }

    return ESP_OK;
}

bool i2c_is_device_connected(i2c_bus_t *bus, uint8_t addr, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_FAIL;

    ret = i2c_master_probe(bus->handle, addr, timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device found at address: 0x%02x", addr);
        return true;
    } else {
        ESP_LOGI(TAG, "No device found at address: 0x%02x", addr);
        return false;
    }
}
