#include "i2c_tools.h"
#include "esp_log.h"

#define I2C_MASTER_TIMEOUT_DEFAULT 1000

static const char *TAG = "i2c_tools";

esp_err_t i2c_master_init(i2c_bus_t *bus)
{
    esp_err_t ret = ESP_FAIL;

    i2c_master_bus_config_t bus_config = {
        .scl_io_num = bus->scl_io_num,
        .sda_io_num = bus->sda_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    
    ret = i2c_new_master_bus(&bus_config, &bus->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized success");

    return ESP_OK;
}

void i2c_master_deinit(i2c_bus_t *bus)
{
    i2c_del_master_bus(bus->handle);
}

esp_err_t i2c_master_scan(i2c_bus_t *bus)
{
    esp_err_t ret = ESP_FAIL;

    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (int i = 0; i < 128; i++) {
        ret = i2c_master_probe(bus->handle, i, I2C_MASTER_TIMEOUT_DEFAULT);
        if (ret == ESP_OK) {
            printf("Found device at address: 0x%02x\n", i);
        }
    }

    return ESP_OK;
}

bool i2c_is_device_connet(i2c_bus_t *bus, uint8_t addr)
{
    esp_err_t ret = ESP_FAIL;

    ret = i2c_master_probe(bus->handle, i2c_addr, I2C_MASTER_TIMEOUT_DEFAULT);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device found at address: 0x%02x", addr);
        return true;
    }
    else {
        ESP_LOGI(TAG, "No device found at address: 0x%02x", addr);
        return false;
    }
}