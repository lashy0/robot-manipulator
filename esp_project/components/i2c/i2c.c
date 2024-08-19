#include "esp_log.h"
#include "i2c.h"

static const char *TAG = "i2c";

esp_err_t i2c_master_init(i2c_bus_t *bus, const i2c_config_bus_t *config)
{
    if (bus->is_initialized) {
        ESP_LOGW(TAG, "I2C bus is already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_FAIL;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = config->port,
        .scl_io_num = config->scl_io_num,
        .sda_io_num = config->sda_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    
    ret = i2c_new_master_bus(&bus_config, &bus->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized successfully");
    bus->is_initialized = true;

    return ESP_OK;
}

void i2c_master_deinit(i2c_bus_t *bus)
{
    if (!bus->is_initialized) {
        ESP_LOGW(TAG, "I2C bus is not initialized");
    }

    esp_err_t ret = i2c_del_master_bus(bus->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize I2C bus");
    }
    else {
        ESP_LOGI(TAG, "I2C bus deinitialized successfully");
        bus->is_initialized = false;
    }
}