#include "i2c_handler.h"

static const char *TAG = "i2c_handler";
static i2c_master_bus_handle_t bus_handle;

esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus initializtion failde");
        return err;
    }
    ESP_LOGI(TAG, "I2C master bus initialized successfully");

    return ESP_OK;
}

i2c_master_bus_handle_t i2c_get_bus_handle(void)
{
    return bus_handle;
}

bool i2c_check_device_connection(uint8_t device_addr)
{
    esp_err_t ret = i2c_master_probe(bus_handle, device_addr, I2C_MASTER_TIMEOUT_MS);
    if (ret == ESP_OK) {
        printf("Device 0x%02x found on the I2C bus\n", device_addr);
        return true;
    }
    else {
        printf("Device 0x%02x not found on the I2C bus\n", device_addr);
        return false;
    }
}

void i2c_scan_device(void)
{
    esp_err_t ret;
    printf("Scanning I2C bus...\n");
    for (int i = 1; i < 127; i++) {
        ret = i2c_master_probe(bus_handle, i, I2C_MASTER_TIMEOUT_MS);
        if (ret == ESP_OK) {
            printf("Found device at address: 0x%02x\n", i);
        }
    }
}