#include "uart_tools.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "uart_tools";

esp_err_t uart_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate, size_t buf_size)
{
    esp_err_t ret = ESP_FAIL;

    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed uart param config");
        return ret;
    }

    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed uart set pin");
        return ret;
    }

    ret = uart_driver_install(uart_num, buf_size * 2, buf_size * 2, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed uart driver install");
        return ret;
    }

    ESP_LOGI(TAG, "UART initialized on UART%d with TX pin: %d, RX pin: %d", uart_num, tx_pin, rx_pin);

    return ESP_OK;
}

esp_err_t uart_send_data(uart_port_t uart_num, char* data)
{
    int len = strlen(data);
    int written = uart_write_bytes(uart_num, data, len);
    if (written != len) {
        ESP_LOGE(TAG, "Failed to write all bytes to UART%d", uart_num);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t uart_receive_data(uart_port_t uart_num, char *buffer, size_t buffer_size, int timeout_ms)
{
    int length = uart_read_bytes(uart_num, (uint8_t*)buffer, buffer_size - 1, pdMS_TO_TICKS(timeout_ms));
    if (length < 0) {
        ESP_LOGE(TAG, "Failed to read from UART%d", uart_num);
        return ESP_FAIL;
    }

    buffer[length] = '\0';

    return ESP_OK;
}