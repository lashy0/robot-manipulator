#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define UART_NUM    UART_NUM_0
#define TX_PIN      GPIO_NUM_21
#define RX_PIN      GPIO_NUM_20
#define BUF_SIZE    1024

static void echo_task(void *arg)
{
    const uart_port_t uart_num = UART_NUM;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Настройка параметров UART
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    
    // Настройка пинов UART (TX и RX)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Установка буферизованного ввода-вывода UART
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));

    const char* test_str = "This is a test string.\n\r";

    while(1) {
        // Отправка данных через UART
        uart_write_bytes(uart_num, test_str, strlen(test_str));

        #if 0
        // Прием данных, если они есть
        uint8_t data[BUF_SIZE];
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len > 0) {
            data[len] = '\0';  // Добавляем нулевой символ для завершения строки
            ESP_LOGI(TAG, "Received: %s", (char *)data);
        }
        #endif

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Задержка 1 секунда
    }
}

void app_main(void)
{
    printf("Hello, world!\n");
    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
}