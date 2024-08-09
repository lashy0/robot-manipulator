#ifndef UART_TOOLS_H
#define UART_TOOLS_H

#include "driver/uart.h"

esp_err_t uart_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate, size_t buf_size);
esp_err_t uart_send_data(uart_port_t uart_num, char *data);
esp_err_t uart_receive_data(uart_port_t uart_num, char *buffer, size_t buffer_size, int timeout_ms);

#endif