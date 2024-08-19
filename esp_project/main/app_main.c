#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include "usb_serial_jtag_tools.h"

static void Receive_callback(void *argument) {
    // Allocate buffers on the heap instead of stack
    uint8_t *rxbuf = (uint8_t *)malloc(4096); // для хранения не обработанных байтов
    char *input_buffer = (char *)malloc(4096); // для хранения вводимых данных в консоле

    if (rxbuf == NULL || input_buffer == NULL) {
        printf("Failed to allocate memory!\n");
        return;
    }

    size_t bytes_read = 0;
    size_t input_length = 0;

    for (;;) {
        // Read data from USB Serial JTAG (non-blocking, short timeout)
        bytes_read = usb_serial_jtag_read_bytes(rxbuf, 4096, 10 / portTICK_PERIOD_MS);
        
        if (bytes_read > 0) {
            // Process received bytes
            for (size_t i = 0; i < bytes_read; i++) {
                // Check for newline or carriage return (Enter key)
                if (rxbuf[i] == '\n' || rxbuf[i] == '\r') {
                    // Null-terminate and process the input when Enter is pressed
                    input_buffer[input_length] = '\0';
                    printf("You entered: %s\n", input_buffer);

                    // Reset input length for the next line of input
                    input_length = 0;
                } else {
                    // Append character to input buffer (unless buffer is full)
                    if (input_length < 4095) {
                        input_buffer[input_length++] = rxbuf[i];
                    }
                }
            }
        }

        // Short delay to allow other tasks to run
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Free allocated buffers (although not reached in this infinite loop)
    free(rxbuf);
    free(input_buffer);
}

void app_main(void) {
    // Initialize the USB Serial JTAG using the component function
    esp_err_t ret = usb_serial_jtag_init();
    if (ret != ESP_OK) {
        printf("Failed to initialize USB Serial JTAG\n");
        return;
    }

    // Create task for receiving data
    xTaskCreate(Receive_callback, "UART receive callback", 8192, NULL, 5, NULL);
}
