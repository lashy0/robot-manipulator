#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "i2c_handler.h"
#include "pca9685.h"

static const char *TAG = "main";

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(pca9685_init());

    // i2c_scan_device();
}
