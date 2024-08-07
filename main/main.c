#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "acs712.h"
#include "i2c_tools.h"

// Parameters ADC ACS712T 5A
#define ACS712_ADC_CHANNEL          ADC_CHANNEL_2
#define ACS712_ADC_UNIT             ADC_UNIT_1
#define ACS712_ADC_ATTEN            ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY          185.0

// Parameters I2C
#define I2C_MASTER_SCL_IO           GPIO_NUM_7
#define I2C_MASTER_SDA_IO           GPIO_NUM_6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

void app_main(void)
{
    // Init I2C
    i2c_bus_t i2c_bus = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_init(&i2c_bus));

    ESP_ERROR_CHECK(i2c_master_scan(&i2c_bus));

    // Init ACS712
    acs712_t acs;

    ESP_ERROR_CHECK(acs712_init(&acs, ACS712_ADC_UNIT, ACS712_ADC_CHANNEL, ACS712_ADC_ATTEN, ACS712_SENSITIVITY));

    while (1) {
        float current;
        int voltage;
        int raw;

        ESP_ERROR_CHECK(acs712_read_raw(&acs, &raw));
        ESP_ERROR_CHECK(acs712_read_voltage(&acs, &voltage));
        ESP_ERROR_CHECK(acs712_read_current(&acs, &current));

        printf("Raw: %d\tVoltage: %d mV\tCurrent: %2f A\n", raw, voltage, current);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    acs712_deinit(&acs);
    i2c_master_deinit(&i2c_bus);
}