#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "acs712.h"
#include "i2c_tools.h"
#include "pca9685.h"
#include "servo_control.h"

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

// Parametrs I2C PCA9685
#define PCA9685_I2C_ADDR            0x40

void app_main(void)
{
    // Init I2C
    // TODO: разделить также как в pca9685
    i2c_bus_t i2c_bus = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_init(&i2c_bus));

    // ESP_ERROR_CHECK(i2c_master_scan(&i2c_bus));

    // Init PCA9685
    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };

    pca9685_handle_t pca9685_handle;

    ESP_ERROR_CHECK(pca9685_init(&pca9685_config, &pca9685_handle));
    ESP_ERROR_CHECK(pca9685_set_pwm_freq(pca9685_handle, 50));

    ESP_ERROR_CHECK(pca9685_set_pwm(pca9685_handle, 0, 0, 512));

    uint16_t on_time;
    uint16_t off_time;

    ESP_ERROR_CHECK(pca9685_get_pwm(pca9685_handle, 0, &on_time, &off_time));
    printf("Channel 0 - On time: %d, Off time: %d\n", on_time, off_time);

    // Init Servo
    servo_t servo_10;
    ESP_ERROR_CHECK(servo_init(&servo_10, pca9685_handle, 10));
    

    // Init ACS712
    // TODO: разделить также как в pca9685
    acs712_t acs;

    ESP_ERROR_CHECK(acs712_init(&acs, ACS712_ADC_UNIT, ACS712_ADC_CHANNEL, ACS712_ADC_ATTEN, ACS712_SENSITIVITY));

    float offset_voltage = acs712_calibrate(&acs);
    printf("Calibrated offset voltage: %.2f mV\n", offset_voltage);

    while (1) {
        ESP_ERROR_CHECK(servo_set_angle(&servo_10, 0));

        vTaskDelay(pdMS_TO_TICKS(300));

        ESP_ERROR_CHECK(servo_set_angle(&servo_10, 90));

        acs712_print_info(&acs, offset_voltage);

        vTaskDelay(pdMS_TO_TICKS(300));
    }

    acs712_deinit(&acs);
    i2c_master_deinit(&i2c_bus);
}