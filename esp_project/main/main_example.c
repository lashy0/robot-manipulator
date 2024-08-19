#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "i2c.h"
#include "i2c_utils.h"
#include "pca9685.h"
#include "acs712.h"
#include "servo.h"

// Parameters I2C bus
#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

// Parametrs I2C PCA9685
#define PCA9685_I2C_ADDR           0x40

// Parameters ADC ACS712T 5A
#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

// Example
void app_main(void)
{
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
    };

    i2c_bus_t i2c_bus = {
        .is_initialized = false,
    };

    printf("I2C\n");

    i2c_master_init(&i2c_bus, &i2c_master_config);

    i2c_is_device_connected(&i2c_bus, PCA9685_I2C_ADDR, 50);

    printf("PCA9685\n");

    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };

    pca9685_t pca9685 = {
        .is_initialized = false,
    };

    pca9685_init(&pca9685_config, &pca9685);

    printf("Init PCA9685\n");

    pca9685_set_pwm_freq(&pca9685, 50);

    printf("ACS712\n");

    acs712_t acs712 = {
        .adc_channel = ACS712_ADC_CHANNEL,
        .sensitivity = ACS712_SENSITIVITY
    };

    acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN);

    int offset_voltage;
    acs712_calibrate_voltage(&acs712, &offset_voltage);
    printf("Calibrated offset voltage: %d mV\n", offset_voltage);

    int raw;
    int voltage;
    float current;

    acs712_read_raw(&acs712, &raw);
    acs712_raw_to_voltage(&acs712, &voltage, raw);
    acs712_raw_to_current(&acs712, &current, offset_voltage, raw);

    printf("Raw: %d\tVoltage: %d mV\tCurrent: %2f A\n", raw, voltage, current);

    servo_t servo;

    servo_init(&servo, pca9685, 10, 180);

    servo_set_angle(&servo, 90);
    vTaskDelay(pdMS_TO_TICKS(1000));

    servo_set_angle(&servo, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    servo_set_angle(&servo, 180);
    vTaskDelay(pdMS_TO_TICKS(1000));

    i2c_master_deinit(&i2c_bus);
    acs712_deinit(&acs712);
}