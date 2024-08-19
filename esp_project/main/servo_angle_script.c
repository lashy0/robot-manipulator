#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include "i2c.h"
#include "pca9685.h"
#include "acs712.h"
#include "servo.h"

#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

#define PCA9685_I2C_ADDR           0x40

#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

servo_t servo;
acs712_t acs712;
int offset_voltage;

// Function to control the servo based on input angle and measure the current
static void set_servo_angle_and_measure_current(float angle) {
    float current;
    int raw;

    // Set the angle of the servo
    if (servo_set_angle(&servo, angle) == ESP_OK) {
        printf("Servo set to angle: %.2f degrees\n", angle);
        
        acs712_read_raw(&acs712, &raw);
        acs712_raw_to_current(&acs712, &current, offset_voltage, raw);
        printf("Measured current: %.2f A\n", current);
    } else {
        printf("Failed to set servo angle\n");
    }
}

static void Receive_callback(void *argument) {
    // Allocate buffers on the heap
    uint8_t *rxbuf = (uint8_t *)malloc(4096);
    char *input_buffer = (char *)malloc(4096);

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
            for (size_t i = 0; i < bytes_read; i++) {
                if (rxbuf[i] == '\n' || rxbuf[i] == '\r') {
                    input_buffer[input_length] = '\0';
                    printf("You entered: %s\n", input_buffer);

                    // Parse the input angle
                    float angle = atof(input_buffer);
                    if (angle >= 0 && angle <= servo.max_angle) {
                        set_servo_angle_and_measure_current(angle);
                    } else {
                        printf("Invalid angle. Please enter a value between 0 and %.2f\n", servo.max_angle);
                    }

                    printf("Done\n");
                    
                    // Reset input length for the next line of input
                    input_length = 0;
                } else {
                    // Append character to input buffer
                    if (input_length < 4095) {
                        input_buffer[input_length++] = rxbuf[i];
                    }
                }
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(rxbuf);
    free(input_buffer);
}

void app_main(void) {
    // Initialize I2C for PCA9685 (Servo Controller)
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
    };

    i2c_bus_t i2c_bus = {
        .is_initialized = false,
    };

    i2c_master_init(&i2c_bus, &i2c_master_config);

    // Initialize PCA9685 for Servo Control
    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };
    
    pca9685_t pca9685 = {
        .is_initialized = false,
    };

    pca9685_init(&pca9685_config, &pca9685);

    pca9685_set_pwm_freq(&pca9685, 50);

    // Initialize the servo (on channel 0, with a max angle of 180 degrees)
    servo_init(&servo, pca9685, 10, 180);

    // Initialize ACS712 current sensor
    acs712.adc_channel = ACS712_ADC_CHANNEL;
    acs712.sensitivity = ACS712_SENSITIVITY;

    acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN);

    acs712_calibrate_voltage(&acs712, &offset_voltage);

    // Set up USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024
    };
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        return;
    }

    esp_vfs_usb_serial_jtag_use_driver();

    // Create task for receiving data
    xTaskCreate(Receive_callback, "USB Serial Receive Callback", 8192, NULL, 5, NULL);
}
