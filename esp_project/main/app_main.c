#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"

#include "acs712.h"
#include "servo.h"
#include "i2c.h"
#include "pca9685.h"

#define PCA9685_I2C_ADDR           0x40

#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

#define BUF_SIZE 4096
#define TASK_STACK_SIZE 4096

static int offset_voltage;
acs712_t acs712;
servo_t servo;

// Фильтр ADC RAW, среднее значение 
#define FILTER_SIZE 20

int average_filter(int data) {
    static int samples[FILTER_SIZE] = {0};
    static int sum = 0;
    static int index = 0;
    static int count = 0;

    sum -= samples[index];
    samples[index] = data;
    sum += data;

    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) {
        count++;
    }

    return sum / count;
}

// Вывод значений ACS712 с применением фильтра
void print_data() {
    int raw_data;
    acs712_read_raw(&acs712, &raw_data);
    int filtererd_data = average_filter(raw_data);
    // int filtererd_data = kalman_update(raw_data);

    int voltage;
    acs712_raw_to_voltage(&acs712, &voltage, filtererd_data);

    float current;
    current = (float)(voltage - offset_voltage) / acs712.sensitivity;

    printf("Raw: %d\t Filtered Raw: %d\tVoltage: %d mV\t Offset Voltage: %d mV\tCurrent: %2f A\n", raw_data, filtererd_data, voltage, offset_voltage, current);
}

static void adc_task(void *pvParameter) {
	while (1) {
        print_data();
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}

static void usb_serial_task(void *argument) {
    //
    uint8_t *rxbuf = (uint8_t *)malloc(BUF_SIZE);
    char *input_buffer = (char *)malloc(BUF_SIZE);

    if (rxbuf == NULL || input_buffer == NULL) {
        printf("Failed to allocate memory!\n");
        return;
    }

    size_t bytes_read = 0;
    size_t input_length = 0;

    for(;;) {
        bytes_read = usb_serial_jtag_read_bytes(rxbuf, BUF_SIZE, 10 / portTICK_PERIOD_MS);

        if (bytes_read > 0) {
            for (size_t i = 0; i < bytes_read; i++) {
                if (rxbuf[i] == '\n' || rxbuf[i] == '\r') {
                    input_buffer[input_length] = '\0';
                    printf("You entered: %s\n", input_buffer);

                    uint8_t channel;
                    float angle;
                    if (sscanf(input_buffer, "%hhu %f", &channel, &angle) == 2) {
                        printf("Servo on channel %d set to angle: %.2f degrees\n", channel, angle);
                    }

                    printf("Done\n");

                    input_length = 0;
                }
                else {
                    if (input_length < BUF_SIZE - 1) {
                        input_buffer[input_length++] = rxbuf[i];
                    }
                }
            }
        }
        // else {
        //     printf("No data received\n");
        // }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(rxbuf);
    free(input_buffer);
}

void app_main(void)
{
    esp_err_t ret;

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

    printf("I2C\n");
    
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

    printf("PCA9685\n");

    // Initialize ACS712
    acs712.adc_channel = ACS712_ADC_CHANNEL;
    acs712.sensitivity = ACS712_SENSITIVITY;

    acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN);

    int raw;
    int sum = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_raw(&acs712, &raw) == ESP_OK) {
            sum += raw;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    int average = sum / samples;
    ret = acs712_raw_to_voltage(&acs712, &offset_voltage, average);
    if (ret != ESP_OK) {
        printf("Failed to convert ADC raw to voltage");
        return;
    }

    printf("Calibrated offset voltage: %d mV\n", offset_voltage);

    printf("ACS712\n");

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);

    // Set up USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE
    };
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        return;
    }

    // usb_serial_jtag_vfs_use_driver();
    esp_vfs_usb_serial_jtag_use_driver();
    printf("Driver initialized successfully");

    // Initialize Servo
    servo_init(&servo, pca9685, 10, 180);

    xTaskCreate(usb_serial_task, "USB Serial Task", 8192, NULL, 5, NULL);
}