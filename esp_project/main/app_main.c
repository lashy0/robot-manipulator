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
#include "servo_pca9685.h"
#include "arm_robot.h"

#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

#define PCA9685_I2C_ADDR           0x40

#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

#define BUF_SIZE 4096
#define TASK_STACK_SIZE 4096

arm_robot_t robot;
acs712_t acs712;

void process_command(char *input) {
    esp_err_t ret;

    if (strncmp(input, "GET_ANGLE", 9) == 0) {
        uint8_t channel;
        float cur_angle;

        // TODO: получше решение для получения значений с USB Serial JTAG
        char subbuff[10];
        memcpy(subbuff, &input[9], 9);
        subbuff[10] = '\0';

        sscanf(subbuff, "%hhu", &channel);

        // TODO: как то избавиться от такой конструкции
        if (robot.base_servo.channel == channel) {
            ret = servo_pca9685_get_angle(&robot.base_servo, &cur_angle, PWM_FREQUENCY);
            printf("Base (pwm 10) servo angle: %.2f\n", cur_angle);
        }
        else if (robot.shoulder_servo.channel == channel) {
            ret = servo_pca9685_get_angle(&robot.shoulder_servo, &cur_angle, PWM_FREQUENCY);
            printf("Shoulder (pwm 11) servo angle: %.2f\n", cur_angle);
        }
        else if (robot.elbow_servo.channel == channel) {
            ret = servo_pca9685_get_angle(&robot.elbow_servo, &cur_angle, PWM_FREQUENCY);
            printf("Elbow (pwm 12) servo angle: %.2f\n", cur_angle);
        }
        else if (robot.wrist_rot_servo.channel == channel) {
            ret = servo_pca9685_get_angle(&robot.wrist_rot_servo, &cur_angle, PWM_FREQUENCY);
            printf("Wrist rot (pwm 13) servo angle: %.2f\n", cur_angle);
        }
        else if (robot.wrist_ver_servo.channel == channel) {
            ret = servo_pca9685_get_angle(&robot.wrist_ver_servo, &cur_angle, PWM_FREQUENCY);
            printf("Wrist ver (pwm 14) servo angle: %.2f\n", cur_angle);
        }
        else if (robot.gripper_servo.channel == channel) {
            ret = servo_pca9685_get_angle(&robot.gripper_servo, &cur_angle, PWM_FREQUENCY);
            printf("Gripper (pwm 15) servo angle: %.2f\n", cur_angle);
        }
        else {
            printf("None\n");
        }
    }
    if (strncmp(input, "SET_ANGLE", 9) == 0) {
        uint8_t channel;
        float angle;

        char subbuff[10];
        memcpy(subbuff, &input[9], 9);
        subbuff[10] = '\0';

        sscanf(subbuff, "%hhu %f", &channel, &angle);
        printf("angle: %.2f\n", angle);

        if (robot.base_servo.channel == channel) {
            ret = servo_pca9685_move_smooth(&robot.base_servo, angle, PWM_FREQUENCY, 2, 50);
        }
        else if (robot.shoulder_servo.channel == channel) {
            ret = servo_pca9685_move_smooth(&robot.shoulder_servo, angle, PWM_FREQUENCY, 2, 50);
        }
        else if (robot.elbow_servo.channel == channel) {
            ret = servo_pca9685_move_smooth(&robot.elbow_servo, angle, PWM_FREQUENCY, 2, 50);
        }
        else if (robot.wrist_rot_servo.channel == channel) {
            ret = servo_pca9685_move_smooth(&robot.wrist_rot_servo, angle, PWM_FREQUENCY, 2, 50);
        }
        else if (robot.wrist_ver_servo.channel == channel) {
            ret = servo_pca9685_move_smooth(&robot.wrist_ver_servo, angle, PWM_FREQUENCY, 2, 50);
        }
        else if (robot.gripper_servo.channel == channel) {
            ret = servo_pca9685_move_smooth(&robot.gripper_servo, angle, PWM_FREQUENCY, 2, 50);
        }
        else {
            printf("None\n");
        }

        printf("Servo on channel %d set to angle: %.2f degress\n", channel, angle);     
    }

    printf("Done\n");
}

static void usb_serial_task(void *arg) {
    uint8_t *rxbuf = (uint8_t *)malloc(BUF_SIZE);
    char *input_buffer = (char *)malloc(BUF_SIZE);

    if (rxbuf == NULL || input_buffer == NULL) {
        printf("Failed to allocate memory!\n");
        return;
    }

    size_t bytes_read = 0;
    size_t input_length = 0;

    while(1) {
        // Read data from USB Serial
        bytes_read = usb_serial_jtag_read_bytes(rxbuf, BUF_SIZE, 10 / portTICK_PERIOD_MS);

        if (bytes_read > 0) {
            for (size_t i = 0; i < bytes_read; i++) {
                if (rxbuf[i] == '\n') {
                    input_buffer[input_length] = '\0';

                    process_command(input_buffer);

                    input_length = 0;
                }
                else {
                    if (input_length < BUF_SIZE - 1) {
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
    esp_err_t ret;

    // Initialize I2C
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
    };

    i2c_bus_t i2c_bus = {
        .is_initialized = false,
    };

    i2c_master_init(&i2c_bus, &i2c_master_config);

    // Initialize PCA9685
    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };
    
    // ???
    pca9685_t pca9685 = {
        .is_initialized = false,
    };

    ret = pca9685_init(&pca9685_config, &pca9685);
    if (ret != ESP_OK) {
        return;
    }
    printf("PCA9685 initialize\n");

    ret = pca9685_set_pwm_freq(&pca9685, 50);
    if (ret != ESP_OK) {
        return;
    }
    printf("PCA9685 set 50 Hz\n");

    // Initialize ACS712 5A
    acs712.adc_channel = ACS712_ADC_CHANNEL;
    acs712.sensitivity = ACS712_SENSITIVITY;

    ret = acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN);
    if (ret != ESP_OK) {
        return;
    }
    printf("ACS712 5A initialize\n");

    // Initialize robot
    arm_robot_init(&robot, pca9685);

    // Set home state robot
    // ret = arm_robot_home_state(&robot);
    // if (ret != ESP_OK) {
    //     return;
    // }
    printf("Arm robot initialize\n");

    // Set up USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE
    };
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        return;
    }

    esp_vfs_usb_serial_jtag_use_driver();
    printf("USB Serial JTAG initialize\n");
    
    // Main
    xTaskCreate(usb_serial_task, "USB Serial Task", BUF_SIZE * 2, NULL, 5, NULL);

    // Deinit
    // ret = pca9685_deinit(&pca9685);
    // if (ret != ESP_OK) {
    //     return;
    // }

    // ret = acs712_deinit(&acs712);
    // if (ret != ESP_OK) {
    //     return;
    // }
}