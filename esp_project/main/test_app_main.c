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

#define BUF_SIZE 128

arm_robot_t robot;
acs712_t acs712;

void servo_move_task(void *arg) {
    servo_t *servo = (servo_t *)arg;

    servo_pca9685_move_smoth_timer(servo);

    vTaskDelete(NULL);
}

void servos_move_task(void *arg) {
    arm_robot_t *robot = (arm_robot_t *)arg;

    arm_robot_movement_timer(robot);

    vTaskDelete(NULL);
}

bool get_movement_status() {
    return robot.is_moving;
}

// TODO: подумать как лучше организовать прошивки
// TODO: разделить на две прошивки, одна для работы с движениями, другая тестовая для проверки работы системы
// TODO: подробней разобраться с task в FreeRTOS
// TODO: попробовать ускорить получение команды ? т.е. сначало считываем что за команда (до встречи пробела) и уже делаем проверку на нее
void process_command(char *input) {

    if (strncmp(input, "GET_ANGLE", 9) == 0) {
        uint8_t channel;
        float cur_angle;

        // TODO: получше решение для получения значений с USB Serial JTAG
        char subbuff[10];
        // strncpy вместо memcpy?
        memcpy(subbuff, &input[9], 9);
        subbuff[10] = '\0';

        sscanf(subbuff, "%hhu", &channel);

        // TODO: переделать
        if (robot.base_servo.channel == channel) {
            servo_pca9685_get_angle(&robot.base_servo, &cur_angle, PWM_FREQUENCY);
            printf("Base (pwm %d) servo angle: %.2f\n", channel, cur_angle);
        }
        else if (robot.shoulder_servo.channel == channel) {
            servo_pca9685_get_angle(&robot.shoulder_servo, &cur_angle, PWM_FREQUENCY);
            printf("Shoulder (pwm %d) servo angle: %.2f\n", channel, cur_angle);
        }
        else if (robot.elbow_servo.channel == channel) {
            servo_pca9685_get_angle(&robot.elbow_servo, &cur_angle, PWM_FREQUENCY);
            printf("Elbow (pwm %d) servo angle: %.2f\n", channel, cur_angle);
        }
        else if (robot.wrist_servo.channel == channel) {
            servo_pca9685_get_angle(&robot.wrist_servo, &cur_angle, PWM_FREQUENCY);
            printf("Wrist rot (pwm %d) servo angle: %.2f\n", channel, cur_angle);
        }
        else if (robot.wrist_rot_servo.channel == channel) {
            servo_pca9685_get_angle(&robot.wrist_rot_servo, &cur_angle, PWM_FREQUENCY);
            printf("Wrist ver (pwm %d) servo angle: %.2f\n", channel, cur_angle);
        }
        else if (robot.gripper_servo.channel == channel) {
            servo_pca9685_get_angle(&robot.gripper_servo, &cur_angle, PWM_FREQUENCY);
            printf("Gripper (pwm %d) servo angle: %.2f\n", channel, cur_angle);
        }
        else {
            printf("None\n");
        }
    }
    else if (strncmp(input, "GET_STATUS_MOVING", 17) == 0) {
        char status_msg[16];
        snprintf(status_msg, sizeof(status_msg), "STATUS %d\n", get_movement_status());
        usb_serial_jtag_write_bytes((const uint8_t *)status_msg, strlen(status_msg), pdMS_TO_TICKS(100));
    }
    else if (strncmp(input, "SET_ANGLES", 10) == 0) {
        float angle_base;
        float angle_shoulder;
        float angle_elbow;
        float angle_wrist;
        float angle_wrist_rot;

        int delay = 10000;

        char subbuff[40];
        memcpy(subbuff, &input[10], 39);
        subbuff[40] = '\0';

        // printf("%s\n", subbuff);

        sscanf(subbuff, "%f %f %f %f %f", &angle_base, &angle_shoulder, &angle_elbow, &angle_wrist, &angle_wrist_rot);
        // printf("Angle base: %.2f; Angle shoulder: %.2f; Angle elbow: %.2f; Angle wrist: %.2f; Angle wrist rotation: %.2f\n", angle_base, angle_shoulder, angle_elbow, angle_wrist, angle_wrist_rot);

        if (!robot.is_moving) {
            printf("Settings angles\n");

            robot.base_target_angle = angle_base;
            robot.shoulder_target_angle = angle_shoulder;
            robot.elbow_target_angle = angle_elbow;
            robot.wrist_target_angle = angle_wrist;
            robot.wrist_rot_target_angle = angle_wrist_rot;
            robot.gripper_target_angle = 140;
            robot.delay = delay;

            xTaskCreate(servos_move_task, "servos move task", 4096, &robot, 2, NULL);
        }
        else {
            printf("None\n");
        }
    }
    else if (strncmp(input, "SET_ANGLE", 9) == 0) {
        uint8_t channel;
        float angle;

        char subbuff[10];
        memcpy(subbuff, &input[9], 9);
        subbuff[10] = '\0';

        sscanf(subbuff, "%hhu %f", &channel, &angle);
        // printf("Setting angle %.2f on channel %d\n", angle, channel);

        float step = 2;
        int delay = 40000; // мс для esp_timer

        // TODO: переделать
        if (robot.base_servo.channel == channel) {
            if (!robot.base_servo.is_busy) {
                printf("Setting angle %.2f on channel %d\n", angle, channel);
                // servo_pca9685_move_smooth(&robot.base_servo, angle, PWM_FREQUENCY, step, delay);

                robot.base_servo.step = step;
                robot.base_servo.delay = delay;
                robot.base_servo.target_angle = angle;
                xTaskCreate(servo_move_task, "base servo move task", 4096, &robot.base_servo, 1, NULL);
            }
            else {
                robot.base_servo.target_angle = angle;
                printf("Updating target angle for base servo to %.2f\n", angle);
            }
        }
        else if (robot.shoulder_servo.channel == channel) {
            if (!robot.shoulder_servo.is_busy) {
                // printf("Setting angle %.2f on channel %d\n", angle, channel);

                robot.shoulder_servo.step = step;
                robot.shoulder_servo.delay = delay;
                robot.shoulder_servo.target_angle = angle;
                xTaskCreate(servo_move_task, "shoulder servo move task", 4096, &robot.shoulder_servo, 1, NULL);
            }
            else {
                robot.shoulder_servo.target_angle = angle;
                // printf("Updating target angle for shoulder servo to %.2f\n", angle);
            }
        }
        else if (robot.elbow_servo.channel == channel) {
            if (!robot.elbow_servo.is_busy) {
                // printf("Setting angle %.2f on channel %d\n", angle, channel);

                robot.elbow_servo.step = step;
                robot.elbow_servo.delay = delay;
                robot.elbow_servo.target_angle = angle;
                xTaskCreate(servo_move_task, "elbow servo move task", 4096, &robot.elbow_servo, 1, NULL);
            }
            else {
                robot.elbow_servo.target_angle = angle;
                // printf("Updating target angle for elbow servo to %.2f\n", angle);
            }
        }
        else if (robot.wrist_servo.channel == channel) {
            if (!robot.wrist_servo.is_busy) {
                // printf("Setting angle %.2f on channel %d\n", angle, channel);

                robot.wrist_servo.step = step;
                robot.wrist_servo.delay = delay;
                robot.wrist_servo.target_angle = angle;
                xTaskCreate(servo_move_task, "wrist servo move task", 4096, &robot.wrist_servo, 1, NULL);
            }
            else {
                robot.wrist_servo.target_angle = angle;
                // printf("Updating target angle for wrist servo to %.2f\n", angle);
            }
        }
        else if (robot.wrist_rot_servo.channel == channel) {
            if (!robot.wrist_rot_servo.is_busy) {
                // printf("Setting angle %.2f on channel %d\n", angle, channel);

                robot.wrist_rot_servo.step = step;
                robot.wrist_rot_servo.delay = delay;
                robot.wrist_rot_servo.target_angle = angle;
                xTaskCreate(servo_move_task, "wrist rotation servo move task", 4096, &robot.wrist_rot_servo, 1, NULL);
            }
            else {
                robot.wrist_rot_servo.target_angle = angle;
                // printf("Updating target angle for wrist rotation servo to %.2f\n", angle);
            }
        }
        else if (robot.gripper_servo.channel == channel) {
            if (!robot.gripper_servo.is_busy) {
                // printf("Setting angle %.2f on channel %d\n", angle, channel);

                robot.gripper_servo.step = step;
                robot.gripper_servo.delay = delay;
                robot.gripper_servo.target_angle = angle;
                xTaskCreate(servo_move_task, "gripper servo move task", 4096, &robot.gripper_servo, 1, NULL);
            }
            else {
                robot.gripper_servo.target_angle = angle;
                // printf("Updating target angle for gripper servo to %.2f\n", angle);
            }
        }
        else {
            printf("None\n");
        }

        // printf("Servo on channel %d set to angle: %.2f degress\n", channel, angle);
    }
}

// TODO: подобрать параметры, чтобы быстрее считывала или не надо так делать?
static void usb_serial_task(void *arg) {

    uint8_t rxbuf[BUF_SIZE];
    char input_buffer[BUF_SIZE];

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

                    // printf("Done\n");

                    input_length = 0;
                }
                else if (input_length < BUF_SIZE - 1) {
                    input_buffer[input_length++] = rxbuf[i];
                }
                else {
                    printf("Input buffer overflow, cleaaring buffer\n");
                    input_length = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// TODO: раскидать по функциям инициализации
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
    ret = arm_robot_home_state(&robot);
    if (ret != ESP_OK) {
        return;
    }
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
    xTaskCreate(usb_serial_task, "USB Serial Task", 4096, NULL, 5, NULL);

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