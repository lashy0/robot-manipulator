#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"

#include "i2c.h"
#include "acs712.h"
#include "pca9685.h"
#include "arm_robot.h"

static const char *TAG = "app_main";

#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

#define PCA9685_I2C_ADDR           0x40

arm_robot_t robot;
acs712_t acs712;

float current;

// Фильтр среднего
esp_err_t acs712_calibrate_voltage(acs712_t *acs712, int samples)
{
    esp_err_t ret;
    int raw;
    int voltage;
    int sum = 0;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_raw(acs712, &raw) == ESP_OK) {
            sum += raw;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else {
            return ESP_FAIL;
        }
    }

    ret = adc_cali_raw_to_voltage(acs712->cali_handle, sum / samples, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    acs712->calibrate_voltage = voltage;

    return ESP_OK;
}
// End

// Вариант функции движения с использованием ACS712
static void move_manipulator_task(void *arg)
{
    arm_robot_t *robot = (arm_robot_t *)arg;
    float current_angle;
    float step = 2.0f;  // Шаг изменения угла
    float current_task;
    bool all_reached = false;
    esp_err_t ret;
    float current_thresh = current;

    // Целевые углы
    float target_angles[4] = {
        robot->manipulator.base_servo.target_angle,
        robot->manipulator.shoulder_servo.target_angle,
        robot->manipulator.elbow_servo.target_angle,
        robot->manipulator.wrist_servo.target_angle
    };

    // Массив указателей на сервоприводы манипулятора
    servo_t *servos[4] = {
        &robot->manipulator.base_servo,
        &robot->manipulator.shoulder_servo,
        &robot->manipulator.elbow_servo,
        &robot->manipulator.wrist_servo
    };

    while (!all_reached) {
        all_reached = true;

        for (int i = 0; i < 4; i++) {
            ret = servo_pca9685_get_angle(servos[i], &current_angle, PWM_FREQUENCY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get current angle for servo %d", i);
                continue;
            }

            // Вычисляем шаг изменения угла
            float increment = (target_angles[i] > current_angle) ? step : -step;
            current_angle += increment;

            // Проверяем, достигли ли мы целевого угла для текущего серв
            if ((increment > 0 && current_angle >= target_angles[i]) ||
                (increment < 0 && current_angle <= target_angles[i])) {
                current_angle = target_angles[i];
            } else {
                all_reached = false;
            }

            ESP_LOGI(TAG, "Manipulator [pwm%d] to current angles: %.2f; target anglse: %.2f", servos[i]->channel, current_angle, target_angles[i]);
            // Устанавливаем текущий угол для текущего сервопривода
            ret = servo_pca9685_set_angle(servos[i], current_angle, PWM_FREQUENCY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set angle for servo %d", i);
            }
        }

        // Считываем силу тока с ACS712
        if (acs712_read_current(&acs712, &current_task) == ESP_OK) {
            ESP_LOGI(TAG, "Current: %.3f A", current_task);

            // Проверяем, меньше ли сила тока порога
            if (current_task < current_thresh) {
                ESP_LOGI(TAG, "Current threshold reached, stopping movement.");
                all_reached = true;  // движение завершено
            }
        } else {
            ESP_LOGE(TAG, "Failed to read current from ACS712");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Отправляем сообщение о завершении движения
    char done_message[] = "DONE\n";
    usb_serial_jtag_write_bytes((const uint8_t *)done_message, sizeof(done_message) - 1, portMAX_DELAY);

    ESP_LOGI(TAG, "Manipulator movement completed.");
    vTaskDelete(NULL);
}

esp_err_t arm_robot_move_manipulator(arm_robot_t *robot, float base_angle, float shoulder_angle, float elbow_angle, float wrist_angle)
{
    if (!robot) {
        ESP_LOGE(TAG, "Invalid robot pointer");
        return ESP_FAIL;
    }

    // Устанавливаем целевые углы для всех сервоприводов
    robot->manipulator.base_servo.target_angle = base_angle;
    robot->manipulator.shoulder_servo.target_angle = shoulder_angle;
    robot->manipulator.elbow_servo.target_angle = elbow_angle;
    robot->manipulator.wrist_servo.target_angle = wrist_angle;

    xTaskCreate(move_manipulator_task, "Move Manipulator Task", 4096, (void *)robot, 5, NULL);

    ESP_LOGI(TAG, "Started moving manipulator to target angles without timer.");
    return ESP_OK;
}
// End

static void parse_command(const char *input)
{
    char command[128];
    strncpy(command, input, sizeof(command) - 1);
    command[sizeof(command) - 1] = '\0';

    // Command format: SET_ANGLE <pwm_id> <angle>
    if (strncmp(command, "SET_ANGLE", 9) == 0) {
        uint8_t channel;
        float angle;

        if (sscanf(command, "SET_ANGLE %hhu %f", &channel, &angle) == 2) {
            // Вызов функции для выставления угла серва
            esp_err_t ret = arm_robot_move_servo_to_angle(&robot, channel, angle);
            // if (ret != ESP_OK) {
            //     ESP_LOGE(TAG, "Failed to move servo to angle %.2f on channel %d", angle, channel);
            // } else {
            //     ESP_LOGI(TAG, "Started moving servo on channel %d to angle %.2f", channel, angle);
            // }
        }
        else {
            ESP_LOGW(TAG, "Invalid SET_ANGLE command format: %s", command);
        }
    }
    // Command format: SET_MANIPULATOR <angle0> <angle1> <angle2> <angle3>
    else if (strncmp(command, "SET_MANIPULATOR", 15) == 0) {
        float angles[4];

        if (sscanf(command, "SET_MANIPULATOR %f %f %f %f", &angles[0], &angles[1], &angles[2], &angles[3]) == 4) {
            // По таймеру
            esp_err_t ret = arm_robot_move_manipulator_to_angles(&robot, angles[0], angles[1], angles[2], angles[3]);
            // Вариант по току
            // esp_err_t ret = arm_robot_move_manipulator(&robot, angles[0], angles[1], angles[2], angles[3]);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to move manipulator to angles");
            } else {
                ESP_LOGI(TAG, "Moving manipulator to angles: base=%.2f, shoulder=%.2f, elbow=%.2f, wrist=%.2f",
                        angles[0], angles[1], angles[2], angles[3]);
            }
        }
        else {
            ESP_LOGW(TAG, "Invalid SET_MANIPULATOR command format: %s", command);
        }
    }
    // Command format: SET_GRIP 0/1
    else if (strncmp(command, "SET_GRIP", 8) == 0) {
        int number;

        if (sscanf(command, "SET_GRIP %d", &number) == 1) {
            // Вызов функции робота для захвата
        }
        else {
            ESP_LOGW(TAG, "Invalid SET_GRIP command format: %s", command);
        }
    }
    else {
        ESP_LOGW(TAG, "Unknow command: %s", command);
    }

}

static void send_current_task(void *arg)
{
    char buffer[64];

    while (1) {
        if (acs712_read_current(&acs712, &current) == ESP_OK) {
            // CURRENT <value>
            int len = snprintf(buffer, sizeof(buffer), "CURRENT %.3f\n", current);

            // portMAX_DELAY
            usb_serial_jtag_write_bytes((uint8_t *)buffer, len, pdMS_TO_TICKS(50));
        }
        else {
            ESP_LOGE(TAG, "Failed to read current from ACS712");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void usb_serial_task(void *arg)
{
    uint8_t rxbuf[128];
    char input_buffer[128];

    size_t bytes_read = 0;
    size_t input_length = 0;

    while(1) {
        bytes_read = usb_serial_jtag_read_bytes(rxbuf, 128, pdMS_TO_TICKS(10));

        if (bytes_read > 0) {
            for (size_t i = 0; i < bytes_read; i++) {
                if (rxbuf[i] == '\n') {
                    input_buffer[input_length] = '\0';

                    parse_command(input_buffer);

                    input_length = 0;
                }
                else if (input_length < 128 - 1) {
                    input_buffer[input_length++] = rxbuf[i];
                }
                else {
                    ESP_LOGW(TAG, "Input buffer overflow, cleaaring buffer");
                    input_length = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main()
{
    // Set level log
    esp_log_level_set("servo_pca9685", ESP_LOG_WARN);
    esp_log_level_set("arm_robot", ESP_LOG_WARN);
    esp_log_level_set("acs712", ESP_LOG_WARN);

    esp_err_t ret;

    // Initialize I2C
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO
    };

    i2c_bus_t i2c_bus = {0};

    ret = i2c_master_init(&i2c_bus, &i2c_master_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialized failed");
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    // End

    // Initialize PCA9685
    pca9685_config_t pca9685_config = {
        .i2c_address = PCA9685_I2C_ADDR,
        .bus_handle = i2c_bus.handle,
        .scl_speed = I2C_MASTER_FREQ_HZ
    };

    pca9685_t pca9685 = {0};

    ret = pca9685_init(&pca9685_config, &pca9685);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 initialized failed");
        return;
    }
    ESP_LOGI(TAG, "PCA9685 initialized successfully");

    ret = pca9685_set_pwm_freq(&pca9685, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    ESP_LOGI(TAG, "PCA9685 set pwm frequency 50 Hz");
    // End

    // Set on_time and off_time pwm
    for (int channel = 0; channel < 16; channel++) {
        ret = pca9685_set_pwm(&pca9685, channel, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed set PWM servo channel %d", channel);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    // End

    // Initialize ACS712 5A
    ret = acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN, ACS712_ADC_CHANNEL, ACS712_SENSITIVITY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ACS712 initialization failed");
        return;
    }
    ESP_LOGI(TAG, "ACS712 5A initialized successfully");

    ESP_LOGI(TAG, "Start calibrate voltage...");
    ret = acs712_calibrate_voltage(&acs712, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate voltage");
        return;
    }

    ESP_LOGI(TAG, "Calibrate voltage: %d mV", acs712.calibrate_voltage);
    // End

    // Initialize robot
    arm_robot_init(&robot, &pca9685);

    // TODO: почему то при не инициализации сервов, нулевой вольтаж получается нормальным с погрешностью в 5-10 mV
    ret = servo_pca9685_set_angle(&robot.manipulator.base_servo, SERVO_BASE_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = servo_pca9685_set_angle(&robot.manipulator.shoulder_servo, SERVO_SHOULDER_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = servo_pca9685_set_angle(&robot.manipulator.elbow_servo, SERVO_ELBOW_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = servo_pca9685_set_angle(&robot.manipulator.wrist_servo, SERVO_WRIST_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = servo_pca9685_set_angle(&robot.arm.wrist_rot_servo, SERVO_WRIST_ROT_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = servo_pca9685_set_angle(&robot.arm.gripper.gripper_servo, SERVO_GRIPPER_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // TODO: доработать данную функцию с учетом загрузки с памяти углы
    // ret = arm_robot_home_state(&robot);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Robot is not set home position");
    //     return;
    // }
    ESP_LOGI(TAG, "Robot is in home position");
    // End

    // Initialize USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = 128,
        .tx_buffer_size = 128
    };
    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial JTAG initialized failed");
        return;
    }

    esp_vfs_usb_serial_jtag_use_driver();
    ESP_LOGI(TAG, "USB Serial JTAG initialized successfully");
    // End

    // Main
    // Задача для отправки данных о силе тока
    // xTaskCreate(send_current_task, "Send Current Task", 4096, NULL, 5, NULL);

    // Задача для приема команд по USB Serial JTAG
    xTaskCreate(usb_serial_task, "USB Serial Task", 4096, NULL, 5, NULL);
    // End
}