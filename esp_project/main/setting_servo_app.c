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

static const char *TAG = "setting_servo";

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

// Фильтр тока
// Параметры для фильтра
#define FILTER_SIZE 10
float current_filter[FILTER_SIZE];
float sum = 0.0f;
int filter_index = 0;
int sample_count = 0;

// Инициафлизация буфера для фильтра
void init_current_filter()
{
    sum = 0.0f;
    sample_count = 0;
    filter_index = 0;

    for (int i = 0; i < FILTER_SIZE; i++) {
        current_filter[i] = 0.0f;
    }
}

// Добавление значения в буфер и обнавление суммы
float apply_current_filter(float value)
{
    sum -= current_filter[filter_index];

    current_filter[filter_index] = value;

    sum += value;

    filter_index = (filter_index + 1) % FILTER_SIZE;

    if (sample_count < FILTER_SIZE) {
        sample_count++;
    }

    return sum / sample_count;
}
// End

// Калибровка для нулевого тока с использованием фильтра среднего
esp_err_t acs712_calibrate_voltage(acs712_t *acs712, int samples)
{
    esp_err_t ret;
    int raw;
    int voltage;
    int sum = 0;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_raw(acs712, &raw) == ESP_OK) {
            sum += raw;
        }
        else {
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    int avg_raw = sum / samples;
    ret = adc_cali_raw_to_voltage(acs712->cali_handle, avg_raw, &voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    acs712->calibrate_voltage = voltage;

    return ESP_OK;
}
// End

// Функция для получения указателя на серво по каналу
servo_t* arm_robot_get_servo_by_channel_switch(arm_robot_t *robot, uint8_t channel)
{
    switch (channel)
    {
    case SERVO_BASE_NUMBER_PWM:
        return &robot->manipulator.base_servo;
    case SERVO_SHOULDER_NUMBER_PWM:
        return &robot->manipulator.shoulder_servo;
    case SERVO_ELBOW_NUMBER_PWM:
        return &robot->manipulator.elbow_servo;
    case SERVO_WRIST_NUMBER_PWM:
        return &robot->manipulator.wrist_servo;
    case SERVO_WRIST_ROT_NUMBER_PWM:
        return &robot->arm.wrist_rot_servo;
    case SERVO_GRIPPER_NUMBER_PWM:
        return &robot->arm.gripper.gripper_servo;
    default:
        ESP_LOGW(TAG, "Unknown servo channel: %d", channel);
        return NULL;
    }
}

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
            esp_err_t ret = arm_robot_move_servo_to_angle(&robot, channel, angle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to move servo to angle %.2f on channel %d", angle, channel);
            } else {
                ESP_LOGI(TAG, "Started moving servo on channel %d to angle %.2f", channel, angle);
            }
        } else {
            ESP_LOGW(TAG, "Invalid SET_ANGLE command format: %s", command);
        }
    }
    // Command format: SET_PULSE_WIDTH <pwm_id> <min_pulse> <max_pulse>
    else if (strncmp(command, "SET_PULSE_WIDTH", 15) == 0) {
        uint8_t channel;
        uint16_t min_pulse, max_pulse;

        if (sscanf(command, "SET_PULSE_WIDTH %hhu %hu %hu", &channel, &min_pulse, &max_pulse) == 3) {
            servo_t *servo = arm_robot_get_servo_by_channel_switch(&robot, channel);
            if (servo) {
                servo->min_pulse_width = min_pulse;
                servo->max_pulse_width = max_pulse;
                ESP_LOGI(TAG, "Set min pulse width for servo %d to %d and max pulse width to %d", channel, min_pulse, max_pulse);
            } else {
                ESP_LOGW(TAG, "Servo with channel %d not found", channel);
            }
        } else {
            ESP_LOGW(TAG, "Invalid SET_PULSE_WIDTH command format: %s", command);
        }
    }
    // Command format: GET_PULSE_WIDTH <pwm_id>
    else if (strncmp(command, "GET_PULSE_WIDTH", 15) == 0) {
        uint8_t channel;

        if (sscanf(command, "GET_PULSE_WIDTH %hhu", &channel) == 1) {
            servo_t *servo = arm_robot_get_servo_by_channel_switch(&robot, channel);
            if (servo) {
                // Формат ответа: GET_PULSE_WIDTH <pwm_id> <min_pulse> <max_pulse>
                char response[128];
                int len = snprintf(response, sizeof(response), "GET_PULSE_WIDTH %d %d %d\n",
                                    channel, servo->min_pulse_width, servo->max_pulse_width);

                // Отправка ответа по USB Serial JTAG
                usb_serial_jtag_write_bytes((const uint8_t *)response, len, portMAX_DELAY);
                ESP_LOGI(TAG, "Sent pulse width for servo %d: min_pulse=%d, max_pulse=%d",
                         channel, servo->min_pulse_width, servo->max_pulse_width);
            } else {
                ESP_LOGW(TAG, "Servo with channel %d not found", channel);
            }
        } else {
            ESP_LOGW(TAG, "Invalid GET_PULSE_WIDTH command format: %s", command);
        }
    }
    else {
        ESP_LOGW(TAG, "Unknown command: %s", command);
    }
}

static void send_current_task(void *arg)
{
    char buffer[64];
    float raw_current;

    while (1) {
        if (acs712_read_current(&acs712, &raw_current) == ESP_OK) {
            // Фильтрация значения тока
            current = apply_current_filter(raw_current);
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

                    // Обработка команд


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

    ret = arm_robot_home_state(&robot);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Robot is not set home position");
        return;
    }
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