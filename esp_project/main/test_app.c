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

#include <math.h>

esp_err_t acs712_calibrate_voltage_session(acs712_t *acs712, int samples, int session_cnt, int thresh_stab)
{
    esp_err_t ret;
    int raw;
    int session_value_arr[session_cnt];
    int stab_cnt = 0;
    bool is_stable = false;
    int attemp = 0;
    int max_attemp = 3; // количество попыток калибровки

    while (attemp < max_attemp && !is_stable) {
        stab_cnt = 0;
        is_stable = true;

        for (int session = 0; session < session_cnt; session++) {
            int raw_sum = 0;

            for (int i = 0; i < samples; i++) {
                if (acs712_read_raw(acs712, &raw) == ESP_OK) {
                    raw_sum += raw;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                else {
                    return ESP_FAIL;
                }
            }
            session_value_arr[session] = raw_sum / samples;

            if (session == 0) {
                continue;
            }

            // проверка отклонения от предыдущей сессии
            int devition = abs(session_value_arr[session] - session_value_arr[session - 1]);

            if (devition > thresh_stab) {
                is_stable = false;
                break;
            }
            else {
                stab_cnt++;
            }
        }

        // если стабилен всей сессии, то откалиброван
        if (is_stable && stab_cnt == (session_cnt - 1)) {
            is_stable = true;
            break;
        }

        // увеличение попытки
        attemp++;
    }

    if (!is_stable) {
        ESP_LOGE(TAG, "FAiled calibrate");
        return ESP_FAIL;
    }

    ret = adc_cali_raw_to_voltage(acs712->cali_handle, raw, &acs712->calibrate_voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}
// End

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
            // vTaskDelay(pdMS_TO_TICKS(10));
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

// Фильтр с применением медианы
static int median_filter(int *arr, int size)
{
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }

    return arr[size / 2];
}

esp_err_t acs712_calibrate_voltage_median(acs712_t *acs712, int samples)
{
    esp_err_t ret;
    int raw_arr[samples];
    int raw;

    for (int i = 0; i < samples; i++) {
        if (acs712_read_raw(acs712, &raw) == ESP_OK) {
            raw_arr[i] = raw;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else {
            return ESP_FAIL;
        }
    }

    raw = median_filter(raw_arr, samples);

    ret = adc_cali_raw_to_voltage(acs712->cali_handle, raw, &acs712->calibrate_voltage);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to convert ADC raw to voltage: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}
// End

static void send_current_task(void *arg)
{
    char buffer[64];

    while (1) {
        if (acs712_read_current(&acs712, &current) == ESP_OK) {
            // CURRENT <value>
            int len = snprintf(buffer, sizeof(buffer), "%.3f\n", current);

            // portMAX_DELAY
            usb_serial_jtag_write_bytes((uint8_t *)buffer, len, pdMS_TO_TICKS(50));
        }
        else {
            ESP_LOGE(TAG, "Failed to read current from ACS712");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main()
{
    // Set level log
    // esp_log_level_set("servo_pca9685", ESP_LOG_WARN);
    // esp_log_level_set("arm_robot", ESP_LOG_WARN);

    esp_err_t ret;

    // Initialize ACS712 5A
    ret = acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN, ACS712_ADC_CHANNEL, ACS712_SENSITIVITY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ACS712 initialization failed");
        return;
    }
    ESP_LOGI(TAG, "ACS712 5A initialized successfully");

    ESP_LOGI(TAG, "Start calibrate voltage...");
    ret = acs712_calibrate_voltage(&acs712, 100);
    // ret = acs712_calibrate_voltage_median(&acs712, 100);
    // ret = acs712_calibrate_voltage_session(&acs712, 100, 5, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate voltage");
        return;
    }

    ESP_LOGI(TAG, "Calibrate voltage: %d mV", acs712.calibrate_voltage);
    // End

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

    // Initialize robot
    arm_robot_init(&robot, &pca9685);

    // ret = arm_robot_home_state(&robot);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Robot is not set home position");
    //     return;
    // }
    // ESP_LOGI(TAG, "Robot is in home position");
    // End

    // Set on_time and off_time pwm
    for (int channel = 0; channel < 16; channel++) {
        ret = pca9685_set_pwm(&pca9685, channel, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failde set PWM servo channel %d", channel);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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
    xTaskCreate(send_current_task, "Send Current Task", 4096, NULL, 5, NULL);
    // End

    robot.manipulator.base_servo.min_pulse_width = 580;
    robot.manipulator.base_servo.max_pulse_width = 2440;

    ret = servo_pca9685_set_angle(&robot.manipulator.base_servo, 90.0, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failde set PWM servo channel %d", robot.manipulator.base_servo.channel);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    float cur_angle;
    servo_pca9685_get_angle(&robot.manipulator.base_servo, &cur_angle, PWM_FREQUENCY);
    printf("Servo (pwm %d) servo angle: %.2f\n", robot.manipulator.base_servo.channel, cur_angle);

    // ret = servo_pca9685_set_angle(&robot.manipulator.base_servo, 90.0, PWM_FREQUENCY);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failde set PWM servo channel %d", robot.manipulator.base_servo.channel);
    //     return;
    // }
    // vTaskDelay(pdMS_TO_TICKS(20));

    // servo_pca9685_get_angle(&robot.manipulator.base_servo, &cur_angle, PWM_FREQUENCY);
    // printf("Servo (pwm %d) servo angle: %.2f\n", robot.manipulator.base_servo.channel, cur_angle);
}