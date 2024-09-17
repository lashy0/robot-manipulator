#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "arm_robot.h"

static const char *TAG = "arm_robot";

float clamp(float value, float min, float max)
{
    return value < min ? min : (value > max ? max : value);
}

esp_err_t move_servo_to_angle(servo_t *servo, float *cur_angle, float target_angle)
{
    esp_err_t ret;

    if (*cur_angle != target_angle) {
        if (*cur_angle > target_angle) (*cur_angle)--;
        if (*cur_angle < target_angle) (*cur_angle)++;

        ret = servo_pca9685_set_angle(servo, *cur_angle, PWM_FREQUENCY);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

void arm_robot_init(arm_robot_t *robot, pca9685_t pca9685)
{
    robot->base_servo.channel = 10;
    robot->shoulder_servo.channel = 11;
    robot->elbow_servo.channel = 12;
    robot->wrist_rot_servo.channel = 13;
    robot->wrist_ver_servo.channel = 14;
    robot->gripper_servo.channel = 15;

    robot->base_servo.pca9685 = pca9685;
    robot->base_servo.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    robot->base_servo.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    robot->base_servo.max_angle = SERVO_MAX_ANGLE;

    robot->shoulder_servo.pca9685 = pca9685;
    robot->shoulder_servo.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    robot->shoulder_servo.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    robot->shoulder_servo.max_angle = SERVO_MAX_ANGLE;

    robot->elbow_servo.pca9685 = pca9685;
    robot->elbow_servo.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    robot->elbow_servo.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    robot->elbow_servo.max_angle = SERVO_MAX_ANGLE;

    robot->wrist_rot_servo.pca9685 = pca9685;
    robot->wrist_rot_servo.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    robot->wrist_rot_servo.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    robot->wrist_rot_servo.max_angle = SERVO_MAX_ANGLE;

    robot->wrist_ver_servo.pca9685 = pca9685;
    robot->wrist_ver_servo.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    robot->wrist_ver_servo.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    robot->wrist_ver_servo.max_angle = SERVO_MAX_ANGLE;

    robot->gripper_servo.pca9685 = pca9685;
    robot->gripper_servo.min_pulse_width = SERVO_MIN_PULSE_WIDTH_US;
    robot->gripper_servo.max_pulse_width = SERVO_MAX_PULSE_WIDTH_US;
    robot->gripper_servo.max_angle = SERVO_MAX_ANGLE;

    ESP_LOGI(TAG, "Arm robot initialized");
}

esp_err_t arm_robot_home_state(arm_robot_t *robot)
{
    esp_err_t ret;
    float cur_base_angle;
    float cur_shoulder_angle;
    float cur_elbow_angle;
    float cur_wrist_rot_angle;
    float cur_wrist_ver_angle;
    float cur_gripper_angle;
    float base_angle = 90;
    float shoulder_angle = 115;
    float elbow_angle = 95;
    float wrist_rot_angle = 75;
    float wrist_ver_angle = 90;
    float gripper_angle = 140;

    ret = servo_pca9685_get_angle(&robot->base_servo, &cur_base_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->shoulder_servo, &cur_shoulder_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->elbow_servo, &cur_elbow_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->wrist_rot_servo, &cur_wrist_rot_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->wrist_ver_servo, &cur_wrist_ver_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->gripper_servo, &cur_gripper_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }

    int exit = 1;

    while (exit) {
        ret = move_servo_to_angle(&robot->base_servo, &cur_base_angle, base_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->shoulder_servo, &cur_shoulder_angle, shoulder_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->elbow_servo, &cur_elbow_angle, elbow_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->wrist_rot_servo, &cur_wrist_rot_angle, wrist_rot_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->wrist_ver_servo, &cur_wrist_ver_angle, wrist_ver_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->gripper_servo, &cur_gripper_angle, gripper_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(40));

        if ((base_angle == cur_base_angle) && (shoulder_angle == cur_shoulder_angle)
        && (elbow_angle == cur_elbow_angle) && (wrist_rot_angle = cur_wrist_rot_angle)
        && (wrist_ver_angle == cur_wrist_ver_angle) && (gripper_angle == cur_gripper_angle)) {
            exit = 0;
        }
    }

    ESP_LOGI(TAG, "End set angle home state");

    return ESP_OK;
}

esp_err_t arm_robot_movement(arm_robot_t *robot, float base_angle, float shoulder_angle, float elbow_angle,
                            float wrist_rot_angle, float wrist_ver_angle, float gripper_angle, int delay)
{
    base_angle = clamp(base_angle, 0, 180);
    shoulder_angle = clamp(shoulder_angle, 15, 165);
    elbow_angle = clamp(elbow_angle, 15, 165);
    wrist_rot_angle = clamp(wrist_rot_angle, 15, 165);
    wrist_ver_angle = clamp(wrist_ver_angle, 0, 180);
    gripper_angle = clamp(gripper_angle, 74, 148);

    esp_err_t ret;
    float cur_base_angle;
    float cur_shoulder_angle;
    float cur_elbow_angle;
    float cur_wrist_rot_angle;
    float cur_wrist_ver_angle;
    float cur_gripper_angle;

    ret = servo_pca9685_get_angle(&robot->base_servo, &cur_base_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->shoulder_servo, &cur_shoulder_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->elbow_servo, &cur_elbow_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->wrist_rot_servo, &cur_wrist_rot_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->wrist_ver_servo, &cur_wrist_ver_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = servo_pca9685_get_angle(&robot->gripper_servo, &cur_gripper_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ret;
    }

    int exit = 1;

    while (exit) {
        ret = move_servo_to_angle(&robot->base_servo, &cur_base_angle, base_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->shoulder_servo, &cur_shoulder_angle, shoulder_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->elbow_servo, &cur_elbow_angle, elbow_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->wrist_rot_servo, &cur_wrist_rot_angle, wrist_rot_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->wrist_ver_servo, &cur_wrist_ver_angle, wrist_ver_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = move_servo_to_angle(&robot->gripper_servo, &cur_gripper_angle, gripper_angle);
        if (ret != ESP_OK) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(delay));

        if ((base_angle == cur_base_angle) && (shoulder_angle == cur_shoulder_angle)
        && (elbow_angle == cur_elbow_angle) && (wrist_rot_angle = cur_wrist_rot_angle)
        && (wrist_ver_angle == cur_wrist_ver_angle) && (gripper_angle == cur_gripper_angle)) {
            exit = 0;
        }
    }

    return ESP_OK;
}