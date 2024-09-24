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

    float increment = (target_angle > *cur_angle) ? 1 : -1;

    if (*cur_angle != target_angle) {
        *cur_angle += increment;
        if ((increment > 0 && *cur_angle > target_angle) || (increment < 0 && *cur_angle < target_angle)) {
            *cur_angle = target_angle;
        }

        ret = servo_pca9685_set_angle(servo, *cur_angle, PWM_FREQUENCY);
        if (ret != ESP_OK) {
            return ret;
        }

        // проверка на ток с откатом на прошлое значение
    }

    return ESP_OK;
}

// TODO: добавить init для реализации серва
void arm_robot_init(arm_robot_t *robot, pca9685_t pca9685)
{
    robot->is_moving = false;

    // Init base
    robot->base_servo.is_busy = false;
    robot->base_servo.pca9685 = pca9685;
    robot->base_servo.channel = SERVO_BASE_NUMBER_PWM;
    robot->base_servo.min_pulse_width = SERVO_BASE_MIN_PULSE_US;
    robot->base_servo.max_pulse_width = SERVO_BASE_MAX_PULSE_US;
    robot->base_servo.max_angle = SERVO_BASE_MAX_ANGLE;
    robot->base_servo.delay = 40;
    robot->base_servo.step = 2;

    robot->base_target_angle = SERVO_BASE_START_ANGLE;

    // Init shoulder
    robot->shoulder_servo.is_busy = false;
    robot->shoulder_servo.pca9685 = pca9685;
    robot->shoulder_servo.channel = SERVO_SHOULDER_NUMBER_PWM;
    robot->shoulder_servo.min_pulse_width = SERVO_SHOULDER_MIN_PULSE_US;
    robot->shoulder_servo.max_pulse_width = SERVO_SHOULDER_MAX_PULSE_US;
    robot->shoulder_servo.max_angle = SERVO_SHOULDER_MAX_ANGLE;
    robot->shoulder_servo.delay = 40;
    robot->shoulder_servo.step = 2;

    robot->shoulder_target_angle = SERVO_SHOULDER_START_ANGLE;

    // Init elbow
    robot->elbow_servo.is_busy = false;
    robot->elbow_servo.pca9685 = pca9685;
    robot->elbow_servo.channel = SERVO_ELBOW_NUMBER_PWM;
    robot->elbow_servo.min_pulse_width = SERVO_ELBOW_MIN_PULSE_US;
    robot->elbow_servo.max_pulse_width = SERVO_ELBOW_MAX_PULSE_US;
    robot->elbow_servo.max_angle = SERVO_ELBOW_MAX_ANGLE;
    robot->elbow_servo.delay = 40;
    robot->elbow_servo.step = 2;

    robot->elbow_target_angle = SERVO_ELBOW_START_ANGLE;

    // Init wrist
    robot->wrist_servo.is_busy = false;
    robot->wrist_servo.pca9685 = pca9685;
    robot->wrist_servo.channel = SERVO_WRIST_NUMBER_PWM;
    robot->wrist_servo.min_pulse_width = SERVO_WRIST_MIN_PULSE_US;
    robot->wrist_servo.max_pulse_width = SERVO_WRIST_MAX_PULSE_US;
    robot->wrist_servo.max_angle = SERVO_WRIST_MAX_ANGLE;
    robot->wrist_servo.delay = 40;
    robot->wrist_servo.step = 2;

    robot->wrist_target_angle = SERVO_WRIST_START_ANGLE;

    // Init wrist rotational
    robot->wrist_rot_servo.is_busy = false;
    robot->wrist_rot_servo.pca9685 = pca9685;
    robot->wrist_rot_servo.channel = SERVO_WRIST_ROT_NUMBER_PWM;
    robot->wrist_rot_servo.min_pulse_width = SERVO_WRIST_ROT_MIN_PULSE_US;
    robot->wrist_rot_servo.max_pulse_width = SERVO_WRIST_ROT_MAX_PULSE_US;
    robot->wrist_rot_servo.max_angle = SERVO_WRIST_ROT_MAX_ANGLE;
    robot->wrist_rot_servo.delay = 40;
    robot->wrist_rot_servo.step = 2;

    robot->wrist_rot_target_angle = SERVO_WRIST_ROT_START_ANGLE;

    // Init gripper
    robot->gripper_servo.is_busy = false;
    robot->gripper_servo.pca9685 = pca9685;
    robot->gripper_servo.channel = SERVO_GRIPPER_NUMBER_PWM;
    robot->gripper_servo.min_pulse_width = SERVO_GRIPPER_MIN_PULSE_US;
    robot->gripper_servo.max_pulse_width = SERVO_GRIPPER_MAX_PULSE_US;
    robot->gripper_servo.max_angle = SERVO_GRIPPER_MAX_ANGLE;
    robot->gripper_servo.delay = 40;
    robot->gripper_servo.step = 2;

    robot->gripper_target_angle = SERVO_GRIPPER_START_ANGLE;

    ESP_LOGI(TAG, "Arm robot initialized");
}

esp_err_t arm_robot_home_state(arm_robot_t *robot)
{
    esp_err_t ret;
    
    ret = servo_pca9685_set_angle(&robot->base_servo, SERVO_BASE_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->shoulder_servo, SERVO_SHOULDER_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->elbow_servo, SERVO_ELBOW_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->wrist_servo, SERVO_WRIST_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->wrist_rot_servo, SERVO_WRIST_ROT_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->gripper_servo, SERVO_GRIPPER_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "End set angle home state");

    return ESP_OK;
}

// TODO: добавить сюда проверки по току или когда выставляется значение угла
void arm_robot_movement(arm_robot_t *robot)
{
    // TODO: изменить углы в проверке
    // robot->base_target_angle = clamp(base_angle, 0, 180);
    // robot->shoulder_target_angle = clamp(shoulder_angle, 15, 165);
    // robot->elbow_target_angle = clamp(elbow_angle, 15, 165);
    // robot->wrist_target_angle = clamp(wrist_angle, 15, 165);
    // robot->wrist_rot_target_angle = clamp(wrist_rot_angle, 0, 180);
    // robot->gripper_target_angle = clamp(gripper_angle, 74, 148);

    if (robot->is_moving) {
        ESP_LOGI(TAG, "Movement in progress, updating target angles");
        return;
        // return ESP_OK;
    }

    robot->is_moving = true; // установка флаг выполнения движения

    esp_err_t ret;
    float cur_base_angle;
    float cur_shoulder_angle;
    float cur_elbow_angle;
    float cur_wrist_angle;
    float cur_wrist_rot_angle;
    float cur_gripper_angle;

    ret = servo_pca9685_get_angle(&robot->base_servo, &cur_base_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
        // return ret;
    }
    ret = servo_pca9685_get_angle(&robot->shoulder_servo, &cur_shoulder_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
        // return ret;
    }
    ret = servo_pca9685_get_angle(&robot->elbow_servo, &cur_elbow_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
        // return ret;
    }
    ret = servo_pca9685_get_angle(&robot->wrist_servo, &cur_wrist_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
        // return ret;
    }
    ret = servo_pca9685_get_angle(&robot->wrist_rot_servo, &cur_wrist_rot_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
        // return ret;
    }
    ret = servo_pca9685_get_angle(&robot->gripper_servo, &cur_gripper_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return;
        // return ret;
    }

    while (robot->is_moving) {
        // printf("Angle base: %.2f; Angle shoulder: %.2f; Angle elbow: %.2f; Angle wrist: %.2f; Angle wrist rotation: %.2f; Angle gripper: %.2f\n", cur_base_angle, cur_shoulder_angle, cur_elbow_angle, cur_wrist_angle, cur_wrist_rot_angle, cur_gripper_angle);

        ret = move_servo_to_angle(&robot->base_servo, &cur_base_angle, robot->base_target_angle);
        if (ret != ESP_OK) {
            return;
            // return ret;
        }

        ret = move_servo_to_angle(&robot->shoulder_servo, &cur_shoulder_angle, robot->shoulder_target_angle);
        if (ret != ESP_OK) {
            return;
            // return ret;
        }

        ret = move_servo_to_angle(&robot->elbow_servo, &cur_elbow_angle, robot->elbow_target_angle);
        if (ret != ESP_OK) {
            return;
            // return ret;
        }

        ret = move_servo_to_angle(&robot->wrist_servo, &cur_wrist_angle, robot->wrist_target_angle);
        if (ret != ESP_OK) {
            return;
            // return ret;
        }

        ret = move_servo_to_angle(&robot->wrist_rot_servo, &cur_wrist_rot_angle, robot->wrist_rot_target_angle);
        if (ret != ESP_OK) {
            return;
            // return ret;
        }

        ret = move_servo_to_angle(&robot->gripper_servo, &cur_gripper_angle, robot->gripper_target_angle);
        if (ret != ESP_OK) {
            return;
            // return ret;
        }

        if ((robot->base_target_angle == cur_base_angle) && (robot->shoulder_target_angle == cur_shoulder_angle)
        && (robot->elbow_target_angle == cur_elbow_angle) && (robot->wrist_target_angle == cur_wrist_angle)
        && (robot->wrist_rot_target_angle == cur_wrist_rot_angle) && (robot->gripper_target_angle == cur_gripper_angle)) {
            robot->is_moving = false;
        }

        vTaskDelay(pdMS_TO_TICKS(robot->delay));
    }

    // return ESP_OK;
}