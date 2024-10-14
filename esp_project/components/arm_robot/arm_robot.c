#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "arm_robot.h"

static const char *TAG = "arm_robot";

#define NUM_SERVO 6

static async_motion_pid_t arm_motion[NUM_SERVO];

void arm_robot_init(arm_robot_t *robot, pca9685_t *pca9685)
{
    // Init base servo
    servo_config_t base_config = {
        .channel = SERVO_BASE_NUMBER_PWM,
        .min_pulse_width = SERVO_BASE_MIN_PULSE_US,
        .max_pulse_width = SERVO_BASE_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_BASE_MAX_ANGLE,
    };

    servo_pca9685_init(&robot->base_servo, pca9685, &base_config);
    robot->base_servo.current_angle = SERVO_BASE_START_ANGLE;

    servo_motion_async_pid_init(&arm_motion[0], &robot->base_servo, 100, 1.5, 0.1, 0.3);
    // End
    
    // Init shoulder servo
    servo_config_t shoulder_config = {
        .channel = SERVO_SHOULDER_NUMBER_PWM,
        .min_pulse_width = SERVO_SHOULDER_MIN_PULSE_US,
        .max_pulse_width = SERVO_SHOULDER_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_SHOULDER_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->shoulder_servo, pca9685, &shoulder_config);
    robot->shoulder_servo.current_angle = SERVO_SHOULDER_START_ANGLE;
    
    servo_motion_async_pid_init(&arm_motion[1], &robot->shoulder_servo, 100, 1.5, 0.1, 0.3);
    // End

    // Init elbow servo
    servo_config_t elbow_config = {
        .channel = SERVO_ELBOW_NUMBER_PWM,
        .min_pulse_width = SERVO_ELBOW_MIN_PULSE_US,
        .max_pulse_width = SERVO_ELBOW_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_ELBOW_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->elbow_servo, pca9685, &elbow_config);
    robot->elbow_servo.current_angle = SERVO_ELBOW_START_ANGLE;

    servo_motion_async_pid_init(&arm_motion[2], &robot->elbow_servo, 50, 0.5, 0.1, 0.2);
    // End

    // Init wrist servo
    servo_config_t wrist_config = {
        .channel = SERVO_WRIST_NUMBER_PWM,
        .min_pulse_width = SERVO_WRIST_MIN_PULSE_US,
        .max_pulse_width = SERVO_WRIST_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_WRIST_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->wrist_servo, pca9685, &wrist_config);
    robot->wrist_servo.current_angle = SERVO_WRIST_START_ANGLE;
    
    servo_motion_async_pid_init(&arm_motion[3], &robot->wrist_servo, 50, 0.5, 0.1, 0.2);
    // End

    // Init wrist rotational servo
    servo_config_t wrist_rot_config = {
        .channel = SERVO_WRIST_ROT_NUMBER_PWM,
        .min_pulse_width = SERVO_WRIST_ROT_MIN_PULSE_US,
        .max_pulse_width = SERVO_WRIST_ROT_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_WRIST_ROT_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->wrist_rot_servo, pca9685, &wrist_rot_config);
    robot->wrist_rot_servo.current_angle = SERVO_WRIST_ROT_START_ANGLE;
    
    servo_motion_async_pid_init(&arm_motion[4], &robot->wrist_rot_servo, 50, 0.5, 0.1, 0.2);
    // End

    // Init gripper servo
    servo_config_t gripper_config = {
        .channel = SERVO_GRIPPER_NUMBER_PWM,
        .min_pulse_width = SERVO_GRIPPER_MIN_PULSE_US,
        .max_pulse_width = SERVO_GRIPPER_MAX_PULSE_US,
        .min_angle = 0.0f,
        .max_angle = SERVO_GRIPPER_MAX_ANGLE,
    };
    servo_pca9685_init(&robot->gripper_servo, pca9685, &gripper_config);
    robot->gripper_servo.current_angle = SERVO_GRIPPER_START_ANGLE;
    robot->gripper_state = GRIPPER_CLOSE;
    
    servo_motion_async_pid_init(&arm_motion[5], &robot->gripper_servo, 50, 0.5, 0.1, 0.2);
    // End
}

// TODO: ммм на один раз грубо говоря
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

    return ESP_OK;
}

esp_err_t arm_robot_move_servo_to_angle(arm_robot_t *robot, uint8_t channel, float angle)
{
    if (channel < NUM_SERVO) {
        servo_smooth_move_async_pid(&arm_motion[channel], angle);
        return ESP_OK;
    }
    else {
        ESP_LOGE(TAG, "Unknown servo channel: %d", channel);
        return ESP_FAIL;
    }
}

bool arm_robot_is_moving(arm_robot_t *robot)
{
    for (int i = 0; i < NUM_SERVO; i++) {
        if (arm_motion[i].is_moving) {
            return true;
        }
    }

    return false;
}

void arm_robot_move_manipulator_to_angles(arm_robot_t *robot, float base_angle, float shoulder_angle, float elbow_angle, float wrist_angle)
{
    float max_delta = base_angle;
    if (shoulder_angle > max_delta) max_delta = shoulder_angle;
    if (elbow_angle > max_delta) max_delta = elbow_angle;
    if (wrist_angle > max_delta) max_delta = wrist_angle;

    float step_delay = max_delta > 0 ? 500 / max_delta : 50;

    for (int i = 0; i < 4; i++) {
        arm_motion[i].step_delay = step_delay;
    }

    servo_smooth_move_async_pid(&arm_motion[0], base_angle);
    servo_smooth_move_async_pid(&arm_motion[1], shoulder_angle);
    servo_smooth_move_async_pid(&arm_motion[2], elbow_angle);
    servo_smooth_move_async_pid(&arm_motion[3], wrist_angle);
}