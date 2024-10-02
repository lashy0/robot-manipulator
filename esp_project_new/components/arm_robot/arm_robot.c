#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "arm_robot.h"

static const char *TAG = "arm_robot";

typedef struct {
    manipulator_t *manipulator;  // Указатель на манипулятор робота
    float target_angles[4];      // Целевые углы для всех четырех сервоприводов
    esp_timer_handle_t move_timer;  // Таймер для управления движением
    bool is_busy;                // Флаг, показывающий, что движение в процессе
} manipulator_movement_t;

static manipulator_movement_t manipulator_movement = {0};


void arm_robot_init(arm_robot_t *robot, pca9685_t *pca9685)
{
    // Init base servo
    servo_config_t base_config = {
        .channel = SERVO_BASE_NUMBER_PWM,
        .min_pulse_width = SERVO_BASE_MIN_PULSE_US,
        .max_pulse_width = SERVO_BASE_MAX_PULSE_US,
        .max_angle = SERVO_BASE_MAX_ANGLE,
        .step = 2.0f,
        .delay = 20
    };

    servo_pca9685_init(&robot->manipulator.base_servo, pca9685, &base_config);
    robot->manipulator.base_servo.current_angle = SERVO_BASE_START_ANGLE;
    robot->manipulator.base_servo.move_timer = NULL;
    // End
    
    // Init shoulder servo
    servo_config_t shoulder_config = {
        .channel = SERVO_SHOULDER_NUMBER_PWM,
        .min_pulse_width = SERVO_SHOULDER_MIN_PULSE_US,
        .max_pulse_width = SERVO_SHOULDER_MAX_PULSE_US,
        .max_angle = SERVO_SHOULDER_MAX_ANGLE,
        .step = 2.0f,
        .delay = 20
    };
    servo_pca9685_init(&robot->manipulator.shoulder_servo, pca9685, &shoulder_config);
    robot->manipulator.shoulder_servo.current_angle = SERVO_SHOULDER_START_ANGLE;
    robot->manipulator.shoulder_servo.move_timer = NULL;
    // End

    // Init elbow servo
    servo_config_t elbow_config = {
        .channel = SERVO_ELBOW_NUMBER_PWM,
        .min_pulse_width = SERVO_ELBOW_MIN_PULSE_US,
        .max_pulse_width = SERVO_ELBOW_MAX_PULSE_US,
        .max_angle = SERVO_ELBOW_MAX_ANGLE,
        .step = 2.0f,
        .delay = 20
    };
    servo_pca9685_init(&robot->manipulator.elbow_servo, pca9685, &elbow_config);
    robot->manipulator.elbow_servo.current_angle = SERVO_ELBOW_START_ANGLE;
    robot->manipulator.elbow_servo.move_timer = NULL;
    // End

    // Init wrist servo
    servo_config_t wrist_config = {
        .channel = SERVO_WRIST_NUMBER_PWM,
        .min_pulse_width = SERVO_WRIST_MIN_PULSE_US,
        .max_pulse_width = SERVO_WRIST_MAX_PULSE_US,
        .max_angle = SERVO_WRIST_MAX_ANGLE,
        .step = 2.0f,
        .delay = 20
    };
    servo_pca9685_init(&robot->manipulator.wrist_servo, pca9685, &wrist_config);
    robot->manipulator.wrist_servo.current_angle = SERVO_WRIST_START_ANGLE;
    robot->manipulator.wrist_servo.move_timer = NULL;
    // End

    // Init wrist rotational servo
    servo_config_t wrist_rot_config = {
        .channel = SERVO_WRIST_ROT_NUMBER_PWM,
        .min_pulse_width = SERVO_WRIST_ROT_MIN_PULSE_US,
        .max_pulse_width = SERVO_WRIST_ROT_MAX_PULSE_US,
        .max_angle = SERVO_WRIST_ROT_MAX_ANGLE,
        .step = 2.0f,
        .delay = 20
    };
    servo_pca9685_init(&robot->arm.wrist_rot_servo, pca9685, &wrist_rot_config);
    robot->arm.wrist_rot_servo.current_angle = SERVO_WRIST_ROT_START_ANGLE;
    robot->arm.wrist_rot_servo.move_timer = NULL;
    // End

    // Init gripper servo
    servo_config_t gripper_config = {
        .channel = SERVO_GRIPPER_NUMBER_PWM,
        .min_pulse_width = SERVO_GRIPPER_MIN_PULSE_US,
        .max_pulse_width = SERVO_GRIPPER_MAX_PULSE_US,
        .max_angle = SERVO_GRIPPER_MAX_ANGLE,
        .step = 2.0f,
        .delay = 20
    };
    servo_pca9685_init(&robot->arm.gripper.gripper_servo, pca9685, &gripper_config);
    robot->arm.gripper.gripper_servo.current_angle = SERVO_GRIPPER_START_ANGLE;
    robot->arm.gripper.state = GRIPPER_CLOSE;
    robot->arm.gripper.gripper_servo.move_timer = NULL;
    // End
}

esp_err_t arm_robot_home_state(arm_robot_t *robot)
{
    esp_err_t ret;
    
    ret = servo_pca9685_set_angle(&robot->manipulator.base_servo, SERVO_BASE_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->manipulator.shoulder_servo, SERVO_SHOULDER_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->manipulator.elbow_servo, SERVO_ELBOW_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->manipulator.wrist_servo, SERVO_WRIST_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->arm.wrist_rot_servo, SERVO_WRIST_ROT_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = servo_pca9685_set_angle(&robot->arm.gripper.gripper_servo, SERVO_GRIPPER_START_ANGLE, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void move_servo_timer_callback(void *arg)
{
    servo_t *servo = (servo_t *)arg;
    float current_angle;
    esp_err_t ret;

    // Получение текущего угла сервопривода
    ret = servo_pca9685_get_angle(servo, &current_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        esp_timer_stop(servo->move_timer);
        servo->is_busy = false;
        ESP_LOGE(TAG, "Failed to get current angle for servo on channel %d", servo->channel);
        return;
    }

    float increment = (servo->target_angle > current_angle) ? servo->step : -servo->step;

    current_angle += increment;

    // Проверка достигли ли целевого угла
    if ((increment > 0 && current_angle >= servo->target_angle) ||
        (increment < 0 && current_angle <= servo->target_angle)) {
        current_angle = servo->target_angle;
        servo_pca9685_set_angle(servo, current_angle, PWM_FREQUENCY);
        ESP_LOGI(TAG, "Moving to angle: %.2f; Target angle: %.2f", current_angle, servo->target_angle);
        esp_timer_stop(servo->move_timer);
        servo->is_busy = false;
        return;
    }

    // Изменение угла
    ret = servo_pca9685_set_angle(servo, current_angle, PWM_FREQUENCY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set angle for servo on channel %d", servo->channel);
        esp_timer_stop(servo->move_timer);
        servo->is_busy = false;
        return;
    }

    ESP_LOGI(TAG, "Moving to angle: %.2f; Target_angle: %.2f; Channel: %d", current_angle, servo->target_angle, servo->channel);
}

static void start_servo_move_smoth_timer(servo_t *servo)
{
    // Прверка занят ли серва
    if (servo->is_busy) {
        ESP_LOGI(TAG, "Servo [pwm %d] is busy. Target angle update.", servo->channel);
        return;
    }

    // Останавливаем и удаляем текущий таймер, если он существует
    if (servo->move_timer != NULL) {
        esp_timer_stop(servo->move_timer);
        esp_timer_delete(servo->move_timer);
        servo->move_timer = NULL;
    }

    // Устанавливаем флаг занятости серво
    servo->is_busy = true;

    // float current_angle;

    // esp_err_t ret = servo_pca9685_get_angle(servo, &current_angle, PWM_FREQUENCY);
    // if (ret != ESP_OK) {
    //     servo->is_busy = false;
    //     return;
    // }

    // servo->current_angle = current_angle;

    // Настройка таймера для плавного движения
    const esp_timer_create_args_t timer_args = {
        .callback = &move_servo_timer_callback,
        .arg = (void *)servo,
        .name = "move_servo_timer"
    };

    if (esp_timer_create(&timer_args, &servo->move_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create move timer for servo on channel %d", servo->channel);
        servo->is_busy = false;
        return;
    }

    if (esp_timer_start_periodic(servo->move_timer, servo->delay * 1000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start move timer for servo on channel %d", servo->channel);
        esp_timer_delete(servo->move_timer);
        servo->move_timer = NULL;
        servo->is_busy = false;
        return;
    }
}

esp_err_t arm_robot_move_servo_to_angle(arm_robot_t *robot, uint8_t channel, float angle)
{
    servo_t *servo = NULL;

    // Выбираем соответствующий сервопривод по каналу
    switch (channel)
    {
    case SERVO_BASE_NUMBER_PWM:
        servo = &robot->manipulator.base_servo;
        break;
    case SERVO_SHOULDER_NUMBER_PWM:
        servo = &robot->manipulator.shoulder_servo;
        break;
    case SERVO_ELBOW_NUMBER_PWM:
        servo = &robot->manipulator.elbow_servo;
        break;
    case SERVO_WRIST_NUMBER_PWM:
        servo = &robot->manipulator.wrist_servo;
        break;
    case SERVO_WRIST_ROT_NUMBER_PWM:
        servo = &robot->arm.wrist_rot_servo;
        break;
    case SERVO_GRIPPER_NUMBER_PWM:
        servo = &robot->arm.gripper.gripper_servo;
        break;
    default:
        ESP_LOGE(TAG, "Unknown servo channel: %d", channel);
        return ESP_FAIL;
    }

    if (servo == NULL) {
        ESP_LOGE(TAG, "Failed to find servo for channel %d", channel);
        return ESP_FAIL;
    }

    servo->target_angle = angle;
    start_servo_move_smoth_timer(servo);

    return ESP_OK;
}

static void move_manipulator_timer_callback(void *arg)
{
    manipulator_t *manipulator = manipulator_movement.manipulator;
    bool all_reached = true;  // Флаг, показывающий, что все сервоприводы достигли целевых углов

    // Массив указателей на сервоприводы манипулятора для упрощения работы
    servo_t *servos[4] = {
        &manipulator->base_servo,
        &manipulator->shoulder_servo,
        &manipulator->elbow_servo,
        &manipulator->wrist_servo
    };

    // Проход по каждому сервоприводу
    for (int i = 0; i < 4; i++) {
        float current_angle;
        esp_err_t ret = servo_pca9685_get_angle(servos[i], &current_angle, PWM_FREQUENCY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get current angle for servo %d", i);
            continue;
        }

        // Вычисляем шаг изменения угла (положительный или отрицательный в зависимости от целевого угла)
        float increment = (manipulator_movement.target_angles[i] > current_angle) ? servos[i]->step : -servos[i]->step;
        current_angle += increment;

        // Проверяем, достигли ли мы целевого угла для текущего сервопривода
        if ((increment > 0 && current_angle >= manipulator_movement.target_angles[i]) ||
            (increment < 0 && current_angle <= manipulator_movement.target_angles[i])) {
            current_angle = manipulator_movement.target_angles[i];  // Устанавливаем угол на целевой
        } else {
            all_reached = false;  // Не все сервоприводы достигли целевых углов
        }

        // Устанавливаем текущий угол для текущего сервопривода
        ret = servo_pca9685_set_angle(servos[i], current_angle, PWM_FREQUENCY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set angle for servo %d", i);
        }
    }

    // Если все сервоприводы достигли целевых углов, останавливаем таймер и завершаем движение
    if (all_reached) {
        esp_timer_stop(manipulator_movement.move_timer);
        manipulator_movement.move_timer = NULL;
        manipulator_movement.is_busy = false;
        ESP_LOGI(TAG, "Manipulator reached all target angles");
    }
}

esp_err_t arm_robot_move_manipulator_to_angles(arm_robot_t *robot, float base_angle, float shoulder_angle, float elbow_angle, float wrist_angle)
{
    if (manipulator_movement.is_busy) {
        ESP_LOGW(TAG, "Manipulator is already moving to target angles.");
        return ESP_FAIL;
    }

    manipulator_movement.manipulator = &robot->manipulator;
    manipulator_movement.target_angles[0] = base_angle;
    manipulator_movement.target_angles[1] = shoulder_angle;
    manipulator_movement.target_angles[2] = elbow_angle;
    manipulator_movement.target_angles[3] = wrist_angle;
    manipulator_movement.is_busy = true;

    const esp_timer_create_args_t timer_args = {
        .callback = &move_manipulator_timer_callback,
        .arg = NULL,
        .name = "move_manipulator_timer"
    };

    if (esp_timer_create(&timer_args, &manipulator_movement.move_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create manipulator move timer");
        manipulator_movement.is_busy = false;
        return ESP_FAIL;
    }

    if (esp_timer_start_periodic(manipulator_movement.move_timer, 20 * 1000) != ESP_OK) {  // 20 мс
        ESP_LOGE(TAG, "Failed to start manipulator move timer");
        esp_timer_delete(manipulator_movement.move_timer);
        manipulator_movement.move_timer = NULL;
        manipulator_movement.is_busy = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Started moving manipulator to target angles: base=%.2f, shoulder=%.2f, elbow=%.2f, wrist=%.2f",
             base_angle, shoulder_angle, elbow_angle, wrist_angle);

    return ESP_OK;
}