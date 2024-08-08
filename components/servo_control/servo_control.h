#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "pca9685.h"

#define SERVO_MIN_PULSE_WIDTH 500
#define SERVO_MAX_PULSE_WIDTH 2500

#define SERVO_MAX_ANGLE 180.0

typedef struct {
    pca9685_handle_t pca9685_handle;
    uint8_t channel;
} servo_t;

esp_err_t servo_init(servo_t *servo, pca9685_handle_t handle, uint8_t channel);
esp_err_t servo_set_angle(servo_t *servo, float angle);

#endif