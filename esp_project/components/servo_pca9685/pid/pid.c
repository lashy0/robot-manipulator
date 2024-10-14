#include "esp_err.h"
#include "esp_log.h"
#include "pid.h"

static const char *TAG = "pid";

void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_err = 0.0f;
    pid->integral = 0.0f;
}

float pid_calculate(pid_controller_t *pid, float target_pos, float current_pos, float delta_time)
{
    float err = target_pos - current_pos;
    pid->integral += err * delta_time;
    float derication = (err - pid->prev_err) / delta_time;
    pid->prev_err = err;

    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * derication;
    return out;
}