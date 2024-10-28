#ifndef PID_H
#define PID_H

/**
 * @brief Structure representing a PID controller
 */
typedef struct {
    float kp;       /**< Proportional coefficient */
    float ki;       /**< Integral coefficient */
    float kd;       /**< Derivation coefficient */
    float prev_err; /**< Previous error value for derivative calculation */
    float integral; /**< Accumulated integral of the error */
} pid_controller_t;

/**
 * @brief Initialize the PID controller with specified coefficients
 * 
 * @param[in] pid Pointer to the PID structure
 * @param[in] kp Proportional coefficient
 * @param[in] ki Integral coefficient
 * @param[in] kd Derivation coefficient
 */
void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Calculate the control output using the PID controller
 * 
 * @param[in] pid Pointer to the PID structure
 * @param[in] target_pos The desired target position
 * @param[in] current_pos The current position
 * @param[in] delta_time Time difference between the current and previous calculation in seconds
 * 
 * @return The control output value
 */
float pid_calculate(pid_controller_t *pid, float target_pos, float current_pos, float delta_time);

#endif