idf_component_register(SRCS "servo_pca9685.c" "servo_motion_async.c" "servo_motion_async_pid.c" "pid/pid.c"
                    INCLUDE_DIRS "include" "pid"
                    REQUIRES pca9685 freertos esp_timer)