idf_component_register(SRCS "servo_pca9685.c" "servo_motion_async.c"
                    INCLUDE_DIRS "include"
                    REQUIRES pca9685 freertos esp_timer)