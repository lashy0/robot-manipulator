idf_component_register(SRCS "servo_pca9685.c" "servo_pca9685_motion.c"
                    INCLUDE_DIRS "include"
                    REQUIRES pca9685 freertos esp_timer)