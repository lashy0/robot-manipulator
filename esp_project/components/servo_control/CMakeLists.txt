idf_component_register(SRCS "servo_control.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos pca9685)