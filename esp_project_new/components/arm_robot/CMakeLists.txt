idf_component_register(SRCS "arm_robot.c"
                    INCLUDE_DIRS "include"
                    REQUIRES servo_pca9685 freertos)