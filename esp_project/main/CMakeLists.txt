idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES i2c acs712 pca9685 servo_pca9685 arm_robot)