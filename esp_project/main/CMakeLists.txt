idf_component_register(SRCS "test_app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vfs i2c pca9685 acs712 servo_pca9685 arm_robot)