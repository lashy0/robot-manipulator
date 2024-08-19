idf_component_register(SRCS "servo_angle_script.c"
                    INCLUDE_DIRS "."
                    REQUIRES vfs i2c pca9685 acs712 servo)