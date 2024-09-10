idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vfs i2c pca9685 acs712 servo arm)