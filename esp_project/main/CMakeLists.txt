idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES i2c pca9685 acs712 servo)