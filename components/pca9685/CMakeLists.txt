idf_component_register(SRCS "pca9685.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos i2c_tools)