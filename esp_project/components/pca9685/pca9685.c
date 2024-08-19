#include "esp_log.h"
#include "pca9685.h"

static const char *TAG = "pca9685";

esp_err_t pca9685_write(pca9685_t *handle, uint8_t reg, uint8_t data)
{
    esp_err_t ret;
    uint8_t buffer[2] = {reg, data};

    ret = i2c_master_transmit(handle->i2c_dev, buffer, sizeof(buffer), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data the device");
        return ret;
    }

    return ESP_OK;
}

esp_err_t pca9685_read(pca9685_t *handle, uint8_t reg, uint8_t *data)
{
    esp_err_t ret;

    ret = i2c_master_transmit(handle->i2c_dev, &reg, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address");
        return ret;
    }

    ret = i2c_master_receive(handle->i2c_dev, data, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data the device");
        return ret;
    }

    return ESP_OK;
}

esp_err_t pca9685_init(pca9685_config_t *config, pca9685_t *handle)
{
    if (handle->is_initialized) {
        ESP_LOGW(TAG, "I2C device is already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    // Configure the I2C device for PCA9685
    i2c_device_config_t i2c_dev_config = {
        .scl_speed_hz = config->scl_speed,
        .device_address = config->i2c_address,
    };

    // Add the PCA9685 device to the I2C bus
    ret = i2c_master_bus_add_device(config->bus_handle, &i2c_dev_config, &handle->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCA9685 device to I2C bus");
        return ret;
    }

    handle->is_initialized = true;

    ret = pca9685_write(handle, PCA9685_MODE1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685");
        free(handle);
        return ret;
    }

    ESP_LOGI(TAG, "PCA9685 initialized successfully");

    return ESP_OK;
}

esp_err_t pca9685_set_pwm(pca9685_t *handle, uint8_t channel, uint16_t on_time, uint16_t off_time)
{
    esp_err_t ret;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;

    uint8_t data[5] = {
        reg,
        on_time & 0xFF,
        (on_time >> 8) & 0xFF,
        off_time & 0xFF,
        (off_time >> 8) & 0xFF
    };

    ret = i2c_master_transmit(handle->i2c_dev, data, sizeof(data), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM register");
        return ret;
    }

    return ESP_OK;
}

esp_err_t pca9685_get_pwm(pca9685_t *handle, uint8_t channel, uint16_t *on_data, uint16_t *off_data)
{
    esp_err_t ret;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4];

    ret = i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, data, 4, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PWM register");
        return ret;
    }

    return ESP_OK;
}

esp_err_t pca9685_reset(pca9685_t *handle)
{
    esp_err_t ret = pca9685_write(handle, PCA9685_MODE1, MODE1_RESTART);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write RESTART bit to MODE1 register");
        return ret;
    }

    ESP_LOGI(TAG, "PCA9685 reset");

    return ESP_OK;
}

esp_err_t pca9685_sleep(pca9685_t *handle)
{
    esp_err_t ret;
    uint8_t mode;

    ret = pca9685_read(handle, PCA9685_MODE1, &mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MODE1 register");
        return ret;
    }

    mode |= MODE1_SLEEP;
    ret = pca9685_write(handle, PCA9685_MODE1, mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SLEEP bit to MODE1 register");
        return ret;
    }

    ESP_LOGI(TAG, "PCA9685 is now in sleep mode");

    return ESP_OK;
}

esp_err_t pca9685_wake(pca9685_t *handle)
{
    esp_err_t ret;
    uint8_t mode;

    ret = pca9685_read(handle, PCA9685_MODE1, &mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MODE1 register");
        return ret;
    }

    mode &= ~MODE1_SLEEP;
    ret = pca9685_write(handle, PCA9685_MODE1, mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear SLEEP bit in MODE1 register");
        return ret;
    }

    ESP_LOGI(TAG, "PCA9685 is now awake");

    return ESP_OK;
}

esp_err_t pca9685_set_pwm_freq(pca9685_t *handle, float freq)
{
    esp_err_t ret;
    uint8_t old_mode;
    uint8_t new_mode;

    float prescaleval = (uint8_t)(FREQUENCY_OSCILLATOR / (4096 * freq)) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    ret = pca9685_read(handle, PCA9685_MODE1, &old_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MODE1 register");
        return ret;
    }

    new_mode = (old_mode &~MODE1_RESTART) | MODE1_SLEEP;
    ret = pca9685_write(handle, PCA9685_MODE1, new_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sleep mode");
        return ret;
    }

    ret = pca9685_write(handle, PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set prescaler");
        return ret;
    }

    ret = pca9685_write(handle, PCA9685_MODE1, old_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore MODE1 register");
        return ret;
    }

    ret = pca9685_write(handle, PCA9685_MODE1, old_mode | MODE1_RESTART | MODE1_AI);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable auto-increment mode");
        return ret;
    }

    ESP_LOGI(TAG, "PWM frequency set to %.2f Hz", freq);

    return ESP_OK;
}

#if 0
void pca9685_deinit(pca9685_t *handle)
{
    if (!handle->is_initialized) {
        ESP_LOGE(TAG, "I2C device is not initialized");
        return;
    }

    esp_err_t ret = pca9685_sleep(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to put PCA9685 into sleep mode");
    }

    ret = i2c_master_bus_rm_device(handle->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove PCA9685 device from I2C bus");
    }

    handle->is_initialized = false;

    ESP_LOGI(TAG, "PCA9685 deinitialized successfully");
}
#endif