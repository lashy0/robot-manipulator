#include "i2c_handler.h"
#include "pca9685.h"

static const char *TAG = "pca9685";
static i2c_master_dev_handle_t pca9685_handle;

// TODO: добавить проверку, что инициализирован I2C
esp_err_t pca9685_init(void)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_get_bus_handle(), &dev_config, &pca9685_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCA9685 device to I2C bus");
        return ret;
    }
    ESP_LOGI(TAG, "PCA9685 device add to I2C bus successfully");

    ret = pca9685_write(PCA9685_MODE1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset PCA9685");
        return ret;
    }

    ret = pca9685_set_pwm_freq(PWM_DEFAULT_FREQ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set default PWM frequency");
        return ret;
    }

    return ESP_OK;
}

esp_err_t pca9685_write(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    esp_err_t ret = i2c_master_transmit(pca9685_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    return ret;
}

esp_err_t pca9685_read(uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit(pca9685_handle, &reg, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_receive(pca9685_handle, data, 1, I2C_MASTER_TIMEOUT_MS);
    return ret;
}

esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG; // PCA9685 16 channel
    }

    uint8_t reg_base = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t buffer[5] = {
        reg_base,
        on & 0xFF,
        on >> 8,
        off & 0xFF,
        off >> 8
    };

    esp_err_t ret = i2c_master_transmit(pca9685_handle, buffer, sizeof(buffer), I2C_MASTER_TIMEOUT_MS);
    return ret;
}

esp_err_t pca9685_get_pwm(uint8_t channel, uint16_t *on, uint16_t *off)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG; // PCA9685 16 channel
    }

    uint8_t reg_base = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t buffer[4];

    esp_err_t ret = i2c_master_transmit(pca9685_handle, &reg_base, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_receive(pca9685_handle, buffer, sizeof(buffer), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    *on = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    *off = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);

    return ESP_OK;
}

// TODO: подробней разобраться с данным кодом
esp_err_t pca9685_set_pwm_freq(uint16_t freq_hz)
{
    // TODO: добавить проверку входного значения freq_hz

    uint8_t prescale_val = (uint8_t)(FREQUENCY_OSCILLATOR / (4096.0 * freq_hz) - 1 + 0.5);

    if (prescale_val < PCA9685_PRESCALE_MIN || prescale_val > PCA9685_PRESCALE_MAX) {
        ESP_LOGE(TAG, "Calculated prescale value out of range");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mode1;
    esp_err_t ret = pca9685_read(PCA9685_MODE1, &mode1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MODE1 register");
        return ret;
    }

    ret = pca9685_write(PCA9685_MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to put device to sleep");
        return ret;
    }

    ret = pca9685_write(PCA9685_PRESCALE, prescale_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write prescale value");
        return ret;
    }

    ret = pca9685_write(PCA9685_MODE1, (mode1 & ~MODE1_SLEEP) | MODE1_AI | MODE1_RESTART);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore MODE1 and enable auto-increment");
        return ret;
    }

    ESP_LOGI(TAG, "PWM frequency set to %d Hz with prescale value %d", freq_hz, prescale_val);
    return ESP_OK;
}