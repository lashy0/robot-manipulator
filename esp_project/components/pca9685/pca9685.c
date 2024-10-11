#include "esp_log.h"
#include "esp_err.h"
#include "pca9685.h"

static const char *TAG = "pca9685";

// TODO: заменить -1 на настраиваемое значение 
esp_err_t pca9685_write(pca9685_t *pca9685, uint8_t reg, uint8_t data)
{
    esp_err_t ret;
    uint8_t buffer[2] = {reg, data};

    ret = i2c_master_transmit(pca9685->i2c_dev, buffer, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data the device: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pca9685_read(pca9685_t *pca9685, uint8_t reg, uint8_t *data)
{
    esp_err_t ret;

    ret = i2c_master_transmit_receive(pca9685->i2c_dev, &reg, 1, data, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data the device: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

void pca9685_set_osc_freq(pca9685_t *pca9685, uint32_t freq)
{
    pca9685->oscillator_freq = freq;
}

void pca9685_get_osc_freq(pca9685_t *pca9685, uint32_t *freq)
{
    *freq = pca9685->oscillator_freq;
}

esp_err_t pca9685_init(pca9685_config_t *config, pca9685_t *pca9685)
{
    if (pca9685->is_initialized) {
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
    ret = i2c_master_bus_add_device(config->bus_handle, &i2c_dev_config, &pca9685->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCA9685 device to I2C bus: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    pca9685->is_initialized = true;

     ret = pca9685_write(pca9685, PCA9685_MODE1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685");
        return ESP_FAIL;
    }

    // Set default value on the structure PCA9685
    pca9685_set_osc_freq(pca9685, FREQUENCY_OSCILLATOR);

    pca9685_set_pwm_freq(pca9685, 50);

    return ESP_OK;
}

esp_err_t pca9685_deinit(pca9685_t *pca9685)
{
    if (pca9685->is_initialized) {
        ESP_LOGE(TAG, "I2C device is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    ret = i2c_master_bus_rm_device(pca9685->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove PCA9685 device: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pca9685_reset(pca9685_t *pca9685)
{
    esp_err_t ret;

    ret = pca9685_write(pca9685, PCA9685_MODE1, MODE1_RESTART);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write RESTART bit to MODE1 register");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PCA9685 reset");

    return ESP_OK;
}

esp_err_t pca9685_sleep(pca9685_t *pca9685)
{
    esp_err_t ret;
    uint8_t mode;

    ret = pca9685_read(pca9685, PCA9685_MODE1, &mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data MODE1 register");
        return ESP_FAIL;
    }

    ret = pca9685_write(pca9685, PCA9685_MODE1, mode | MODE1_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SLEEP bit to MODE register");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PCA9685 in sleep mode");

    return ESP_OK;
}

esp_err_t pca9685_wake(pca9685_t *pca9685)
{
    esp_err_t ret;
    uint8_t mode;

    ret = pca9685_read(pca9685, PCA9685_MODE1, &mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data MODE1 register");
        return ESP_FAIL;
    }

    ret = pca9685_write(pca9685, PCA9685_MODE1, mode & ~MODE1_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to cleare SLEEP bit in MODE1 register");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PCA9685 in now awake");

    return ESP_OK;
}

esp_err_t pca9685_get_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t *on_time, uint16_t *off_time)
{
    esp_err_t ret;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4];

    ret = i2c_master_transmit_receive(pca9685->i2c_dev, &reg, 1, data, 4, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PWM register: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    *on_time = (data[1] << 8) | data[0];
    *off_time = (data[3] << 8) | data[2];

    return ESP_OK;
}

esp_err_t pca9685_set_pwm(pca9685_t *pca9685, uint8_t channel, uint16_t on_time, uint16_t off_time)
{
    esp_err_t ret;
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[5] = {
        reg,
        on_time,
        on_time >> 8,
        off_time,
        off_time >> 8
    };

    ret = i2c_master_transmit(pca9685->i2c_dev, data, 5, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM register: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pca9685_set_pwm_freq(pca9685_t *pca9685, float freq)
{
    if (freq <= 0.0) {
        ESP_LOGE(TAG, "Inwalid PWM frequency: %.2f. Must be greater than 0", freq);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t old_mode;
    uint8_t new_mode;

    if (freq < MIN_PWM_FREQUENCY) freq = MIN_PWM_FREQUENCY;
    if (freq > MAX_PWM_FREQUENCY) freq = MAX_PWM_FREQUENCY;

    float prescaleval = ((pca9685->oscillator_freq / (4096.0 * freq)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    ret = pca9685_read(pca9685, PCA9685_MODE1, &old_mode);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    new_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    ret = pca9685_write(pca9685, PCA9685_MODE1, new_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sleep mode");
        return ESP_FAIL;
    }

    ret = pca9685_write(pca9685, PCA9685_PRESCALE, prescale);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set prescaler");
        return ESP_FAIL;
    }

    ret = pca9685_write(pca9685, PCA9685_MODE1, old_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore MODE1 register");
        return ESP_FAIL;
    }

    ret = pca9685_write(pca9685, PCA9685_MODE1, old_mode | MODE1_RESTART | MODE1_AI);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MODE1 register to rurn on auto increment");
        return ESP_FAIL;
    }

    pca9685->pwm_freq = freq;

    ESP_LOGI(TAG, "PWM frequency set to %.2f", freq);

    return ESP_OK;
}