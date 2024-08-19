Скрипт servo_set_angle.py запускать с залитой программой servo_angle_script.c в main.\
При измерение силы тока при установке угла сервопривода, сила тока особо не меняется.\
Или я как то не верно делаю замер, либо не верная реализация работы с ACS712.

Реализация ```servo``` очень простая, из-за чего движение сервопривода резкие.

## Компоненты

Написал компоненты для управления девайсами подключенными.

### PCA9685

PCA9685 - это 16-канальный 12-разрядный ШИМ-драйвер, используемый для управления такими устройствами, как сервоприводы и светодиоды.
Реализация взаимодействия по I2C.

**Основыные функции**:
- ```pca9685_init()```: Добавляет в инициализируную I2C шину PCA9685
- ```pca9685_set_pwm()```: Устанавливает PWM сигнал для определенного канала
- ```pca9685_get_pwm```: Считывает текущий PWM мигнал для указанного канала
- ```pca9685_set_pwm_freq```: Устанавливает частоту PWM
- ```pca9685_write```: Записывает байт данных в указанный регистор
- ```pca9685_read```: Считывает байт данных с указанного регистра 

### ASC712

ACS712 - это датчик тока на основе эффекта Холла, который измеряет ток, протекающий по цепи.

За пример работы с данным датчиком брал пример представленный по данной [ссылке](https://electronics.stackexchange.com/questions/422086/esp32-using-acs712-give-wrong-values).\
И если я правильно расмотрел маркировку чипа ACS712, то используется вариант на 5A.\
Еще [ссылка](https://3d-diy.ru/wiki/arduino-datchiki/datchik-toka-acs712/) про работу с ACS712.

**Основные функции**:
- ```acs712_init()```: Инициализирует ADC, используемый для считывания
- ```acs712_read_current```: Считывает ток, протекающий через датчик
- ```acs712_calibrate_voltage```: Калибрует напряжение смещения датчика для полкчения более точных показаний тока
- ```acs712_read_voltage```: Считывает текущий вольтаж
- ```acs712_rad_raw```: Считывает текущий сигнал с ADC

### Servo

Для управления сервоприводами с помощью PCA9685.

**Основные функции**:
- ```servo_init```: Инициализация сервопривода по определенному PWM каналу
- ```servo_set_angle```: Устанавливет угол наклона сервопривода

### Пример использования

```
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "i2c.h"
#include "pca9685.h"
#include "acs712.h"
#include "servo.h"

// Аддрес I2C дефолтный для PCA9685
#define PCA9685_I2C_ADDR           0x40

// Параметры подключения по I2C
#define I2C_MASTER_SCL_IO          GPIO_NUM_7
#define I2C_MASTER_SDA_IO          GPIO_NUM_6 
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000

// Параметры для ACS712
#define ACS712_ADC_CHANNEL         ADC_CHANNEL_2
#define ACS712_ADC_UNIT            ADC_UNIT_1
#define ACS712_ADC_ATTEN           ADC_ATTEN_DB_12
#define ACS712_SENSITIVITY         185.0

void app_main(void)
{
    i2c_config_bus_t i2c_master_config = {
        .port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
    };

    i2c_bus_t i2c_bus = {
        .is_initialized = false,
    };

    // Создаем соединение с I2C шиной
    i2c_master_init(&i2c_bus, &i2c_master_config);

    // Подключаем PCA9685 к шине I2C
    i2c_is_device_connected(&i2c_bus, PCA9685_I2C_ADDR, 50);

    // Выставляем частоту для PCA9685, чтобы работать с сервоприводами
    pca9685_set_pwm_freq(&pca9685, 50);

    // Инициализация датчика ACS712
    acs712_t acs712 = {
        .adc_channel = ACS712_ADC_CHANNEL,
        .sensitivity = ACS712_SENSITIVITY
    };

    acs712_init(&acs712, ACS712_ADC_UNIT, ACS712_ADC_ATTEN);

    // Получение вольтажа с помощью ACS712
    int offset_voltage;
    acs712_calibrate_voltage(&acs712, &offset_voltage);
    printf("Calibrated offset voltage: %d mV\n", offset_voltage);

    // Инициализация сервопривода под номерм 10 (серв в основании)
    // Задаем максимальный угол поворота
    servo_t servo;
    servo_init(&servo, pca9685, 10, 180);

    // Поаорот сервопривода на 90 градусов
    servo_set_angle(&servo, 90);
    vTaskDelay(pdMS_TO_TICKS(1000));

}
```