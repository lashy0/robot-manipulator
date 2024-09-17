Запуск скрипта для управления сервами или получения текущего значения угла сервопривода:

```
python -m tools.scripts.servo_set_angle --port <свое значение> --baudrate 115200
```

Две команды:
- get_angle <номер pwm>
- set_angle <номер pwm> <угол>