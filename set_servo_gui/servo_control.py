import serial
import time
from PyQt5.QtWidgets import (
    QWidget, QSlider, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QComboBox
)
from PyQt5.QtCore import Qt
import serial.tools
import serial.tools.list_ports

BAUD_RATE = 115200


class ServoControl(QWidget):
    def __init__(self):
        super().__init__()

        self.serial_port = None
        self.ser = None

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Работа с портом
        self.port_selector = QComboBox()
        self.__refresh_ports()
        layout.addWidget(self.port_selector)

        # Подключение к выбранеому порту
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.__connect_to_port)
        layout.addWidget(self.connect_button)

        # Создание слайдера для управления сервоприводами
        self.sliders = []
        self.labels = []
        self.angle_labels = []

        for i in range(10, 16):
            h_layout = QHBoxLayout()
            label = QLabel(f"Servo {i}")
            self.labels.append(label)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)
            # slider.setEnabled(False)
            slider.valueChanged.connect(lambda value, servo=i-10: self.__update_servo(servo, value))

            angle_label = QLabel(f"Angle: {slider.value()}")
            self.angle_labels.append(angle_label)

            h_layout.addWidget(label)
            h_layout.addWidget(slider)
            h_layout.addWidget(angle_label)
            self.sliders.append(slider)
            layout.addLayout(h_layout)

        self.setLayout(layout)
        self.setWindowTitle("Servo Control")
        self.show()

    def __refresh_ports(self):
        # Обновление списка доступных COM портов
        ports = serial.tools.list_ports.comports()
        self.port_selector.clear()
        for port in ports:
            self.port_selector.addItem(port.device)

    def __connect_to_port(self):
        # Подключение к выбранному порту
        self.serial_port = self.port_selector.currentText()
        try:
            self.ser = serial.Serial(self.serial_port, BAUD_RATE, timeout=1)
            print(f"Connected to {self.serial_port}")
            self.connect_button.setEnabled(False)
            self.port_selector.setEnabled(False)
            for slider in self.sliders:
                slider.setEnabled(True)
        except serial.SerialException as e:
            print(f"Error opening serial port {self.serial_port}: {e}")
    
    def __update_servo(self, servo, angle):
        # Отправка значений угла на esp32c3
        if self.ser is not None and self.ser.is_open:
            command_str = f"{servo} {angle}"
            self.ser.write(command_str.encode('utf-8'))
            print(f"Sent: {command_str.strip()}")

            self.angle_labels[servo].setText(f"Angle: {angle}")

            while True:
                response = self.ser.readlines().decode('utf-8').strip()
                if response:
                    print(f"Received: {response}")
                    if "Done" in response:
                        print("Done")
                        break
                else:
                    print("No response received")
                    break
        else:
            print("Serial port not open")
    
    def closeEvent(self, event):
        # Закрыть порт при закрытии окна
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        event.accept()
