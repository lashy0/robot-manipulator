import serial
from PyQt5.QtWidgets import (
    QWidget, QSlider, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QComboBox
)
from PyQt5.QtCore import Qt
import serial.tools
import serial.tools.list_ports

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from device import SerialDevice

BAUD_RATE = 115200


class ServoControl(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.device: SerialDevice = None

        self.init_ui()
    
    def init_ui(self) -> None:
        layout = QVBoxLayout()

        # COM port selection field
        self.port_selector = QComboBox()
        self.__refresh_ports()
        layout.addWidget(self.port_selector)

        # The button for connecting to the device
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.__connect_to_port)
        layout.addWidget(self.connect_button)

        # Sliders for servo control
        self.sliders = []
        self.angle_labels = []

        for i in range(10, 16):
            h_layout = QHBoxLayout()
            label = QLabel(f"PWM {i}")
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)
            slider.setEnabled(False)
            slider.valueChanged.connect(lambda value, pwm=i: self.__update_servo(pwm, value))
            self.sliders.append(slider)

            angle_label = QLabel(f"Angle: {slider.value()}")
            self.angle_labels.append(angle_label)

            h_layout.addWidget(label)
            h_layout.addWidget(slider)
            h_layout.addWidget(angle_label)

            layout.addLayout(h_layout)
        
        self.setLayout(layout)
        self.setWindowTitle("Servo Control")

        self.resize(400, 300)
    
    def __refresh_ports(self) -> None:
        ports = serial.tools.list_ports.comports()
        self.port_selector.clear()
        for port in ports:
            self.port_selector.addItem(port.device)
    
    def __connect_to_port(self) -> None:
        port = self.port_selector.currentText()
        self.device = SerialDevice(port, BAUD_RATE)
        self.device.open()

        if self.device.is_connected():
            self.connect_button.setEnabled(False)
            self.port_selector.setEnabled(False)
            for slider in self.sliders:
                slider.setEnabled(True)
        else:
            ...
    
    def __update_servo(self, pwm_id: int, angle: int) -> None:
        if self.device and self.device.is_connected():
            command_str = f"{pwm_id} {angle}"
            self.device.send_data(command_str)
            print(f"Sent: {command_str.strip()}")

            self.angle_labels[pwm_id-10].setText(f"Angle: {angle}")

            response = self.device.read_data()
            if response:
                print(f"Received: {response}")
                if "Done" in response:
                    print("Done")
            else:
                print("No response received")
        else:
            print("Serial port not open")
    
    def closeEvent(self, event) -> None:
        if self.device and self.device.is_connected():
            self.device.close()
        event.accept()

if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv)
    control = ServoControl()
    control.show()

    sys.exit(app.exec_())
