import sys
import time
import signal
import argparse

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLineEdit, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from ..utils import SerialDevice


def clear_buffer(device: SerialDevice) -> None:
    while True:
        data = device.read_data()
        if not data:
            break
    
    print("Buffer clear")


# Поток для отправки данных на ESP32
class WriteThread(QThread):
    write_signal = pyqtSignal(str)

    def __init__(self, device: SerialDevice):
        super().__init__()
        self.device = device
        self.running = True

    def run(self):
        while self.running:
            time.sleep(0.1)

    def stop(self):
        self.running = False

    def add_command(self, command):
        try:
            self.device.write_data(command)
            print(f"Sent: {command.strip()}")
        except Exception as e:
            print(f"Error sending command: {e}")


# Поток для чтения данных с ESP32
class DataThread(QThread):
    data_received = pyqtSignal(float)

    def __init__(self, device: SerialDevice):
        super().__init__()
        self.device = device
        self.running = True

    def run(self):
        while self.running:
            data = self.device.read_data()
            try:
                if data:
                    random_value = float(data)
                    self.data_received.emit(random_value)
            except ValueError:
                pass
            time.sleep(0.1)

    def stop(self):
        self.running = False


# Класс для графика
class RealTimePlot(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)

        self.setParent(parent)
        self.data = []

        self.ax.set_title('Real-Time Data')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')

    def update_plot(self, value):
        self.data.append(value)
        if len(self.data) > 100:
            self.data.pop(0)
        
        self.ax.clear()
        self.ax.plot(self.data, 'b-')
        self.ax.set_title('Real-Time Data')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        self.draw()


# Основной класс приложения
class MainWindow(QMainWindow):
    def __init__(self, device: SerialDevice):
        super().__init__()

        self.device = device

        self.initUI()

        self.data_thread = DataThread(self.device)
        self.data_thread.data_received.connect(self.plot.update_plot)
        self.data_thread.start()

        self.write_thread = WriteThread(self.device)
        self.write_thread.start()

    def initUI(self):
        self.setWindowTitle('Real-Time Plot and Control')

        self.plot = RealTimePlot(self)

        # Слайдеры для работы с сервами
        self.sliders = []
        self.slider_labels = []
        self.labels_angle = []
        self.channel_start = 10
        self.num_servos = 6
        self.servo_angles = [90] * self.num_servos

        sliders_layout = QVBoxLayout()
        for i in range(self.num_servos):
            h_layout = QHBoxLayout()
            slider_label = QLabel(f'Servo {i + 1} (PWM {self.channel_start + i}):')
            # slider_label.setFixedWidth(150)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(180)
            slider.setValue(self.servo_angles[i])
            slider.valueChanged.connect(lambda value, channel=self.channel_start + i: self.on_slider_change(channel, value))
            self.sliders.append(slider)
            self.slider_labels.append(slider_label)

            angle_label = QLabel(f"Angle: {slider.value()}")
            self.labels_angle.append(angle_label)

            h_layout.addWidget(slider_label)
            h_layout.addWidget(slider)
            h_layout.addWidget(angle_label)
            sliders_layout.addLayout(h_layout)

        # Поле ввода и кнопка для отправки данных
        self.input_line = QLineEdit(self)
        self.input_line.setPlaceholderText('Enter data to send')
        self.send_button = QPushButton('Send to ESP32', self)
        self.send_button.clicked.connect(self.send_data_to_esp)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.plot)
        layout.addLayout(sliders_layout)
        layout.addWidget(self.input_line)
        layout.addWidget(self.send_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.setGeometry(100, 100, 800, 600)
    
    def on_slider_change(self, channel, value):
        self.labels_angle[channel - self.channel_start].setText(f"Angle: {value}")

        self.servo_angles[channel - self.channel_start] = value
        command = f"SET_ANGLE {channel} {value}\n"
        self.write_thread.add_command(command)
        # self.device.write_data(command)
        # print(f"Sent: {command.strip()}")

    def send_data_to_esp(self):
        data_to_send = self.input_line.text()
        if data_to_send:
            self.device.write_data(data_to_send + '\n')
            self.input_line.clear()

    def closeEvent(self, event):
        self.device.disconnect()
        print("Closing application...")
        
        self.data_thread.stop()
        self.data_thread.wait()
        print("DataThread stopped")

        self.write_thread.stop()
        self.write_thread.wait()
        print("WriteThread stopped")

        event.accept()


def signal_handler(signal, frame):
    print("\nExiting...")
    QApplication.quit()


def main():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--port", required=True, help="The port to which the device is connected"
    )
    parser.add_argument(
        "--baudrate", type=int, default=9600, help="Data transfer rate (default is 9600)"
    )
    parser.add_argument(
        "--timeout", type=int, default=1, help="Connection timeout (default is 1 sec)"
    )

    args = parser.parse_args()

    device = SerialDevice(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout
    )
    device.connect()

    if not device.is_connected():
        print("Failed to connect to the device")
        exit()
    
    # clear_buffer(device)

    # Ctrl + C
    signal.signal(signal.SIGINT, signal_handler)

    # Запуск приложения
    app = QApplication(sys.argv)
    window = MainWindow(device)
    window.show()
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt as e:
        pass


if __name__ == '__main__':
    main()
