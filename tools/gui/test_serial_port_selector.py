import sys
from PyQt5.QtWidgets import (
    QApplication, QMessageBox
)

from ..utils.device import SerialDevice
from ..utils.logger import Logger

from .components import SerialPortSelector


class DeviceHandler:
    def __init__(self, selector):
        self.device = SerialDevice()
        # Logger.configure_logger_for_module("SerialDevice", level="DEBUG")
        self.selector = selector

    def connect(self, port):
        try:
            self.device.connect(port)
            success = self.device.is_connected()
            self.selector.connection_result_signal.emit(success)
            if success:
                    QMessageBox.information(None, "Connection", f"Successfully connected to {port}")
        except Exception as e:
            QMessageBox.critical(None, "Error", f"Failed to connect to {port}: {e}")
            self.selector.connection_result_signal.emit(False)

    def disconnect(self):
        self.device.disconnect()
        self.selector.connection_result_signal.emit(False)
        QMessageBox.information(None, "Device Disconnection", "Disconnected successfully.")


if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = SerialPortSelector()
    device_handle = DeviceHandler(window)
    window.connect_signal.connect(device_handle.connect)
    window.disconnect_signal.connect(device_handle.disconnect)
    window.show()

    sys.exit(app.exec_())
