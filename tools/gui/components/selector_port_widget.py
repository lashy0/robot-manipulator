from serial.tools import list_ports
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton
)
from PyQt5.QtCore import QTimer, pyqtSignal


class SerialPortSelector(QWidget):
    """
    Widget for selecting and connecting to a serial COM port.

    Attributes:
        connect_signal (pyqtSignal): Signal emitted when a connection is requested, with the selected COM port as argument.
        disconnect_signal (pyqtSignal): Signal emitted when a disconnection is requested.
        connection_result_signal (pyqtSignal): Signal emitted to indicate the result of a connection attempt.
    """
    connect_signal = pyqtSignal(str)
    disconnect_signal = pyqtSignal()
    connection_result_signal = pyqtSignal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)

        hbox_layout = QHBoxLayout()
        # Combox COM-port
        self.combobox = QComboBox()
        self.combobox.setEditable(True)
        self.combobox.lineEdit().setReadOnly(True)
        self.combobox.lineEdit().setPlaceholderText("Select COM port")
        hbox_layout.addWidget(self.combobox)

        # Button Connect/Disconnect
        self.button = QPushButton("Connect")
        self.button.clicked.connect(self.handle_button)
        hbox_layout.addWidget(self.button)

        layout.addLayout(hbox_layout)

        # Timer update list COM-port
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_com_ports)
        self.timer.start(1000)

        self.update_com_ports()

        # Connect the connection result signal
        self.connection_result_signal.connect(self.handle_connection_result)
    
    def update_com_ports(self) -> None:
        """
        Updates the list of available COM ports in the combo box.
        """
        current_ports = set(port.device for port in list_ports.comports())
        existing_ports = set(self.combobox.itemText(i) for i in range(self.combobox.count()))

        if current_ports != existing_ports:
            self.combobox.clearEditText()
            self.combobox.clear()
            self.combobox.addItems(current_ports)
    
    def handle_button(self) -> None:
        """
        Handles the Connect/Disconnect button click event.
        Emits the appropriate signal based on the current state.
        """
        if self.button.text() == "Connect":
            selected_port = self.combobox.currentText().strip()
            if selected_port and selected_port != "Select COM port":
                self.connect_signal.emit(selected_port)
        else:
            self.disconnect_signal.emit()
    
    def handle_connection_result(self, success: bool) -> None:
        """
        Handles the result of a connection attempt.
        Updates the button text based on the connection result.

        Args:
            success (bool): True if the connection was successful, False otherwise.
        """
        if success:
            self.button.setText("Disconnect")
        else:
            self.button.setText("Connect")
