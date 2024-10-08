import time
import serial

from .logger import Logger


class SerialDevice:
    def __init__(self, port: str = None, baudrate: int = 9600, timeout: float = 1.0):
        self._serial = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._logger = Logger().get_logger("SerialDevice", level="ERROR")
    
    def is_connected(self) -> bool:
        return self._serial and self._serial.is_open
    
    def connect(self, port: str = None) -> None:
        if port:
            self.port = port
        
        if not self.port:
            self._logger.error("No COM port specified")
            return

        if not self.is_connected():
            try:
                self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._logger.debug(f"Connected to {self.port} at {self.baudrate} baudrate")
            except serial.SerialException as e:
                self._logger.error(f"Failed to connect to {self.port}: {e}")
    
    def disconnect(self) -> None:
        if self.is_connected():
            self._serial.close()
            self._logger.debug(f"Disconnected from {self.port}")
    
    def reconnect(self, retrise: int = 3, delay: float = 1.0) -> None:
        for attempt in range(retrise):
            self.disconnect()
            time.sleep(delay)
            self.connect()
            if self.is_connected():
                self._logger.debug(f"Reconnected to {self.port} on attempt {attempt + 1}")
                return
            self._logger.error(f"Failed to reconnect to {self.port} after {retrise} attempts")
    
    def write_data(self, data: str, timeout: float = None) -> None:
        """Writes data to the serial device"""
        if self.is_connected():
            try:
                if timeout:
                    self._serial.timeout = timeout
                self._serial.write(data.encode('utf-8'))
                self._logger.debug(f"Sent: {data}")
            except serial.SerialException as e:
                self._logger.error(f"Failed to write data: {e}")
        else:
            self._logger.error(f"Device {self.port} is not connected")
    
    def read_data(self, timeout: float = None) -> str:
        """Reads data from the serial device"""
        if self.is_connected():
            try:
                if timeout:
                    self._serial.timeout = timeout
                data = self._serial.readline().decode('utf-8').strip()
                self._logger.debug(f"Received: {data}")
                return data
            except serial.SerialException as e:
                self._logger.error(f"Failed to read data: {e}")
                return ""
        else:
            self._logger.error(f"Device {self.port} is not connected")
            return ""
    
    def flush_input(self) -> None:
        """Flush the input buffer of the serial connection"""
        if self.is_connected():
            self._serial.reset_input_buffer()
            self._logger.debug("Input buffer reset")
    
    def flush_output(self) -> None:
        """Flush the output buffer of the serial connection"""
        if self.is_connected():
            self._serial.reset_output_buffer()
            self._logger.debug("Output buffer reset")
