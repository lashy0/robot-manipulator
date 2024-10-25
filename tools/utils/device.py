import time
import serial

from .logger import Logger


_logger = Logger().get_logger("SerialDevice", level="ERROR")


class SerialDevice:
    """Class to manage a serial connection to a device.

    Attributes:
        port (str): 
            The COM port for the serial connection.
        baudrate (int):
            The baud rate for the serial connection.
        timeout (float):
            The read timeout for the serial connection.
        _serial (serial.Serial):
            The pySerial instance representing the connection.
    """
    def __init__(self, port: str = None, baudrate: int = 9600, timeout: float = 1.0):
        """Initialize the SerialDevice.

        Args:
            port:
                The COM port to connect.
            baudrate:
                The baud rate the connection.
            timeout:
                The timeout for reading from the serial connection.
        """
        self._serial = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
    
    def is_connected(self) -> bool:
        """Check if the serial connection is open

        Returns:
            bool: True if the connection is open, False otherwise.
        """
        return self._serial and self._serial.is_open
    
    def connect(self, port: str = None) -> None:
        """Connects to the serial device.

        Args:
            port (str, optional):
                The COM port to connect to.
        
        Raises:
            serial.SerialException:
                If the connection to the serial port fails.
        """
        if port:
            self.port = port
        
        if not self.port:
            _logger.error("No COM port specified")
            return

        if not self.is_connected():
            try:
                self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                _logger.debug(f"Connected to {self.port} at {self.baudrate} baudrate")
            except serial.SerialException as e:
                _logger.error(f"Failed to connect to {self.port}: {e}")
                raise
    
    def disconnect(self) -> None:
        """Disconnects from the serial device if connected."""
        if self.is_connected():
            self._serial.close()
            _logger.debug(f"Disconnected from {self.port}")
    
    def reconnect(self, retrise: int = 3, delay: float = 1.0) -> None:
        """Attempts to reconnect to the serial device.

        Args:
            retrise (int):
                Number of reconnection attempts.
            delay (float):
                Delay between reconnection attempts is seconds.
        """
        for attempt in range(retrise):
            self.disconnect()
            time.sleep(delay)
            self.connect()
            if self.is_connected():
                _logger.debug(f"Reconnected to {self.port} on attempt {attempt + 1}")
                return
            _logger.error(f"Failed to reconnect to {self.port} after {retrise} attempts")
    
    def write_data(self, data: str, timeout: float = None) -> None:
        """Writes data to the serial device.
        
        Args:
            data (str):
                The data to be written to the serial device.
            timeout (flaot, optional):
                Timeout for writing data.
        
        Raises:
            serial.SerialException:
                If writing to the serial port fails.
        """
        if self.is_connected():
            try:
                if timeout:
                    self._serial.timeout = timeout
                self._serial.write(data.encode('utf-8'))
                _logger.debug(f"Sent to {self.port}: {data}")
            except serial.SerialException as e:
                _logger.error(f"Failed to write data to {self.port}: {e}")
                raise
        else:
            _logger.error(f"Device {self.port} is not connected")
    
    def read_data(self, timeout: float = None) -> str:
        """Reads data from the serial device.

        Args:
            timeout (float, optional):
                Timeout for reading data.
        
        Returns:
            str: The data read from the serial devic, or an empty string if the read fails.
        
        Raises:
            serial.SerialException:
                If reading from the serial port fails.
        """
        if self.is_connected():
            try:
                if timeout:
                    self._serial.timeout = timeout
                data = self._serial.readline().decode('utf-8').strip()
                _logger.debug(f"Received from {self.port}: {data}")
                return data
            except serial.SerialException as e:
                _logger.error(f"Failed to read data from {self.port}: {e}")
                raise
        else:
            _logger.error(f"Device {self.port} is not connected")
            return ""
    
    def flush_input(self) -> None:
        """Flushes the input buffer of the serial connection.

        Raises:
            serial.SerialException:
                If flushing the input buffer fails.
        """
        if self.is_connected():
            try:
                self._serial.reset_input_buffer()
                _logger.debug(f"Input buffer reset for {self.port}")
            except serial.SerialException as e:
                _logger.error(f"Failed to flush input buffer for {self.port}: {e}")
                raise
    
    def flush_output(self) -> None:
        """Flushes the output buffer of the serial connection.

        Raises:
            serial.SerialException:
                If flushing the output buffer fails.
        """
        if self.is_connected():
            try:
                self._serial.reset_output_buffer()
                _logger.debug(f"Output buffer reset for {self.port}")
            except serial.SerialException as e:
                _logger.error(f"Failed to flush output buffer for {self.port}: {e}")
                raise
