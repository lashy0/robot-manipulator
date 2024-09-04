import serial


class SerialDevice:
    def __init__(self, port: str, baudrate: int = 9600, timeout:int = 1) -> None:
        """
        Initializes the SerialDevice

        Args:
            port (str): The name of the COM port.
            baudrate (int): The baud rate for the  serial connection.
            timeout (int): The timeout value for the serial connection in seconds.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect = None
    
    def open(self) -> None:
        """Opens a connection to the serial device."""
        try:
            self.connect = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            if self.connect.is_open:
                print(f"Connected to {self.port}")
            else:
                raise Exception("Failed to open the connection")
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
    
    def close(self):
        """Close the connection to the serial device"""
        if self.connect and self.connect.is_open:
            self.connect.close()
            print(f"Connected to {self.port} is closed")

    def is_connected(self):
        """Checks if the serial connection is open"""
        return self.connect and self.connect.is_open
    
    def send_data(self, data):
        """
        Sends data to the serial device.

        Args:
            data (str): The data to be send as a string.
        """
        if self.is_connected:
            if isinstance(data, str):
                data = data.encode('utf-8')
            self.connect.write(data)
            # print(f"Data sent: {data}")
        else:
            print("The connection is nit established")
    
    def read_data(self) -> str:
        """
        Reads data from the serial device.

        Returns:
            str: The data received from the device, or None if no data was received.
        """
        if self.is_connected:
            try:
                data = self.connect.readline().decode('utf-8').strip()
                # print(f"Data received: {data}")
                return data
            except serial.SerialException as e:
                print(f"Error reading data from thr port {self.port}: {e}")
        else:
            print("The connection is nit established")
            return None
