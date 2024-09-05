import serial
import time
import re

SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

LOG_PATTERN = re.compile(r"^[IWF] \(\d+\).*")


def send_angle_to_esp32(channel, angle):
    """ Sends the angle value to the ESP32 over USB Serial JTAG and reads responses """
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            # Send the angle value followed by newline to ESP32
            command_str = f"{channel} {angle}\n"
            ser.write(command_str.encode('utf-8'))
            print(f"Sent: {command_str.strip()}")

            while True:
                response = ser.readline().decode('utf-8').strip()
                if response:
                    
                    # TODO: не работает
                    if LOG_PATTERN.match(response):
                        continue
                    
                    if "Done" in response:
                        print("Done")
                        break
                    
                    print(f"Received: {response}")
                else:
                    print("No response received")
                    break
    except serial.SerialException as e:
        print(f"Error opening or using the serial port: {e}")


if __name__ == "__main__":
    while True:
        try:
            # Get angle input from the user
            input_data = input("Enter servo channel (0-15) and angle (0 to 180 degrees): ")
            print(input_data)
            channel, angle = input_data.split()
            
            channel = int(channel)
            angle = float(angle)
            
            if 0 <= channel <= 15 and 0 <= angle <= 180:
                send_angle_to_esp32(channel, angle)
            else:
                print("Please enter a valid channel (0 to 15) and angle (0 to 180 degrees)")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

        time.sleep(1)