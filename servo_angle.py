import serial
import time
import re

SERIAL_PORT = 'COM6'
BAUD_RATE = 115200

LOG_PATTERN = re.compile(r"^[IWF] \(\d+\).*")


def send_angle_to_esp32(angle):
    """ Sends the angle value to the ESP32 over USB Serial JTAG and reads responses """
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            # Send the angle value followed by newline to ESP32
            angle_str = str(angle) + '\n'
            ser.write(angle_str.encode('utf-8'))
            print(f"Sent: {angle_str.strip()}")

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
            angle = float(input("Enter servo angle (0 to 180 degrees): "))
            if 0 <= angle <= 180:
                send_angle_to_esp32(angle)
            else:
                print("Please enter a value between 0 and 180")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

        time.sleep(1)