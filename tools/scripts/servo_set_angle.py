import time
import argparse

from ..utils import SerialDevice
from ..utils import Logger


def clear_buffer(device: SerialDevice) -> None:
    while True:
        data = device.read_data()
        if not data:
            break
    
    print("Buffer clear")


def send_angles_to_esp32(device: SerialDevice, cmd: str, base: float, shoulder: float, elbow: float, wrist: float, wrist_rot: float):
    command = f"{cmd} {base} {shoulder} {elbow} {wrist} {wrist_rot}\n"
    device.write_data(command)

    while True:
        response = device.read_data()
        if response:
            if "Done" in response:
                print("Done")
                break
            # print(f"Received: {response}")
        else:
            print("No response received")
            break


def send_angle_to_esp32(device: SerialDevice, cmd: str, channel: int, angle: float):
    command = f"{cmd} {channel} {angle}\n"
    device.write_data(command)
    # print(f"Sent: {command.strip()}")

    while True:
        response = device.read_data()
        if response:
            if "Done" in response:
                print("Done")
                break
            # print(f"Received: {response}")
        else:
            print("No response received")
            break

def get_angle_to_esp32(device: SerialDevice, cmd: str, channel: int):
    command = f"{cmd} {channel}\n"
    device.write_data(command)
    print(f"Sent: {command.strip()}")

    while True:
        response = device.read_data()
        if response:
            if "Done" in response:
                print("Done")
                break
            # print(f"Received: {response}")
        else:
            print("No response received")
            break


# TODO: снова проблема с получением обратно значений от esp32 (отставание появляется)
if __name__ == "__main__":
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
    
    while True:
        # clear_buffer(device)

        try:
            input_data = input("Enter command (SET/GET_) servo channel (0-15) and angle (0 to 180 degrees): ")
            commands = input_data.split()

            command = commands[0].upper()
            channel = int(commands[1])

            if command == "SET_ANGLE":
                angle = float(commands[2])

                if 0 <= channel <= 15 and 0 <= angle <= 180:
                    send_angle_to_esp32(device, command, channel, angle)
                else:
                    print("Please enter a valid channel (0 to 15) and angle (0 to 180 degrees)")
            elif command == "GET_ANGLE":
                if 0 <= channel <= 15:
                    get_angle_to_esp32(device, command, channel)
                else:
                    print("Please enter a valid channel (0 to 15)")
            elif command == "SET_ANGLES":
                angle_base = float(commands[1])
                angle_shoulder = float(commands[2])
                angle_elbow = float(commands[3])
                angle_wrist= float(commands[4])
                angle_wrist_rot = float(commands[5])

                send_angles_to_esp32(device, command, angle_base, angle_shoulder, angle_elbow, angle_wrist, angle_wrist_rot)
            else:
                print("Please enter a valid command")

        except ValueError:
            print("Invalid input. Please enter a numbers")
        except KeyboardInterrupt:
            print("\nExiting...")
            device.disconnect()
            break

        time.sleep(0.5)
