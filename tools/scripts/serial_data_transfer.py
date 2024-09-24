import time
import argparse

from ..utils import SerialDevice


def clear_buffer(device: SerialDevice) -> None:
    while True:
        data = device.read_data()
        if not data:
            break
    
    print("Buffer clear")


def send_data(device: SerialDevice, message: str) -> None:
    if message:
        message = f"{message}\n"
        device.write_data(message)
        print(f"Sent: {message}")

        while True:
            response = device.read_data()

            if response:
                print(f"Received: {response}")

                if "Done" in response:
                    print("Done")
                    break
            else:
                print("No data received")
                break
    else:
        print("There is no message to send")


def read_data(device: SerialDevice) -> None:
    response = device.read_data()
    if response:
        print(f"Received: {response}")
    else:
        print("No data received")


def main():
    parser = argparse.ArgumentParser(description="Serial communication with esp32 device")
    parser.add_argument(
        "--port", type=str, required=True, help="The port to which the device is connected"
    )
    parser.add_argument(
        "--baudrate", type=int, default=9600, help="Data transfer rate (default is 9600)"
    )
    parser.add_argument(
        "--timeout", type=float, default=1.0, help="Connection timeout (default is 1.0)"
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

    clear_buffer(device)
    
    try:
        while True:
            command = input("Enter the command ('send <message>' or 'read') or 'exit' to exit: ").strip()

            print(command)

            if command.lower() == 'exit':
                print("Completion of the program...")
                break

            if command.startswith("send"):
                _, message = command.split(maxsplit=1)
                send_data(device, message)
            elif command.lower() == "read":
                read_data(device)
            else:
                print("The wrong command. Use 'send <message>' or 'read'")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        device.disconnect()
    
    time.sleep(0.5)


if __name__ == "__main__":
    main()