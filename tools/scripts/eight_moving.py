import numpy as np
import argparse
import time
from ikpy.chain import Chain
from ikpy.link import URDFLink

from ..utils import SerialDevice 


robot_chain = Chain(name='robot_arm', links=[
    URDFLink(
        name="base",
        bounds=(-np.pi/2, np.pi/2), # от -90 до 90
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    # Первое звено (соединение с базой, плечо)
    URDFLink(
        name="shoulder",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.02],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
    ),
    # Второе звено (локоть)
    URDFLink(
        name="elbow",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.105],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    # Третье звено (запястье)
    URDFLink(
        name="wrist",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.096],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    # Четвертое звено (крутит захват)
    URDFLink(
        name="wrist_rotational",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.065],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    # Манипулятор (захват)
    URDFLink(
        name="gripper",
        bounds=None,
        origin_translation=[0, 0, 0.11],
        origin_orientation=[0, 0, 0],
        rotation=None,
        joint_type='fixed',
    )
])

# TODO: сделать функцию, которая делает точность для float до 2 знаков после запятой
# TODO: запускать отдельной таской проверку выполнения движения?
# TODO: вынести в отдельный файл модель руки в ikpy
# TODO попробовать делать больше точек движения?
# TODO: настроить точность максимального и минимального угла сервоприводов
# TODO: переделать работу со сервом и app_main, получше продумать команды в прошивке
# TODO: сделать генератор анимации движения по полученных углам + отображение точек движения
# TODO: мб уменьшить шаг и вместе с ним время ожидания
# TODO: вынести команды в отдельный файл в утилиты
# TODO: нужно учитывать время работы и может быть высчитывать его???
# Функция для отправки углов на микроконтроллер
def send_angles_to_robot(device : SerialDevice, joint_angles):
    for idx, angles in enumerate(joint_angles):
        # Ожидаем, пока is_moving не станет False
        while get_robot_movement_status(device):
            time.sleep(0.1)

        # Отправка углов на микроконтроллер
        base_angle = round(float(90 + angles[0]), 2)
        shoulder_angle = round(float(90 + angles[1]), 2)
        elbow_angle = round(float(90 + angles[2]), 2)
        wrist_angle = round(float(90 + angles[3]), 2)
        wrist_rot_angle = round(float(90 + angles[4]), 2)
        gripper_angle = round(float(angles[5]), 2)

        # Формирование и отправка команд на каждый сервопривод
        command = f"SET_ANGLES {base_angle} {shoulder_angle} {elbow_angle} {wrist_angle} {wrist_rot_angle}\n"

        device.write_data(command)
        # print(f"Sent: {command.strip()}")
        # print(f"len {len(joint_angles)}, idx: {idx}")
        if (idx == len(joint_angles) - 1):
            time.sleep(3)
        else:
            time.sleep(0.1)  # Небольшая задержка между отправками команд


# Функция для проверки состояния движения
def get_robot_movement_status(device: SerialDevice):
    device.write_data("GET_STATUS_MOVING\n")
    time.sleep(0.1)  # Небольшая задержка перед чтением ответа

    response = device.read_data()
    if response and response.startswith("STATUS"):
        try:
            # Пример: ответ "STATUS 0" или "STATUS 1"
            status = int(response.split()[1])
            return bool(status)  # Возвращаем True, если движение в процессе
        except (IndexError, ValueError):
            pass
    
    return False  # По умолчанию считаем, что движение завершено или произошла ошибка

# Пример использования функции отправки углов на микроконтроллер
if __name__ == '__main__':
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
    try:
        width = 0.1
        shift = 0.2
        points_number = 100
        t = np.linspace(0, 2 * np.pi, points_number)
        x = [shift] * points_number
        y = width * np.sin(2 * t)
        z = width * np.sin(t) + shift

        target_positions = np.vstack((x, y, z)).T

        initial_position = [0, 0, 0]

        joint_angles = []

        for idx, target in enumerate(target_positions):
            angles = robot_chain.inverse_kinematics(target_position=target)

            if angles is not None:
                joint_angles.append(angles)
                initial_position = angles
            else:
                print(f"точка {idx} недостижима: {target}")
                joint_angles.append(initial_position)
        
        # Преобразование углов в градусы
        joint_angles_degrees = np.degrees(joint_angles)

        # print(joint_angles_degrees)

        # Отправка углов на робота с проверкой статуса движения
        send_angles_to_robot(device, joint_angles_degrees)

        # Начальное состояние
        command = f"SET_ANGLES 90 90 90 90 90\n"
        device.write_data(command)
        while get_robot_movement_status(device):
            time.sleep(0.05)

        # Закрытие соединения после отправки
        device.disconnect()

    except KeyboardInterrupt as e:
        print("\nExiting...")
        device.disconnect()
