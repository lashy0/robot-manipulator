from typing import List

from ..utils import SerialDevice


def set_angle(device: SerialDevice, pwm_id: int, angle: float) -> None:
    """
    Set the angle for a specific PWM.

    Args:
        device (SerialDevice): The serial device to send the command to
        pwm_id (int): The ID of the PWM to set the angle for
        angle (int): The angle to set
    """
    command = f"SET_ANGLE {pwm_id} {angle}\n"
    device.write_data(command)


def set_manipulator(device: SerialDevice, angles: List[float]) -> None:
    """
    Set angles for the manipulator.

    Args:
        device (SerialDevice): The serial device to send the command to
        angles (List[float]): A list of angles for the manipulator joints. Must contain exactly 4 elements.
    """
    if len(angles) != 4:
        raise ValueError(
            "The angles list must contain exactly 4 elements."
        )
    
    command = f"SET_MANIPULATOR {angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
    device.write_data(command)


def set_grip(device: SerialDevice, state: bool) -> None:
    """
    Set the state of the gripper.

    Args:
        device (SerialDevice): The serial device to send the command to
        state (bool): The state of the grip. True for gripping (1), False for releasing (0)
    """
    command = f"SET_GRIP {int(state)}\n"
    device.write_data(command)
