from typing import List

from ..utils import SerialDevice


def set_angle(device: SerialDevice, pwm_id: int, angle: float) -> None:
    """Set the angle for a specific PWM.

    Sends a command to adjust the angle for a specified PWM ID on the connected device.

    Args
    ----
    device : SerialDevice 
        The serial device to send the command to.
    
    pwm_id : int
        The ID of the PWM to set the angle for, must be between 0 and 15.
    
    angle : float
        The angle to set, limited to a range of 0 to 180 degress.
    
    Raises
    ------
    TypeError
        If 'pwm_id' is not an integer or 'angle is not a float or integer.
    
    ValueError
        If 'pwm_id' is outside the range [0, 15] or 'angle' is not within [0.0, 180.0].
    """
    if not isinstance(pwm_id, int) or not isinstance(angle, (int, float)):
        raise TypeError(
            "pwm_id must be an integer and angle must be a float or integer"
        )

    if pwm_id < 0 or pwm_id > 15:
        raise ValueError(
            "PWM id must be between 0 and 15"
        )
    
    if angle < 0.0 or angle > 180.0:
        raise ValueError(
            "Angle must be between 0 and 180 degress"
        )

    command = f"SET_ANGLE {pwm_id} {angle}\n"
    device.write_data(command)


def set_manipulator(device: SerialDevice, angles: List[float]) -> None:
    """Set angles for the manipulator.

    Configures the manipulator joints to specific angles, sending a command
    to adjust all joint angles simultaneously.

    Args
    ----
    device : SerialDevice
        The serial device to send the command to.
    
    angles : List[float]
        A list of four angles for the manipulator joints. 
        Each value must be between 0 and 180 degress.
    
    Raises
    ------
    TypeError
        If 'angles' is not a list of floats or intagers.
    
    ValueError
        If 'angles' does not contain exactly 4 elements, or if any angle
        is outside the range [0.0, 180.0].
    """
    if not isinstance(angles, list):
        raise TypeError(
            "Angles must be a list of floats"
        )

    if len(angles) != 4:
        raise ValueError(
            "The angles list must contain exactly 4 elements."
        )

    for i, angle in enumerate(angles):
        if not isinstance(angle, (int, float)):
            raise TypeError(
                f"Angle at index {i} must bea a float or integer"
            )
        if angle < 0.0 or angle > 180.0:
            raise ValueError(
                f"Angle at index {i} must be between 0 and 180 degress"
            )
    
    command = f"SET_MANIPULATOR {angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
    device.write_data(command)


def set_gripper(device: SerialDevice, state: bool) -> None:
    """Set the state of the gripper.

    Controls the gripper, setting it to either grip or release.

    Args
    ----
    device : SerialDevice
        The serial device to send the command to.
    
    state : bool
        The state of the gripper: True for grip (1), False for releasing (0).
    
    Raises
    ------
    TypeError
        If 'state' is not a boolean value.
    """
    if not isinstance(state, bool):
        raise TypeError(
            "State must be a boolean value"
        )

    command = f"SET_GRIPPER {int(state)}\n"
    device.write_data(command)
