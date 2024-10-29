import numpy as np

from ..robot_arm.model import ROBOT_CHAIN, START_ANGLE_POSITION
from ..robot_arm.utils import RobotArmPlot, robot_arm_animate_show


def calculate_steps(
        initial_angles: np.ndarray,
        target_angles: np.ndarray,
        max_angle_per_step: float
    ) -> int:
    """Calculate the number of steps required to move from initial angles to target angles.

    Args
    ----
    initial_angles : np.ndarray
        The initail joint angles of the robot arm.
    
    target_angles : np.ndarray
        The target joint angles of the robot arm.
    
    max_angle_per_step : float
        The maximum angle that each joint can move per step radians.
    
    Returns
    -------
    steps: int
        The number of steps needed to reach the target angles from the initial angles.
    """
    angle_differences = np.abs(np.array(target_angles) - np.array(initial_angles))
    steps = int(np.ceil(np.max(angle_differences) / max_angle_per_step))
    return steps


def interpolate_angles(
        initial_angles: np.ndarray,
        target_angles: np.ndarray,
        steps: int
    ) -> np.ndarray:
    """Interpolate the angles between initial and target positions.

    Args
    ----
    initial_angles : np.ndarray
        The initail joint angles of the robot arm.
    
    target_angles : np.ndarray
        The target joint angles of the robot arm.
    
    steps : int
        The number of steps over witch to interpolate the angles.
    
    Returns
    -------
    interpolated_angles : np.ndarray
        A sequence of interpolated angles from initial to target positions.
    """
    return np.linspace(initial_angles, target_angles, steps)


if __name__ == '__main__':
    plotter = RobotArmPlot(ROBOT_CHAIN, START_ANGLE_POSITION)

    target_position = [0.15, -0.15, 0.15]
    target_angles_radians = ROBOT_CHAIN.inverse_kinematics(target_position)

    max_angle_per_step_deg = 1
    max_angle_per_step = np.deg2rad(max_angle_per_step_deg)
    print(f'Max angle per step: {max_angle_per_step_deg} degrees ({max_angle_per_step:.5f} radians)')
    
    steps = calculate_steps(START_ANGLE_POSITION, target_angles_radians, max_angle_per_step)
    print(f"Number of steps for animation: {steps}")
    
    interpolated_angles = interpolate_angles(START_ANGLE_POSITION, target_angles_radians, steps)

    print('Target joint angles:')
    print(f'  In radians: {np.array2string(target_angles_radians, precision=2, separator=", ")}')
    target_angles_degrees = np.rad2deg(target_angles_radians)
    print(f'  In degrees: {np.array2string(target_angles_degrees, precision=2, separator=", ")}')


    robot_arm_animate_show(plotter, interpolated_angles, 30, show_trajectory=True)
