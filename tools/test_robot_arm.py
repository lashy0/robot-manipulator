import numpy as np

from .robot_arm.model import ROBOT_CHAIN, START_ANGLE_POSITION
from .robot_arm.utils import RobotArm3DPlot, robot_arm_animate_show


# Вычисление количества шагов для анимации
def calculate_steps(initial_angles, target_angles, max_angle_per_step):
    # Вычисляем разницу углов
    angle_differences = np.abs(np.array(target_angles) - np.array(initial_angles))
    # Определяем максимальное количество шагов по любому углу
    steps = int(np.ceil(np.max(angle_differences) / max_angle_per_step))
    return steps


def interpolate_angles(initial_angles, target_angles, steps):
        return np.linspace(initial_angles, target_angles, steps)


if __name__ == '__main__':
    plotter = RobotArm3DPlot(ROBOT_CHAIN, initial_angles=START_ANGLE_POSITION)

    target_position = [0.15, -0.15, 0.15]
    target_angles_radians = ROBOT_CHAIN.inverse_kinematics(target_position)

    max_angle_per_step_deg = 1
    max_angle_per_step = np.deg2rad(max_angle_per_step_deg)
    print(f'Угол {max_angle_per_step_deg} в радианах {max_angle_per_step}')
    
    steps = calculate_steps(START_ANGLE_POSITION, target_angles_radians, max_angle_per_step)
    print(f'Количество шагов для анимации: {steps}')
    
    interpolated_angles = interpolate_angles(START_ANGLE_POSITION, target_angles_radians, steps)

    print(target_angles_radians)

    robot_arm_animate_show(plotter, interpolated_angles, 30, show_trajectory=True)
