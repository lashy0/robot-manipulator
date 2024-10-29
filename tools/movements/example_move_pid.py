import numpy as np

from ..robot_arm import ROBOT_CHAIN, START_ANGLE_POSITION
from ..robot_arm.utils import RobotArmPlot, robot_arm_animate_show
from ..pid import PIDController


# Параметры PID для каждого звена
pid_controllers = [
    PIDController(kp=0.25, ki=0.01, kd=0.01),
    PIDController(kp=0.25, ki=0.01, kd=0.01),
    PIDController(kp=0.25, ki=0.01, kd=0.01),
    PIDController(kp=0.25, ki=0.01, kd=0.01)
]

# Инициализация начальных значений
current_angles = START_ANGLE_POSITION.copy()
# target_position = [0.15, 0.0, 0.2]
target_position = [0.15, -0.15, 0.15]
target_angles = ROBOT_CHAIN.inverse_kinematics(target_position)

# Инициализация визуализации
robot_plot = RobotArmPlot(ROBOT_CHAIN, START_ANGLE_POSITION)
fig = robot_plot._fig

# Параметры моделирования
dt = 0.02  # шаг времени в секундах
max_speed = np.deg2rad(4)

# Параметры для плавного старта
ramp_up_steps = 50
current_ramp_step = 1

angles_history = []

# От atol зависит как плавно будет сбрасывать скорость в конце и сколько итераций пройдет
while not np.allclose(current_angles, target_angles, atol=1e-2):
    # Увеличение коэффициента для плавного старта
    ramp_factor = min(1.0, current_ramp_step / ramp_up_steps)
    current_ramp_step += 1
    print(f"ramp_factor: {ramp_factor}")

    for i in range(len(pid_controllers)):
        error = target_angles[i] - current_angles[i]

        control_signal = pid_controllers[i].update(error, dt)

        # Плавное увеличение скорости
        control_signal *= ramp_factor

        # Ограничение скорости
        if max_speed is not None:
            control_signal = np.clip(control_signal, -max_speed, max_speed)

        current_angles[i] += control_signal

        # Ограничение по углам для звеньев ROBOT_CHAIN
        min_angle, max_angle = ROBOT_CHAIN.links[i].bounds
        current_angles[i] = np.clip(current_angles[i], min_angle, max_angle)

    angles_history.append(current_angles.copy())

print(f"The number of steps to animate: {len(angles_history)}")

total_time = len(angles_history) * dt
print(f"Total running time of the movement: {total_time:.2f} seconds")

angles_history = np.array(angles_history)

robot_arm_animate_show(robot_plot, angles_history, dt * 1000, show_trajectory=True)
