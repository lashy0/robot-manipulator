import numpy as np

from ..robot_arm import ROBOT_CHAIN, START_ANGLE_POSITION
from ..robot_arm.utils import RobotArmPlot, robot_arm_animate_show
from ..pid import PIDController

from .utils import generate_interpolated_figure8_trajectory


# Инициализация PID для каждого звена для движения манипулятора
pid_controllers = [
    PIDController(kp=0.25, ki=0.001, kd=0.01),
    PIDController(kp=0.25, ki=0.001, kd=0.01),
    PIDController(kp=0.25, ki=0.001, kd=0.01),
    PIDController(kp=0.25, ki=0.001, kd=0.01)
]

# Инициализация начальных значений
current_angles = START_ANGLE_POSITION.copy()
previous_angles = current_angles.copy()

# Определение параметров для траектории восемь
width = 0.1 # Ширина восьмерки
shift = [0.15, 0, 0.2] # Смещение XYZ
points_number = 20 # Количество точек траектории
points_interp = 100

# Расчет траектории восьмерки
figure_eight_trajectory = generate_interpolated_figure8_trajectory(
    points_number, points_interp, width, shift
)

# Инициализация визуализации
robot_plot = RobotArmPlot(ROBOT_CHAIN, START_ANGLE_POSITION)
fig = robot_plot._fig
ax = robot_plot._ax

# Отображение траектории восьмерки
ax.plot(figure_eight_trajectory[:, 0], figure_eight_trajectory[:, 1], figure_eight_trajectory[:, 2], linestyle='dotted', color='r')

# Параметры моделирования
dt = 0.02  # шаг времени в секундах
simulation_time = 10  # общее время моделирования в секундах
# steps = int(simulation_time / dt)  # количество шагов в моделировании
steps = len(figure_eight_trajectory)
max_speed = np.deg2rad(1)

angles_history = []

# Возврат в начальное положение перед началом движения
initial_position = figure_eight_trajectory[0]

target_angles = ROBOT_CHAIN.inverse_kinematics(initial_position, initial_position=current_angles)

while not np.allclose(current_angles, target_angles, atol=1e-1):
    for i in range(len(pid_controllers)):
        error = target_angles[i] - current_angles[i]
        control_signal = pid_controllers[i].update(error, dt)

        control_signal = np.clip(control_signal, -max_speed, max_speed)

        current_angles[i] += control_signal

        min_angle, max_angle = ROBOT_CHAIN.links[i].bounds
        current_angles[i] = np.clip(current_angles[i], min_angle, max_angle)

    angles_history.append(current_angles.copy())

# Подсчет движения при работе ПИД регулятора на каждом сервоприводе
for step in range(steps):
    # target_position = figure_eight_trajectory[step % len(figure_eight_trajectory)]
    target_position = figure_eight_trajectory[step]

    target_angles = ROBOT_CHAIN.inverse_kinematics(target_position, initial_position=current_angles)

    # Переход к целевым углам с использованием PID до достижения заданной точности
    while not np.allclose(current_angles, target_angles, atol=1e-1):
        for i in range(len(pid_controllers)):
            error = target_angles[i] - current_angles[i]
            control_signal = pid_controllers[i].update(error, dt)

            control_signal = np.clip(control_signal, -max_speed, max_speed)

            current_angles[i] += control_signal

            # Ограничение угла в пределах допустимых значений звена
            min_angle, max_angle = ROBOT_CHAIN.links[i].bounds
            current_angles[i] = np.clip(current_angles[i], min_angle, max_angle)

        # Сохранение текущих углов в историю
        angles_history.append(current_angles.copy())
    
    for i in range(len(pid_controllers)):
        pid_controllers[i].reset()

# Возврат в начальное положение после окончания движения
target_angles = START_ANGLE_POSITION

while not np.allclose(current_angles, target_angles, atol=1e-1):
    for i in range(len(pid_controllers)):
        error = target_angles[i] - current_angles[i]
        control_signal = pid_controllers[i].update(error, dt)

        current_angles[i] += control_signal

        control_signal = np.clip(control_signal, -max_speed, max_speed)

        min_angle, max_angle = ROBOT_CHAIN.links[i].bounds
        current_angles[i] = np.clip(current_angles[i], min_angle, max_angle)

    angles_history.append(current_angles.copy())


angles_history = np.array(angles_history)

total_time = len(angles_history) * dt
print(f"Total running time of the movement: {total_time:.2f} seconds")

robot_arm_animate_show(robot_plot, angles_history, dt * 1000)
