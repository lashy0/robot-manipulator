import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QSizePolicy, QPushButton
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

# Matplotlib imports for embedding in PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from ...robot_arm import ROBOT_CHAIN
from ...robot_arm.utils import RobotArm3DPlot


class RobotArmWidget(QWidget):
    DEGREE_MULTIPLIER = 10  # множитель для преобразования углов слайдера
    STEP_SIZE = 2           # шаг слайдера
    ANIMATION_INTERVAL = 100 # интервал анимации в миллисекундах
    ANIMATION_STEPS = 100   # количество шагов анимации

    # Определяем сигнал, который будет испускаться при изменении угла конкретного сервопривода
    angle_changed = pyqtSignal(int, float)  # (номер сервопривода, новый угол в градусах)

    def __init__(self, robot_arm_plot: RobotArm3DPlot, parent=None):
        super().__init__(parent)

        self.robot_plot = robot_arm_plot
        self.fig, self.ax = self.robot_plot.get_plot_objects()
        self.chain = self.robot_plot.get_chain()
        self.canvas = FigureCanvas(self.fig)

        # Set up the main layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.canvas)
        self.setLayout(main_layout)

        # Create joint sliders
        self._create_joint_sliders()

        # Create reset button
        self._create_reset_button()

        # Initial joint angles (full array including fixed joints)
        self.current_angles = np.zeros(len(self.chain.links))
        self.previous_angles = np.zeros(len(self.chain.links))  # Хранение предыдущих значений углов

        # Plot the initial state
        self.robot_plot.plot(self.current_angles)
        self.canvas.draw()

    def _create_joint_sliders(self):
        """
        Creates sliders for each movable joint to control the angles interactively.
        """
        self.sliders = []
        self.angle_labels = []
        self.slider_layout = QVBoxLayout()

        for i in range(len(self.chain.links)):
            if self.chain.active_links_mask[i]:
                link = self.chain.links[i]
                min_angle, max_angle = link.bounds

                min_angle_deg = int(np.rad2deg(min_angle) * self.DEGREE_MULTIPLIER)
                max_angle_deg = int(np.rad2deg(max_angle) * self.DEGREE_MULTIPLIER)

                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(min_angle_deg)
                slider.setMaximum(max_angle_deg)
                slider.setValue(0)
                slider.setSingleStep(self.STEP_SIZE)
                # Должен отправлять значение при отпускание слайдера
                # slider.sliderReleased.connect(lambda index=i: self._update_angle_from_slider(index))
                slider.valueChanged.connect(lambda value, index=i: self._update_angle_from_slider(index, value))

                slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

                # Label for the joint
                joint_name = f'Joint {len(self.sliders)}'
                label = QLabel(joint_name)
                label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
                # Label to display the current angle
                angle_label = QLabel('0.00°')
                angle_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
                angle_label.setFixedWidth(50)
                self.angle_labels.append(angle_label)

                h_layout = QHBoxLayout()
                h_layout.addWidget(label)
                h_layout.addWidget(slider)
                h_layout.addWidget(angle_label)

                h_layout.setStretch(0, 0) # Метка соединения
                h_layout.setStretch(1, 1) # Слайдер
                h_layout.setStretch(2, 0) # Метка угла

                self.slider_layout.addLayout(h_layout)
                self.sliders.append(slider)

        self.layout().addLayout(self.slider_layout)
    
    def toggle_sliders(self, enable: bool):
        for slider in self.sliders:
            slider.setEnabled(enable)
    
    def update_joint_angle(self, joint_index: int, angle: float):
        if joint_index < 0 or joint_index >= len(self.chain.links):
            print(f"Invalid joint index: {joint_index}")
            return

        # Устанавливаем значение угла в слайдере
        self.sliders[joint_index].setValue(int(angle * self.DEGREE_MULTIPLIER))
        self.angle_labels[joint_index].setText(f'{angle:.1f}°')

        # Переводим угол в радианы
        angle_in_radians = np.deg2rad(angle)

        # Обновляем текущие углы соединений
        self.current_angles[joint_index] = angle_in_radians

        # Перерисовываем модель
        self.robot_plot.plot(self.current_angles)
        self.canvas.draw()


    def _create_reset_button(self):
        """
        Creates the reset button that sets the robot arm to the zero position.
        """
        self.reset_button = QPushButton('Reset')
        self.reset_button.clicked.connect(self.animate_reset_to_zero)
        self.layout().addWidget(self.reset_button)

    def _update_angle_from_slider(self, joint_index, slider_value):
        """
        Обновляет угол конкретного соединения при изменении значения на слайдере.
        """
        # Конвертируем значение слайдера в градусы
        angle_in_degrees = slider_value / self.DEGREE_MULTIPLIER
        angle_in_radians = np.deg2rad(angle_in_degrees)
        
        # Обновляем метку угла
        self.angle_labels[joint_index].setText(f'{angle_in_degrees:.1f}°')

        # Проверяем, изменился ли угол на данном слайдере по сравнению с предыдущим значением
        if np.isclose(self.previous_angles[joint_index], angle_in_degrees, atol=0.1):
            return  # Если угол изменился незначительно, не отправляем команду

        # Устанавливаем новый угол в массиве текущих значений
        self.current_angles[joint_index] = angle_in_radians
        self.previous_angles[joint_index] = angle_in_degrees

        # Обновляем 3D модель
        self.set_joint_angles(self.current_angles)

        # сигнал для изменения конкретного угла (номер соединения, угол в градусах)
        self.angle_changed.emit(joint_index, angle_in_degrees)

    def set_joint_angles(self, angles):
        """
        Updates the robot arm with new joint angles.

        Args:
            angles (np.ndarray): An array of joint angles for the movable joints.
        """
        # Create a full angles array including zeros for fixed joints
        full_angles = []
        movable_joint_index = 0

        for i in range(len(self.chain.links)):
            if self.chain.active_links_mask[i]:
                full_angles.append(angles[movable_joint_index])
                movable_joint_index += 1
            else:
                full_angles.append(0.0)  # Fixed link, angle is zero

        full_angles = np.array(full_angles)

        self.current_angles = full_angles
        self.robot_plot.plot(self.current_angles)
        self.canvas.draw()

    def animate_reset_to_zero(self):
        """
        Animate the robot arm moving back to the zero position using dynamic step calculation.
        """
        current_angles = np.array([slider.value() / self.DEGREE_MULTIPLIER for slider in self.sliders])
        target_angles = np.zeros_like(current_angles)  # Target angles (zero for all joints)

        angle_step = 1.0  # Задаем шаг изменения угла

        # Определяем максимальное изменение угла среди всех суставов
        max_angle_change = np.max(np.abs(current_angles - target_angles))

        # Динамически рассчитываем количество шагов для анимации
        if max_angle_change > 0:
            self.ANIMATION_STEPS = int(np.ceil(max_angle_change / angle_step))
        else:
            self.ANIMATION_STEPS = 1  # Если углы уже в нулевом состоянии, шагов не нужно

        # Создаем последовательность углов для каждой итерации анимации (линейная интерполяция)
        self.angles_sequence = np.linspace(current_angles, target_angles, self.ANIMATION_STEPS)

        # Инициализируем анимацию
        self.animation_step = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self._perform_animation_step)
        self.timer.start(self.ANIMATION_INTERVAL)

    def _perform_animation_step(self):
        """
        Perform a single step of the animation towards the zero position.
        """
        if self.animation_step < len(self.angles_sequence):
            new_angles = self.angles_sequence[self.animation_step]
            new_angles_in_radians = np.deg2rad(new_angles)

            # Update the sliders and angle labels
            for i, angle in enumerate(new_angles):
                self.sliders[i].setValue(int(angle * self.DEGREE_MULTIPLIER))
                self.angle_labels[i].setText(f'{angle:.1f}°')

            # Update the robot arm
            self.set_joint_angles(new_angles_in_radians)

            # Increment the animation step
            self.animation_step += 1
        else:
            # Stop the timer when the animation is done
            self.timer.stop()


if __name__ == '__main__':
    plotter = RobotArm3DPlot(ROBOT_CHAIN)

    app = QApplication(sys.argv)

    window = QWidget()
    layout = QVBoxLayout()
    robot_widget = RobotArmWidget(plotter)

    layout.addWidget(robot_widget)
    window.setLayout(layout)
    window.setWindowTitle('Robot Arm Visualization')

    window.show()

    sys.exit(app.exec_())
