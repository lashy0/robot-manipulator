import numpy as np
from ikpy.chain import Chain
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from ...robot_arm import ROBOT_CHAIN, START_ANGLE_POSITION
from ...robot_arm.utils import RobotArm3DPlot


# TODO: подумать как лучше смотреть и обрабатывать текущий угол активного звена
class RobotArm3DPlotQt(QWidget):
    joint_angle_changed = pyqtSignal(int, float)
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.robot_arm = ROBOT_CHAIN
        self.current_angles = np.array(START_ANGLE_POSITION, dtype=np.float64)
        layout = QVBoxLayout(self)

        # Create RobotArm3DPlot
        self.plotter = RobotArm3DPlot(self.robot_arm, initial_angles=START_ANGLE_POSITION)
        self.fig, self.ax = self.plotter.get_plot_objects()
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        layout.addWidget(self.canvas)

        # Create Slider
        self.sliders = []
        self.slider_labels = []

        for i in range(len(self.robot_arm.links)):
            if self.robot_arm.active_links_mask[i]:
                link = self.robot_arm.links[i]
                min_angle, max_angle = link.bounds
                min_angle = int(np.rad2deg(min_angle))
                max_angle = int(np.rad2deg(max_angle))

                hbox_layout = QHBoxLayout()

                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(min_angle)
                slider.setMaximum(max_angle)
                slider.setValue(int(np.rad2deg(START_ANGLE_POSITION[i])))
                slider.valueChanged.connect(lambda value, joint_index=i: self.on_slider_value_changed(joint_index, value))
                hbox_layout.addWidget(slider)
                self.sliders.append(slider)

                label = QLabel(f"Joint {i}: {int(np.rad2deg(START_ANGLE_POSITION[i]))}°")
                label.setMinimumWidth(70)
                label.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
                hbox_layout.addWidget(label)
                self.slider_labels.append(label)

                hbox_layout.setStretch(0, 1)
                hbox_layout.setStretch(1, 0)

                layout.addLayout(hbox_layout)
        
        # Update plotter timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plotter)
        self.timer.start(30)

        # Connect signal func
        self.connection_status_changed.connect(self.on_connection_status_changed)

    def on_connection_status_changed(self, is_connect: bool):
        for slider in self.sliders:
            slider.setEnabled(is_connect)

    def on_slider_value_changed(self, joint_index: int, value: int):
        # TODO: подправить + logger
        if joint_index < 0 or joint_index >= len(self.robot_arm.links):
            print(f"Invalid joint index: {joint_index}")
            return
        
        self.current_angles[joint_index] = np.deg2rad(value)
        self.slider_labels[joint_index].setText(f"Joint {joint_index}: {value}°")
        self.update_plotter()
        self.joint_angle_changed.emit(joint_index, float(value))
        
    def update_plotter(self):
        self.plotter.plot(self.current_angles)
        self.canvas.draw()


if __name__ == '__main__':
    import sys
    from PyQt5.QtWidgets import (
        QApplication
    )

    app = QApplication(sys.argv)
    window = RobotArm3DPlotQt()
    window.show()

    window.connection_status_changed.emit(False)

    sys.exit(app.exec_())
