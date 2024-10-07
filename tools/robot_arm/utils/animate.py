import numpy as np
from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .plot import RobotArm3DPlot


def robot_arm_animate(
    robot_arm: RobotArm3DPlot,
    angle_seq: List[np.ndarray],
    interval: int = 100,
    repeat: bool = True
) -> None:
    """
    Animates the robotic arm motion using a RobotArm3DPlot object.

    Args:
        arm_plot (RobotArm3DPlot): The RobotArm3DPlot instance for visualization.
        angles_seq (List[np.ndarray]): A sequence of joint angle arrays for each frame.
        interval (int, optional): Delay between frames in milliseconds.
        repeat (bool, optional): Whether the animation should repeat after completing.
    """
    def update(frame: int) -> None:
        robot_arm.plot(angle_seq[frame])
        plt.draw()
    
    ani = FuncAnimation(
        robot_arm.fig,
        update,
        frames=len(angle_seq),
        interval=interval,
        repeat=repeat
    )

    robot_arm.show()
