import numpy as np
from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .plot import RobotArm3DPlot


def robot_arm_animate_show(
    robot_arm: RobotArm3DPlot,
    angle_seq: List[np.ndarray],
    interval: int = 100,
    repeat: bool = True,
    **kwargs
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
    
    if not all(len(angles) == len(robot_arm.chain) for angles in angle_seq):
        raise ValueError("Each array of angles in angle_seq must correspond to the number of robot_arm links.")
    
    ani = FuncAnimation(
        robot_arm.fig,
        update,
        frames=len(angle_seq),
        interval=interval,
        repeat=repeat,
        **kwargs
    )

    robot_arm.show()
