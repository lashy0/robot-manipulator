import numpy as np
from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Optional

from .plot import RobotArm3DPlot


def robot_arm_animate_show(
    robot_arm: RobotArm3DPlot,
    angle_seq: List[np.ndarray],
    interval: Optional[int] = 100,
    repeat: Optional[bool] = True,
    show_trajectory: Optional[bool] = False,
) -> None:
    """
    Animates the robotic arm motion using a RobotArm3DPlot object.

    Args:
        arm_plot (RobotArm3DPlot): The RobotArm3DPlot instance for visualization.
        angles_seq (List[np.ndarray]): A sequence of joint angle arrays for each frame.
        interval (Optional[int]): Delay between frames in milliseconds.
        repeat (Optional[bool]): Whether the animation should repeat after completing.
        show_trajectory (Optional[bool]): Whether to display the trajectory to the target point.
    """
    if show_trajectory:
        # Plot the trajectory line up to the current frame
        trajectory_points = [robot_arm.chain.forward_kinematics(angles)[:3, 3] for angles in angle_seq]
        trajectory_points = np.array(trajectory_points)
        trajectory_line, = robot_arm.ax.plot(
            trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], linestyle='--', color='red', alpha=0.5
        )

        # Mark the target point
        target_position = trajectory_points[-1]
        robot_arm.ax.scatter(target_position[0], target_position[1], target_position[2], color='red', s=100, label='Target')
        plt.draw()
    
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
        repeat=repeat
    )

    robot_arm.show()
