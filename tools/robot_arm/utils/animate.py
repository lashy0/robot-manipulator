import numpy as np
from typing import List, Optional
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .plot import RobotArmPlot


def robot_arm_animate_show(
    plotter: RobotArmPlot,
    joint_angles_seq: List[np.ndarray],
    interval: Optional[int] = 100,
    repeat: Optional[bool] = True,
    show_trajectory: Optional[bool] = False,
) -> None:
    """Animate the robotic arm moving through a sequence of joint angles.

    This function animates the robot arms movement through a given sequence of joint angles using Matplotlib.
    The animation can optionally show the trajectory of the end-effector and repeat the movement continuously.

    Args
    ----
    plotter : RobotArmPlot
        The RobotArmPlot instance for plotting the robot arm.
    
    joint_angles_seq : List[np.ndarray]
        A list of numpy arrays, each representing a set of joint angles for the robot arm.
        Each array must have a length that matches the number of links in the robot arm.
    
    interval : int, optional
        The interval between frames in the animation, in milliseconds.
        Default is 100.
    
    repeat : bool, optional
        Whether the animation should repeat after reaching the end of the sequence.
        Default is True.
    
    show_trajector : bool, optional
        Whether to show the trajectory of the end-effector during the animation. If True, a dashed red line will
        indicate the path of the end-effector, and the final target position will be marked.
        Default is False.
    
    Raises
    ------
    ValueError
        If any array in `joint_angles_seq` does not match the number of links in the robot arm.
    """
    if not all(len(angles) == len(plotter.robot.links) for angles in joint_angles_seq):
        raise ValueError("Each array of angles in joint_angles_seq must correspond to the number of robot_arm links.")
    
    if show_trajectory:
        # Plot the trajectory line up to the current frame
        trajectory_points = [plotter.robot.forward_kinematics(angles)[:3, 3] for angles in joint_angles_seq]
        trajectory_points = np.array(trajectory_points)
        trajectory_line, = plotter._ax.plot(
            trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], linestyle='--', color='red', alpha=0.5
        )

        # Mark the target point
        target_position = trajectory_points[-1]
        plotter._ax.scatter(target_position[0], target_position[1], target_position[2], color='red', s=100, label='Target')
        plt.draw()
    
    def update(frame: int) -> None:
        plotter.draw(joint_angles_seq[frame])
    
    ani = FuncAnimation(
        plotter._fig,
        update,
        frames=len(joint_angles_seq),
        interval=interval,
        repeat=repeat
    )

    plt.show()
