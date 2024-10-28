import numpy as np
from ikpy.chain import Chain
from ikpy.utils.plot import init_3d_figure
from typing import Optional
import matplotlib.pyplot as plt


class RobotArmPlot:
    """Class to visualize to robot arm chain using IKPy and Matplotlib.

    Attributes
    ----------
    robot : ikpy.chain.Chain
        The IKPy Chain object representing the robot arm.
    
    init_angles : np.ndarray
        Initial joint angles pf the robot arm.
    
    _fig : matplotlib.figure.Figure
        Matplotlib figure used for ploting.
    
    _ax: mpl_toolkits.mplot3d.Axes3D
        3D Axes used for the robot visualization.
    
    _directions_colors : list
        Colors used to represent the directions XYZ of the end-effector.
    
    _lines : mpl_toolkits.mplot3d.art3d.Line3D
        Line3D object representing the links of the robot.
    
    _scatter : mpl_toolkits.mplot3d.art3d.Path3DCollection
        Scatter object representing the nodes (joints) of the robot.
    
    _tip_axes : list
        List of Line3D objects representing the x, y, z axes of the end-effector.
    """
    _directions_colors = ["green", "cyan", "orange"]

    def __init__(
        self,
        robot: Chain,
        init_angles: Optional[np.ndarray] = None
    ) -> None:
        """Initialize the RobotArmPlot.

        Args
        ----
        robot : ikpy.chain.Chain
            TThe IKPy Chain object representing the robot arm.
        
        init_angles : np.ndarray, optional
            Initial joint angles of the robot arm.
            Defaults to zero for all joints if not provided.
        """
        self.robot = robot

        if init_angles is None:
            init_angles = np.zeros(len(self.robot))
        self.init_angles = init_angles
    
        self._fig, self._ax = init_3d_figure()

        self._init_plot_chain()
        self._set_axis_limits()
    
    def _set_axis_limits(self) -> None:
        """Set axis limits for the 3D plot based on the maximum reach of the robot arm."""
        max_reach = sum(link.length for link in self.robot.links if hasattr(link, 'length'))

        buffer = 0.1 * max_reach
        self._ax.set_xlim([-max_reach - buffer, max_reach + buffer])
        self._ax.set_ylim([-max_reach - buffer, max_reach + buffer])
        self._ax.set_zlim([0, max_reach + buffer])

    def _init_plot_chain(self) -> None:
        """Initialize the plot of the robot chain, including links, nodes and end-effector axes."""
        frames = self.robot.forward_kinematics(self.init_angles, full_kinematics=True)

        position = np.array([frame[:3, 3] for frame in frames])

        # XYZ coordinates
        xs, ys, zs = position[:, 0], position[:, 1], position[:, 2]

        # Create plot the chain
        self.lines = self._ax.plot(xs, ys, zs, linewidth=5, label="robot")[0]
        # Create plot the nodes of the chain
        self.scatter = self._ax.scatter(xs, ys, zs, s=55, c=self.lines.get_color())

        # Create tip axes
        end_effector_frame = frames[-1]
        origin = end_effector_frame[:3, 3]
        directions = [
            origin + end_effector_frame[:3, i] * 0.1 for i in range(3)
        ]

        self.tip_axes = [
            self._ax.plot(
                [origin[0], direction[0]],
                [origin[1], direction[1]],
                [origin[2], direction[2]],
                linestyle='dashed', c=color
            )[0] for direction, color in zip(directions, self._directions_colors)
        ]
    
    def draw(self, joint_angles: np.ndarray) -> None:
        """Update the plot of the robot arm to reflect new joint angles.

        Args
        ----
        joint_angles : np.ndarray
            Array of new joint angles for the robot arm.
        """
        frames = self.robot.forward_kinematics(joint_angles, full_kinematics=True)
        position = np.array([frame[:3, 3] for frame in frames])

        # XYZ coordinates
        xs, ys, zs = position[:, 0], position[:, 1], position[:, 2]

        # Update chain lines
        self.lines.set_data(xs, ys)
        self.lines.set_3d_properties(zs)

        # Update scatter points
        self.scatter._offsets3d = (xs, ys, zs)

        # Update tip axes
        end_effector_frame = frames[-1]
        origin = end_effector_frame[:3, 3]
        directions = [
            origin + end_effector_frame[:3, i] * 0.1 for i in range(3)
        ]

        for tip_axis, direction in zip(self.tip_axes, directions):
            tip_axis.set_data([origin[0], direction[0]], [origin[1], direction[1]])
            tip_axis.set_3d_properties([origin[2], direction[2]])

        plt.draw()
