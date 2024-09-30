import numpy as np
from ikpy.chain import Chain
from ikpy.utils.plot import init_3d_figure
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D


class RobotArmPlot:
    """
    A class to visualize and animate a robotic arm manipulator using ikpy and matplotlib.

    Parameters:
        chain (Chain): The ikpy chain representing the robotic arm.
        xlim (Tuple[float, float]): Limits for the X-axis in the plot.
        ylim (Tuple[float, float]): Limits for the Y-axis in the plot.
        zlim (Tuple[float, float]): Limits for the Z-axis in the plot.
        fig (Figure, optional): The matplotlib figure object. If None, a new figure is created.
        ax (Axes3D, optional): The 3D axes object for plotting. If None, new axes are created.
    """
    def __init__(
        self,
        chain: Chain,
        xlim: Tuple[float, float] = (-0.5, 0.5),
        ylim: Tuple[float, float] = (-0.5, 0.5),
        zlim: Tuple[float, float] = (0, 0.5),
        fig: Optional[Figure] = None,
        ax: Optional[Axes3D] = None,
    ) -> None:
        self.chain = chain
        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim

        if fig is None or ax is None:
            self.fig, self.ax = self._init_plot()
        else:
            self.fig, self.ax = fig, ax

        # Initialize graphical elements (lines, scatter points, axes)
        self._init_plot_elements()

    def _init_plot(self) -> Tuple[Figure, Axes3D]:
        """
        Initializes the 3D plot with labels and axis limits.

        Returns:
            Tuple[Figure, Axes3D]: The matplotlib figure and 3D axes objects.
        """
        fig, ax = init_3d_figure()

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim(self.xlim)
        ax.set_ylim(self.ylim)
        ax.set_zlim(self.zlim)

        return fig, ax

    def _init_plot_elements(self) -> None:
        """
        Initializes the graphical elements for the robot arm visualization,
        including the arm links, joints, and the coordinate axes at the end effector.
        """
        # Get initial joint positions with zero angles
        initial_angles = np.zeros(len(self.chain))
        frames = self.chain.forward_kinematics(initial_angles, full_kinematics=True)
        # Extract positions from transformation matrices
        positions = [frame[:3, 3] for frame in frames]

        # XYZ coordinates
        xs = [pos[0] for pos in positions]
        ys = [pos[1] for pos in positions]
        zs = [pos[2] for pos in positions]

        # Create a line object to represent the robot arm links
        (self.line,) = self.ax.plot(xs, ys, zs, linewidth=5, label="Robot Arm")

        # Create a scatter object to represent the robot arm joints
        self.scatter = self.ax.scatter(xs, ys, zs, s=55, c=self.line.get_color())

        # Initialize lines for the coordinate axes at the end effector
        self.tip_axes = []
        directions_colors = ["green", "cyan", "orange"]  # Цвета для осей X, Y, Z
        for color in directions_colors:
            # Create empty lines for axes, to be updated later
            line, = self.ax.plot([], [], [], linestyle='dashed', c=color)
            self.tip_axes.append(line)

    def plot(self, angles: np.ndarray) -> None:
        """
        Updates the plot to display the robotic arm at the specified joint angles.

        Args:
            angles (np.ndarray): An array of joint angles for the robotic arm.
        """
        if len(angles) != len(self.chain):
            raise ValueError(
                f"Expected {len(self.chain)} angles, got {len(angles)}"
            )

         # Compute new joint positions
        frames = self.chain.forward_kinematics(angles, full_kinematics=True)
        # Extract positions
        positions = [frame[:3, 3] for frame in frames]

        # XYZ coordinates
        xs = [pos[0] for pos in positions]
        ys = [pos[1] for pos in positions]
        zs = [pos[2] for pos in positions]

        # Update the line data (robot arm links)
        self.line.set_data(xs, ys)
        self.line.set_3d_properties(zs)

        # Update the scatter data (robot arm joints)
        self.scatter._offsets3d = (xs, ys, zs)

        # Update the coordinate axes at the end effector
        # Get the end effector frame (transformation matrix)
        end_effector_frame = frames[-1]

        # Position of the end effector
        position = end_effector_frame[:3, 3]

        # Orientation (rotation matrix) of the end effector
        rotation = end_effector_frame[:3, :3]

        # Length of the axes
        axis_length = 0.1

        # Calculate the end points for the X, Y, Z axes
        x_axis_end = position + rotation[:, 0] * axis_length
        y_axis_end = position + rotation[:, 1] * axis_length
        z_axis_end = position + rotation[:, 2] * axis_length

        axes_ends = [x_axis_end, y_axis_end, z_axis_end]

        # Update the data for each axis line
        for idx, axis_line in enumerate(self.tip_axes):
            axis_end = axes_ends[idx]
            x_data = [position[0], axis_end[0]]
            y_data = [position[1], axis_end[1]]
            z_data = [position[2], axis_end[2]]

            axis_line.set_data(x_data, y_data)
            axis_line.set_3d_properties(z_data)

        # Обновляем границы и метки осей
        # self.ax.set_xlim(self.xlim)
        # self.ax.set_ylim(self.ylim)
        # self.ax.set_zlim(self.zlim)

        # Перерисовываем фигуру
        # plt.draw()

    def animate(
        self,
        angles_seq: List[np.ndarray],
        interval: int = 100,
        repeat: bool = True
    ) -> None:
        """
        Animates the robotic arm motion based on a sequence of joint angles.

        Args:
            angles_seq (List[np.ndarray]): A sequence of joint angle arrays for each frame.
            interval (int, optional): Delay between frames in milliseconds.
            repeat (bool, optional): Whether the animation should repeat after completing.
        """
        def update(frame: int) -> None:
            self.plot(angles_seq[frame])

        # Create the animation object
        ani = FuncAnimation(
            self.fig,
            update,
            frames=len(angles_seq),
            interval=interval,
            repeat=repeat
        )

        self.show()

    def show(self) -> None:
        """Displays the plot window."""
        plt.show()