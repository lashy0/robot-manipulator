import numpy as np
from ikpy.chain import Chain
from ikpy.utils.plot import init_3d_figure
from typing import Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D


class RobotArm3DPlot:
    """
    A class to visualize and animate a robotic arm manipulator using ikpy and matplotlib.

    Parameters:
        chain (Chain): The ikpy chain representing the robotic arm.
        xlim (Optional[Tuple[float, float]]): Limits for the X-axis in the plot.
        ylim (Optional[Tuple[float, float]]): Limits for the Y-axis in the plot.
        zlim (Optional[Tuple[float, float]]): Limits for the Z-axis in the plot.
        initial_angles (Optional[np,ndarray]): Initial joint angles for the robotic arm.
    """
    def __init__(
        self,
        chain: Chain,
        xlim: Optional[Tuple[float, float]] = (-0.5, 0.5),
        ylim: Optional[Tuple[float, float]] = (-0.5, 0.5),
        zlim: Optional[Tuple[float, float]] = (0, 0.5),
        initial_angles: Optional[np.ndarray] = None
    ) -> None:
        self.chain = chain
        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim

        if initial_angles is None:
            initial_angles = np.zeros(len(self.chain))
        self.initial_angles = initial_angles

        self.fig, self.ax = self._init_plot()

        # Initialize graphical elements (lines, scatter points, axes)
        self._init_plot_elements()

    def get_plot_objects(self) -> Tuple[Figure, Axes3D]:
        """
        Returns the current figure and axis objects for future use.

        Returns:
            Tuple[Figure, Axes3D]: matplotlib figure and 3D axes.
        """
        return self.fig, self.ax
    
    def get_chain(self) -> Chain:
        return self.chain

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
        frames = self.chain.forward_kinematics(self.initial_angles, full_kinematics=True)
        # Extract positions from transformation matrices
        positions = np.array([frame[:3, 3] for frame in frames])

        # XYZ coordinates
        xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]

        # Create a line object to represent the robot arm links
        (self.line,) = self.ax.plot(xs, ys, zs, linewidth=5, label="Robot Arm")

        # Create a scatter object to represent the robot arm joints
        self.scatter = self.ax.scatter(xs, ys, zs, s=55, c=self.line.get_color())

        # Initialize lines for the coordinate axes at the end effector
        directions_colors = ["green", "cyan", "orange"]  # Цвета для осей X, Y, Z
        self.tip_axes = [
            self.ax.plot([], [], [], linestyle='dashed', c=color)[0]
            for color in directions_colors
        ]

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
        positions = np.array([frame[:3, 3] for frame in frames])

        # XYZ coordinates
        xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]

        # Update the line data (robot arm links)
        self.line.set_data_3d(xs, ys, zs)

        # Update the scatter data (robot arm joints)
        # self.scatter.set_offsets(np.c_[xs, ys])
        # self.scatter.set_3d_properties(zs, 'z')
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
        axes_ends = [position + rotation[:, i] * axis_length for i in range(3)]

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
        # self.draw()

    def show(self) -> None:
        """Displays the plot window."""
        plt.show()
    
    def draw(self) -> None:
        """Refreshes and displays the current state of the plot."""
        plt.draw()
    
    def set_view(self, elev: float, azim: float) -> None:
        """
        Sets the view of the graph with the specified viewing angle.

        Args:
            elev (float): Height above the graph
            azim (float): The angle of rotation of the camera
        """
        self.ax.view_init(elev=elev, azim=azim)
    
    # TODO: вариант анимации уже по готовым углам, но тут надо проверять
    def animate(self, angles_seq: np.ndarray, interval: float = 0.05) -> None:
        """
        Animates the manipulator, smoothly switching between the specified positions of the joints.

        Args:
            angles_seq (np.ndarray): A sequence of arrays of joint angles
            interval (float): The interval between animation steps in seconds
        """
        for angles in angles_seq:
            self.plot(angles)
            plt.pause(interval)
