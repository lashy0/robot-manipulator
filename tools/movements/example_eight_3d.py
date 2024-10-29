from .utils import generate_figure8_trajectory, generate_interpolated_figure8_trajectory
from .utils import plot_trajectory


num_points = 100
width = 0.1
center_offset = [0.2, 0, 0.2]
interp_points = 300

trajectory = generate_figure8_trajectory(num_points, width, center_offset)
plot_trajectory(
    trajectory, xlim=[-0.5, 0.5], ylim=[-0.5, 0.5], zlim=[0, 0.5]
)

interp_trajectory = generate_interpolated_figure8_trajectory(
    num_points, interp_points, width, center_offset
)
plot_trajectory(
    interp_trajectory, [-0.5, 0.5], [-0.5, 0.5], [0, 0.5]
)
