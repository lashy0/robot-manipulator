import numpy as np
from typing import Union, List
from scipy.interpolate import CubicSpline


def generate_figure8_trajectory(
    num_points: int,
    width: float,
    center_offset: Union[np.ndarray, List[float]]
) -> np.ndarray:
    """Generate a figure-eight trajectory in 3D space.

    Args
    ----
    num_points : int
        Number of points in the trajectory.
    
    width : float
        Width of the figure-eight (in meters).
    
    center_offset : np.ndarray, list
        Offset of the center of the figure-eight in the format [x, y, z].
    
    Returns
    -------
    trajectory : np.ndarray
        Trajectory as an array of points with coordinates (num_points, 3).
    """
    if isinstance(center_offset, list):
        center_offset = np.array(center_offset)

    if len(center_offset) != 3:
        raise ValueError("'center_offset' must have a length of 3 (x, y, z)")
    
    t = np.linspace(0, 2 * np.pi, num_points)

    x = np.full(num_points, center_offset[0])
    y = width * np.sin(2 * t) + center_offset[1]
    z = width * np.sin(t) + center_offset[2]

    trajectory = np.vstack((x, y, z)).T

    return trajectory


def generate_interpolated_figure8_trajectory(
    num_points: int,
    interp_points: int,
    width: float,
    center_offset: Union[np.ndarray, List[float]]
) -> np.ndarray:
    """Generates a figure-eight trajectory in 3D space with interpolation.
    
    Args
    ----
    num_points : int
        Number of points in the trajectory.
    
    interp_points : int
        Number of points in the interpolated trajectory.
    
    width : float
        Width of the figure-eight (in meters).
    
    center_offset : np.ndarray, list
        Offset of the center of the figure-eight in the format [x, y, z].
    
    Returns
    -------
    trajectory : np.ndarray
        Interpolated trajectory as an array of points with coordinates (interp_points, 3).
    """
    original_trajectory = generate_figure8_trajectory(num_points, width, center_offset)

    t_original = np.linspace(0, 2 * np.pi, num_points)
    t_interp = np.linspace(0, 2 * np.pi, interp_points)

    cs_x = CubicSpline(t_original, original_trajectory[:, 0])
    cs_y = CubicSpline(t_original, original_trajectory[:, 1])
    cs_z = CubicSpline(t_original, original_trajectory[:, 2])

    x_interp = cs_x(t_interp)
    y_interp = cs_y(t_interp)
    z_interp = cs_z(t_interp)

    trajectory = np.vstack((x_interp, y_interp, z_interp)).T

    return trajectory
