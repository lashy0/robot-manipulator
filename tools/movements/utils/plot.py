import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, List


def plot_trajectory(
    trajectory: np.ndarray,
    xlim: Optional[List[float]] = None,
    ylim: Optional[List[float]] = None,
    zlim: Optional[List[float]] = None
) -> None:
    """Plots a 3D trajectory.

    Args
    ----
    trajectory : np.ndarray
        Trajectory to plot with coordinates (n_points, 3).
    
    xlim : list, optiona
        Limits for the X axis as [xmin, xmax].
    
    ylim : list, optional
        Limits for the Y axis as [ymin, ymax].
    
    zlim : list, optional
        Limits for the Z axis as [zmin, zmax].
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], linestyle='dotted')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set axis limits if provided
    if xlim:
        ax.set_xlim3d(xlim)
    if ylim:
        ax.set_ylim3d(ylim)
    if zlim:
        ax.set_zlim3d(zlim)

    plt.show()
