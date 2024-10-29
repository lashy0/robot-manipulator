import numpy as np
import matplotlib.pyplot as plt


def _pid_plot(
    time_points : np.ndarray,
    values : np.ndarray,
    control_signals : np.ndarray,
    target_value : float
):
    """Utility function to plot the results of the PID controller simulation.

    Args
    ----
    time_points : np.ndarray
        The array of time points for the simulation.
    
    values : np.ndarray
        The array of system calues over time.
    
    control_signals : np.ndarray
        The array of control signal values over time.
    
    target_value : float
        The target value that the PID controller is trying to achieve.
    """
    plt.figure(figsize=(12, 6))

    # Value over time
    plt.subplot(2, 1, 1)
    plt.plot(time_points, values, label='Values', color='b')
    plt.axhline(target_value, color='r', linestyle='--', label='Target value')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('PID Comtoller simulation')
    plt.legend()
    plt.grid(True)

    # Control signal over time
    plt.subplot(2, 1, 2)
    plt.plot(time_points, control_signals, label='Control Signal', color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Signal')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
