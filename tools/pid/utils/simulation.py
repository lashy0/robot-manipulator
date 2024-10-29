import numpy as np
from typing import Optional

from ..pid_controller import PIDController
from .plot import _pid_plot


def servo_simulation(
    pid: PIDController,
    target_angle : float,
    initial_angle : float,
    time_simulation : float,
    dt : float,
    max_speed : Optional[float] = None
) -> None:
    """Run a simulation of a PID controller trying to reach a target angle for a servo motor.
    
    Args
    ----
    pid : PIDController
        The PID controller instance to be used for the simulation.
    
    target_angle : float
        The desired target angle that the PID controller is trying to achieve (degrees).
    
    initial_angle : float
        The initial angle of the servo motor (degrees).
    
    time_simulation : float
        The total time for the simulation (seconds).
    
    dt : float
        The time step for the simulation (seconds).
    
    max_speed : float, optional
        The maximum speed of the servo motor in degrees per second.
    """
    time_points = np.arange(0, time_simulation, dt)
    angle_values = np.zeros_like(time_points)
    control_signals = np.zeros_like(time_points)

    current_angle = initial_angle

    for i, _ in enumerate(time_points):
        if i == 0:
            angle_values[i] = initial_angle
            control_signals[i] = 0
        
        # Calculate error between target and current angle
        error = target_angle - current_angle

        # Get control signal from the PID controller 
        control_signal = pid.update(error, dt)

        # Limit the control signal to the maximum servo speed
        if max_speed is not None:
            control_signal = np.clip(control_signal, -max_speed, max_speed)
        
        control_signals[i] = control_signal
        
        # Simulate the servo response
        current_angle += control_signal
        angle_values[i] = current_angle
    
    final_error = target_angle - current_angle
    print("Simulation complete")
    print(f"Final signal: {current_angle:.2f} degrees")
    print(f"Target_angle: {target_angle:.2f} degrees")
    print(f"Final error: {final_error} degrees")
    print(f"Max control signal: {np.max(control_signals):.2f}")
    print(f"Min control signal: {np.min(control_signals):.2f}")

    # Plot result
    _pid_plot(time_points, angle_values, control_signals, target_angle)
