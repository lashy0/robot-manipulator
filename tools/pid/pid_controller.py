import numpy as np
from typing import Optional


class PIDController:
    """Class to represent a PID (Proportional, Integral, Derivative) controller.

    Atributes
    ---------
    kp : float
        The proportional gain of the PID controller.
    
    ki : float
        The integral gain of the PID controller.
    
    kd : float
        The derivation gain of the PID controller.
    
    integral_limit : float, optional
        The maximum value for the integral term to prevent wind-up.
    
    _prev_err : float
        The error from the previos time step.
    
    _integral : float
        The accumulated integral term to account for past errors.
    """
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: Optional[float] = None
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self._prev_err = None
        self._integral = 0.0

    def update(self, error: float, dt: float) -> float:
        """Updates the PID controller output given the current error and time step.

        Args
        ----
        error : float
            The current error value.
        
        dt : float
            The time step between the current and previous error calculations.
        
        Returns
        -------
        out : float
            The output value from the PID controller to adjust the system.
        """
        # Integral term with anti-windup
        self._integral += error * dt
        if self.integral_limit is not None:
            self._integral = np.clip(self._integral, -self.integral_limit, self.integral_limit)
        # Derivative term
        derivative = 0.0 if self._prev_err is None else (error - self._prev_err) / dt
        self._prev_err = error
        # PID controller output
        return self.kp * error + self.ki * self._integral + self.kd * derivative
    
    def reset(self) -> None:
        """Reset the PID controllers integral and previous error terms."""
        self._prev_err = None
        self._integral = 0.0
