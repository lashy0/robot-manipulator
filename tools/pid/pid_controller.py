from typing import Optional


class PIDController:
    """
    A class to represent a PID (Proportional, Integral, Derivative) controller

    Attributes:
        Kp (float): The proportional gain of the PID controller
        Ki (float): The integral gain of the PID controller
        Kd (float): The derivation gain of the PID controller
        max_step (float): The maximum allowable change in the output
        setpoint (float): The target value that the PID controller aims to achieve
        _integral (float): The accumulated integral term to account for past errors
        _prev_err (float): The error from the previos time step (used for calculating the derivative term)
    """
    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        max_step: Optional[float]= None,
    ) -> None:
        """
        Args:
            Kp (float): Proportional coefficient PID
            Ki (float): Integral coefficient PID
            Kd (float): Derivative coefficient PID
            max_step (Optional[float]): Maximum change in output per update to ensure smooth movement
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_step = max_step
        self.setpoint = 0.0

        self._integral = 0.0
        self._prev_err = None
    
    def update(self, current_value: float, delay_time: float) -> float:
        """
        Update the PID controller based on the current value of the system
        This calculates the control signal needed to move the system closer to the setpoint

        Args:
            current_value (float): The current value of the controlled variable
            delay_time (float): Time delay between controller updates in seconds
        
        Returns:
            float: The calculated control signal, which will be applied to the system
        """
        
        error = self.setpoint - current_value
        
        self._integral += error * delay_time
        
        if self._prev_err is None:
            derication = 0.0
        else:
            derication = (error - self._prev_err) / delay_time
        
        output = self.Kp * error + self.Ki * self._integral + self.Kd * derication

        if self.max_step is not None:
            if output > self.max_step:
                output = self.max_step
            elif output < -self.max_step:
                output = -self.max_step
        
        self._prev_err = error

        return output

    def set_setpoint(self, setpoint: float) -> None:
        """
        Set a new target setpoint for the PID controller

        Args:
            setpoint (float): The target value to achieve
        """
        self.setpoint = setpoint
    
    def reset(self) -> None:
        """Reset the internal state of the PID controller (integral, previous error)"""
        self._integral = 0.0
        self._prev_err = None
