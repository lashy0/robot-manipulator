import numpy as np
from typing import Tuple

from .pid_controller import PIDController


class PIDSimulation:
    """
    A class to simulation the response of a system controlled by a PID controller over time

    This class runs a simulation based on the provided PID controller settings and computrs
    the system response

    Attributes:
        pid (PIDController): The PID controller used in the simulation
        delay_time (float): The delay time each simulation step
        duration (float): The total duration of the simulation
        current_value (float): The initial value of the system being controlled
        value_data (np.ndarray): An array that stores the values of the system response over time
        time_data (np.ndarray): An array that stores the time points of the simulation
    """
    def __init__(
        self,
        pid: PIDController,
        delay_time: float,
        duration: float
    ) -> None:
        """
        Args:
            pid (PIDController): The PID controller instance to use for the simulation
            delay_time (float): The delay time between each simulation step
            duration (float): The total duration of the simulation
        """
        self.pid = pid
        self.delay_time = delay_time
        self.duration = duration
        self.current_value = 0.0

        self._initial_state()

    def _initial_state(self) -> None:
        """Initializes the value data and time data arrays"""
        self.value_data = np.array([], dtype=float)
        self.time_data = np.array([], dtype=float)

    def set_current_value(self, current_value: float) -> None:
        """
        Sets the current value of the system to a new value

        Args:
            current_value (float): The new current value to set
        """
        self.current_value = current_value
    
    def set_duration(self, duration: float) -> None:
        """
        Sets the duration of the simulation

        Args:
            duration (float): The new duration value to set
        """
        if duration <= 0:
            raise ValueError(
                "duration must be positive"
            )
        self.duration = duration
    
    def set_delay_time(self, delay_time: float) -> None:
        """
        Sets the delay time between each simulation step

        Args:
            delay_time (float): The new delay time value to set
        """
        if delay_time <= 0:
            raise ValueError(
                "delay time must be positive"
            )
        self.delay_time = delay_time
    
    def run_simulation(self) -> None:
        """
        Run PID controller simulation
        This method computes the system response over time based on the PID controller settings
        """
        self.reset_results()

        tiem_steps = np.linspace(0, self.duration, int(self.duration / self.delay_time))
        self.value_data = np.zeros(tiem_steps.size, dtype=float)
        self.time_data = np.zeros(tiem_steps.size, dtype=float)
        current_value = self.current_value

        for i, time_step in enumerate(tiem_steps):
            if i == 0:
                self.value_data[i] = current_value
                self.time_data[i] = time_step
                continue
            control_signal = self.pid.update(current_value, self.delay_time)
            current_value += control_signal

            self.value_data[i] = current_value
            self.time_data[i] = time_step
    
    def get_results(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Retrieves results of the simulation

        Returns:
            Tuple[np.ndarray, np.ndarray]: Arrays containing the time data and the value data
        """
        return self.time_data, self.value_data
    
    def reset_results(self) -> None:
        """Reset the simulation result, clearing stored time and value data"""
        self._initial_state()
    
    def is_results_empty(self) -> bool:
        """
        Check if simulation results is empty

        Returns:
            bool: True if the simulation data is empty. False otherwise
        """
        return self.time_data.size == 0 or self.value_data.size == 0
