import matplotlib.pyplot as plt
from typing import Optional

from ..pid_controller import PIDController
from ..pid_simulation import PIDSimulation


class PIDControllerPlot:
    """
    A class to plot and visualize the response a PID controller over time

    This class uses matplotlib to create a plot that shows the system response,
    as controlled by the PID controller

    Attributes:
        pid (PIDController): The PID controller used for the simulation
        simulation (PIDSimulation): The PID simulation instance to generate plot data
        fig (Figure): The matplotlib figure object for the plot
        ax_value (Axes): The matplotlib axes object for plotting the PID response
        line_value (Line2D): The line representing the current value of the controlled system
        line_sentpoint (Line2D): The line representing the setpoint value in the plot
    """
    def __init__(self, pid: PIDController, delay_time: float, duration: Optional[float] = 2.0) -> None:
        """
        Args:
            pid (PIDController): The PID Controller instance to use for the plot
            dealay_time (float): The delay time between each simulation step
            duration (Optional[float]): The duration of the simulation
        """
        self.pid = pid
        self.simulation = PIDSimulation(pid, delay_time, duration)

        self.fig, self.ax_value = plt.subplots(figsize=(10, 6))

        self._initial_plot()

    def _initial_plot(self) -> None:
        self.line_value, = self.ax_value.plot([], [], label="Current Value", color='b')
        self.line_setpoint = self.ax_value.axhline(self.pid.setpoint, color='r', linestyle='--', label="Setpoint")
        self.ax_value.set_xlabel("Time (s)")
        self.ax_value.set_ylabel("Value")
        self.ax_value.set_title("PID Controller")
        self.ax_value.legend()
    
    def set_delay_time(self, delay_time: float) -> None:
        """
        Sets a new delay time for the simulation

        Args:
            delay_time (float): The new delay time value to set
        """
        self.simulation.set_delay_time(delay_time)

    def set_current_value(self, current_value: float) -> None:
        """
        Sets a new current value for the simulation

        Args:
            current_value (float): The new current value to set
        """
        self.simulation.set_current_value(current_value)

    def plot(self) -> None:
        """
        Plots the results of the PID simulation ny updating the data on graph
        If the simulation has not yet run, it rums the simulation first 
        """
        if self.simulation.is_results_empty():
            self.simulation.run_simulation()
        
        time_data, value_data = self.simulation.get_results()

        self.line_value.set_data(time_data, value_data)
        self.ax_value.relim()
        self.ax_value.autoscale_view()
        plt.draw()

    def show(self) -> None:
        """Display the plot window"""
        plt.show()
    
    def reset(self) -> None:
        """Reset the plot and simulation results to their initial state"""
        self.simulation.reset_results()
        self.ax_value.clear()
        self._initial_plot()
        plt.draw()
