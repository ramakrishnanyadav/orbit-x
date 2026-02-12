"""
Physics simulation and trajectory propagation.

Provides abstract interface for dynamics simulation across domains.
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Any
import numpy as np

from .models import State
from .constraints import Constraint


class DynamicsSimulator(ABC):
    """
    Abstract interface for vehicle dynamics simulation.
    
    Propagates state forward in time given control inputs.
    """
    
    @abstractmethod
    def propagate(self, state: State, control: Any, dt: float) -> State:
        """
        Advance state forward by time dt given control input.
        
        Args:
            state: Current state
            control: Control input (domain-specific)
            dt: Time step (seconds)
        
        Returns:
            New state after propagation
        """
        pass
    
    def propagate_trajectory(self, initial_state: State, controls: List[Any], 
                            dt: float) -> List[State]:
        """
        Propagate full trajectory from initial state.
        
        Args:
            initial_state: Starting state
            controls: Sequence of control inputs
            dt: Time step for each control
        
        Returns:
            List of states including initial state
        """
        states = [initial_state.copy()]
        current = initial_state.copy()
        
        for control in controls:
            current = self.propagate(current, control, dt)
            states.append(current.copy())
        
        return states
    
    def validate_trajectory(self, trajectory: List[State], 
                           constraints: List[Constraint]) -> Tuple[bool, List[str]]:
        """
        Check if trajectory satisfies all constraints.
        
        Args:
            trajectory: List of states
            constraints: List of constraints to check
        
        Returns:
            (is_valid, violation_messages)
        """
        is_valid = True
        violations = []
        
        for i, state in enumerate(trajectory):
            for constraint in constraints:
                satisfied, margin = constraint.check(state)
                if not satisfied and constraint.severity == 'hard':
                    is_valid = False
                    violations.append(
                        f"t={state.time:.1f}s: {constraint.name} violated (margin={margin:.2f})"
                    )
        
        return is_valid, violations
    
    @abstractmethod
    def get_energy_cost(self, state: State, control: Any, dt: float) -> float:
        """
        Compute energy/fuel cost for a control action.
        
        Args:
            state: Current state
            control: Control input
            dt: Time duration
        
        Returns:
            Energy consumed (Joules, kg of fuel, etc.)
        """
        pass


class RK4Integrator:
    """
    4th-order Runge-Kutta numerical integrator.
    
    Provides accurate numerical integration for dynamics.
    """
    
    @staticmethod
    def integrate(state_vec: np.ndarray, derivative_func, t: float, dt: float) -> np.ndarray:
        """
        Perform one RK4 integration step.
        
        Args:
            state_vec: State vector
            derivative_func: Function computing state derivative: f(state, t) -> state_dot
            t: Current time
            dt: Time step
        
        Returns:
            New state vector
        """
        k1 = derivative_func(state_vec, t)
        k2 = derivative_func(state_vec + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = derivative_func(state_vec + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = derivative_func(state_vec + dt * k3, t + dt)
        
        return state_vec + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


class AtmosphereModel:
    """
    Simple exponential atmosphere model.
    
    Used for aerodynamic and drag calculations.
    """
    
    # Sea level conditions
    RHO_0 = 1.225  # kg/m^3
    SCALE_HEIGHT = 8500  # meters
    
    @staticmethod
    def density(altitude: float) -> float:
        """
        Compute atmospheric density at given altitude.
        
        Args:
            altitude: Altitude above sea level (meters)
        
        Returns:
            Air density (kg/m^3)
        """
        return AtmosphereModel.RHO_0 * np.exp(-altitude / AtmosphereModel.SCALE_HEIGHT)
    
    @staticmethod
    def temperature(altitude: float) -> float:
        """
        Compute temperature at given altitude (simplified).
        
        Args:
            altitude: Altitude above sea level (meters)
        
        Returns:
            Temperature (Kelvin)
        """
        # Linear lapse rate: -6.5 K/km up to 11km
        T_0 = 288.15  # K at sea level
        lapse_rate = -0.0065  # K/m
        
        if altitude < 11000:
            return T_0 + lapse_rate * altitude
        else:
            return T_0 + lapse_rate * 11000  # Isothermal above tropopause
