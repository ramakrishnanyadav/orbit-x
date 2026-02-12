"""
Adaptive numerical integration with error control.

Implements Runge-Kutta-Fehlberg 4(5) method for high-accuracy
state propagation with automatic step size adjustment.
"""

import numpy as np
from typing import Callable, Tuple, List
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


class IntegrationError(Exception):
    """Raised when numerical integration fails."""
    pass


@dataclass
class IntegrationResult:
    """Result of adaptive integration."""
    states: List
    times: np.ndarray
    step_sizes: np.ndarray
    num_steps: int
    num_rejections: int


class AdaptiveIntegrator:
    """
    Adaptive timestep integration using Runge-Kutta-Fehlberg 4(5).
    
    Features:
    - 5th order accurate solution
    - 4th order embedded error estimate
    - Automatic step doubling/halving
    - Configurable error tolerances
    
    Reference: Fehlberg, E. (1969). "Low-order classical Runge-Kutta formulas
               with stepsize control and their application to some heat transfer
               problems." NASA Technical Report R-315.
    """
    
    # Butcher tableau coefficients for RK45
    A = np.array([
        [0, 0, 0, 0, 0, 0],
        [1/4, 0, 0, 0, 0, 0],
        [3/32, 9/32, 0, 0, 0, 0],
        [1932/2197, -7200/2197, 7296/2197, 0, 0, 0],
        [439/216, -8, 3680/513, -845/4104, 0, 0],
        [-8/27, 2, -3544/2565, 1859/4104, -11/40, 0]
    ])
    
    # Weights for 5th order solution
    C5 = np.array([16/135, 0, 6656/12825, 28561/56430, -9/50, 2/55])
    
    # Weights for 4th order solution
    C4 = np.array([25/216, 0, 1408/2565, 2197/4104, -1/5, 0])
    
    # Time increments
    B = np.array([0, 1/4, 3/8, 12/13, 1, 1/2])
    
    def __init__(self, rtol: float = 1e-6, atol: float = 1e-9, 
                 safety_factor: float = 0.9, max_step_increase: float = 2.0,
                 min_step_decrease: float = 0.2, min_dt: float = 1e-10):
        """
        Initialize adaptive integrator.
        
        Args:
            rtol: Relative error tolerance (default: 1e-6)
            atol: Absolute error tolerance (default: 1e-9)
            safety_factor: Safety margin for step size adjustment (default: 0.9)
            max_step_increase: Maximum factor to increase step size (default: 2.0)
            min_step_decrease: Minimum factor to decrease step size (default: 0.2)
            min_dt: Minimum allowable timestep (default: 1e-10 seconds)
        """
        self.rtol = rtol
        self.atol = atol
        self.safety_factor = safety_factor
        self.max_step_increase = max_step_increase
        self.min_step_decrease = min_step_decrease
        self.min_dt = min_dt
        
        self.num_rejections = 0
        self.num_steps = 0
    
    def propagate(self, state, dynamics_func: Callable, t_start: float, 
                  t_end: float, dt_initial: float = None) -> IntegrationResult:
        """
        Propagate state from t_start to t_end with adaptive timesteps.
        
        Args:
            state: Initial state object
            dynamics_func: Function f(state, t) that returns state derivatives
            t_start: Initial time
            t_end: Final time
            dt_initial: Initial timestep guess (default: (t_end - t_start) / 100)
        
        Returns:
            IntegrationResult containing trajectory and metadata
        """
        if dt_initial is None:
            dt_initial = (t_end - t_start) / 100.0
        
        dt = min(dt_initial, t_end - t_start)
        t = t_start
        
        states = [state]
        times = [t]
        step_sizes = []
        
        self.num_steps = 0
        self.num_rejections = 0
        
        while t < t_end:
            # Don't overshoot final time
            if t + dt > t_end:
                dt = t_end - t
            
            # Attempt integration step
            state_new, error, success = self._rkf45_step(
                state, dynamics_func, t, dt
            )
            
            if success:
                # Accept step
                state = state_new
                t += dt
                states.append(state)
                times.append(t)
                step_sizes.append(dt)
                self.num_steps += 1
                
                # Suggest next timestep based on error
                dt = self._adjust_timestep(dt, error)
            else:
                # Reject step and retry with smaller dt
                self.num_rejections += 1
                dt = self._adjust_timestep(dt, error)
                
                if dt < self.min_dt:
                    raise IntegrationError(
                        f"Timestep too small ({dt:.2e} s) at t={t:.2f}. "
                        f"Integration likely unstable. Try reducing tolerances."
                    )
        
        logger.info(
            f"Integration complete: {self.num_steps} steps, "
            f"{self.num_rejections} rejections, "
            f"mean dt={np.mean(step_sizes):.2e} s"
        )
        
        return IntegrationResult(
            states=states,
            times=np.array(times),
            step_sizes=np.array(step_sizes),
            num_steps=self.num_steps,
            num_rejections=self.num_rejections
        )
    
    def _rkf45_step(self, state, dynamics_func: Callable, t: float, 
                    dt: float) -> Tuple:
        """
        Single Runge-Kutta-Fehlberg 4(5) step.
        
        Returns:
            (state_new, error, success)
        """
        # Convert state to vector for computation
        y = self._state_to_vector(state)
        
        # Compute k values (6 stages)
        k = np.zeros((6, len(y)))
        
        for i in range(6):
            # Build intermediate state
            y_temp = y.copy()
            for j in range(i):
                y_temp += dt * self.A[i, j] * k[j]
            
            # Convert back to state object
            state_temp = self._vector_to_state(y_temp, state)
            
            # Evaluate dynamics
            dydt = dynamics_func(state_temp, t + self.B[i] * dt)
            k[i] = self._state_to_vector(dydt)
        
        # 5th order solution
        y_5 = y + dt * np.sum(self.C5[:, np.newaxis] * k, axis=0)
        
        # 4th order solution (for error estimate)
        y_4 = y + dt * np.sum(self.C4[:, np.newaxis] * k, axis=0)
        
        # Error estimate
        error = np.abs(y_5 - y_4)
        
        # Compute tolerance
        tolerance = self.atol + self.rtol * np.abs(y_5)
        
        # Check if step is acceptable
        error_ratio = np.max(error / tolerance)
        success = error_ratio <= 1.0
        
        # Convert back to state
        state_new = self._vector_to_state(y_5, state)
        
        return state_new, error_ratio, success
    
    def _adjust_timestep(self, dt: float, error_ratio: float) -> float:
        """
        Adjust timestep based on error estimate.
        
        Uses PI controller for smooth step size changes.
        """
        if error_ratio == 0:
            # Perfect accuracy, increase step aggressively
            factor = self.max_step_increase
        else:
            # Standard step size adjustment
            factor = self.safety_factor * (1.0 / error_ratio) ** 0.2
        
        # Clamp adjustment factor
        factor = np.clip(factor, self.min_step_decrease, self.max_step_increase)
        
        return dt * factor
    
    @staticmethod
    def _state_to_vector(state) -> np.ndarray:
        """
        Convert state object to flat numpy array.
        
        This is a generic implementation. Domain-specific integrators
        should override this method.
        """
        if hasattr(state, 'to_vector'):
            return state.to_vector()
        
        # Fallback: extract all numeric attributes
        vector = []
        for attr in ['position', 'velocity', 'quaternion', 'angular_velocity']:
            if hasattr(state, attr):
                val = getattr(state, attr)
                if isinstance(val, np.ndarray):
                    vector.extend(val.flatten())
                else:
                    vector.append(val)
        
        return np.array(vector, dtype=float)
    
    @staticmethod
    def _vector_to_state(vector: np.ndarray, template_state):
        """
        Convert flat numpy array back to state object.
        
        Uses template_state as a guide for structure.
        """
        if hasattr(template_state, 'from_vector'):
            return template_state.from_vector(vector)
        
        # Fallback: reconstruct based on template
        state_dict = {}
        idx = 0
        
        for attr in ['position', 'velocity', 'quaternion', 'angular_velocity']:
            if hasattr(template_state, attr):
                val = getattr(template_state, attr)
                if isinstance(val, np.ndarray):
                    size = val.size
                    state_dict[attr] = vector[idx:idx+size].reshape(val.shape)
                    idx += size
                else:
                    state_dict[attr] = vector[idx]
                    idx += 1
        
        # Create new state with updated values
        import copy
        new_state = copy.deepcopy(template_state)
        for attr, val in state_dict.items():
            setattr(new_state, attr, val)
        
        return new_state


class FixedStepRK4:
    """
    Classic 4th-order Runge-Kutta with fixed timestep.
    
    Faster than RK45 but less accurate. Use when:
    - Problem is known to be smooth
    - Timestep requirements are well-understood
    - Maximum performance is needed
    """
    
    def __init__(self, dt: float):
        """
        Initialize fixed-step RK4 integrator.
        
        Args:
            dt: Fixed timestep (seconds)
        """
        self.dt = dt
    
    def propagate(self, state, dynamics_func: Callable, t_start: float, 
                  t_end: float) -> IntegrationResult:
        """
        Propagate state from t_start to t_end with fixed timesteps.
        """
        t = t_start
        states = [state]
        times = [t]
        
        num_steps = int(np.ceil((t_end - t_start) / self.dt))
        
        for i in range(num_steps):
            dt = min(self.dt, t_end - t)
            state = self._rk4_step(state, dynamics_func, t, dt)
            t += dt
            states.append(state)
            times.append(t)
        
        return IntegrationResult(
            states=states,
            times=np.array(times),
            step_sizes=np.full(num_steps, self.dt),
            num_steps=num_steps,
            num_rejections=0
        )
    
    def _rk4_step(self, state, dynamics_func: Callable, t: float, dt: float):
        """Single RK4 step."""
        k1 = dynamics_func(state, t)
        
        state_temp = self._add_scaled(state, k1, dt/2)
        k2 = dynamics_func(state_temp, t + dt/2)
        
        state_temp = self._add_scaled(state, k2, dt/2)
        k3 = dynamics_func(state_temp, t + dt/2)
        
        state_temp = self._add_scaled(state, k3, dt)
        k4 = dynamics_func(state_temp, t + dt)
        
        # Combine: state_new = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        return self._rk4_combine(state, k1, k2, k3, k4, dt)
    
    @staticmethod
    def _add_scaled(state, derivative, scale: float):
        """Add scaled derivative to state."""
        import copy
        new_state = copy.deepcopy(state)
        
        # This is a simplified version - real implementation would handle
        # all state components properly
        if hasattr(new_state, 'position') and hasattr(derivative, 'position'):
            new_state.position = state.position + scale * derivative.position
        if hasattr(new_state, 'velocity') and hasattr(derivative, 'velocity'):
            new_state.velocity = state.velocity + scale * derivative.velocity
        
        return new_state
    
    @staticmethod
    def _rk4_combine(state, k1, k2, k3, k4, dt: float):
        """Combine RK4 stages."""
        import copy
        new_state = copy.deepcopy(state)
        
        if hasattr(new_state, 'position'):
            new_state.position = (state.position + 
                                 dt/6 * (k1.position + 2*k2.position + 
                                        2*k3.position + k4.position))
        if hasattr(new_state, 'velocity'):
            new_state.velocity = (state.velocity + 
                                 dt/6 * (k1.velocity + 2*k2.velocity + 
                                        2*k3.velocity + k4.velocity))
        
        return new_state
