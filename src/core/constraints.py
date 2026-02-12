"""
Constraint abstraction framework.

Provides a unified interface for expressing and checking constraints
across different mission types.
"""

from abc import ABC, abstractmethod
from typing import Tuple, List, Any, Optional
import numpy as np
from shapely.geometry import Point, Polygon

from .models import State


class Constraint(ABC):
    """
    Abstract base class for all constraints.
    
    Constraints can be 'hard' (must be satisfied) or 'soft' (preference).
    """
    
    def __init__(self, name: str, severity: str = 'hard'):
        """
        Initialize constraint.
        
        Args:
            name: Human-readable constraint name
            severity: 'hard' or 'soft'
        """
        self.name = name
        if severity not in ['hard', 'soft']:
            raise ValueError(f"Severity must be 'hard' or 'soft', got {severity}")
        self.severity = severity
    
    @abstractmethod
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """
        Check if constraint is satisfied.
        
        Args:
            state: Current state
            action: Optional action being taken
        
        Returns:
            (is_satisfied, margin): 
                - is_satisfied: True if constraint is met
                - margin: How far from violation (positive = safe, negative = violated)
        """
        pass
    
    def encode_linear(self, optimizer: Any) -> List[Any]:
        """
        Encode constraint as linear inequalities for MILP solver.
        
        Returns:
            List of linear constraint objects
        """
        # Default: not linearly encodable
        return []
    
    def get_repair_hint(self, state: State, action: Any = None) -> str:
        """
        Suggest how to fix a constraint violation.
        
        Returns:
            Human-readable repair suggestion
        """
        return f"Constraint '{self.name}' violated. No repair hint available."
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}', severity='{self.severity}')"


class NoFlyZoneConstraint(Constraint):
    """
    Spatial constraint: prohibited regions.
    
    Uses Shapely for efficient geometric operations.
    """
    
    def __init__(self, polygon: Polygon, altitude_range: Tuple[float, float] = None, 
                 name: str = "NoFlyZone"):
        """
        Initialize no-fly zone constraint.
        
        Args:
            polygon: Shapely Polygon defining the zone (lat/lon coordinates)
            altitude_range: (min_alt, max_alt) in meters. None = all altitudes
            name: Constraint name
        """
        super().__init__(name, severity='hard')
        self.polygon = polygon
        self.altitude_range = altitude_range
        
        # Pre-compute buffer for safety margin
        self.buffer_distance = 100  # meters (converted to degrees approximately)
        self.buffer_degrees = self.buffer_distance / 111000  # rough conversion
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if state is outside no-fly zone."""
        # Extract lat/lon from position
        if len(state.position) >= 2:
            lat, lon = state.position[0], state.position[1]
            point = Point(lat, lon)
        else:
            return (True, float('inf'))
        
        # Check if inside polygon
        inside = self.polygon.contains(point)
        
        # Check altitude if specified
        in_alt_range = True
        if self.altitude_range is not None and state.altitude is not None:
            alt_min, alt_max = self.altitude_range
            in_alt_range = alt_min <= state.altitude <= alt_max
        
        violated = inside and in_alt_range
        
        # Compute margin (distance to boundary)
        if violated:
            # Inside: negative margin
            margin = -self.polygon.exterior.distance(point) * 111000  # convert to meters
        else:
            # Outside: positive margin
            margin = self.polygon.exterior.distance(point) * 111000
        
        return (not violated, margin)
    
    def get_repair_hint(self, state: State, action: Any = None) -> str:
        """Suggest moving away from no-fly zone."""
        return f"Move at least {self.buffer_distance}m away from {self.name}"


class ResourceConstraint(Constraint):
    """
    Constraint on consumable resources (fuel, battery, data storage).
    """
    
    def __init__(self, resource_type: str, min_level: float, 
                 max_level: float = float('inf'), name: str = None):
        """
        Initialize resource constraint.
        
        Args:
            resource_type: 'fuel', 'battery', 'data_storage', etc.
            min_level: Minimum allowed level
            max_level: Maximum allowed level
            name: Optional constraint name
        """
        name = name or f"{resource_type}_limit"
        super().__init__(name, severity='hard')
        self.resource_type = resource_type
        self.min_level = min_level
        self.max_level = max_level
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if resource level is within bounds."""
        # Try different locations for resource value
        current = None
        
        if self.resource_type == 'fuel' and state.fuel is not None:
            current = state.fuel
        elif self.resource_type == 'battery' and state.battery_SOC is not None:
            current = state.battery_SOC
        elif self.resource_type == 'data_storage' and state.data_storage is not None:
            current = state.data_storage
        elif self.resource_type in state.resources:
            current = state.resources[self.resource_type]
        
        if current is None:
            # Resource not tracked in this state
            return (True, float('inf'))
        
        satisfied = self.min_level <= current <= self.max_level
        
        # Margin is minimum distance to either bound
        margin = min(current - self.min_level, self.max_level - current)
        
        return (satisfied, margin)
    
    def get_repair_hint(self, state: State, action: Any = None) -> str:
        """Suggest resource management action."""
        satisfied, margin = self.check(state, action)
        if not satisfied:
            if margin < 0:
                return f"Resource {self.resource_type} out of bounds. Current margin: {margin:.2f}"
        return ""


class TimeWindowConstraint(Constraint):
    """
    Temporal constraint: activity must occur within a time window.
    """
    
    def __init__(self, activity: str, earliest: float, latest: float, 
                 name: str = None):
        """
        Initialize time window constraint.
        
        Args:
            activity: Activity identifier
            earliest: Earliest allowed time (seconds)
            latest: Latest allowed time (seconds)
            name: Optional constraint name
        """
        name = name or f"{activity}_time_window"
        super().__init__(name, severity='hard')
        self.activity = activity
        self.earliest = earliest
        self.latest = latest
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if current time is within window."""
        # Only check if action matches the activity
        if action is not None and hasattr(action, 'type'):
            if action.type != self.activity:
                return (True, float('inf'))
        
        satisfied = self.earliest <= state.time <= self.latest
        
        # Compute margin
        if state.time < self.earliest:
            margin = self.earliest - state.time
        elif state.time > self.latest:
            margin = state.time - self.latest
        else:
            margin = min(state.time - self.earliest, self.latest - state.time)
        
        return (satisfied, margin)
    
    def get_repair_hint(self, state: State, action: Any = None) -> str:
        """Suggest timing adjustment."""
        if state.time < self.earliest:
            return f"Wait {self.earliest - state.time:.1f}s before {self.activity}"
        elif state.time > self.latest:
            return f"Activity {self.activity} must complete by t={self.latest:.1f}s"
        return ""


class TurnRateConstraint(Constraint):
    """
    Kinematic constraint: maximum turn rate for aircraft.
    """
    
    def __init__(self, max_turn_rate: float, name: str = "max_turn_rate"):
        """
        Initialize turn rate constraint.
        
        Args:
            max_turn_rate: Maximum turn rate in deg/s
            name: Constraint name
        """
        super().__init__(name, severity='hard')
        self.max_rate = max_turn_rate
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if turn rate is within limits."""
        if action is None or not hasattr(action, 'heading_change'):
            return (True, float('inf'))
        
        dt = getattr(action, 'duration', 1.0)
        if dt <= 0:
            return (True, float('inf'))
        
        turn_rate = abs(action.heading_change) / dt
        satisfied = turn_rate <= self.max_rate
        margin = self.max_rate - turn_rate
        
        return (satisfied, margin)
    
    def get_repair_hint(self, state: State, action: Any = None) -> str:
        """Suggest reducing turn rate."""
        return f"Reduce turn rate to <= {self.max_rate} deg/s"


class AltitudeConstraint(Constraint):
    """
    Constraint on altitude bounds.
    """
    
    def __init__(self, min_altitude: float, max_altitude: float, 
                 name: str = "altitude_limits"):
        """
        Initialize altitude constraint.
        
        Args:
            min_altitude: Minimum allowed altitude (meters)
            max_altitude: Maximum allowed altitude (meters)
            name: Constraint name
        """
        super().__init__(name, severity='hard')
        self.min_alt = min_altitude
        self.max_alt = max_altitude
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if altitude is within bounds."""
        if state.altitude is None:
            return (True, float('inf'))
        
        satisfied = self.min_alt <= state.altitude <= self.max_alt
        margin = min(state.altitude - self.min_alt, self.max_alt - state.altitude)
        
        return (satisfied, margin)
    
    def get_repair_hint(self, state: State, action: Any = None) -> str:
        """Suggest altitude adjustment."""
        if state.altitude < self.min_alt:
            return f"Climb to at least {self.min_alt}m"
        elif state.altitude > self.max_alt:
            return f"Descend to below {self.max_alt}m"
        return ""


class BankAngleConstraint(Constraint):
    """
    Constraint on maximum bank angle for aircraft.
    """
    
    def __init__(self, max_bank_angle: float, name: str = "max_bank_angle"):
        """
        Initialize bank angle constraint.
        
        Args:
            max_bank_angle: Maximum bank angle in degrees
            name: Constraint name
        """
        super().__init__(name, severity='hard')
        self.max_bank = max_bank_angle
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if bank angle is within limits."""
        if action is None or not hasattr(action, 'bank_angle'):
            return (True, float('inf'))
        
        bank = abs(action.bank_angle)
        satisfied = bank <= self.max_bank
        margin = self.max_bank - bank
        
        return (satisfied, margin)


class SlewRateConstraint(Constraint):
    """
    Constraint on spacecraft slew rate (attitude change rate).
    """
    
    def __init__(self, max_slew_rate: float, min_slew_time: float = 0.0,
                 name: str = "slew_rate_limit"):
        """
        Initialize slew rate constraint.
        
        Args:
            max_slew_rate: Maximum slew rate in deg/s
            min_slew_time: Minimum time for slew maneuver (seconds)
            name: Constraint name
        """
        super().__init__(name, severity='hard')
        self.max_rate = max_slew_rate
        self.min_time = min_slew_time
    
    def check(self, state: State, action: Any = None) -> Tuple[bool, float]:
        """Check if slew maneuver is feasible."""
        if action is None or not hasattr(action, 'slew_angle'):
            return (True, float('inf'))
        
        slew_angle = abs(action.slew_angle)
        dt = getattr(action, 'duration', 0.0)
        
        # Check rate limit
        if dt > 0:
            slew_rate = slew_angle / dt
            rate_ok = slew_rate <= self.max_rate
        else:
            rate_ok = slew_angle == 0
        
        # Check minimum time
        min_required_time = slew_angle / self.max_rate + self.min_time
        time_ok = dt >= min_required_time
        
        satisfied = rate_ok and time_ok
        margin = min_required_time - dt if not time_ok else 0.0
        
        return (satisfied, -margin if not satisfied else margin)


def check_all_constraints(state: State, constraints: List[Constraint], 
                         action: Any = None) -> Tuple[bool, Dict[str, Tuple[bool, float]]]:
    """
    Check all constraints and return detailed results.
    
    Args:
        state: Current state
        constraints: List of constraints to check
        action: Optional action being evaluated
    
    Returns:
        (all_satisfied, results_dict):
            - all_satisfied: True if all hard constraints met
            - results_dict: {constraint_name: (satisfied, margin)}
    """
    all_satisfied = True
    results = {}
    
    for constraint in constraints:
        satisfied, margin = constraint.check(state, action)
        results[constraint.name] = (satisfied, margin)
        
        if constraint.severity == 'hard' and not satisfied:
            all_satisfied = False
    
    return all_satisfied, results
