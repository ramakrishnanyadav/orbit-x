"""
Core domain models for unified mission planning.

This module defines the fundamental abstractions that work across
aircraft and spacecraft domains.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional
from abc import ABC, abstractmethod
import numpy as np


@dataclass
class State:
    """
    Unified state representation for vehicles.
    
    Different domains use different subsets of these fields:
    - Aircraft: position, velocity, heading, fuel, time
    - Spacecraft: position_ECI, velocity_ECI, quaternion, angular_velocity, 
                  battery_SOC, data_storage, time
    """
    # Common fields
    time: float = 0.0  # seconds since mission start
    
    # Spatial state (can be lat/lon/alt or ECI coordinates)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Aircraft-specific
    heading: Optional[float] = None  # degrees
    fuel: Optional[float] = None  # kg or liters
    altitude: Optional[float] = None  # meters
    
    # Spacecraft-specific
    position_ECI: Optional[np.ndarray] = None  # km, Earth-Centered Inertial
    velocity_ECI: Optional[np.ndarray] = None  # km/s
    quaternion: Optional[np.ndarray] = None  # attitude representation
    angular_velocity: Optional[np.ndarray] = None  # rad/s
    battery_SOC: Optional[float] = None  # State of Charge (0-1)
    data_storage: Optional[float] = None  # MB used
    
    # Generic resource tracking
    resources: Dict[str, float] = field(default_factory=dict)
    
    # Metadata
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def copy(self) -> 'State':
        """Create a deep copy of this state."""
        import copy
        return copy.deepcopy(self)
    
    def distance_to(self, other: 'State') -> float:
        """Compute Euclidean distance to another state."""
        return np.linalg.norm(self.position - other.position)


@dataclass
class Vehicle:
    """
    Unified vehicle representation.
    
    Contains physical properties and capabilities common to all vehicle types.
    """
    name: str
    vehicle_type: str  # 'aircraft' or 'spacecraft'
    mass: float  # kg
    
    # Geometric properties
    cross_sectional_area: float = 0.0  # m^2
    drag_coefficient: float = 2.2
    
    # Performance limits
    max_speed: Optional[float] = None  # m/s
    max_acceleration: Optional[float] = None  # m/s^2
    max_turn_rate: Optional[float] = None  # deg/s
    max_bank_angle: Optional[float] = None  # degrees (aircraft)
    max_slew_rate: Optional[float] = None  # deg/s (spacecraft)
    
    # Propulsion
    max_thrust: Optional[float] = None  # N
    fuel_capacity: Optional[float] = None  # kg
    battery_capacity: Optional[float] = None  # Wh
    
    # Domain-specific properties
    properties: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate vehicle configuration."""
        if self.vehicle_type not in ['aircraft', 'spacecraft']:
            raise ValueError(f"Invalid vehicle_type: {self.vehicle_type}")


@dataclass
class Waypoint:
    """A target location or point of interest."""
    id: str
    position: np.ndarray  # lat/lon/alt or ECI coordinates
    name: Optional[str] = None
    priority: int = 1
    time_window: Optional[tuple] = None  # (earliest, latest) in seconds
    required_duration: float = 0.0  # seconds to spend at waypoint
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Mission:
    """
    Unified mission specification.
    
    Defines objectives, constraints, and success criteria.
    """
    name: str
    vehicle: Vehicle
    start_state: State
    waypoints: List[Waypoint]
    
    # Mission parameters
    max_duration: float  # seconds
    return_to_start: bool = True
    
    # Objectives and weights
    objectives: Dict[str, float] = field(default_factory=lambda: {
        'minimize_time': 1.0,
        'minimize_fuel': 0.5,
        'minimize_risk': 0.3,
    })
    
    # Constraints (will be Constraint objects)
    constraints: List[Any] = field(default_factory=list)
    
    # Environment
    environment: Dict[str, Any] = field(default_factory=dict)
    
    # Metadata
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def add_constraint(self, constraint: Any):
        """Add a constraint to the mission."""
        self.constraints.append(constraint)
    
    def get_objective_value(self) -> float:
        """Compute weighted objective value (to be implemented by planners)."""
        return 0.0


class StateSpace(ABC):
    """
    Abstract interface for state space representation.
    
    Defines the valid configuration space for a vehicle.
    """
    
    @abstractmethod
    def validate_state(self, state: State) -> bool:
        """Check if a state is valid."""
        pass
    
    @abstractmethod
    def distance_metric(self, s1: State, s2: State) -> float:
        """Compute distance between two states."""
        pass
    
    @abstractmethod
    def interpolate(self, s1: State, s2: State, alpha: float) -> State:
        """
        Interpolate between two states.
        
        Args:
            s1: Start state
            s2: End state
            alpha: Interpolation parameter in [0, 1]
        
        Returns:
            Interpolated state
        """
        pass
    
    @property
    @abstractmethod
    def dimension(self) -> int:
        """Dimensionality of the state space."""
        pass


@dataclass
class MissionResult:
    """
    Results from mission planning and execution.
    
    Contains the planned trajectory, performance metrics, and validation data.
    """
    success: bool
    mission: Mission
    
    # Planned trajectory
    states: List[State] = field(default_factory=list)
    controls: List[Any] = field(default_factory=list)
    
    # Performance metrics
    total_time: float = 0.0
    total_distance: float = 0.0
    fuel_used: float = 0.0
    energy_used: float = 0.0
    
    # Objective values
    objective_value: float = 0.0
    objective_breakdown: Dict[str, float] = field(default_factory=dict)
    
    # Constraint satisfaction
    constraint_violations: int = 0
    constraint_margins: Dict[str, float] = field(default_factory=dict)
    
    # Computational metrics
    planning_time: float = 0.0  # seconds
    algorithm_used: str = ""
    iterations: int = 0
    
    # Validation data
    is_feasible: bool = True
    validation_report: Dict[str, Any] = field(default_factory=dict)
    
    # Metadata
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def summary(self) -> str:
        """Generate a human-readable summary."""
        lines = [
            f"Mission: {self.mission.name}",
            f"Success: {self.success}",
            f"Algorithm: {self.algorithm_used}",
            f"Planning Time: {self.planning_time:.2f}s",
            f"Total Mission Time: {self.total_time:.1f}s ({self.total_time/60:.1f} min)",
            f"Total Distance: {self.total_distance/1000:.2f} km",
            f"Fuel Used: {self.fuel_used:.2f} kg",
            f"Constraint Violations: {self.constraint_violations}",
            f"Objective Value: {self.objective_value:.3f}",
        ]
        return "\n".join(lines)
