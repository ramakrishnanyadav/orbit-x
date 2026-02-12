"""
Aircraft domain implementation.

Includes dynamics simulation, wind modeling, and aerodynamics.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, Callable
import numpy as np
from scipy.interpolate import RegularGridInterpolator

from ..core.models import State, Vehicle
from ..core.simulator import DynamicsSimulator, RK4Integrator, AtmosphereModel


@dataclass
class AircraftControl:
    """Control input for aircraft."""
    thrust: float  # N
    bank_angle: float  # degrees
    duration: float = 1.0  # seconds
    heading_change: float = 0.0  # degrees (computed)


class WindField:
    """
    3D wind field with spatial and temporal variation.
    
    Supports uniform, grid-based, and analytical wind models.
    """
    
    def __init__(self, wind_type: str = 'uniform', **kwargs):
        """
        Initialize wind field.
        
        Args:
            wind_type: 'uniform', 'grid', or 'analytical'
            **kwargs: Type-specific parameters
        """
        self.wind_type = wind_type
        
        if wind_type == 'uniform':
            # Uniform wind: constant velocity vector
            self.wind_vector = np.array(kwargs.get('wind_vector', [0, 0, 0]))
        
        elif wind_type == 'grid':
            # Grid-based wind field
            self.lat_grid = kwargs.get('lat_grid', np.linspace(0, 1, 10))
            self.lon_grid = kwargs.get('lon_grid', np.linspace(0, 1, 10))
            self.alt_grid = kwargs.get('alt_grid', np.linspace(0, 1000, 5))
            
            # Wind components (u=east, v=north, w=up)
            self.u_grid = kwargs.get('u_grid', np.zeros((10, 10, 5)))
            self.v_grid = kwargs.get('v_grid', np.zeros((10, 10, 5)))
            self.w_grid = kwargs.get('w_grid', np.zeros((10, 10, 5)))
            
            # Create interpolators
            self.u_interp = RegularGridInterpolator(
                (self.lat_grid, self.lon_grid, self.alt_grid), 
                self.u_grid, bounds_error=False, fill_value=0
            )
            self.v_interp = RegularGridInterpolator(
                (self.lat_grid, self.lon_grid, self.alt_grid), 
                self.v_grid, bounds_error=False, fill_value=0
            )
            self.w_interp = RegularGridInterpolator(
                (self.lat_grid, self.lon_grid, self.alt_grid), 
                self.w_grid, bounds_error=False, fill_value=0
            )
        
        elif wind_type == 'analytical':
            # Analytical wind model
            self.wind_function = kwargs.get('wind_function', lambda pos, t: np.zeros(3))
        
        # Temporal variation parameters
        self.temporal_amplitude = kwargs.get('temporal_amplitude', 0.0)
        self.temporal_period = kwargs.get('temporal_period', 3600.0)  # seconds
        self.temporal_phase = kwargs.get('temporal_phase', 0.0)
    
    def get_wind(self, position: np.ndarray, time: float = 0.0) -> np.ndarray:
        """
        Get wind vector at given position and time.
        
        Args:
            position: [lat, lon, alt] or [x, y, z]
            time: Current time (seconds)
        
        Returns:
            Wind velocity vector [u, v, w] in m/s
        """
        # Base wind from spatial field
        if self.wind_type == 'uniform':
            wind = self.wind_vector.copy()
        
        elif self.wind_type == 'grid':
            # Interpolate from grid
            lat, lon, alt = position[0], position[1], position[2] if len(position) > 2 else 0
            u = float(self.u_interp([lat, lon, alt]))
            v = float(self.v_interp([lat, lon, alt]))
            w = float(self.w_interp([lat, lon, alt]))
            wind = np.array([u, v, w])
        
        elif self.wind_type == 'analytical':
            wind = self.wind_function(position, time)
        else:
            wind = np.zeros(3)
        
        # Add temporal variation
        if self.temporal_amplitude > 0:
            phase = 2 * np.pi * time / self.temporal_period + self.temporal_phase
            multiplier = 1 + self.temporal_amplitude * np.sin(phase)
            wind *= multiplier
        
        return wind
    
    def perturb(self, wind_multiplier: float = 1.0, wind_rotation: float = 0.0):
        """
        Create perturbed wind field for Monte Carlo analysis.
        
        Args:
            wind_multiplier: Scale factor for wind magnitude
            wind_rotation: Rotation angle in degrees
        """
        if self.wind_type == 'uniform':
            # Scale and rotate
            mag = np.linalg.norm(self.wind_vector[:2])
            if mag > 0:
                angle = np.arctan2(self.wind_vector[1], self.wind_vector[0])
                angle += np.deg2rad(wind_rotation)
                new_mag = mag * wind_multiplier
                self.wind_vector[0] = new_mag * np.cos(angle)
                self.wind_vector[1] = new_mag * np.sin(angle)


class AircraftDynamics(DynamicsSimulator):
    """
    Aircraft dynamics simulator with point-mass model.
    
    Includes:
    - 3DOF or 6DOF kinematics
    - Aerodynamics (lift, drag)
    - Propulsion (thrust, fuel consumption)
    - Wind effects
    """
    
    def __init__(self, vehicle: Vehicle, wind_field: Optional[WindField] = None):
        """
        Initialize aircraft dynamics.
        
        Args:
            vehicle: Aircraft vehicle specification
            wind_field: Optional wind field model
        """
        self.vehicle = vehicle
        self.wind_field = wind_field or WindField('uniform')
        
        # Aerodynamic parameters
        self.S = vehicle.properties.get('wing_area', 20.0)  # m^2
        self.C_L = vehicle.properties.get('C_L', 0.5)  # lift coefficient
        self.C_D_0 = vehicle.properties.get('C_D_0', 0.025)  # parasitic drag
        self.k = vehicle.properties.get('k', 0.05)  # induced drag factor
        self.e = vehicle.properties.get('e', 0.8)  # Oswald efficiency
        self.AR = vehicle.properties.get('AR', 8.0)  # aspect ratio
        
        # Propulsion parameters
        self.TSFC = vehicle.properties.get('TSFC', 0.5)  # kg/(N*hr) or (1/hr) for electric
        self.eta_prop = vehicle.properties.get('eta_prop', 0.8)  # propulsive efficiency
        
        # Limits
        self.max_load_factor = vehicle.properties.get('max_load_factor', 2.5)
        
        # Constants
        self.g = 9.81  # m/s^2
    
    def propagate(self, state: State, control: AircraftControl, dt: float) -> State:
        """
        Propagate aircraft state forward by dt.
        
        Uses point-mass 3DOF model with aerodynamics.
        """
        # Extract current state
        lat, lon, alt = state.position[0], state.position[1], state.position[2]
        
        # Velocity in body frame (simplified: heading aligned with velocity)
        V = np.linalg.norm(state.velocity)  # airspeed magnitude
        heading = state.heading or 0.0
        
        # Get wind at current position
        wind = self.wind_field.get_wind(state.position, state.time)
        
        # Atmospheric properties
        rho = AtmosphereModel.density(alt)
        
        # Aerodynamic forces
        q = 0.5 * rho * V**2  # dynamic pressure
        
        # Lift
        L = q * self.S * self.C_L
        
        # Drag (parasitic + induced)
        C_D = self.C_D_0 + self.k * self.C_L**2
        D = q * self.S * C_D
        
        # Thrust
        thrust = control.thrust
        
        # Weight
        weight = self.vehicle.mass * self.g
        
        # Bank angle
        bank = np.deg2rad(control.bank_angle)
        
        # Forces in wind frame
        # Assuming small angle approximations and coordinated turn
        F_x = thrust * np.cos(heading) - D  # horizontal, along velocity
        F_y = thrust * np.sin(bank)  # lateral
        F_z = L * np.cos(bank) - weight  # vertical
        
        # Accelerations
        a_x = F_x / self.vehicle.mass
        a_y = F_y / self.vehicle.mass
        a_z = F_z / self.vehicle.mass
        
        # Turn rate (coordinated turn)
        if V > 1.0:  # avoid division by zero
            turn_rate = self.g * np.tan(bank) / V  # rad/s
        else:
            turn_rate = 0.0
        
        # Update heading
        new_heading = heading + np.rad2deg(turn_rate * dt)
        new_heading = new_heading % 360
        
        # Update velocity (airspeed)
        # Simplified: change in speed magnitude
        dV = a_x * dt
        new_V = max(0.1, V + dV)  # minimum speed
        
        # Update position (including wind drift)
        # Convert heading to velocity components
        v_east = new_V * np.sin(np.deg2rad(new_heading))
        v_north = new_V * np.cos(np.deg2rad(new_heading))
        v_up = a_z * dt  # simplified vertical
        
        velocity_air = np.array([v_north, v_east, v_up])
        velocity_ground = velocity_air + wind[:3]
        
        new_position = state.position + velocity_ground * dt
        
        # Ensure altitude doesn't go negative
        if new_position[2] < 0:
            new_position[2] = 0
            v_up = 0
        
        # Fuel consumption
        if state.fuel is not None:
            fuel_rate = (thrust * self.TSFC) / 3600  # kg/s
            new_fuel = max(0, state.fuel - fuel_rate * dt)
        else:
            new_fuel = None
        
        # Battery consumption (for electric aircraft)
        if state.battery_SOC is not None and self.vehicle.battery_capacity:
            # Power = Thrust * Velocity / efficiency
            power = (thrust * new_V) / self.eta_prop  # Watts
            energy_used = power * dt / 3600  # Wh
            soc_decrease = energy_used / self.vehicle.battery_capacity
            new_soc = max(0, state.battery_SOC - soc_decrease)
        else:
            new_soc = None
        
        # Create new state
        new_state = State(
            time=state.time + dt,
            position=new_position,
            velocity=velocity_ground,
            heading=new_heading,
            altitude=new_position[2],
            fuel=new_fuel,
            battery_SOC=new_soc,
            resources=state.resources.copy(),
            metadata=state.metadata.copy()
        )
        
        return new_state
    
    def get_energy_cost(self, state: State, control: AircraftControl, dt: float) -> float:
        """Compute fuel/energy cost for control action."""
        thrust = control.thrust
        
        if state.fuel is not None:
            # Fuel-based
            fuel_rate = (thrust * self.TSFC) / 3600  # kg/s
            return fuel_rate * dt
        else:
            # Battery-based
            V = np.linalg.norm(state.velocity)
            power = (thrust * V) / self.eta_prop
            return power * dt / 3600  # Wh
    
    def compute_turn_radius(self, speed: float, bank_angle: float) -> float:
        """
        Compute turn radius for coordinated turn.
        
        Args:
            speed: Airspeed (m/s)
            bank_angle: Bank angle (degrees)
        
        Returns:
            Turn radius (meters)
        """
        if abs(bank_angle) < 0.1:
            return float('inf')
        
        bank_rad = np.deg2rad(bank_angle)
        R = speed**2 / (self.g * np.tan(bank_rad))
        return abs(R)
    
    def compute_min_turn_radius(self, speed: float) -> float:
        """
        Compute minimum turn radius at given speed.
        
        Uses maximum bank angle.
        """
        max_bank = self.vehicle.max_bank_angle or 45.0
        return self.compute_turn_radius(speed, max_bank)
    
    def is_maneuver_feasible(self, state1: State, state2: State, speed: float) -> bool:
        """
        Check if maneuver between two states is dynamically feasible.
        
        Args:
            state1: Start state
            state2: End state
            speed: Planned airspeed
        
        Returns:
            True if maneuver respects turn radius constraints
        """
        # Compute required turn
        heading1 = state1.heading or 0.0
        dx = state2.position[0] - state1.position[0]
        dy = state2.position[1] - state1.position[1]
        heading2 = np.rad2deg(np.arctan2(dy, dx))
        
        heading_change = (heading2 - heading1 + 180) % 360 - 180  # [-180, 180]
        
        # Check if turn radius is sufficient
        min_radius = self.compute_min_turn_radius(speed)
        distance = np.linalg.norm([dx, dy])
        
        # Simple check: distance should be greater than turn radius for large turns
        if abs(heading_change) > 45 and distance < min_radius:
            return False
        
        return True


@dataclass
class AircraftStateSpace:
    """State space for aircraft (lat, lon, alt, heading, speed, fuel)."""
    
    lat_bounds: Tuple[float, float] = (0, 90)
    lon_bounds: Tuple[float, float] = (-180, 180)
    alt_bounds: Tuple[float, float] = (0, 5000)
    speed_bounds: Tuple[float, float] = (10, 50)
    
    @property
    def dimension(self) -> int:
        return 6  # lat, lon, alt, heading, speed, fuel
    
    def validate_state(self, state: State) -> bool:
        """Check if state is within bounds."""
        lat, lon, alt = state.position[0], state.position[1], state.position[2]
        
        if not (self.lat_bounds[0] <= lat <= self.lat_bounds[1]):
            return False
        if not (self.lon_bounds[0] <= lon <= self.lon_bounds[1]):
            return False
        if not (self.alt_bounds[0] <= alt <= self.alt_bounds[1]):
            return False
        
        return True
