"""
Spacecraft domain implementation.

Includes orbital mechanics, power management, and attitude dynamics.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, List
import numpy as np
from datetime import datetime, timedelta

from ..core.models import State, Vehicle, Waypoint
from ..core.simulator import DynamicsSimulator, RK4Integrator


@dataclass
class SpacecraftControl:
    """Control input for spacecraft."""
    torque: np.ndarray  # N*m, body frame
    power_mode: str = 'idle'  # 'idle', 'observation', 'downlink'
    duration: float = 1.0  # seconds
    slew_angle: float = 0.0  # degrees


@dataclass
class GroundStation:
    """Ground station for communications."""
    id: str
    name: str
    latitude: float  # degrees
    longitude: float  # degrees
    min_elevation: float = 10.0  # degrees
    max_data_rate: float = 2.0  # Mbps


@dataclass
class GroundTarget:
    """Ground target for observation."""
    id: str
    name: str
    latitude: float  # degrees
    longitude: float  # degrees
    priority: int = 1
    science_value: float = 100.0
    min_revisit_time: float = 12 * 3600  # seconds


class OrbitalMechanics:
    """
    Orbital mechanics calculations.
    
    Implements two-body dynamics with J2 perturbation.
    """
    
    # Earth parameters
    MU = 398600.4418  # km^3/s^2, gravitational parameter
    R_E = 6378.137  # km, Earth radius
    J2 = 1.08263e-3  # J2 zonal harmonic
    OMEGA_E = 7.2921159e-5  # rad/s, Earth rotation rate
    
    @staticmethod
    def two_body_acceleration(r: np.ndarray) -> np.ndarray:
        """
        Compute two-body gravitational acceleration.
        
        Args:
            r: Position vector in ECI (km)
        
        Returns:
            Acceleration vector (km/s^2)
        """
        r_norm = np.linalg.norm(r)
        return -OrbitalMechanics.MU / r_norm**3 * r
    
    @staticmethod
    def j2_acceleration(r: np.ndarray) -> np.ndarray:
        """
        Compute J2 perturbation acceleration.
        
        Args:
            r: Position vector in ECI (km)
        
        Returns:
            Perturbation acceleration (km/s^2)
        """
        r_norm = np.linalg.norm(r)
        x, y, z = r[0], r[1], r[2]
        
        factor = (3/2) * OrbitalMechanics.J2 * (OrbitalMechanics.MU / r_norm**2) * \
                 (OrbitalMechanics.R_E / r_norm)**2
        
        a_x = factor * x/r_norm * (5*(z/r_norm)**2 - 1)
        a_y = factor * y/r_norm * (5*(z/r_norm)**2 - 1)
        a_z = factor * z/r_norm * (5*(z/r_norm)**2 - 3)
        
        return np.array([a_x, a_y, a_z])
    
    @staticmethod
    def atmospheric_drag_acceleration(r: np.ndarray, v: np.ndarray, 
                                     C_d: float, A: float, mass: float) -> np.ndarray:
        """
        Compute atmospheric drag acceleration (simplified).
        
        Args:
            r: Position vector in ECI (km)
            v: Velocity vector in ECI (km/s)
            C_d: Drag coefficient
            A: Cross-sectional area (m^2)
            mass: Spacecraft mass (kg)
        
        Returns:
            Drag acceleration (km/s^2)
        """
        altitude = np.linalg.norm(r) - OrbitalMechanics.R_E  # km
        
        # Simple exponential atmosphere
        rho_0 = 1.225e-9  # kg/km^3 at sea level (converted)
        H = 8.5  # km, scale height
        rho = rho_0 * np.exp(-altitude / H)
        
        # Relative velocity (ignoring Earth rotation for simplicity)
        v_rel = v
        v_rel_norm = np.linalg.norm(v_rel)
        
        if v_rel_norm < 1e-6:
            return np.zeros(3)
        
        # Drag force: F = -0.5 * C_d * A * rho * v^2 * v_hat
        # Convert A from m^2 to km^2
        A_km2 = A * 1e-6
        drag_accel = -0.5 * (C_d * A_km2 / mass) * rho * v_rel_norm * v_rel
        
        return drag_accel
    
    @staticmethod
    def eci_to_ecef(r_eci: np.ndarray, t: float) -> np.ndarray:
        """
        Convert ECI to ECEF coordinates.
        
        Args:
            r_eci: Position in ECI (km)
            t: Time since epoch (seconds)
        
        Returns:
            Position in ECEF (km)
        """
        theta = OrbitalMechanics.OMEGA_E * t  # Earth rotation angle
        
        # Rotation matrix (Z-axis rotation)
        R = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        
        return R @ r_eci
    
    @staticmethod
    def ecef_to_lla(r_ecef: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert ECEF to latitude/longitude/altitude.
        
        Args:
            r_ecef: Position in ECEF (km)
        
        Returns:
            (latitude, longitude, altitude) in degrees and km
        """
        x, y, z = r_ecef
        
        # Longitude
        lon = np.arctan2(y, x)
        
        # Latitude (iterative solution)
        p = np.sqrt(x**2 + y**2)
        lat = np.arctan2(z, p)
        
        # Altitude (simplified)
        alt = np.linalg.norm(r_ecef) - OrbitalMechanics.R_E
        
        return np.rad2deg(lat), np.rad2deg(lon), alt
    
    @staticmethod
    def compute_elevation_angle(sat_eci: np.ndarray, ground_lat: float, 
                               ground_lon: float, t: float) -> float:
        """
        Compute elevation angle from ground station to satellite.
        
        Args:
            sat_eci: Satellite position in ECI (km)
            ground_lat: Ground station latitude (degrees)
            ground_lon: Ground station longitude (degrees)
            t: Time since epoch (seconds)
        
        Returns:
            Elevation angle (degrees)
        """
        # Convert satellite to ECEF
        sat_ecef = OrbitalMechanics.eci_to_ecef(sat_eci, t)
        
        # Ground station in ECEF
        lat_rad = np.deg2rad(ground_lat)
        lon_rad = np.deg2rad(ground_lon)
        
        gs_ecef = OrbitalMechanics.R_E * np.array([
            np.cos(lat_rad) * np.cos(lon_rad),
            np.cos(lat_rad) * np.sin(lon_rad),
            np.sin(lat_rad)
        ])
        
        # Vector from ground station to satellite
        range_vec = sat_ecef - gs_ecef
        range_norm = np.linalg.norm(range_vec)
        
        if range_norm < 1e-6:
            return 90.0
        
        # Local vertical at ground station
        vertical = gs_ecef / np.linalg.norm(gs_ecef)
        
        # Elevation angle
        sin_el = np.dot(range_vec, vertical) / range_norm
        elevation = np.rad2deg(np.arcsin(np.clip(sin_el, -1, 1)))
        
        return elevation


class SpacecraftDynamics(DynamicsSimulator):
    """
    Spacecraft dynamics simulator.
    
    Includes:
    - Orbital propagation (two-body + J2 + drag)
    - Attitude dynamics (simplified)
    - Power budget (solar + battery)
    - Thermal management (duty cycle)
    """
    
    def __init__(self, vehicle: Vehicle, epoch: datetime = None):
        """
        Initialize spacecraft dynamics.
        
        Args:
            vehicle: Spacecraft vehicle specification
            epoch: Mission epoch (for time calculations)
        """
        self.vehicle = vehicle
        self.epoch = epoch or datetime(2025, 1, 1)
        
        # Orbital parameters
        self.C_d = vehicle.drag_coefficient
        self.A = vehicle.cross_sectional_area  # m^2
        
        # Attitude parameters
        self.I_xx = vehicle.properties.get('I_xx', 0.01)  # kg*m^2
        self.I_yy = vehicle.properties.get('I_yy', 0.01)
        self.I_zz = vehicle.properties.get('I_zz', 0.015)
        self.max_slew_rate = vehicle.max_slew_rate or 5.0  # deg/s
        self.min_slew_time = vehicle.properties.get('min_slew_time', 10.0)  # s
        
        # Power parameters
        self.A_solar = vehicle.properties.get('solar_panel_area', 0.03)  # m^2
        self.eta_solar = vehicle.properties.get('solar_efficiency', 0.28)
        self.battery_capacity = vehicle.battery_capacity or 20.0  # Wh
        self.min_SOC = vehicle.properties.get('min_SOC', 0.2)
        
        # Payload power
        self.P_camera = vehicle.properties.get('camera_power', 5.0)  # W
        self.P_transmitter = vehicle.properties.get('transmitter_power', 8.0)  # W
        self.P_avionics = vehicle.properties.get('avionics_power', 2.0)  # W
        
        # Solar constant
        self.SOLAR_CONSTANT = 1361  # W/m^2
    
    def propagate(self, state: State, control: SpacecraftControl, dt: float) -> State:
        """
        Propagate spacecraft state forward by dt.
        
        Uses two-body + J2 + drag orbital dynamics.
        """
        # Extract orbital state
        r = state.position_ECI.copy() if state.position_ECI is not None else np.zeros(3)
        v = state.velocity_ECI.copy() if state.velocity_ECI is not None else np.zeros(3)
        
        # Compute accelerations
        a_2body = OrbitalMechanics.two_body_acceleration(r)
        a_j2 = OrbitalMechanics.j2_acceleration(r)
        a_drag = OrbitalMechanics.atmospheric_drag_acceleration(
            r, v, self.C_d, self.A, self.vehicle.mass
        )
        
        a_total = a_2body + a_j2 + a_drag
        
        # Integrate using simple Euler (RK4 would be better for production)
        new_v = v + a_total * dt
        new_r = r + v * dt + 0.5 * a_total * dt**2
        
        # Attitude propagation (simplified: assume instant slew for now)
        q = state.quaternion.copy() if state.quaternion is not None else np.array([1, 0, 0, 0])
        omega = state.angular_velocity.copy() if state.angular_velocity is not None else np.zeros(3)
        
        # For simplicity: attitude assumed to track nadir pointing
        # Production code would integrate quaternion kinematics
        
        # Power budget calculation
        in_eclipse = self.is_in_eclipse(new_r, state.time + dt)
        
        if in_eclipse:
            P_solar = 0.0
        else:
            # Compute sun angle (simplified: assume panels normal to sun)
            sun_angle_factor = 0.7  # Average over orbit
            P_solar = self.eta_solar * self.A_solar * self.SOLAR_CONSTANT * sun_angle_factor
        
        # Power consumption based on mode
        if control.power_mode == 'observation':
            P_out = self.P_camera + self.P_avionics
        elif control.power_mode == 'downlink':
            P_out = self.P_transmitter + self.P_avionics
        else:  # idle
            P_out = self.P_avionics
        
        # Battery update
        P_net = P_solar - P_out
        E_change = P_net * dt / 3600  # Wh
        
        current_SOC = state.battery_SOC if state.battery_SOC is not None else 0.8
        new_SOC = current_SOC + E_change / self.battery_capacity
        new_SOC = np.clip(new_SOC, 0.0, 1.0)
        
        # Data storage update
        current_storage = state.data_storage if state.data_storage is not None else 0.0
        
        if control.power_mode == 'observation':
            # Generate data
            data_rate = 10.0  # Mbps (from camera)
            data_generated = data_rate * dt / 8  # MB
            new_storage = current_storage + data_generated
        elif control.power_mode == 'downlink':
            # Downlink data
            downlink_rate = 2.0  # Mbps
            data_downlinked = downlink_rate * dt / 8  # MB
            new_storage = max(0, current_storage - data_downlinked)
        else:
            new_storage = current_storage
        
        # Create new state
        new_state = State(
            time=state.time + dt,
            position=new_r[:3],  # Store in position for compatibility
            velocity=new_v[:3],
            position_ECI=new_r,
            velocity_ECI=new_v,
            quaternion=q,
            angular_velocity=omega,
            battery_SOC=new_SOC,
            data_storage=new_storage,
            resources=state.resources.copy(),
            metadata=state.metadata.copy()
        )
        
        return new_state
    
    def is_in_eclipse(self, r_eci: np.ndarray, t: float) -> bool:
        """
        Determine if spacecraft is in Earth's shadow (simplified).
        
        Args:
            r_eci: Spacecraft position in ECI (km)
            t: Time since epoch (seconds)
        
        Returns:
            True if in eclipse
        """
        # Simplified cylindrical shadow model
        # Sun direction (simplified: assume sun in +X direction for now)
        # Production code would use proper ephemeris
        sun_dir = np.array([1, 0, 0])
        
        # Project spacecraft onto plane perpendicular to sun
        r_perp = r_eci - np.dot(r_eci, sun_dir) * sun_dir
        r_perp_norm = np.linalg.norm(r_perp)
        
        # Check if behind Earth and within shadow cylinder
        if np.dot(r_eci, sun_dir) < 0 and r_perp_norm < OrbitalMechanics.R_E:
            return True
        
        return False
    
    def compute_ground_track(self, r_eci: np.ndarray, t: float) -> Tuple[float, float, float]:
        """
        Compute ground track (lat/lon/alt) of spacecraft.
        
        Args:
            r_eci: Position in ECI (km)
            t: Time since epoch (seconds)
        
        Returns:
            (latitude, longitude, altitude) in degrees and km
        """
        r_ecef = OrbitalMechanics.eci_to_ecef(r_eci, t)
        return OrbitalMechanics.ecef_to_lla(r_ecef)
    
    def compute_access_windows(self, orbit_states: List[State], 
                              ground_station: GroundStation) -> List[Tuple[float, float, float]]:
        """
        Compute access windows to a ground station.
        
        Args:
            orbit_states: List of states over time
            ground_station: Ground station specification
        
        Returns:
            List of (AOS_time, LOS_time, max_elevation) tuples
        """
        access_windows = []
        in_pass = False
        pass_start = 0.0
        max_el = 0.0
        
        for state in orbit_states:
            if state.position_ECI is None:
                continue
            
            elevation = OrbitalMechanics.compute_elevation_angle(
                state.position_ECI, 
                ground_station.latitude,
                ground_station.longitude,
                state.time
            )
            
            if elevation >= ground_station.min_elevation:
                if not in_pass:
                    # AOS (Acquisition of Signal)
                    in_pass = True
                    pass_start = state.time
                    max_el = elevation
                else:
                    # Update max elevation
                    max_el = max(max_el, elevation)
            else:
                if in_pass:
                    # LOS (Loss of Signal)
                    in_pass = False
                    access_windows.append((pass_start, state.time, max_el))
        
        return access_windows
    
    def get_energy_cost(self, state: State, control: SpacecraftControl, dt: float) -> float:
        """Compute energy cost for control action."""
        if control.power_mode == 'observation':
            power = self.P_camera + self.P_avionics
        elif control.power_mode == 'downlink':
            power = self.P_transmitter + self.P_avionics
        else:
            power = self.P_avionics
        
        return power * dt / 3600  # Wh


def create_circular_orbit(altitude: float, inclination: float, 
                         RAAN: float = 0.0, true_anomaly: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Create initial state for circular orbit.
    
    Args:
        altitude: Orbit altitude above Earth (km)
        inclination: Orbit inclination (degrees)
        RAAN: Right Ascension of Ascending Node (degrees)
        true_anomaly: Initial true anomaly (degrees)
    
    Returns:
        (position_ECI, velocity_ECI) in km and km/s
    """
    # Orbital radius
    r = OrbitalMechanics.R_E + altitude
    
    # Circular orbital velocity
    v_mag = np.sqrt(OrbitalMechanics.MU / r)
    
    # Convert angles to radians
    inc_rad = np.deg2rad(inclination)
    raan_rad = np.deg2rad(RAAN)
    ta_rad = np.deg2rad(true_anomaly)
    
    # Position in orbital plane
    r_orbital = r * np.array([np.cos(ta_rad), np.sin(ta_rad), 0])
    
    # Velocity in orbital plane (perpendicular to position)
    v_orbital = v_mag * np.array([-np.sin(ta_rad), np.cos(ta_rad), 0])
    
    # Rotation matrices
    R_inc = np.array([
        [1, 0, 0],
        [0, np.cos(inc_rad), -np.sin(inc_rad)],
        [0, np.sin(inc_rad), np.cos(inc_rad)]
    ])
    
    R_raan = np.array([
        [np.cos(raan_rad), -np.sin(raan_rad), 0],
        [np.sin(raan_rad), np.cos(raan_rad), 0],
        [0, 0, 1]
    ])
    
    # Transform to ECI
    r_eci = R_raan @ R_inc @ r_orbital
    v_eci = R_raan @ R_inc @ v_orbital
    
    return r_eci, v_eci
