"""
Real orbital visibility calculator using actual orbital mechanics.

This module computes target visibility and ground station contact windows
using proper orbital propagation and elevation angle calculations.
"""

import numpy as np
from datetime import datetime, timedelta, timezone
from typing import List, Dict, Tuple
import logging

logger = logging.getLogger(__name__)


class OrbitVisibilityCalculator:
    """
    Compute visibility windows using real orbital mechanics.
    
    Uses simplified two-body orbital propagation with elevation angle
    calculations for target visibility.
    """
    
    def __init__(self):
        # Earth parameters
        self.R_earth = 6378.137  # km
        self.mu = 398600.4418  # km^3/s^2 (gravitational parameter)
        self.omega_earth = 7.2921159e-5  # rad/s (Earth rotation rate)
    
    def compute_target_windows(self, target_lat: float, target_lon: float,
                               orbit_altitude: float, orbit_inclination: float,
                               duration_days: int = 7, min_elevation: float = 10.0,
                               timestep: float = 10.0) -> List[Dict]:
        """
        Compute visibility windows for a ground target.
        
        Args:
            target_lat: Target latitude (degrees)
            target_lon: Target longitude (degrees)
            orbit_altitude: Orbital altitude (km)
            orbit_inclination: Orbital inclination (degrees)
            duration_days: Mission duration (days)
            min_elevation: Minimum elevation angle (degrees)
            timestep: Propagation timestep (seconds)
        
        Returns:
            List of visibility windows with start, end, max elevation, duration
        """
        windows = []
        
        # Orbital parameters
        a = self.R_earth + orbit_altitude  # Semi-major axis
        n = np.sqrt(self.mu / a**3)  # Mean motion (rad/s)
        period = 2 * np.pi / n  # Orbital period (seconds)
        
        # Convert inclination to radians
        inc_rad = np.deg2rad(orbit_inclination)
        
        # Target position in ECEF
        target_ecef = self._lla_to_ecef(target_lat, target_lon, 0.0)
        
        # Simulate orbit
        total_seconds = duration_days * 86400
        num_steps = int(total_seconds / timestep)
        
        current_time = datetime.now(timezone.utc)
        in_pass = False
        pass_start = None
        pass_elevations = []
        
        for step in range(num_steps):
            t = step * timestep
            
            # Simple circular orbit propagation
            # Mean anomaly
            M = n * t
            
            # For circular orbit, E = M
            theta = M  # True anomaly
            
            # Position in orbital plane
            r = a
            x_orbit = r * np.cos(theta)
            y_orbit = r * np.sin(theta)
            z_orbit = 0.0
            
            # Rotate to ECI (simplified - just inclination rotation)
            x_eci = x_orbit
            y_eci = y_orbit * np.cos(inc_rad)
            z_eci = y_orbit * np.sin(inc_rad)
            
            sat_eci = np.array([x_eci, y_eci, z_eci])
            
            # Convert to ECEF (account for Earth rotation)
            theta_earth = self.omega_earth * t
            sat_ecef = self._eci_to_ecef(sat_eci, theta_earth)
            
            # Compute elevation angle
            elevation = self._compute_elevation(sat_ecef, target_ecef)
            
            # Detect pass start/end
            if elevation >= min_elevation:
                if not in_pass:
                    # Pass started
                    in_pass = True
                    pass_start = current_time + timedelta(seconds=t)
                    pass_elevations = [elevation]
                else:
                    pass_elevations.append(elevation)
            else:
                if in_pass:
                    # Pass ended
                    in_pass = False
                    pass_end = current_time + timedelta(seconds=t)
                    duration_sec = (pass_end - pass_start).total_seconds()
                    
                    windows.append({
                        'start_time': pass_start,
                        'end_time': pass_end,
                        'max_elevation': max(pass_elevations),
                        'duration': duration_sec
                    })
        
        # Close final pass if still in one
        if in_pass:
            pass_end = current_time + timedelta(seconds=total_seconds)
            duration_sec = (pass_end - pass_start).total_seconds()
            windows.append({
                'start_time': pass_start,
                'end_time': pass_end,
                'max_elevation': max(pass_elevations),
                'duration': duration_sec
            })
        
        logger.info(f"  Computed {len(windows)} visibility windows for target at ({target_lat:.2f}, {target_lon:.2f})")
        
        return windows
    
    def _lla_to_ecef(self, lat: float, lon: float, alt: float) -> np.ndarray:
        """Convert latitude/longitude/altitude to ECEF."""
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)
        
        N = self.R_earth  # For sphere
        
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N + alt) * np.sin(lat_rad)
        
        return np.array([x, y, z])
    
    def _eci_to_ecef(self, r_eci: np.ndarray, theta: float) -> np.ndarray:
        """Convert ECI to ECEF with Earth rotation."""
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        R = np.array([
            [cos_theta, sin_theta, 0],
            [-sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        
        return R @ r_eci
    
    def _compute_elevation(self, sat_ecef: np.ndarray, target_ecef: np.ndarray) -> float:
        """
        Compute elevation angle from target to satellite.
        
        Returns elevation in degrees.
        """
        # Vector from target to satellite
        range_vec = sat_ecef - target_ecef
        range_mag = np.linalg.norm(range_vec)
        
        # Local up direction at target (radial from Earth center)
        up = target_ecef / np.linalg.norm(target_ecef)
        
        # Elevation angle
        sin_elev = np.dot(range_vec, up) / range_mag
        elevation_rad = np.arcsin(np.clip(sin_elev, -1.0, 1.0))
        
        return np.rad2deg(elevation_rad)
