"""
Celestial ephemeris calculations for sun and moon positions.

Implements Meeus analytical algorithms for high accuracy without
requiring large ephemeris files.
"""

import numpy as np
from datetime import datetime, timezone
from typing import Tuple
import logging

logger = logging.getLogger(__name__)


class SunEphemeris:
    """
    Analytical sun position calculator.
    
    Implements Meeus "Astronomical Algorithms" (1998) Chapter 25.
    Accuracy: 0.01° over 100 years.
    
    Much faster than numerical integration and doesn't require
    large ephemeris files (JPL DE440).
    """
    
    # Astronomical constants
    AU_TO_KM = 149597870.7  # 1 AU in kilometers
    
    def __init__(self):
        """Initialize sun ephemeris calculator."""
        pass
    
    def get_position_eci(self, time_utc: datetime) -> np.ndarray:
        """
        Compute sun position vector in ECI (J2000) coordinates.
        
        Args:
            time_utc: UTC time (datetime object)
        
        Returns:
            r_sun: Position vector [x, y, z] in km (ECI frame)
        
        Example:
            >>> sun = SunEphemeris()
            >>> time = datetime(2025, 6, 21, 12, 0, 0, tzinfo=timezone.utc)
            >>> r_sun = sun.get_position_eci(time)
            >>> print(f"Sun distance: {np.linalg.norm(r_sun):.0f} km")
        """
        # Convert to Julian Date
        jd = self._datetime_to_jd(time_utc)
        
        # Julian centuries from J2000.0
        T = (jd - 2451545.0) / 36525.0
        
        # Mean longitude of the sun (degrees)
        L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T**2
        L0 = self._normalize_angle(L0)
        
        # Mean anomaly (degrees)
        M = 357.52911 + 35999.05029 * T - 0.0001537 * T**2
        M = self._normalize_angle(M)
        M_rad = np.deg2rad(M)
        
        # Equation of center (degrees)
        C = ((1.914602 - 0.004817 * T - 0.000014 * T**2) * np.sin(M_rad) +
             (0.019993 - 0.000101 * T) * np.sin(2 * M_rad) +
             0.000289 * np.sin(3 * M_rad))
        
        # True longitude (degrees)
        true_longitude = L0 + C
        true_longitude_rad = np.deg2rad(true_longitude)
        
        # Distance to sun (AU)
        # Using eccentricity of Earth's orbit
        e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T**2
        nu = M + C  # True anomaly
        nu_rad = np.deg2rad(nu)
        
        R_au = (1.000001018 * (1 - e**2)) / (1 + e * np.cos(nu_rad))
        R_km = R_au * self.AU_TO_KM
        
        # Position in ecliptic coordinates
        x_ecl = R_km * np.cos(true_longitude_rad)
        y_ecl = R_km * np.sin(true_longitude_rad)
        z_ecl = 0  # Sun in ecliptic plane by definition
        
        # Convert ecliptic to equatorial (ECI) coordinates
        # Account for obliquity of ecliptic (tilt of Earth's axis)
        epsilon = self._obliquity(T)
        epsilon_rad = np.deg2rad(epsilon)
        
        # Rotation matrix from ecliptic to equatorial
        x_eq = x_ecl
        y_eq = y_ecl * np.cos(epsilon_rad) - z_ecl * np.sin(epsilon_rad)
        z_eq = y_ecl * np.sin(epsilon_rad) + z_ecl * np.cos(epsilon_rad)
        
        return np.array([x_eq, y_eq, z_eq])
    
    def is_in_eclipse(self, r_sat_eci: np.ndarray, r_sun_eci: np.ndarray,
                     r_earth: float = 6378.137) -> bool:
        """
        Determine if satellite is in Earth's shadow (eclipse).
        
        Uses cylindrical shadow model (acceptable for LEO).
        
        Args:
            r_sat_eci: Satellite position in ECI (km)
            r_sun_eci: Sun position in ECI (km)
            r_earth: Earth radius (km, default: 6378.137)
        
        Returns:
            True if satellite is in eclipse, False otherwise
        """
        # Vector from sun to satellite
        r_sun_to_sat = r_sat_eci - r_sun_eci
        
        # Sun direction (unit vector)
        sun_dir = -r_sun_eci / np.linalg.norm(r_sun_eci)
        
        # Check if satellite is on night side
        if np.dot(r_sat_eci, sun_dir) < 0:
            # Satellite might be in shadow
            # Compute perpendicular distance from sun-earth line
            projection = np.dot(r_sat_eci, sun_dir) * sun_dir
            perpendicular = r_sat_eci - projection
            dist_from_axis = np.linalg.norm(perpendicular)
            
            # In shadow if closer to axis than Earth radius
            if dist_from_axis < r_earth:
                return True
        
        return False
    
    def compute_beta_angle(self, r_eci: np.ndarray, v_eci: np.ndarray,
                          time_utc: datetime) -> float:
        """
        Compute beta angle (angle between orbit plane and sun vector).
        
        High beta → more sunlight → more solar power
        Low beta → frequent eclipses → power-constrained
        
        Args:
            r_eci: Satellite position (km)
            v_eci: Satellite velocity (km/s)
            time_utc: UTC time
        
        Returns:
            beta: Beta angle (degrees, -90 to 90)
        """
        # Orbit normal vector
        h = np.cross(r_eci, v_eci)  # Angular momentum
        h_unit = h / np.linalg.norm(h)
        
        # Sun direction
        r_sun = self.get_position_eci(time_utc)
        sun_unit = r_sun / np.linalg.norm(r_sun)
        
        # Beta angle is 90° minus angle between orbit normal and sun
        sin_beta = np.dot(h_unit, sun_unit)
        beta = np.rad2deg(np.arcsin(sin_beta))
        
        return beta
    
    @staticmethod
    def _obliquity(T: float) -> float:
        """
        Compute obliquity of ecliptic (tilt of Earth's axis).
        
        Args:
            T: Julian centuries from J2000.0
        
        Returns:
            epsilon: Obliquity (degrees)
        """
        # IAU formula (accurate to 0.01 arcsec over centuries)
        epsilon = 23.439291 - 0.0130042 * T - 1.64e-7 * T**2 + 5.04e-7 * T**3
        return epsilon
    
    @staticmethod
    def _datetime_to_jd(time_utc: datetime) -> float:
        """
        Convert UTC datetime to Julian Date.
        
        Args:
            time_utc: UTC time (datetime object)
        
        Returns:
            jd: Julian Date
        
        Reference: Meeus, "Astronomical Algorithms" Chapter 7
        """
        year = time_utc.year
        month = time_utc.month
        day = time_utc.day
        
        # Adjust for January/February
        if month <= 2:
            year -= 1
            month += 12
        
        # Gregorian calendar correction
        A = int(year / 100)
        B = 2 - A + int(A / 4)
        
        # Julian Day Number at 0h
        jd_0h = (int(365.25 * (year + 4716)) +
                int(30.6001 * (month + 1)) +
                day + B - 1524.5)
        
        # Add time of day (fraction of day)
        day_fraction = (time_utc.hour +
                       time_utc.minute / 60.0 +
                       time_utc.second / 3600.0 +
                       time_utc.microsecond / 3600.0e6) / 24.0
        
        jd = jd_0h + day_fraction
        
        return jd
    
    @staticmethod
    def _normalize_angle(angle_deg: float) -> float:
        """Normalize angle to [0, 360) degrees."""
        return angle_deg % 360.0


class MoonEphemeris:
    """
    Analytical moon position calculator.
    
    Simplified lunar ephemeris for basic calculations.
    Accuracy: ~1° (sufficient for general orbit analysis).
    """
    
    AU_TO_KM = 149597870.7
    MOON_MEAN_DISTANCE = 384400.0  # km
    
    def get_position_eci(self, time_utc: datetime) -> np.ndarray:
        """
        Compute moon position vector in ECI coordinates.
        
        Args:
            time_utc: UTC time
        
        Returns:
            r_moon: Position vector [x, y, z] in km (ECI frame)
        
        Note: This is a simplified model. For high-precision lunar
              calculations, use JPL ephemeris.
        """
        jd = SunEphemeris._datetime_to_jd(time_utc)
        T = (jd - 2451545.0) / 36525.0
        
        # Mean longitude
        L_prime = 218.316 + 481267.881 * T
        L_prime = L_prime % 360.0
        L_prime_rad = np.deg2rad(L_prime)
        
        # Mean anomaly
        M_prime = 134.963 + 477198.868 * T
        M_prime = M_prime % 360.0
        M_prime_rad = np.deg2rad(M_prime)
        
        # Mean distance
        F = 93.272 + 483202.019 * T
        F = F % 360.0
        F_rad = np.deg2rad(F)
        
        # Ecliptic longitude (simplified)
        lambda_moon = L_prime + 6.289 * np.sin(M_prime_rad)
        lambda_rad = np.deg2rad(lambda_moon)
        
        # Ecliptic latitude (simplified)
        beta = 5.128 * np.sin(F_rad)
        beta_rad = np.deg2rad(beta)
        
        # Distance (simplified)
        r_moon = self.MOON_MEAN_DISTANCE * (1 - 0.0549 * np.cos(M_prime_rad))
        
        # Position in ecliptic coordinates
        x_ecl = r_moon * np.cos(beta_rad) * np.cos(lambda_rad)
        y_ecl = r_moon * np.cos(beta_rad) * np.sin(lambda_rad)
        z_ecl = r_moon * np.sin(beta_rad)
        
        # Convert to equatorial (ECI)
        epsilon = SunEphemeris._obliquity(T)
        epsilon_rad = np.deg2rad(epsilon)
        
        x_eq = x_ecl
        y_eq = y_ecl * np.cos(epsilon_rad) - z_ecl * np.sin(epsilon_rad)
        z_eq = y_ecl * np.sin(epsilon_rad) + z_ecl * np.cos(epsilon_rad)
        
        return np.array([x_eq, y_eq, z_eq])


# Convenience functions
def get_sun_position(time_utc: datetime) -> np.ndarray:
    """
    Get sun position in ECI coordinates.
    
    Args:
        time_utc: UTC time
    
    Returns:
        r_sun: Position (km) in ECI frame
    """
    sun = SunEphemeris()
    return sun.get_position_eci(time_utc)


def check_eclipse(r_sat_eci: np.ndarray, time_utc: datetime) -> bool:
    """
    Check if satellite is in Earth's shadow.
    
    Args:
        r_sat_eci: Satellite position in ECI (km)
        time_utc: UTC time
    
    Returns:
        True if in eclipse, False otherwise
    """
    sun = SunEphemeris()
    r_sun = sun.get_position_eci(time_utc)
    return sun.is_in_eclipse(r_sat_eci, r_sun)
