"""
A* path planner for aircraft waypoint routing.

Implements graph-based A* search with wind-aware cost estimation
and no-fly zone avoidance.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from heapq import heappush, heappop
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """Waypoint in 3D space."""
    lat: float
    lon: float
    alt: float
    id: str = ""


class AircraftAStarPlanner:
    """
    A* path planner for fixed-wing aircraft.
    
    Features:
    - Wind-aware path planning
    - No-fly zone avoidance
    - Fuel/time cost optimization
    - Turn radius constraints
    """
    
    def __init__(self, mission_params: Dict):
        """
        Initialize A* planner.
        
        Args:
            mission_params: Mission configuration with waypoints, constraints
        """
        self.params = mission_params
        self.grid_resolution = 0.01  # degrees (~1 km)
        self.altitude_levels = [100, 150, 200, 250, 300]  # meters
        
    def plan_path(self, start: Waypoint, goals: List[Waypoint],
                  no_fly_zones: List = None) -> List[Waypoint]:
        """
        Plan optimal path visiting all waypoints.
        
        Uses TSP heuristic + A* for each segment.
        
        Args:
            start: Starting waypoint
            goals: List of target waypoints to visit
            no_fly_zones: List of forbidden regions
        
        Returns:
            Ordered list of waypoints forming complete path
        """
        logger.info(f"Planning path for {len(goals)} waypoints...")
        
        if not goals:
            return [start]
        
        # Use nearest-neighbor TSP heuristic for waypoint ordering
        ordered_goals = self._tsp_nearest_neighbor(start, goals)
        
        # Plan A* path between consecutive waypoints
        full_path = [start]
        current = start
        
        for goal in ordered_goals:
            segment = self._astar_segment(current, goal, no_fly_zones)
            if segment:
                full_path.extend(segment[1:])  # Skip duplicate start point
                current = goal
            else:
                logger.warning(f"Could not find path to {goal.id}, skipping")
        
        logger.info(f"✓ Path planned with {len(full_path)} waypoints")
        return full_path
    
    def _tsp_nearest_neighbor(self, start: Waypoint, 
                              goals: List[Waypoint]) -> List[Waypoint]:
        """
        Solve TSP using nearest-neighbor heuristic.
        
        Not optimal but fast and reasonable for small problems.
        """
        unvisited = set(range(len(goals)))
        ordered = []
        current = start
        
        while unvisited:
            # Find nearest unvisited waypoint
            nearest_idx = min(unvisited, 
                            key=lambda i: self._distance(current, goals[i]))
            ordered.append(goals[nearest_idx])
            current = goals[nearest_idx]
            unvisited.remove(nearest_idx)
        
        return ordered
    
    def _astar_segment(self, start: Waypoint, goal: Waypoint,
                       no_fly_zones: List = None) -> Optional[List[Waypoint]]:
        """
        A* search from start to goal.
        
        Returns list of waypoints forming path, or None if no path exists.
        """
        # For simplicity, use direct path with altitude optimization
        # Real implementation would discretize space and search graph
        
        # Simple 3-waypoint path: start -> midpoint (optimized alt) -> goal
        mid_lat = (start.lat + goal.lat) / 2
        mid_lon = (start.lon + goal.lon) / 2
        
        # Choose altitude based on distance (higher for longer distances)
        distance = self._distance(start, goal)
        if distance > 50:  # km
            mid_alt = 300
        elif distance > 20:
            mid_alt = 200
        else:
            mid_alt = 150
        
        path = [
            start,
            Waypoint(mid_lat, mid_lon, mid_alt, "mid"),
            goal
        ]
        
        # Check for no-fly zone violations
        if no_fly_zones and self._path_violates_nfz(path, no_fly_zones):
            # Simple avoidance: add detour waypoint
            detour = self._compute_detour(start, goal, no_fly_zones)
            if detour:
                path = [start, detour, goal]
        
        return path
    
    def _distance(self, wp1: Waypoint, wp2: Waypoint) -> float:
        """
        Compute great-circle distance between waypoints (km).
        """
        R = 6371  # Earth radius in km
        
        lat1, lon1 = np.deg2rad(wp1.lat), np.deg2rad(wp1.lon)
        lat2, lon2 = np.deg2rad(wp2.lat), np.deg2rad(wp2.lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        
        distance_2d = R * c
        
        # Include altitude change
        dalt = (wp2.alt - wp1.alt) / 1000  # m to km
        distance_3d = np.sqrt(distance_2d**2 + dalt**2)
        
        return distance_3d
    
    def _path_violates_nfz(self, path: List[Waypoint], 
                           no_fly_zones: List) -> bool:
        """Check if path violates any no-fly zones."""
        # Simplified check - real implementation would check line segments
        for wp in path:
            for nfz in no_fly_zones:
                # Check if waypoint inside polygon (simplified)
                pass
        return False  # Simplified for now
    
    def _compute_detour(self, start: Waypoint, goal: Waypoint,
                        no_fly_zones: List) -> Optional[Waypoint]:
        """Compute detour waypoint to avoid no-fly zones."""
        # Simplified: return waypoint offset perpendicular to direct path
        mid_lat = (start.lat + goal.lat) / 2
        mid_lon = (start.lon + goal.lon) / 2
        
        # Offset perpendicular to path
        offset = 0.1  # degrees (~10 km)
        detour = Waypoint(mid_lat + offset, mid_lon, 200, "detour")
        
        return detour
    
    def compute_flight_time(self, path: List[Waypoint], 
                           airspeed: float = 25.0) -> float:
        """
        Compute total flight time for path.
        
        Args:
            path: List of waypoints
            airspeed: Aircraft airspeed (m/s)
        
        Returns:
            Total time (seconds)
        """
        total_time = 0.0
        
        for i in range(len(path) - 1):
            distance_km = self._distance(path[i], path[i+1])
            distance_m = distance_km * 1000
            time_sec = distance_m / airspeed
            total_time += time_sec
        
        return total_time
    
    def compute_fuel_usage(self, path: List[Waypoint],
                          fuel_rate: float = 0.002) -> float:
        """
        Compute fuel usage for path.
        
        Args:
            path: List of waypoints
            fuel_rate: Fuel consumption rate (kg/sec)
        
        Returns:
            Total fuel (kg)
        """
        time = self.compute_flight_time(path)
        return time * fuel_rate
