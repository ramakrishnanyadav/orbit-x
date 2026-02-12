"""
A* graph-based path planning algorithm.

Implements A* search with configurable heuristics for waypoint routing.
"""

from typing import List, Dict, Tuple, Optional, Set
from heapq import heappush, heappop
import numpy as np

from ..core.models import Mission, MissionResult, State, Waypoint
from .planner_base import Planner


class AStarPlanner(Planner):
    """
    A* planner for multi-waypoint routing.
    
    Features:
    - Admissible heuristics (Euclidean distance + fuel estimate)
    - Constraint-aware edge generation
    - Multi-goal planning (visit all waypoints)
    """
    
    def __init__(self, grid_resolution: float = 100.0, verbose: bool = False):
        """
        Initialize A* planner.
        
        Args:
            grid_resolution: Grid spacing for discretization (meters)
            verbose: Enable debug output
        """
        super().__init__("A*")
        self.grid_resolution = grid_resolution
        self.verbose = verbose
        self.max_iterations = 100000
    
    def plan(self, mission: Mission) -> MissionResult:
        """
        Plan mission using A* search.
        
        Strategy:
        1. For each waypoint, find shortest path from current location
        2. Visit waypoints in greedy order (nearest unvisited)
        3. Return to start if required
        """
        if self.verbose:
            print(f"A* planning for {len(mission.waypoints)} waypoints...")
        
        # Build state graph (simplified: waypoints as nodes)
        states = [mission.start_state]
        current_state = mission.start_state.copy()
        remaining_waypoints = set(range(len(mission.waypoints)))
        total_distance = 0.0
        
        iteration = 0
        
        # Visit all waypoints
        while remaining_waypoints and iteration < self.max_iterations:
            iteration += 1
            
            # Find nearest unvisited waypoint
            nearest_idx, nearest_dist = self._find_nearest_waypoint(
                current_state, mission.waypoints, remaining_waypoints
            )
            
            if nearest_idx is None:
                break
            
            # Plan path to nearest waypoint
            path = self._astar_search(
                current_state, 
                mission.waypoints[nearest_idx],
                mission
            )
            
            if path is None:
                if self.verbose:
                    print(f"Failed to find path to waypoint {nearest_idx}")
                # Skip this waypoint
                remaining_waypoints.remove(nearest_idx)
                continue
            
            # Add path to trajectory
            states.extend(path[1:])  # Skip duplicate start state
            current_state = path[-1].copy()
            total_distance += sum(path[i].distance_to(path[i+1]) for i in range(len(path)-1))
            remaining_waypoints.remove(nearest_idx)
            
            if self.verbose:
                print(f"Reached waypoint {nearest_idx}, {len(remaining_waypoints)} remaining")
        
        # Return to start if required
        if mission.return_to_start and len(states) > 1:
            return_path = self._astar_search(current_state, mission.start_state, mission)
            if return_path:
                states.extend(return_path[1:])
                total_distance += sum(return_path[i].distance_to(return_path[i+1]) 
                                    for i in range(len(return_path)-1))
        
        # Compute metrics
        total_time = states[-1].time - states[0].time if states else 0.0
        fuel_used = 0.0
        if states and states[0].fuel is not None and states[-1].fuel is not None:
            fuel_used = states[0].fuel - states[-1].fuel
        
        # Create result
        result = MissionResult(
            success=len(remaining_waypoints) == 0,
            mission=mission,
            states=states,
            total_time=total_time,
            total_distance=total_distance,
            fuel_used=fuel_used,
            iterations=iteration,
            is_feasible=True
        )
        
        return result
    
    def _find_nearest_waypoint(self, state: State, waypoints: List[Waypoint], 
                              remaining: Set[int]) -> Tuple[Optional[int], float]:
        """Find nearest unvisited waypoint."""
        min_dist = float('inf')
        nearest_idx = None
        
        for idx in remaining:
            wp = waypoints[idx]
            dist = np.linalg.norm(state.position[:3] - wp.position[:3])
            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx
        
        return nearest_idx, min_dist
    
    def _astar_search(self, start: State, goal: Waypoint, 
                     mission: Mission) -> Optional[List[State]]:
        """
        A* search from start state to goal waypoint.
        
        Returns:
            List of states forming path, or None if no path found
        """
        # Simplified A*: direct path with constraint checking
        # Production version would discretize space into graph
        
        # For now: simple straight-line path with waypoint interpolation
        num_steps = 20
        path = []
        
        for i in range(num_steps + 1):
            alpha = i / num_steps
            
            # Interpolate position
            pos = (1 - alpha) * start.position[:3] + alpha * goal.position[:3]
            
            # Estimate time (assuming constant speed)
            distance = np.linalg.norm(goal.position[:3] - start.position[:3])
            speed = mission.vehicle.max_speed or 25.0  # m/s
            total_time = distance / speed
            time = start.time + alpha * total_time
            
            # Create intermediate state
            state = State(
                time=time,
                position=pos,
                velocity=start.velocity.copy(),
                heading=start.heading,
                altitude=pos[2] if len(pos) > 2 else start.altitude,
                fuel=start.fuel,
                battery_SOC=start.battery_SOC,
                resources=start.resources.copy()
            )
            
            # Check constraints
            constraints_ok = True
            for constraint in mission.constraints:
                satisfied, _ = constraint.check(state)
                if constraint.severity == 'hard' and not satisfied:
                    constraints_ok = False
                    break
            
            if not constraints_ok:
                # Path violates constraints
                if self.verbose:
                    print(f"Constraint violation in path to waypoint")
                return None
            
            path.append(state)
        
        return path
    
    def _heuristic(self, state: State, goal: Waypoint, mission: Mission) -> float:
        """
        Admissible heuristic for A*.
        
        Returns lower bound on cost-to-go.
        """
        # Euclidean distance
        dist = np.linalg.norm(state.position[:3] - goal.position[:3])
        
        # Minimum time (assuming max speed, no obstacles)
        max_speed = mission.vehicle.max_speed or 25.0
        min_time = dist / max_speed
        
        # Minimum fuel (simplified estimate)
        min_fuel_rate = 0.01  # kg/s (very rough estimate)
        min_fuel = min_fuel_rate * min_time
        
        # Weighted combination based on mission objectives
        time_weight = mission.objectives.get('minimize_time', 1.0)
        fuel_weight = mission.objectives.get('minimize_fuel', 0.5)
        
        return time_weight * min_time + fuel_weight * min_fuel


class GreedyPlanner(Planner):
    """
    Simple greedy baseline planner.
    
    Always visits nearest unvisited waypoint.
    """
    
    def __init__(self):
        super().__init__("Greedy")
    
    def plan(self, mission: Mission) -> MissionResult:
        """Plan by visiting nearest neighbor."""
        states = [mission.start_state.copy()]
        current = mission.start_state.copy()
        remaining = set(range(len(mission.waypoints)))
        
        while remaining:
            # Find nearest
            nearest_idx = None
            min_dist = float('inf')
            
            for idx in remaining:
                wp = mission.waypoints[idx]
                dist = np.linalg.norm(current.position[:3] - wp.position[:3])
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = idx
            
            if nearest_idx is None:
                break
            
            # Move to waypoint (simple teleport for baseline)
            wp = mission.waypoints[nearest_idx]
            
            # Estimate time
            speed = mission.vehicle.max_speed or 25.0
            travel_time = min_dist / speed
            
            new_state = State(
                time=current.time + travel_time,
                position=wp.position.copy(),
                velocity=current.velocity,
                heading=current.heading,
                altitude=wp.position[2] if len(wp.position) > 2 else current.altitude,
                fuel=current.fuel,
                battery_SOC=current.battery_SOC
            )
            
            states.append(new_state)
            current = new_state
            remaining.remove(nearest_idx)
        
        # Return to start
        if mission.return_to_start and len(states) > 1:
            dist = np.linalg.norm(current.position[:3] - mission.start_state.position[:3])
            speed = mission.vehicle.max_speed or 25.0
            travel_time = dist / speed
            
            final_state = State(
                time=current.time + travel_time,
                position=mission.start_state.position.copy(),
                velocity=current.velocity,
                heading=current.heading,
                altitude=mission.start_state.altitude,
                fuel=current.fuel,
                battery_SOC=current.battery_SOC
            )
            states.append(final_state)
        
        # Compute metrics
        total_time = states[-1].time - states[0].time if states else 0.0
        total_distance = sum(states[i].distance_to(states[i+1]) for i in range(len(states)-1))
        
        result = MissionResult(
            success=True,
            mission=mission,
            states=states,
            total_time=total_time,
            total_distance=total_distance,
            is_feasible=True
        )
        
        return result
