"""
Objective function definitions for mission optimization.

Supports multi-objective optimization with configurable weights.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple
import numpy as np

from .models import State, Mission, MissionResult


class ObjectiveFunction(ABC):
    """
    Abstract base class for objective functions.
    
    Objective functions evaluate the quality of a mission plan.
    """
    
    def __init__(self, name: str, weight: float = 1.0):
        """
        Initialize objective function.
        
        Args:
            name: Objective identifier
            weight: Relative importance (used in multi-objective optimization)
        """
        self.name = name
        self.weight = weight
    
    @abstractmethod
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """
        Evaluate objective for a trajectory.
        
        Args:
            states: Planned trajectory
            mission: Mission specification
        
        Returns:
            Objective value (lower is better for minimization)
        """
        pass
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}', weight={self.weight})"


class TimeObjective(ObjectiveFunction):
    """Minimize total mission time."""
    
    def __init__(self, weight: float = 1.0):
        super().__init__("minimize_time", weight)
    
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """Return total mission duration."""
        if not states:
            return float('inf')
        return states[-1].time - states[0].time


class FuelObjective(ObjectiveFunction):
    """Minimize fuel consumption."""
    
    def __init__(self, weight: float = 1.0):
        super().__init__("minimize_fuel", weight)
    
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """Return total fuel consumed."""
        if not states:
            return float('inf')
        
        initial_fuel = states[0].fuel if states[0].fuel is not None else 0.0
        final_fuel = states[-1].fuel if states[-1].fuel is not None else 0.0
        
        return initial_fuel - final_fuel


class DistanceObjective(ObjectiveFunction):
    """Minimize total distance traveled."""
    
    def __init__(self, weight: float = 1.0):
        super().__init__("minimize_distance", weight)
    
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """Return total path length."""
        if len(states) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(states)):
            total_distance += states[i].distance_to(states[i-1])
        
        return total_distance


class EnergyObjective(ObjectiveFunction):
    """Minimize energy consumption."""
    
    def __init__(self, weight: float = 1.0):
        super().__init__("minimize_energy", weight)
    
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """Return total energy consumed."""
        if not states:
            return float('inf')
        
        # For battery-powered vehicles
        initial_soc = states[0].battery_SOC if states[0].battery_SOC is not None else 1.0
        final_soc = states[-1].battery_SOC if states[-1].battery_SOC is not None else 1.0
        
        battery_capacity = mission.vehicle.battery_capacity or 1.0
        energy_used = (initial_soc - final_soc) * battery_capacity
        
        return energy_used


class RiskObjective(ObjectiveFunction):
    """
    Minimize risk (proximity to obstacles, constraint violations).
    """
    
    def __init__(self, weight: float = 1.0):
        super().__init__("minimize_risk", weight)
    
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """
        Compute cumulative risk along trajectory.
        
        Risk increases when close to no-fly zones or constraint boundaries.
        """
        total_risk = 0.0
        
        for state in states:
            # Check proximity to constraint boundaries
            for constraint in mission.constraints:
                satisfied, margin = constraint.check(state)
                if margin < 100:  # Within 100m/100s/etc of boundary
                    # Risk increases as margin decreases
                    risk = max(0, 100 - margin) / 100
                    total_risk += risk
        
        return total_risk


class ScienceValueObjective(ObjectiveFunction):
    """
    Maximize science value (for spacecraft observation missions).
    
    This is a maximization objective, so we negate it.
    """
    
    def __init__(self, weight: float = 1.0):
        super().__init__("maximize_science_value", weight)
    
    def evaluate(self, states: List[State], mission: Mission) -> float:
        """
        Return negative science value (for minimization).
        
        Science value based on targets observed and downlinked.
        """
        # Count waypoints visited
        visited_waypoints = set()
        
        for state in states:
            # Check which waypoints were visited
            for wp in mission.waypoints:
                distance = np.linalg.norm(state.position[:3] - wp.position[:3])
                if distance < 1000:  # Within 1km
                    visited_waypoints.add(wp.id)
        
        # Sum science value
        total_value = sum(wp.metadata.get('science_value', wp.priority) 
                         for wp in mission.waypoints 
                         if wp.id in visited_waypoints)
        
        # Return negative (since we minimize)
        return -total_value


class MultiObjective:
    """
    Combine multiple objectives with weights.
    
    Supports Pareto analysis and weighted sum approaches.
    """
    
    def __init__(self, objectives: List[ObjectiveFunction]):
        """
        Initialize multi-objective function.
        
        Args:
            objectives: List of objective functions with weights
        """
        self.objectives = objectives
    
    def evaluate(self, states: List[State], mission: Mission) -> Tuple[float, Dict[str, float]]:
        """
        Evaluate all objectives.
        
        Args:
            states: Planned trajectory
            mission: Mission specification
        
        Returns:
            (total_weighted_value, individual_values)
        """
        values = {}
        total = 0.0
        
        for obj in self.objectives:
            value = obj.evaluate(states, mission)
            values[obj.name] = value
            total += obj.weight * value
        
        return total, values
    
    def get_pareto_front(self, solutions: List[List[State]], 
                        mission: Mission) -> List[int]:
        """
        Find Pareto-optimal solutions.
        
        Args:
            solutions: List of candidate trajectories
            mission: Mission specification
        
        Returns:
            Indices of Pareto-optimal solutions
        """
        n = len(solutions)
        is_dominated = [False] * n
        
        # Evaluate all solutions
        evaluations = []
        for sol in solutions:
            _, values = self.evaluate(sol, mission)
            evaluations.append([values[obj.name] for obj in self.objectives])
        
        # Check dominance
        for i in range(n):
            if is_dominated[i]:
                continue
            for j in range(n):
                if i == j or is_dominated[j]:
                    continue
                
                # Check if j dominates i
                dominates = True
                strictly_better = False
                for k in range(len(self.objectives)):
                    if evaluations[j][k] > evaluations[i][k]:
                        dominates = False
                        break
                    if evaluations[j][k] < evaluations[i][k]:
                        strictly_better = True
                
                if dominates and strictly_better:
                    is_dominated[i] = True
                    break
        
        return [i for i in range(n) if not is_dominated[i]]
