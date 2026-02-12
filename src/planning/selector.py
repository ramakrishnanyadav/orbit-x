"""
Intelligent planner selection based on problem characteristics.
"""

from typing import Type
from ..core.models import Mission
from .planner_base import Planner
from .astar_planner import AStarPlanner, GreedyPlanner
from .milp_planner import MILPPlanner


class PlannerSelector:
    """
    Automatically select best planning algorithm for a given mission.
    
    Decision logic based on:
    - Problem size (number of waypoints)
    - Constraint complexity
    - State space dimensionality
    - Computational budget
    """
    
    def __init__(self, verbose: bool = False):
        """
        Initialize planner selector.
        
        Args:
            verbose: Enable decision logging
        """
        self.verbose = verbose
    
    def select_algorithm(self, mission: Mission) -> Planner:
        """
        Choose best planning algorithm for mission.
        
        Args:
            mission: Mission specification
        
        Returns:
            Instantiated planner
        """
        # Problem size metrics
        n_waypoints = len(mission.waypoints)
        n_constraints = len(mission.constraints)
        
        # Analyze constraints
        has_nonlinear = any(
            hasattr(c, 'is_nonlinear') and c.is_nonlinear() 
            for c in mission.constraints
        )
        
        has_temporal = any(
            'time' in c.name.lower() or 'window' in c.name.lower()
            for c in mission.constraints
        )
        
        # Decision logic
        decision = None
        planner = None
        
        if n_waypoints == 0:
            decision = "No waypoints - using trivial planner"
            planner = GreedyPlanner()
        
        elif n_waypoints <= 10 and not has_nonlinear and not has_temporal:
            decision = f"Small problem (n={n_waypoints}), linear constraints -> MILP"
            planner = MILPPlanner(verbose=self.verbose)
        
        elif n_waypoints <= 50:
            decision = f"Medium problem (n={n_waypoints}) -> A*"
            planner = AStarPlanner(verbose=self.verbose)
        
        elif n_waypoints > 50:
            decision = f"Large problem (n={n_waypoints}) -> Greedy heuristic"
            planner = GreedyPlanner()
        
        else:
            decision = "Default -> A*"
            planner = AStarPlanner(verbose=self.verbose)
        
        if self.verbose:
            print(f"Planner selection: {decision}")
        
        return planner
    
    def plan(self, mission: Mission) -> 'MissionResult':
        """
        Select planner and plan mission.
        
        Args:
            mission: Mission specification
        
        Returns:
            Mission planning result
        """
        planner = self.select_algorithm(mission)
        return planner.plan_with_timing(mission)
