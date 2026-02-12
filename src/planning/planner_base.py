"""
Abstract base class for all planning algorithms.
"""

from abc import ABC, abstractmethod
from typing import List, Optional
import time

from ..core.models import Mission, MissionResult, State


class Planner(ABC):
    """
    Abstract interface for mission planning algorithms.
    
    All planners must implement the plan() method.
    """
    
    def __init__(self, name: str):
        """
        Initialize planner.
        
        Args:
            name: Planner identifier
        """
        self.name = name
        self.verbose = False
    
    @abstractmethod
    def plan(self, mission: Mission) -> MissionResult:
        """
        Plan a mission.
        
        Args:
            mission: Mission specification
        
        Returns:
            MissionResult with planned trajectory and metrics
        """
        pass
    
    def plan_with_timing(self, mission: Mission) -> MissionResult:
        """
        Plan mission and record computational time.
        
        Args:
            mission: Mission specification
        
        Returns:
            MissionResult with planning_time populated
        """
        start_time = time.time()
        result = self.plan(mission)
        end_time = time.time()
        
        result.planning_time = end_time - start_time
        result.algorithm_used = self.name
        
        return result
    
    def validate_result(self, result: MissionResult) -> bool:
        """
        Validate that result satisfies all constraints.
        
        Args:
            result: Planning result
        
        Returns:
            True if all hard constraints satisfied
        """
        if not result.states:
            return False
        
        for state in result.states:
            for constraint in result.mission.constraints:
                satisfied, margin = constraint.check(state)
                if constraint.severity == 'hard' and not satisfied:
                    if self.verbose:
                        print(f"Constraint violation: {constraint.name} at t={state.time:.1f}s")
                    return False
        
        return True
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}')"
