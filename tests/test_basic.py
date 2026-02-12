"""
Basic integration tests for ORBIT-X system.

Tests core functionality to ensure system works end-to-end.
"""

import pytest
import numpy as np
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from core.models import Mission, Vehicle, Waypoint, State
from core.constraints import AltitudeConstraint, ResourceConstraint
from planning.astar_planner import AStarPlanner, GreedyPlanner
from planning.selector import PlannerSelector


class TestCoreModels:
    """Test core data structures."""
    
    def test_state_creation(self):
        """Test State creation and methods."""
        state = State(
            time=0.0,
            position=np.array([37.7749, -122.4194, 100.0]),
            velocity=np.array([25.0, 0.0, 0.0]),
            heading=0.0,
            fuel=5.0
        )
        
        assert state.time == 0.0
        assert len(state.position) == 3
        assert state.fuel == 5.0
    
    def test_vehicle_creation(self):
        """Test Vehicle creation."""
        vehicle = Vehicle(
            name="Test UAV",
            vehicle_type="aircraft",
            mass=25.0,
            max_speed=30.0
        )
        
        assert vehicle.name == "Test UAV"
        assert vehicle.vehicle_type == "aircraft"
        assert vehicle.mass == 25.0
    
    def test_waypoint_creation(self):
        """Test Waypoint creation."""
        wp = Waypoint(
            id="wp1",
            name="Test Waypoint",
            position=np.array([37.8, -122.4, 150.0]),
            priority=10
        )
        
        assert wp.id == "wp1"
        assert wp.priority == 10


class TestConstraints:
    """Test constraint framework."""
    
    def test_altitude_constraint(self):
        """Test altitude constraint checking."""
        constraint = AltitudeConstraint(min_altitude=50.0, max_altitude=400.0)
        
        # Valid state
        state_ok = State(
            time=0.0,
            position=np.array([0, 0, 100.0]),
            velocity=np.zeros(3),
            altitude=100.0
        )
        satisfied, margin = constraint.check(state_ok)
        assert satisfied
        assert margin > 0
        
        # Invalid state (too high)
        state_bad = State(
            time=0.0,
            position=np.array([0, 0, 500.0]),
            velocity=np.zeros(3),
            altitude=500.0
        )
        satisfied, margin = constraint.check(state_bad)
        assert not satisfied
    
    def test_resource_constraint(self):
        """Test resource constraint checking."""
        constraint = ResourceConstraint('fuel', min_level=0.5, max_level=10.0)
        
        # Valid state
        state_ok = State(
            time=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            fuel=2.0
        )
        satisfied, margin = constraint.check(state_ok)
        assert satisfied
        
        # Invalid state (too low fuel)
        state_bad = State(
            time=0.0,
            position=np.zeros(3),
            velocity=np.zeros(3),
            fuel=0.1
        )
        satisfied, margin = constraint.check(state_bad)
        assert not satisfied


class TestPlanners:
    """Test planning algorithms."""
    
    def test_greedy_planner(self):
        """Test greedy planner on simple mission."""
        # Create simple mission
        vehicle = Vehicle(
            name="Test UAV",
            vehicle_type="aircraft",
            mass=25.0,
            max_speed=30.0
        )
        
        start = State(
            time=0.0,
            position=np.array([0.0, 0.0, 100.0]),
            velocity=np.array([25.0, 0.0, 0.0]),
            fuel=5.0
        )
        
        waypoints = [
            Waypoint(id="wp1", position=np.array([0.01, 0.0, 100.0]), priority=1),
            Waypoint(id="wp2", position=np.array([0.0, 0.01, 100.0]), priority=1),
        ]
        
        mission = Mission(
            name="Test Mission",
            vehicle=vehicle,
            start_state=start,
            waypoints=waypoints,
            max_duration=3600.0
        )
        
        # Plan with greedy
        planner = GreedyPlanner()
        result = planner.plan(mission)
        
        assert result.success
        assert len(result.states) > 0
        assert result.total_time > 0
    
    def test_astar_planner(self):
        """Test A* planner on simple mission."""
        vehicle = Vehicle(
            name="Test UAV",
            vehicle_type="aircraft",
            mass=25.0,
            max_speed=30.0
        )
        
        start = State(
            time=0.0,
            position=np.array([0.0, 0.0, 100.0]),
            velocity=np.array([25.0, 0.0, 0.0]),
            fuel=5.0
        )
        
        waypoints = [
            Waypoint(id="wp1", position=np.array([0.01, 0.0, 100.0]), priority=1),
        ]
        
        mission = Mission(
            name="Test Mission",
            vehicle=vehicle,
            start_state=start,
            waypoints=waypoints,
            max_duration=3600.0
        )
        
        # Plan with A*
        planner = AStarPlanner(verbose=False)
        result = planner.plan(mission)
        
        assert result.success
        assert len(result.states) > 0
    
    def test_planner_selector(self):
        """Test automatic planner selection."""
        vehicle = Vehicle(
            name="Test UAV",
            vehicle_type="aircraft",
            mass=25.0,
            max_speed=30.0
        )
        
        start = State(
            time=0.0,
            position=np.array([0.0, 0.0, 100.0]),
            velocity=np.array([25.0, 0.0, 0.0]),
            fuel=5.0
        )
        
        # Small problem (should use MILP or A*)
        waypoints_small = [
            Waypoint(id=f"wp{i}", position=np.array([0.01*i, 0.0, 100.0]), priority=1)
            for i in range(3)
        ]
        
        mission_small = Mission(
            name="Small Mission",
            vehicle=vehicle,
            start_state=start,
            waypoints=waypoints_small,
            max_duration=3600.0
        )
        
        selector = PlannerSelector(verbose=False)
        planner = selector.select_algorithm(mission_small)
        
        assert planner is not None
        assert planner.name in ['MILP', 'A*', 'Greedy']


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
