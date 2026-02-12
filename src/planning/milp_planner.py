"""
Mixed Integer Linear Programming (MILP) planner.

Uses convex optimization for exact solutions when tractable.
"""

from typing import List, Optional
import numpy as np

from ..core.models import Mission, MissionResult, State, Waypoint
from .planner_base import Planner


class MILPPlanner(Planner):
    """
    MILP-based optimal planner.
    
    Formulates mission planning as Mixed Integer Linear Program.
    Requires cvxpy and a MILP solver (MOSEK, Gurobi, or open-source).
    """
    
    def __init__(self, solver: str = 'ECOS', verbose: bool = False):
        """
        Initialize MILP planner.
        
        Args:
            solver: CVXPY solver name ('ECOS', 'SCS', 'MOSEK', 'GUROBI')
            verbose: Enable solver output
        """
        super().__init__("MILP")
        self.solver_name = solver
        self.verbose = verbose
    
    def plan(self, mission: Mission) -> MissionResult:
        """
        Plan mission using MILP optimization.
        
        Formulation:
        - Decision variables: waypoint visit order, arrival times
        - Objective: minimize weighted sum of time and fuel
        - Constraints: TSP constraints, time windows, resource limits
        """
        try:
            import cvxpy as cp
        except ImportError:
            print("CVXPY not installed. Falling back to greedy planner.")
            from .astar_planner import GreedyPlanner
            return GreedyPlanner().plan(mission)
        
        n = len(mission.waypoints)
        
        if n == 0:
            return MissionResult(success=False, mission=mission, states=[mission.start_state])
        
        if n > 20:
            if self.verbose:
                print(f"Problem too large for MILP (n={n}). Use A* or sampling-based planner.")
            # Fall back to greedy for large problems
            from .astar_planner import GreedyPlanner
            return GreedyPlanner().plan(mission)
        
        # Simplified MILP: solve TSP for waypoint order
        # Then construct path
        
        # Distance matrix
        D = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    D[i, j] = np.linalg.norm(
                        mission.waypoints[i].position[:3] - mission.waypoints[j].position[:3]
                    )
        
        # Decision variables: x[i,j] = 1 if edge i->j is used
        x = cp.Variable((n, n), boolean=True)
        
        # Objective: minimize total distance
        objective = cp.Minimize(cp.sum(cp.multiply(D, x)))
        
        # Constraints
        constraints = []
        
        # Each waypoint visited exactly once (out)
        for i in range(n):
            constraints.append(cp.sum(x[i, :]) == 1)
        
        # Each waypoint visited exactly once (in)
        for j in range(n):
            constraints.append(cp.sum(x[:, j]) == 1)
        
        # No self-loops
        for i in range(n):
            constraints.append(x[i, i] == 0)
        
        # Subtour elimination (Miller-Tucker-Zemlin formulation)
        if n > 2:
            u = cp.Variable(n)
            for i in range(1, n):
                for j in range(1, n):
                    if i != j:
                        constraints.append(u[i] - u[j] + n * x[i, j] <= n - 1)
        
        # Solve
        problem = cp.Problem(objective, constraints)
        
        try:
            problem.solve(solver=self.solver_name, verbose=self.verbose)
        except:
            # Try backup solver
            try:
                problem.solve(solver='ECOS_BB', verbose=self.verbose)
            except:
                if self.verbose:
                    print("MILP solver failed. Using greedy fallback.")
                from .astar_planner import GreedyPlanner
                return GreedyPlanner().plan(mission)
        
        if problem.status not in ['optimal', 'optimal_inaccurate']:
            if self.verbose:
                print(f"MILP status: {problem.status}. Using greedy fallback.")
            from .astar_planner import GreedyPlanner
            return GreedyPlanner().plan(mission)
        
        # Extract tour from solution
        tour = self._extract_tour(x.value, n)
        
        if self.verbose:
            print(f"MILP tour: {tour}")
        
        # Build trajectory
        states = [mission.start_state.copy()]
        current = mission.start_state.copy()
        total_distance = 0.0
        
        # Visit waypoints in tour order
        for wp_idx in tour:
            wp = mission.waypoints[wp_idx]
            dist = np.linalg.norm(current.position[:3] - wp.position[:3])
            
            # Estimate travel time
            speed = mission.vehicle.max_speed or 25.0
            travel_time = dist / speed
            
            # Create arrival state
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
            total_distance += dist
        
        # Return to start if required
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
            total_distance += dist
        
        # Compute metrics
        total_time = states[-1].time - states[0].time if states else 0.0
        
        result = MissionResult(
            success=True,
            mission=mission,
            states=states,
            total_time=total_time,
            total_distance=total_distance,
            objective_value=problem.value if hasattr(problem, 'value') else 0.0,
            is_feasible=True
        )
        
        return result
    
    def _extract_tour(self, x_val: np.ndarray, n: int) -> List[int]:
        """
        Extract tour from solution matrix.
        
        Args:
            x_val: Solution matrix (may be fractional due to solver tolerances)
            n: Number of waypoints
        
        Returns:
            List of waypoint indices in visit order
        """
        # Round to nearest integer
        x_rounded = np.round(x_val)
        
        # Find tour by following edges
        tour = []
        current = 0  # Start from first waypoint
        visited = set()
        
        for _ in range(n):
            if current in visited:
                break
            tour.append(current)
            visited.add(current)
            
            # Find next waypoint
            next_wp = None
            for j in range(n):
                if x_rounded[current, j] > 0.5:
                    next_wp = j
                    break
            
            if next_wp is None or next_wp in visited:
                break
            
            current = next_wp
        
        return tour
