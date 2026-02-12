"""
Constraint relaxation and infeasibility diagnosis.

Provides intelligent analysis and repair suggestions when missions
have conflicting or unsatisfiable constraints.
"""

import numpy as np
from typing import List, Dict, Tuple, Set, Optional
from dataclasses import dataclass, field
from collections import defaultdict
import logging

from .constraints import Constraint

logger = logging.getLogger(__name__)


@dataclass
class ConflictInfo:
    """Information about a constraint conflict."""
    constraint_name: str
    constraint_type: str
    deficit: float  # How much constraint is violated by
    severity: str  # 'hard' or 'soft'
    penalty_weight: float = 1.0
    description: str = ""


@dataclass
class ConflictReport:
    """Detailed report of constraint conflicts and repair suggestions."""
    conflicts: List[ConflictInfo]
    minimal_conflicts: List[ConflictInfo]
    repair_suggestions: List[str]
    estimated_success_rate: float  # Probability of success if suggestions applied
    
    def __str__(self) -> str:
        """Format report as human-readable text."""
        report = "="*60 + "\n"
        report += "CONSTRAINT CONFLICT ANALYSIS\n"
        report += "="*60 + "\n\n"
        
        report += f"Total Conflicts Found: {len(self.conflicts)}\n"
        report += f"Minimal Conflicting Set: {len(self.minimal_conflicts)}\n"
        report += f"Estimated Success Rate After Fixes: {self.estimated_success_rate:.1%}\n\n"
        
        if self.minimal_conflicts:
            report += "CRITICAL CONFLICTS (must resolve):\n"
            report += "-"*60 + "\n"
            for i, conflict in enumerate(self.minimal_conflicts, 1):
                report += f"{i}. {conflict.constraint_name} ({conflict.constraint_type})\n"
                report += f"   Severity: {conflict.severity}\n"
                report += f"   Deficit: {conflict.deficit:.4f}\n"
                if conflict.description:
                    report += f"   Description: {conflict.description}\n"
                report += "\n"
        
        if self.repair_suggestions:
            report += "REPAIR SUGGESTIONS:\n"
            report += "-"*60 + "\n"
            for i, suggestion in enumerate(self.repair_suggestions, 1):
                report += f"{i}. {suggestion}\n"
            report += "\n"
        
        report += "="*60 + "\n"
        
        return report


class ConstraintRelaxer:
    """
    Intelligent constraint relaxation for infeasible problems.
    
    Strategy:
    1. Identify which constraints are violated
    2. Find minimal conflicting set (smallest set that must be relaxed)
    3. Rank constraints by relaxation cost
    4. Suggest specific parameter adjustments
    """
    
    def __init__(self, planner=None):
        """
        Initialize constraint relaxer.
        
        Args:
            planner: Optional planner instance for testing feasibility
        """
        self.planner = planner
    
    def analyze_infeasibility(self, mission, failed_plan=None) -> ConflictReport:
        """
        Diagnose why no feasible solution exists.
        
        Args:
            mission: Mission object with constraints
            failed_plan: Optional failed plan attempt (for detailed analysis)
        
        Returns:
            ConflictReport with detailed analysis and suggestions
        """
        logger.info("Analyzing constraint conflicts...")
        
        conflicts = []
        
        # Check each constraint independently
        for constraint in mission.constraints:
            violation = self._check_constraint_satisfaction(constraint, mission)
            
            if violation:
                conflicts.append(ConflictInfo(
                    constraint_name=constraint.name,
                    constraint_type=type(constraint).__name__,
                    deficit=violation['deficit'],
                    severity=constraint.severity,
                    penalty_weight=getattr(constraint, 'penalty_weight', 1.0),
                    description=violation.get('description', '')
                ))
        
        logger.info(f"Found {len(conflicts)} constraint violations")
        
        # Find minimal conflicting set
        minimal_conflicts = self._find_minimal_conflicts(conflicts, mission)
        
        logger.info(f"Minimal conflicting set: {len(minimal_conflicts)} constraints")
        
        # Generate repair suggestions
        suggestions = self._generate_repair_suggestions(minimal_conflicts, mission)
        
        # Estimate success rate if suggestions applied
        success_rate = self._estimate_success_rate(minimal_conflicts, mission)
        
        return ConflictReport(
            conflicts=conflicts,
            minimal_conflicts=minimal_conflicts,
            repair_suggestions=suggestions,
            estimated_success_rate=success_rate
        )
    
    def solve_with_relaxation(self, mission, max_iterations: int = 10):
        """
        Attempt to solve by iteratively relaxing constraints.
        
        Algorithm:
        1. Try with all constraints → if feasible, done
        2. Rank soft constraints by penalty weight
        3. Iteratively relax lowest-penalty constraints
        4. Return best solution found
        
        Args:
            mission: Mission to plan
            max_iterations: Maximum constraint relaxations to try
        
        Returns:
            solution: Best plan found
            relaxed_constraints: List of constraints that were relaxed
        
        Raises:
            InfeasibilityError: If no solution found even with all soft constraints relaxed
        """
        if self.planner is None:
            raise ValueError("Planner must be set to use solve_with_relaxation")
        
        logger.info("Attempting to solve with constraint relaxation...")
        
        # Try with all constraints first
        try:
            solution = self.planner.plan(mission)
            if solution.is_feasible:
                logger.info("✓ Solution found with all constraints satisfied")
                return solution, []
        except Exception as e:
            logger.warning(f"Initial planning failed: {e}")
        
        # Separate constraints by severity
        hard_constraints = [c for c in mission.constraints if c.severity == 'hard']
        soft_constraints = [c for c in mission.constraints if c.severity == 'soft']
        
        logger.info(f"Hard constraints: {len(hard_constraints)}, "
                   f"Soft constraints: {len(soft_constraints)}")
        
        # Sort soft constraints by penalty (relax cheapest first)
        soft_sorted = sorted(soft_constraints, 
                            key=lambda c: getattr(c, 'penalty_weight', 1.0))
        
        relaxed = []
        best_solution = None
        best_score = float('-inf')
        
        for i, constraint in enumerate(soft_sorted):
            if i >= max_iterations:
                break
            
            # Remove this constraint temporarily
            mission.constraints.remove(constraint)
            relaxed.append(constraint.name)
            
            logger.info(f"Relaxing constraint {i+1}/{len(soft_sorted)}: {constraint.name}")
            
            try:
                solution = self.planner.plan(mission)
                
                if solution.is_feasible:
                    # Score solution (lower penalty is better)
                    score = solution.objective_value - sum(
                        getattr(c, 'penalty_weight', 1.0) 
                        for c in soft_sorted[:i+1]
                    )
                    
                    if score > best_score:
                        best_solution = solution
                        best_score = score
                    
                    logger.info(f"✓ Feasible solution found (score: {score:.2f})")
                    
                    # If score is good enough, stop
                    if score > 0.8 * solution.objective_value:
                        break
            
            except Exception as e:
                logger.warning(f"Planning failed even with relaxation: {e}")
                continue
        
        # Restore original constraints
        mission.constraints.extend([c for c in soft_sorted if c.name in relaxed])
        
        if best_solution is not None and best_solution.is_feasible:
            logger.info(f"✓ Solution found with {len(relaxed)} constraints relaxed")
            return best_solution, relaxed
        
        # Still infeasible - need to relax hard constraints or problem is impossible
        logger.error("No feasible solution found even with all soft constraints relaxed")
        
        conflict_report = self.analyze_infeasibility(mission)
        
        raise InfeasibilityError(
            "Mission is infeasible even with maximum constraint relaxation.\n" +
            str(conflict_report)
        )
    
    def _check_constraint_satisfaction(self, constraint: Constraint, 
                                      mission) -> Optional[Dict]:
        """
        Check if a single constraint can be satisfied.
        
        Returns:
            None if satisfied, dict with violation info otherwise
        """
        # This is a simplified check - real implementation would test
        # constraint against mission parameters
        
        try:
            margin = constraint.get_margin(mission)
            
            if margin < 0:
                return {
                    'deficit': abs(margin),
                    'description': f"Constraint violated by {abs(margin):.4f}"
                }
        except AttributeError:
            # Constraint doesn't have get_margin method
            # Would need domain-specific checks here
            pass
        
        return None
    
    def _find_minimal_conflicts(self, conflicts: List[ConflictInfo], 
                               mission) -> List[ConflictInfo]:
        """
        Find minimal conflicting set.
        
        This is an NP-hard problem (related to hitting set), so we use
        a greedy heuristic:
        1. Sort conflicts by deficit (most severe first)
        2. Greedily select conflicts that "cover" the most violations
        
        Returns:
            List of conflicts that form a minimal set
        """
        if not conflicts:
            return []
        
        # For now, return hard constraint violations
        # (these are always in the minimal set)
        minimal = [c for c in conflicts if c.severity == 'hard']
        
        # If no hard violations, include worst soft violations
        if not minimal:
            sorted_conflicts = sorted(conflicts, key=lambda c: -c.deficit)
            # Include top 3 worst violations
            minimal = sorted_conflicts[:min(3, len(sorted_conflicts))]
        
        return minimal
    
    def _generate_repair_suggestions(self, conflicts: List[ConflictInfo], 
                                     mission) -> List[str]:
        """
        Generate specific, actionable repair suggestions.
        
        Args:
            conflicts: List of constraint conflicts
            mission: Mission object
        
        Returns:
            List of human-readable suggestions
        """
        suggestions = []
        
        for conflict in conflicts:
            ctype = conflict.constraint_type
            deficit = conflict.deficit
            
            if 'Fuel' in ctype or 'Battery' in ctype:
                required_increase = deficit * 1.2  # 20% safety margin
                suggestions.append(
                    f"Increase {ctype.replace('Constraint', '').lower()} "
                    f"capacity by {required_increase:.2f} units "
                    f"(current deficit: {deficit:.2f})"
                )
            
            elif 'TimeWindow' in ctype:
                time_extension = int(np.ceil(deficit * 1.2))
                suggestions.append(
                    f"Relax time window constraint by {time_extension} seconds "
                    f"(current violation: {deficit:.0f} s)"
                )
            
            elif 'NoFlyZone' in ctype:
                buffer_increase = int(np.ceil(deficit * 1.5))
                suggestions.append(
                    f"Increase no-fly zone safety buffer by {buffer_increase} meters "
                    f"OR re-route to avoid penetration of {deficit:.0f} m"
                )
            
            elif 'Altitude' in ctype:
                suggestions.append(
                    f"Adjust altitude limits to accommodate {deficit:.0f} m violation "
                    f"OR select lower-altitude waypoints"
                )
            
            elif 'Slew' in ctype or 'Turn' in ctype:
                suggestions.append(
                    f"Reduce required slew/turn rate by {deficit:.2f} deg/s "
                    f"OR increase time between observations"
                )
            
            else:
                # Generic suggestion
                suggestions.append(
                    f"Relax {conflict.constraint_name} constraint "
                    f"(current violation: {deficit:.4f})"
                )
        
        # Add strategic suggestions
        if len(conflicts) > 5:
            suggestions.append(
                "Consider simplifying mission: reduce number of waypoints/targets"
            )
        
        if any('Fuel' in c.constraint_type or 'Battery' in c.constraint_type 
               for c in conflicts):
            suggestions.append(
                "Consider adding refueling/recharging waypoint"
            )
        
        return suggestions
    
    def _estimate_success_rate(self, conflicts: List[ConflictInfo], 
                              mission) -> float:
        """
        Estimate probability of success if repair suggestions are applied.
        
        Uses heuristic based on:
        - Number of conflicts
        - Severity of violations
        - Type of constraints involved
        
        Returns:
            Probability between 0 and 1
        """
        if not conflicts:
            return 1.0
        
        # Base success rate depends on number of conflicts
        base_rate = max(0.1, 1.0 - 0.15 * len(conflicts))
        
        # Penalize hard constraint violations more
        hard_penalty = sum(1 for c in conflicts if c.severity == 'hard') * 0.1
        base_rate -= hard_penalty
        
        # Penalize large deficits
        avg_deficit = np.mean([c.deficit for c in conflicts])
        if avg_deficit > 10.0:
            base_rate *= 0.7
        elif avg_deficit > 5.0:
            base_rate *= 0.85
        
        return np.clip(base_rate, 0.0, 1.0)


class InfeasibilityError(Exception):
    """Raised when mission is infeasible even with constraint relaxation."""
    
    def __init__(self, message: str, conflict_report: ConflictReport = None):
        super().__init__(message)
        self.conflict_report = conflict_report


# Utility functions for constraint margin analysis

def analyze_constraint_margins(mission, solution) -> Dict[str, float]:
    """
    Analyze how close each constraint is to being violated.
    
    Args:
        mission: Mission object
        solution: Solved trajectory
    
    Returns:
        Dict mapping constraint names to margin values (0-1 scale)
        where 1 = well satisfied, 0 = at limit
    """
    margins = {}
    
    for constraint in mission.constraints:
        try:
            # Get minimum margin across entire trajectory
            min_margin = float('inf')
            
            for state in solution.trajectory:
                satisfied, margin = constraint.check(state, None)
                if margin < min_margin:
                    min_margin = margin
            
            # Normalize to 0-1 scale (heuristic)
            if min_margin == float('inf'):
                normalized_margin = 1.0
            else:
                # Assume margins > 10 are "comfortable"
                normalized_margin = min(min_margin / 10.0, 1.0)
            
            margins[constraint.name] = normalized_margin
        
        except Exception as e:
            logger.warning(f"Could not compute margin for {constraint.name}: {e}")
            margins[constraint.name] = 0.0
    
    return margins


def identify_critical_constraints(mission, solution, threshold: float = 0.2) -> List[str]:
    """
    Identify constraints that are close to being violated.
    
    Args:
        mission: Mission object
        solution: Solved trajectory
        threshold: Margin threshold (default: 0.2 = 20%)
    
    Returns:
        List of constraint names that are "critical" (low margin)
    """
    margins = analyze_constraint_margins(mission, solution)
    
    critical = [
        name for name, margin in margins.items()
        if margin < threshold
    ]
    
    return critical
