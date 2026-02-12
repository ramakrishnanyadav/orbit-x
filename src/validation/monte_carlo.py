"""
Monte Carlo validation and robustness analysis.

Performs uncertainty quantification by running many perturbed scenarios.
"""

import copy
from typing import Dict, List, Any, Callable
import numpy as np
import pandas as pd
from tqdm import tqdm

from ..core.models import Mission, MissionResult


class MonteCarloValidator:
    """
    Monte Carlo robustness validator.
    
    Runs mission plan under perturbed conditions to assess reliability.
    """
    
    def __init__(self, n_runs: int = 100, seed: int = 42):
        """
        Initialize Monte Carlo validator.
        
        Args:
            n_runs: Number of simulation runs
            seed: Random seed for reproducibility
        """
        self.n_runs = n_runs
        self.seed = seed
        self.results = []
    
    def run_analysis(self, mission: Mission, planner: Any, 
                    perturbation_config: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Run Monte Carlo analysis.
        
        Args:
            mission: Nominal mission specification
            planner: Planning algorithm
            perturbation_config: Parameters for uncertainty generation
        
        Returns:
            Dictionary with statistical results
        """
        np.random.seed(self.seed)
        
        # Default perturbation config
        if perturbation_config is None:
            perturbation_config = {
                'wind_multiplier_range': (0.7, 1.3),
                'wind_rotation_range': (-20, 20),
                'mass_multiplier_range': (0.95, 1.05),
                'drag_multiplier_range': (0.9, 1.1),
                'fuel_multiplier_range': (0.98, 1.02),
            }
        
        print(f"Running {self.n_runs} Monte Carlo simulations...")
        
        self.results = []
        
        for i in tqdm(range(self.n_runs), desc="Monte Carlo"):
            # Perturb mission
            perturbed_mission = self._perturb_mission(mission, i, perturbation_config)
            
            # Plan with nominal planner
            try:
                result = planner.plan(perturbed_mission)
                
                self.results.append({
                    'run_id': i,
                    'success': result.success and result.is_feasible,
                    'total_time': result.total_time,
                    'total_distance': result.total_distance,
                    'fuel_used': result.fuel_used,
                    'energy_used': result.energy_used,
                    'constraint_violations': result.constraint_violations,
                    'objective_value': result.objective_value,
                })
            except Exception as e:
                self.results.append({
                    'run_id': i,
                    'success': False,
                    'total_time': 0,
                    'total_distance': 0,
                    'fuel_used': 0,
                    'energy_used': 0,
                    'constraint_violations': 999,
                    'objective_value': float('inf'),
                })
        
        return self._analyze_results()
    
    def _perturb_mission(self, mission: Mission, seed: int, 
                        config: Dict[str, Any]) -> Mission:
        """
        Create perturbed version of mission.
        
        Args:
            mission: Nominal mission
            seed: Random seed for this run
            config: Perturbation configuration
        
        Returns:
            Perturbed mission
        """
        np.random.seed(self.seed + seed)
        
        perturbed = copy.deepcopy(mission)
        
        # Perturb wind field (if aircraft mission)
        if hasattr(perturbed.environment, 'wind_field'):
            wind_mult = np.random.uniform(*config['wind_multiplier_range'])
            wind_rot = np.random.uniform(*config['wind_rotation_range'])
            perturbed.environment['wind_multiplier'] = wind_mult
            perturbed.environment['wind_rotation'] = wind_rot
        
        # Perturb vehicle mass
        mass_mult = np.random.uniform(*config['mass_multiplier_range'])
        perturbed.vehicle.mass *= mass_mult
        
        # Perturb drag coefficient
        drag_mult = np.random.uniform(*config['drag_multiplier_range'])
        perturbed.vehicle.drag_coefficient *= drag_mult
        
        # Perturb initial fuel
        if perturbed.start_state.fuel is not None:
            fuel_mult = np.random.uniform(*config['fuel_multiplier_range'])
            perturbed.start_state.fuel *= fuel_mult
        
        return perturbed
    
    def _analyze_results(self) -> Dict[str, Any]:
        """
        Analyze Monte Carlo results.
        
        Returns:
            Statistical summary
        """
        df = pd.DataFrame(self.results)
        
        # Success rate
        success_rate = df['success'].mean()
        
        # Statistics for successful runs
        successful = df[df['success']]
        
        if len(successful) == 0:
            return {
                'success_rate': 0.0,
                'n_runs': self.n_runs,
                'n_successful': 0,
                'mean_time': 0,
                'std_time': 0,
                'mean_fuel': 0,
                'std_fuel': 0,
            }
        
        report = {
            'success_rate': success_rate,
            'n_runs': self.n_runs,
            'n_successful': len(successful),
            
            # Time statistics
            'mean_time': successful['total_time'].mean(),
            'std_time': successful['total_time'].std(),
            'min_time': successful['total_time'].min(),
            'max_time': successful['total_time'].max(),
            'percentile_95_time': successful['total_time'].quantile(0.95),
            
            # Fuel statistics
            'mean_fuel': successful['fuel_used'].mean(),
            'std_fuel': successful['fuel_used'].std(),
            'min_fuel': successful['fuel_used'].min(),
            'max_fuel': successful['fuel_used'].max(),
            'worst_case_fuel': successful['fuel_used'].max(),
            'percentile_95_fuel': successful['fuel_used'].quantile(0.95),
            
            # Distance statistics
            'mean_distance': successful['total_distance'].mean(),
            'std_distance': successful['total_distance'].std(),
            
            # Constraint violations
            'violation_rate': (df['constraint_violations'] > 0).mean(),
        }
        
        # Failure mode analysis
        failures = df[~df['success']]
        if len(failures) > 0:
            report['n_failures'] = len(failures)
            report['failure_reasons'] = "See detailed logs"
        
        return report
    
    def print_report(self, report: Dict[str, Any]):
        """Print human-readable report."""
        print("\n" + "="*60)
        print("MONTE CARLO VALIDATION REPORT")
        print("="*60)
        print(f"Total Runs: {report['n_runs']}")
        print(f"Successful: {report['n_successful']} ({report['success_rate']*100:.1f}%)")
        print(f"\nMission Time:")
        print(f"  Mean: {report['mean_time']:.1f} ± {report['std_time']:.1f} s")
        print(f"  Range: [{report['min_time']:.1f}, {report['max_time']:.1f}] s")
        print(f"  95th percentile: {report['percentile_95_time']:.1f} s")
        print(f"\nFuel Consumption:")
        print(f"  Mean: {report['mean_fuel']:.2f} ± {report['std_fuel']:.2f} kg")
        print(f"  Worst case: {report['worst_case_fuel']:.2f} kg")
        print(f"  95th percentile: {report['percentile_95_fuel']:.2f} kg")
        print(f"\nConstraint Violations: {report['violation_rate']*100:.1f}%")
        print("="*60 + "\n")
