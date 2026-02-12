"""
Baseline comparison utilities.

Compare advanced planners against simple baselines.
"""

from typing import List, Dict
import pandas as pd

from ..core.models import Mission, MissionResult
from ..planning.planner_base import Planner


class BaselineComparator:
    """
    Compare multiple planners on the same mission.
    
    Useful for validating that advanced algorithms outperform baselines.
    """
    
    def __init__(self):
        """Initialize comparator."""
        self.results = []
    
    def compare(self, mission: Mission, planners: List[Planner]) -> pd.DataFrame:
        """
        Run all planners and compare results.
        
        Args:
            mission: Mission to plan
            planners: List of planners to compare
        
        Returns:
            DataFrame with comparison metrics
        """
        self.results = []
        
        for planner in planners:
            print(f"Running {planner.name}...")
            result = planner.plan_with_timing(mission)
            
            self.results.append({
                'method': planner.name,
                'success': result.success,
                'time_s': result.total_time,
                'time_min': result.total_time / 60,
                'distance_km': result.total_distance / 1000,
                'fuel_kg': result.fuel_used,
                'violations': result.constraint_violations,
                'planning_time_s': result.planning_time,
                'objective_value': result.objective_value,
            })
        
        df = pd.DataFrame(self.results)
        return df
    
    def print_comparison(self, df: pd.DataFrame, baseline_method: str = 'Greedy'):
        """
        Print comparison table with improvements over baseline.
        
        Args:
            df: Comparison DataFrame
            baseline_method: Name of baseline method for computing improvements
        """
        print("\n" + "="*80)
        print("PLANNER COMPARISON")
        print("="*80)
        print(df.to_string(index=False))
        print("="*80)
        
        # Compute improvements over baseline
        if baseline_method in df['method'].values:
            baseline = df[df['method'] == baseline_method].iloc[0]
            
            print(f"\nIMPROVEMENTS OVER {baseline_method.upper()}:")
            print("-"*80)
            
            for _, row in df.iterrows():
                if row['method'] == baseline_method:
                    continue
                
                time_imp = (baseline['time_s'] - row['time_s']) / baseline['time_s'] * 100
                fuel_imp = (baseline['fuel_kg'] - row['fuel_kg']) / baseline['fuel_kg'] * 100 if baseline['fuel_kg'] > 0 else 0
                
                print(f"{row['method']:15s}: Time {time_imp:+6.1f}%  |  Fuel {fuel_imp:+6.1f}%")
            
            print("="*80 + "\n")
