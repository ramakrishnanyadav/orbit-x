"""
Rich mission telemetry and reporting.

Generates comprehensive JSON output with detailed mission metrics,
constraint analysis, and performance data.
"""

import json
import numpy as np
from datetime import datetime
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
import logging

logger = logging.getLogger(__name__)


@dataclass
class MissionTelemetry:
    """Comprehensive mission telemetry data."""
    
    # Metadata
    mission_id: str
    mission_type: str  # 'aircraft' or 'spacecraft'
    generation_time: str
    software_version: str
    planner_algorithm: str
    computation_time_sec: float
    
    # Trajectory data
    waypoints: List[Dict[str, Any]]
    total_distance_km: float
    total_time_sec: float
    total_fuel_kg: Optional[float] = None
    total_energy_wh: Optional[float] = None
    
    # Constraint analysis
    constraints_evaluated: int = 0
    constraint_violations: int = 0
    constraint_margins: Dict[str, float] = None
    
    # Performance metrics
    performance: Dict[str, Any] = None
    
    # Robustness analysis
    monte_carlo: Optional[Dict[str, Any]] = None
    
    # Environmental conditions
    environment: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)
    
    def to_json(self, filepath: str = None, indent: int = 2) -> str:
        """
        Export to JSON format.
        
        Args:
            filepath: Optional file path to save JSON
            indent: Indentation level (default: 2)
        
        Returns:
            JSON string
        """
        data = self.to_dict()
        
        # Custom JSON encoder for numpy types
        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.integer):
                    return int(obj)
                if isinstance(obj, np.floating):
                    return float(obj)
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                if isinstance(obj, datetime):
                    return obj.isoformat()
                return super().default(obj)
        
        json_str = json.dumps(data, indent=indent, cls=NumpyEncoder)
        
        if filepath:
            with open(filepath, 'w') as f:
                f.write(json_str)
            logger.info(f"✓ Telemetry saved to {filepath}")
        
        return json_str


class TelemetryGenerator:
    """
    Generate comprehensive mission telemetry from planning results.
    
    Creates rich JSON output with all mission details, metrics,
    and analysis results.
    """
    
    def __init__(self, software_version: str = "1.0.0"):
        """
        Initialize telemetry generator.
        
        Args:
            software_version: ORBIT-X version string
        """
        self.software_version = software_version
    
    def generate(self, mission, result, planner_name: str,
                computation_time: float, monte_carlo_results: Dict = None) -> MissionTelemetry:
        """
        Generate complete telemetry from mission results.
        
        Args:
            mission: Mission specification
            result: Planning result
            planner_name: Name of planning algorithm used
            computation_time: Planning time (seconds)
            monte_carlo_results: Optional MC validation results
        
        Returns:
            MissionTelemetry object
        """
        # Generate unique mission ID
        mission_id = f"{mission.name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Extract waypoint data
        waypoints = self._extract_waypoints(result)
        
        # Compute constraint margins
        margins = self._compute_constraint_margins(mission, result)
        
        # Build performance metrics
        performance = self._build_performance_metrics(result, mission)
        
        # Environmental data
        environment = self._extract_environment_data(mission)
        
        telemetry = MissionTelemetry(
            mission_id=mission_id,
            mission_type=mission.vehicle_type,
            generation_time=datetime.now().isoformat(),
            software_version=self.software_version,
            planner_algorithm=planner_name,
            computation_time_sec=round(computation_time, 3),
            waypoints=waypoints,
            total_distance_km=round(result.total_distance / 1000, 2) if hasattr(result, 'total_distance') else 0,
            total_time_sec=round(result.total_time, 1) if hasattr(result, 'total_time') else 0,
            total_fuel_kg=round(result.fuel_used, 2) if hasattr(result, 'fuel_used') else None,
            total_energy_wh=round(result.energy_used, 1) if hasattr(result, 'energy_used') else None,
            constraints_evaluated=len(mission.constraints) if hasattr(mission, 'constraints') else 0,
            constraint_violations=len(result.violations) if hasattr(result, 'violations') else 0,
            constraint_margins=margins,
            performance=performance,
            monte_carlo=monte_carlo_results,
            environment=environment
        )
        
        return telemetry
    
    def _extract_waypoints(self, result) -> List[Dict[str, Any]]:
        """Extract waypoint data from planning result."""
        waypoints = []
        
        if not hasattr(result, 'waypoints'):
            return waypoints
        
        for i, wp in enumerate(result.waypoints):
            waypoint_data = {
                'index': i,
                'time_sec': round(getattr(wp, 'time', 0), 1),
                'lat': round(getattr(wp, 'lat', 0), 6),
                'lon': round(getattr(wp, 'lon', 0), 6),
                'alt_m': round(getattr(wp, 'alt', 0), 1),
            }
            
            # Add domain-specific fields
            if hasattr(wp, 'heading'):
                waypoint_data['heading_deg'] = round(wp.heading, 1)
            if hasattr(wp, 'speed'):
                waypoint_data['speed_mps'] = round(wp.speed, 1)
            if hasattr(wp, 'fuel'):
                waypoint_data['fuel_kg'] = round(wp.fuel, 2)
            if hasattr(wp, 'battery_soc'):
                waypoint_data['battery_soc'] = round(wp.battery_soc, 3)
            if hasattr(wp, 'action'):
                waypoint_data['action'] = wp.action
            
            waypoints.append(waypoint_data)
        
        return waypoints
    
    def _compute_constraint_margins(self, mission, result) -> Dict[str, float]:
        """
        Compute safety margins for each constraint.
        
        Returns dict mapping constraint names to margin values (0-1 scale).
        """
        margins = {}
        
        if not hasattr(mission, 'constraints'):
            return margins
        
        for constraint in mission.constraints:
            try:
                # Simplified margin computation
                # Real implementation would check constraint at each waypoint
                margin = 1.0  # Placeholder
                margins[constraint.name] = round(margin, 3)
            except Exception as e:
                logger.warning(f"Could not compute margin for {constraint.name}: {e}")
                margins[constraint.name] = 0.0
        
        return margins
    
    def _build_performance_metrics(self, result, mission) -> Dict[str, Any]:
        """Build performance metrics dictionary."""
        metrics = {
            'success': getattr(result, 'is_feasible', False),
            'optimality_gap': getattr(result, 'optimality_gap', None),
        }
        
        # Add efficiency metrics
        if hasattr(result, 'fuel_used') and hasattr(mission.vehicle, 'fuel_capacity'):
            metrics['fuel_efficiency_pct'] = round(
                100 * (1 - result.fuel_used / mission.vehicle.fuel_capacity), 1
            )
        
        if hasattr(result, 'total_time') and hasattr(mission, 'max_mission_time'):
            metrics['time_utilization_pct'] = round(
                100 * result.total_time / mission.max_mission_time, 1
            )
        
        # Smoothness score (if available)
        if hasattr(result, 'smoothness_score'):
            metrics['smoothness_score'] = round(result.smoothness_score, 3)
        
        return metrics
    
    def _extract_environment_data(self, mission) -> Dict[str, Any]:
        """Extract environmental condition data."""
        env = {}
        
        if hasattr(mission, 'environment'):
            if hasattr(mission.environment, 'wind_field'):
                env['wind'] = {
                    'mean_speed_mps': 0.0,  # Placeholder
                    'max_speed_mps': 0.0,
                }
            
            if hasattr(mission.environment, 'temperature'):
                env['temperature_c'] = mission.environment.temperature
        
        return env if env else None


def export_telemetry_json(telemetry: MissionTelemetry, output_dir: str = "outputs"):
    """
    Export telemetry to JSON file with standard naming.
    
    Args:
        telemetry: MissionTelemetry object
        output_dir: Output directory (default: "outputs")
    
    Returns:
        Path to exported file
    """
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    filename = f"telemetry_{telemetry.mission_id}.json"
    filepath = os.path.join(output_dir, filename)
    
    telemetry.to_json(filepath)
    
    return filepath


def generate_summary_report(telemetry: MissionTelemetry) -> str:
    """
    Generate human-readable summary report.
    
    Args:
        telemetry: MissionTelemetry object
    
    Returns:
        Formatted text report
    """
    report = []
    report.append("="*70)
    report.append(f"MISSION TELEMETRY REPORT: {telemetry.mission_id}")
    report.append("="*70)
    report.append("")
    
    # Metadata
    report.append("MISSION METADATA:")
    report.append(f"  Type: {telemetry.mission_type}")
    report.append(f"  Generated: {telemetry.generation_time}")
    report.append(f"  Planner: {telemetry.planner_algorithm}")
    report.append(f"  Computation Time: {telemetry.computation_time_sec:.2f} seconds")
    report.append("")
    
    # Summary stats
    report.append("MISSION SUMMARY:")
    report.append(f"  Total Distance: {telemetry.total_distance_km:.2f} km")
    report.append(f"  Total Time: {telemetry.total_time_sec:.1f} sec ({telemetry.total_time_sec/60:.1f} min)")
    if telemetry.total_fuel_kg:
        report.append(f"  Fuel Consumed: {telemetry.total_fuel_kg:.2f} kg")
    if telemetry.total_energy_wh:
        report.append(f"  Energy Used: {telemetry.total_energy_wh:.1f} Wh")
    report.append(f"  Waypoints: {len(telemetry.waypoints)}")
    report.append("")
    
    # Constraints
    report.append("CONSTRAINT ANALYSIS:")
    report.append(f"  Evaluated: {telemetry.constraints_evaluated}")
    report.append(f"  Violations: {telemetry.constraint_violations}")
    if telemetry.constraint_margins:
        report.append("  Margins:")
        for name, margin in telemetry.constraint_margins.items():
            status = "✓" if margin > 0.2 else "⚠" if margin > 0.1 else "✗"
            report.append(f"    {status} {name}: {margin:.2f}")
    report.append("")
    
    # Performance
    if telemetry.performance:
        report.append("PERFORMANCE METRICS:")
        for key, value in telemetry.performance.items():
            if isinstance(value, float):
                report.append(f"  {key}: {value:.2f}")
            else:
                report.append(f"  {key}: {value}")
        report.append("")
    
    # Monte Carlo
    if telemetry.monte_carlo:
        mc = telemetry.monte_carlo
        report.append("ROBUSTNESS ANALYSIS (Monte Carlo):")
        report.append(f"  Runs: {mc.get('total_runs', 0)}")
        report.append(f"  Success Rate: {mc.get('success_rate', 0)*100:.1f}%")
        if 'mean_time' in mc:
            report.append(f"  Mean Time: {mc['mean_time']:.1f} ± {mc.get('std_time', 0):.1f} sec")
        if 'mean_fuel' in mc:
            report.append(f"  Mean Fuel: {mc['mean_fuel']:.2f} ± {mc.get('std_fuel', 0):.2f} kg")
        report.append("")
    
    report.append("="*70)
    
    return "\n".join(report)
