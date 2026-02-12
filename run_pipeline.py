#!/usr/bin/env python3
"""
ORBIT-X Mission Planning Pipeline

End-to-end mission planning, validation, and export.
"""

import argparse
import sys
from pathlib import Path
import numpy as np
from shapely.geometry import Polygon

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from core.models import Mission, Vehicle, Waypoint, State
from core.constraints import NoFlyZoneConstraint, AltitudeConstraint, ResourceConstraint
from planning.selector import PlannerSelector
from planning.astar_planner import AStarPlanner, GreedyPlanner
from planning.milp_planner import MILPPlanner
from validation.monte_carlo import MonteCarloValidator
from validation.baseline import BaselineComparator
from utils.export import MissionExporter
from utils.plotting import MissionVisualizer


def create_sample_aircraft_mission():
    """Create a sample aircraft patrol mission."""
    
    # Define vehicle
    vehicle = Vehicle(
        name="UAV-1",
        vehicle_type="aircraft",
        mass=25.0,  # kg
        cross_sectional_area=0.5,  # m^2
        drag_coefficient=0.3,
        max_speed=30.0,  # m/s
        max_bank_angle=45.0,  # degrees
        fuel_capacity=5.0,  # kg
        properties={
            'wing_area': 2.0,
            'C_L': 0.5,
            'C_D_0': 0.025,
            'k': 0.05,
            'TSFC': 0.5,
            'eta_prop': 0.8,
        }
    )
    
    # Start state
    start = State(
        time=0.0,
        position=np.array([37.7749, -122.4194, 100.0]),  # San Francisco, 100m alt
        velocity=np.array([25.0, 0.0, 0.0]),  # 25 m/s north
        heading=0.0,
        altitude=100.0,
        fuel=5.0,
    )
    
    # Waypoints (patrol mission)
    waypoints = [
        Waypoint(
            id="wp1",
            name="North Point",
            position=np.array([37.8050, -122.4200, 150.0]),
            priority=10
        ),
        Waypoint(
            id="wp2",
            name="East Point",
            position=np.array([37.7750, -122.3900, 150.0]),
            priority=8
        ),
        Waypoint(
            id="wp3",
            name="South Point",
            position=np.array([37.7500, -122.4100, 120.0]),
            priority=9
        ),
    ]
    
    # Create mission
    mission = Mission(
        name="Aircraft Patrol Mission",
        vehicle=vehicle,
        start_state=start,
        waypoints=waypoints,
        max_duration=3600.0,  # 1 hour
        return_to_start=True,
        objectives={
            'minimize_time': 1.0,
            'minimize_fuel': 0.5,
        }
    )
    
    # Add constraints
    mission.add_constraint(AltitudeConstraint(min_altitude=50.0, max_altitude=400.0))
    mission.add_constraint(ResourceConstraint('fuel', min_level=0.5, max_level=10.0))
    
    # No-fly zone (simplified polygon)
    nfz_polygon = Polygon([
        (37.77, -122.42),
        (37.77, -122.40),
        (37.78, -122.40),
        (37.78, -122.42),
    ])
    mission.add_constraint(NoFlyZoneConstraint(nfz_polygon, altitude_range=(0, 500)))
    
    return mission


def create_sample_spacecraft_mission():
    """Create a sample spacecraft observation mission."""
    from domains.spacecraft import create_circular_orbit
    
    # Define spacecraft
    vehicle = Vehicle(
        name="CubeSat-1",
        vehicle_type="spacecraft",
        mass=5.0,  # kg
        cross_sectional_area=0.01,  # m^2
        drag_coefficient=2.2,
        max_slew_rate=5.0,  # deg/s
        battery_capacity=20.0,  # Wh
        properties={
            'I_xx': 0.01,
            'I_yy': 0.01,
            'I_zz': 0.015,
            'min_slew_time': 10.0,
            'solar_panel_area': 0.03,
            'solar_efficiency': 0.28,
            'min_SOC': 0.2,
            'camera_power': 5.0,
            'transmitter_power': 8.0,
            'avionics_power': 2.0,
        }
    )
    
    # Initial orbit (LEO, 550 km altitude, 53° inclination)
    r_eci, v_eci = create_circular_orbit(
        altitude=550.0,  # km
        inclination=53.0,  # degrees
        RAAN=0.0,
        true_anomaly=0.0
    )
    
    start = State(
        time=0.0,
        position=r_eci,
        velocity=v_eci,
        position_ECI=r_eci,
        velocity_ECI=v_eci,
        quaternion=np.array([1, 0, 0, 0]),
        angular_velocity=np.zeros(3),
        battery_SOC=0.8,
        data_storage=0.0,
    )
    
    # Ground targets
    waypoints = [
        Waypoint(
            id="target_1",
            name="New York",
            position=np.array([40.7128, -74.0060, 0.0]),
            priority=10,
            metadata={'science_value': 100}
        ),
        Waypoint(
            id="target_2",
            name="London",
            position=np.array([51.5074, -0.1278, 0.0]),
            priority=8,
            metadata={'science_value': 80}
        ),
    ]
    
    mission = Mission(
        name="Spacecraft Observation Mission",
        vehicle=vehicle,
        start_state=start,
        waypoints=waypoints,
        max_duration=7 * 24 * 3600.0,  # 7 days
        return_to_start=False,
        objectives={
            'maximize_science_value': 1.0,
            'minimize_energy': 0.3,
        }
    )
    
    return mission


def main():
    """Main pipeline execution."""
    parser = argparse.ArgumentParser(description='ORBIT-X Mission Planning Pipeline')
    parser.add_argument('--mission', choices=['aircraft', 'spacecraft'], 
                       default='aircraft', help='Mission type')
    parser.add_argument('--planner', choices=['auto', 'astar', 'milp', 'greedy'],
                       default='auto', help='Planning algorithm')
    parser.add_argument('--monte-carlo', action='store_true',
                       help='Run Monte Carlo validation')
    parser.add_argument('--n-runs', type=int, default=50,
                       help='Number of Monte Carlo runs')
    parser.add_argument('--compare', action='store_true',
                       help='Compare multiple planners')
    parser.add_argument('--output-dir', type=str, default='outputs',
                       help='Output directory')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output_dir) / args.mission
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("="*70)
    print("ORBIT-X MISSION PLANNING SYSTEM")
    print("="*70)
    
    # Create mission
    print(f"\nCreating {args.mission} mission...")
    if args.mission == 'aircraft':
        mission = create_sample_aircraft_mission()
    else:
        mission = create_sample_spacecraft_mission()
    
    print(f"Mission: {mission.name}")
    print(f"Waypoints: {len(mission.waypoints)}")
    print(f"Constraints: {len(mission.constraints)}")
    
    # Select planner
    if args.planner == 'auto':
        selector = PlannerSelector(verbose=True)
        planner = selector.select_algorithm(mission)
    elif args.planner == 'astar':
        planner = AStarPlanner(verbose=True)
    elif args.planner == 'milp':
        planner = MILPPlanner(verbose=True)
    else:  # greedy
        planner = GreedyPlanner()
    
    # Plan mission
    print(f"\n{'='*70}")
    print(f"Planning with {planner.name}...")
    print(f"{'='*70}\n")
    
    result = planner.plan_with_timing(mission)
    
    # Print results
    print("\n" + result.summary())
    
    # Export results
    print(f"\n{'='*70}")
    print("Exporting Results...")
    print(f"{'='*70}\n")
    
    MissionExporter.to_csv(result, str(output_dir / 'flight_plan.csv'))
    MissionExporter.to_json(result, str(output_dir / 'mission_result.json'))
    MissionExporter.to_report(result, str(output_dir / 'report.txt'))
    MissionExporter.to_kml(result, str(output_dir / 'trajectory.kml'))
    
    # Generate visualizations
    print(f"\n{'='*70}")
    print("Generating Visualizations...")
    print(f"{'='*70}\n")
    
    MissionVisualizer.plot_2d_trajectory(result, str(output_dir / 'trajectory_2d.png'))
    MissionVisualizer.plot_altitude_profile(result, str(output_dir / 'altitude_profile.png'))
    MissionVisualizer.plot_resource_usage(result, str(output_dir / 'resource_usage.png'))
    MissionVisualizer.plot_mission_summary(result, str(output_dir / 'summary.png'))
    
    # Baseline comparison
    if args.compare:
        print(f"\n{'='*70}")
        print("Comparing Planners...")
        print(f"{'='*70}\n")
        
        comparator = BaselineComparator()
        planners = [GreedyPlanner(), AStarPlanner(), MILPPlanner()]
        df = comparator.compare(mission, planners)
        comparator.print_comparison(df)
        
        # Save comparison
        df.to_csv(output_dir / 'planner_comparison.csv', index=False)
    
    # Monte Carlo validation
    if args.monte_carlo:
        print(f"\n{'='*70}")
        print(f"Running Monte Carlo Validation ({args.n_runs} runs)...")
        print(f"{'='*70}\n")
        
        validator = MonteCarloValidator(n_runs=args.n_runs)
        mc_report = validator.run_analysis(mission, planner)
        validator.print_report(mc_report)
        
        # Save Monte Carlo results
        import json
        with open(output_dir / 'monte_carlo_report.json', 'w') as f:
            json.dump(mc_report, f, indent=2)
    
    print(f"\n{'='*70}")
    print(f"✓ Pipeline complete! Results saved to: {output_dir}")
    print(f"{'='*70}\n")


if __name__ == '__main__':
    main()
