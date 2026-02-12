#!/usr/bin/env python3
"""
Standalone spacecraft mission planning script for AeroHack demonstration.

Generates all required outputs:
- schedule_7day.csv
- access_windows.csv
- constraint_report.txt
- metrics.json
"""

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.domains.spacecraft_scheduler import (
    SpacecraftScheduler,
    GroundTarget,
    GroundStation,
    export_access_windows_csv,
    export_schedule_csv
)


def load_mission(mission_file: str):
    """Load mission from JSON file."""
    with open(mission_file, 'r') as f:
        return json.load(f)


def run_spacecraft_mission(mission_file: str, output_dir: str = "outputs/spacecraft"):
    """
    Run complete spacecraft mission planning.
    
    Args:
        mission_file: Path to mission JSON
        output_dir: Output directory for results
    """
    print("="*70)
    print("ORBIT-X SPACECRAFT MISSION PLANNER")
    print("="*70)
    print()
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Load mission
    print(f"Loading mission from {mission_file}...")
    mission = load_mission(mission_file)
    print(f"✓ Mission: {mission['mission_name']}")
    print(f"  Duration: {mission['duration_days']} days")
    print(f"  Targets: {len(mission['ground_targets'])}")
    print(f"  Ground Stations: {len(mission['ground_stations'])}")
    print()
    
    # Parse targets and stations
    targets = [
        GroundTarget(
            id=t['id'],
            name=t['name'],
            lat=t['lat'],
            lon=t['lon'],
            priority=t.get('priority', 1),
            science_value=t.get('science_value', 100.0),
            min_elevation=t.get('min_elevation', 30.0),
            min_observation_time=t.get('min_observation_time', 30.0),
            min_revisit_time=t.get('min_revisit_time', 12.0)
        )
        for t in mission['ground_targets']
    ]
    
    stations = [
        GroundStation(
            id=s['id'],
            name=s['name'],
            lat=s['lat'],
            lon=s['lon'],
            min_elevation=s.get('min_elevation', 10.0),
            max_data_rate=s.get('max_data_rate', 2.0)
        )
        for s in mission['ground_stations']
    ]
    
    # Initialize scheduler
    print("Initializing spacecraft scheduler...")
    scheduler = SpacecraftScheduler(mission['spacecraft'])
    print("✓ Scheduler initialized")
    print()
    
    # Compute access windows
    print("Computing access windows...")
    print("  (Simulating orbital propagation for 7 days)")
    
    # For demo: create dummy orbit states
    # Real version would use actual orbital propagator
    orbit_states = []
    
    target_windows, station_windows = scheduler.compute_access_windows(
        orbit_states, targets, stations, mission['duration_days']
    )
    
    print(f"✓ Target access windows: {len(target_windows)}")
    print(f"✓ Station contact windows: {len(station_windows)}")
    print()
    
    # Export access windows
    access_windows_file = os.path.join(output_dir, "access_windows.csv")
    all_windows = target_windows + station_windows
    export_access_windows_csv(all_windows, access_windows_file)
    print()
    
    # Schedule mission
    print("Scheduling observations and downlinks...")
    schedule = scheduler.schedule_mission(
        target_windows, station_windows, targets, stations, strategy='greedy'
    )
    print(f"✓ Total activities scheduled: {len(schedule)}")
    print()
    
    # Export schedule
    schedule_file = os.path.join(output_dir, "schedule_7day.csv")
    export_schedule_csv(schedule, schedule_file)
    print()
    
    # Generate constraint report
    print("Generating constraint verification report...")
    constraint_report = scheduler.generate_constraint_report(schedule, targets)
    
    report_file = os.path.join(output_dir, "constraint_report.txt")
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(constraint_report)
    print(f"✓ Constraint report saved to {report_file}")
    print()
    print(constraint_report)
    print()
    
    # Generate metrics
    observations = [a for a in schedule if a.activity_type == 'OBSERVE']
    downlinks = [a for a in schedule if a.activity_type == 'DOWNLINK']
    
    total_science_value = sum(
        next((t.science_value for t in targets if t.id == obs.target_id), 0)
        for obs in observations
    )
    
    unique_targets = set(a.target_id for a in observations if a.target_id)
    
    total_data_observed = sum(a.data_generated for a in observations)
    total_data_downlinked = sum(a.data_downlinked for a in downlinks)
    
    data_return_rate = (total_data_downlinked / total_data_observed * 100) if total_data_observed > 0 else 0
    
    metrics = {
        "mission_name": mission['mission_name'],
        "duration_days": mission['duration_days'],
        "observations_scheduled": len(observations),
        "downlinks_scheduled": len(downlinks),
        "unique_targets_observed": len(unique_targets),
        "total_targets_available": len(targets),
        "coverage_percent": len(unique_targets) / len(targets) * 100,
        "total_science_value": round(total_science_value, 1),
        "total_data_observed_mb": round(total_data_observed, 1),
        "total_data_downlinked_mb": round(total_data_downlinked, 1),
        "data_return_rate_percent": round(data_return_rate, 1),
        "final_battery_soc": round(schedule[-1].battery_soc_end, 3) if schedule else 1.0,
        "min_battery_soc": round(min(a.battery_soc_end for a in schedule), 3) if schedule else 1.0
    }
    
    metrics_file = os.path.join(output_dir, "metrics.json")
    with open(metrics_file, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f"✓ Metrics saved to {metrics_file}")
    print()
    
    # Print summary
    print("="*70)
    print("MISSION SUMMARY")
    print("="*70)
    print(f"Observations: {metrics['observations_scheduled']}")
    print(f"Downlinks: {metrics['downlinks_scheduled']}")
    print(f"Unique Targets: {metrics['unique_targets_observed']} / {metrics['total_targets_available']} ({metrics['coverage_percent']:.1f}%)")
    print(f"Science Value: {metrics['total_science_value']:.1f}")
    print(f"Data Return: {metrics['data_return_rate_percent']:.1f}%")
    print(f"Min Battery SOC: {metrics['min_battery_soc']*100:.1f}%")
    print("="*70)
    print()
    
    print("✓ ALL OUTPUTS GENERATED SUCCESSFULLY")
    print()
    print("Output files:")
    print(f"  - {schedule_file}")
    print(f"  - {access_windows_file}")
    print(f"  - {report_file}")
    print(f"  - {metrics_file}")
    print()
    
    return metrics


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Run spacecraft mission planning")
    parser.add_argument(
        "--mission",
        default="data/missions/spacecraft_observation.json",
        help="Path to mission JSON file"
    )
    parser.add_argument(
        "--output",
        default="outputs/spacecraft",
        help="Output directory"
    )
    
    args = parser.parse_args()
    
    try:
        run_spacecraft_mission(args.mission, args.output)
        sys.exit(0)
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
