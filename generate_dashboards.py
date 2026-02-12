#!/usr/bin/env python3
"""
ORBIT-X Dashboard Generator

Generates professional interactive HTML dashboards for both aircraft and spacecraft missions.
Run this after mission execution to create visualization dashboards.

Usage:
    python generate_dashboards.py
    python generate_dashboards.py --mission spacecraft
    python generate_dashboards.py --mission aircraft
"""

import argparse
import os
import sys
import logging
from pathlib import Path

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from utils.dashboard import MissionDashboardGenerator

logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description='Generate ORBIT-X mission dashboards')
    parser.add_argument('--mission', choices=['aircraft', 'spacecraft', 'both'], 
                       default='both', help='Which mission dashboard to generate')
    parser.add_argument('--output-dir', default='outputs', help='Output directory')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("ORBIT-X PROFESSIONAL DASHBOARD GENERATOR")
    print("=" * 70)
    print()
    
    generator = MissionDashboardGenerator()
    
    if args.mission in ['spacecraft', 'both']:
        print("Generating Spacecraft Mission Dashboard...")
        try:
            spacecraft_data = {
                'metrics_file': os.path.join(args.output_dir, 'spacecraft', 'metrics.json'),
                'schedule_file': os.path.join(args.output_dir, 'spacecraft', 'schedule_7day.csv'),
                'windows_file': os.path.join(args.output_dir, 'spacecraft', 'access_windows.csv'),
            }
            
            output_file = generator.create_spacecraft_dashboard(
                spacecraft_data,
                os.path.join(args.output_dir, 'spacecraft', 'dashboard.html')
            )
            
            print(f"✓ Spacecraft dashboard: {output_file}")
            print(f"  → Open in browser to view interactive visualization")
            
        except FileNotFoundError as e:
            print(f"✗ Spacecraft data not found: {e}")
            print(f"  → Run 'python run_spacecraft_mission.py' first")
    
    if args.mission in ['aircraft', 'both']:
        print("\nGenerating Aircraft Mission Dashboard...")
        try:
            aircraft_data = {
                'metrics_file': os.path.join(args.output_dir, 'aircraft', 'metrics.json'),
                'plan_file': os.path.join(args.output_dir, 'aircraft', 'flight_plan.csv'),
                'monte_carlo_file': os.path.join(args.output_dir, 'aircraft', 'monte_carlo_results.json'),
            }
            
            output_file = generator.create_aircraft_dashboard(
                aircraft_data,
                os.path.join(args.output_dir, 'aircraft', 'dashboard.html')
            )
            
            print(f"✓ Aircraft dashboard: {output_file}")
            print(f"  → Open in browser to view interactive visualization")
            
        except FileNotFoundError as e:
            print(f"✗ Aircraft data not found: {e}")
            print(f"  → Run 'python run_pipeline.py --mission aircraft' first")
    
    print()
    print("=" * 70)
    print("DASHBOARD GENERATION COMPLETE")
    print("=" * 70)
    print()
    print("Next steps:")
    print("  1. Open dashboard HTML files in your browser")
    print("  2. Interactive features: zoom, pan, hover for details")
    print("  3. Export as static images if needed (right-click)")
    print()


if __name__ == '__main__':
    main()
