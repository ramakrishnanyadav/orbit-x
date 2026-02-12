"""
Export mission results to various formats.

Supports: CSV, JSON, KML, GMAT scripts.
"""

import json
import csv
from typing import List
from pathlib import Path
import numpy as np

from ..core.models import MissionResult, State


class MissionExporter:
    """Export mission results to standard formats."""
    
    @staticmethod
    def to_csv(result: MissionResult, output_path: str):
        """
        Export trajectory to CSV file.
        
        Args:
            result: Mission result
            output_path: Output file path
        """
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Header
            writer.writerow([
                'time_sec', 'lat', 'lon', 'alt_m', 'heading_deg', 
                'speed_mps', 'fuel_kg', 'battery_soc', 'action'
            ])
            
            # Data rows
            for i, state in enumerate(result.states):
                lat = state.position[0] if len(state.position) > 0 else 0
                lon = state.position[1] if len(state.position) > 1 else 0
                alt = state.position[2] if len(state.position) > 2 else 0
                
                heading = state.heading or 0.0
                speed = np.linalg.norm(state.velocity) if state.velocity is not None else 0.0
                fuel = state.fuel if state.fuel is not None else 0.0
                soc = state.battery_SOC if state.battery_SOC is not None else 0.0
                
                # Determine action
                if i == 0:
                    action = "start"
                elif i == len(result.states) - 1:
                    action = "end"
                else:
                    action = "transit"
                
                writer.writerow([
                    f"{state.time:.1f}",
                    f"{lat:.6f}",
                    f"{lon:.6f}",
                    f"{alt:.1f}",
                    f"{heading:.1f}",
                    f"{speed:.2f}",
                    f"{fuel:.3f}",
                    f"{soc:.3f}",
                    action
                ])
        
        print(f"Exported trajectory to {output_path}")
    
    @staticmethod
    def to_json(result: MissionResult, output_path: str):
        """
        Export mission result to JSON.
        
        Args:
            result: Mission result
            output_path: Output file path
        """
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        
        data = {
            'mission_name': result.mission.name,
            'success': result.success,
            'algorithm': result.algorithm_used,
            'planning_time_s': result.planning_time,
            'metrics': {
                'total_time_s': result.total_time,
                'total_distance_m': result.total_distance,
                'fuel_used_kg': result.fuel_used,
                'energy_used_wh': result.energy_used,
                'constraint_violations': result.constraint_violations,
                'objective_value': result.objective_value,
            },
            'trajectory': [
                {
                    'time': state.time,
                    'position': state.position.tolist() if isinstance(state.position, np.ndarray) else state.position,
                    'velocity': state.velocity.tolist() if isinstance(state.velocity, np.ndarray) else state.velocity,
                    'heading': state.heading,
                    'altitude': state.altitude,
                    'fuel': state.fuel,
                    'battery_soc': state.battery_SOC,
                }
                for state in result.states
            ]
        }
        
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Exported mission to {output_path}")
    
    @staticmethod
    def to_kml(result: MissionResult, output_path: str):
        """
        Export trajectory to KML for Google Earth.
        
        Args:
            result: Mission result
            output_path: Output file path
        """
        try:
            import simplekml
        except ImportError:
            print("simplekml not installed. Skipping KML export.")
            return
        
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        
        kml = simplekml.Kml()
        
        # Create line string for path
        coords = []
        for state in result.states:
            lat = state.position[0] if len(state.position) > 0 else 0
            lon = state.position[1] if len(state.position) > 1 else 0
            alt = state.position[2] if len(state.position) > 2 else 0
            coords.append((lon, lat, alt))  # KML uses (lon, lat, alt)
        
        linestring = kml.newlinestring(name=f"{result.mission.name} Path")
        linestring.coords = coords
        linestring.altitudemode = simplekml.AltitudeMode.absolute
        linestring.style.linestyle.width = 3
        linestring.style.linestyle.color = simplekml.Color.red
        
        # Add waypoint markers
        for i, state in enumerate(result.states):
            if i % max(1, len(result.states) // 10) == 0:  # Sample every 10th point
                lat = state.position[0] if len(state.position) > 0 else 0
                lon = state.position[1] if len(state.position) > 1 else 0
                alt = state.position[2] if len(state.position) > 2 else 0
                
                pnt = kml.newpoint(name=f"t={state.time:.0f}s")
                pnt.coords = [(lon, lat, alt)]
                pnt.altitudemode = simplekml.AltitudeMode.absolute
        
        kml.save(output_path)
        print(f"Exported KML to {output_path}")
    
    @staticmethod
    def to_report(result: MissionResult, output_path: str):
        """
        Generate text report.
        
        Args:
            result: Mission result
            output_path: Output file path
        """
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w') as f:
            f.write("="*70 + "\n")
            f.write(f"MISSION PLANNING REPORT: {result.mission.name}\n")
            f.write("="*70 + "\n\n")
            
            f.write(f"Success: {result.success}\n")
            f.write(f"Algorithm: {result.algorithm_used}\n")
            f.write(f"Planning Time: {result.planning_time:.2f} seconds\n\n")
            
            f.write("PERFORMANCE METRICS:\n")
            f.write("-"*70 + "\n")
            f.write(f"Total Mission Time: {result.total_time:.1f} s ({result.total_time/60:.1f} min)\n")
            f.write(f"Total Distance: {result.total_distance/1000:.2f} km\n")
            f.write(f"Fuel Used: {result.fuel_used:.2f} kg\n")
            f.write(f"Energy Used: {result.energy_used:.2f} Wh\n")
            f.write(f"Constraint Violations: {result.constraint_violations}\n")
            f.write(f"Objective Value: {result.objective_value:.3f}\n\n")
            
            if result.objective_breakdown:
                f.write("OBJECTIVE BREAKDOWN:\n")
                f.write("-"*70 + "\n")
                for key, value in result.objective_breakdown.items():
                    f.write(f"  {key}: {value:.3f}\n")
                f.write("\n")
            
            if result.constraint_margins:
                f.write("CONSTRAINT MARGINS:\n")
                f.write("-"*70 + "\n")
                for key, value in result.constraint_margins.items():
                    f.write(f"  {key}: {value:.2f}\n")
                f.write("\n")
            
            f.write("="*70 + "\n")
        
        print(f"Exported report to {output_path}")
