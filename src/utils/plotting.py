"""
Visualization utilities for mission results.

Generates 2D/3D plots, timelines, and analysis charts.
"""

from typing import List, Optional
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

from ..core.models import MissionResult, State, Waypoint


class MissionVisualizer:
    """Generate visualizations for mission results."""
    
    @staticmethod
    def plot_2d_trajectory(result: MissionResult, output_path: str = None, 
                          show_waypoints: bool = True, show_constraints: bool = True):
        """
        Plot 2D trajectory (top-down view).
        
        Args:
            result: Mission result
            output_path: Optional path to save figure
            show_waypoints: Show waypoint markers
            show_constraints: Show no-fly zones
        """
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Extract trajectory
        lats = [s.position[0] for s in result.states]
        lons = [s.position[1] for s in result.states]
        
        # Plot trajectory
        ax.plot(lons, lats, 'b-', linewidth=2, label='Trajectory')
        ax.plot(lons[0], lats[0], 'go', markersize=12, label='Start')
        ax.plot(lons[-1], lats[-1], 'ro', markersize=12, label='End')
        
        # Plot waypoints
        if show_waypoints and result.mission.waypoints:
            wp_lats = [wp.position[0] for wp in result.mission.waypoints]
            wp_lons = [wp.position[1] for wp in result.mission.waypoints]
            ax.plot(wp_lons, wp_lats, 'k^', markersize=10, label='Waypoints')
            
            # Label waypoints
            for i, wp in enumerate(result.mission.waypoints):
                ax.annotate(f'WP{i}', 
                          (wp.position[1], wp.position[0]),
                          xytext=(5, 5), textcoords='offset points')
        
        # Plot no-fly zones
        if show_constraints:
            from ..core.constraints import NoFlyZoneConstraint
            for constraint in result.mission.constraints:
                if isinstance(constraint, NoFlyZoneConstraint):
                    # Extract polygon coordinates
                    coords = list(constraint.polygon.exterior.coords)
                    poly = MplPolygon(coords, alpha=0.3, facecolor='red', 
                                    edgecolor='darkred', linewidth=2, 
                                    label='No-Fly Zone')
                    ax.add_patch(poly)
        
        ax.set_xlabel('Longitude (deg)', fontsize=12)
        ax.set_ylabel('Latitude (deg)', fontsize=12)
        ax.set_title(f'Mission Trajectory: {result.mission.name}', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.axis('equal')
        
        plt.tight_layout()
        
        if output_path:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved 2D trajectory plot to {output_path}")
        else:
            plt.show()
        
        plt.close()
    
    @staticmethod
    def plot_altitude_profile(result: MissionResult, output_path: str = None):
        """
        Plot altitude vs time.
        
        Args:
            result: Mission result
            output_path: Optional path to save figure
        """
        fig, ax = plt.subplots(figsize=(12, 6))
        
        times = [s.time for s in result.states]
        altitudes = [s.position[2] if len(s.position) > 2 else 0 for s in result.states]
        
        ax.plot(times, altitudes, 'b-', linewidth=2)
        ax.fill_between(times, altitudes, alpha=0.3)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Altitude (m)', fontsize=12)
        ax.set_title('Altitude Profile', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if output_path:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved altitude profile to {output_path}")
        else:
            plt.show()
        
        plt.close()
    
    @staticmethod
    def plot_resource_usage(result: MissionResult, output_path: str = None):
        """
        Plot fuel/battery usage over time.
        
        Args:
            result: Mission result
            output_path: Optional path to save figure
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        times = [s.time for s in result.states]
        
        # Fuel plot
        fuels = [s.fuel if s.fuel is not None else 0 for s in result.states]
        if any(f > 0 for f in fuels):
            ax1.plot(times, fuels, 'r-', linewidth=2)
            ax1.set_ylabel('Fuel (kg)', fontsize=12)
            ax1.set_title('Fuel Usage', fontsize=12, fontweight='bold')
            ax1.grid(True, alpha=0.3)
        else:
            ax1.text(0.5, 0.5, 'No fuel data', ha='center', va='center', 
                    transform=ax1.transAxes, fontsize=14)
            ax1.set_ylabel('Fuel (kg)', fontsize=12)
        
        # Battery plot
        socs = [s.battery_SOC if s.battery_SOC is not None else 0 for s in result.states]
        if any(s > 0 for s in socs):
            ax2.plot(times, [s*100 for s in socs], 'g-', linewidth=2)
            ax2.set_ylabel('Battery SOC (%)', fontsize=12)
            ax2.set_title('Battery State of Charge', fontsize=12, fontweight='bold')
            ax2.axhline(y=20, color='r', linestyle='--', label='Min SOC')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
        else:
            ax2.text(0.5, 0.5, 'No battery data', ha='center', va='center',
                    transform=ax2.transAxes, fontsize=14)
            ax2.set_ylabel('Battery SOC (%)', fontsize=12)
        
        ax2.set_xlabel('Time (s)', fontsize=12)
        
        plt.tight_layout()
        
        if output_path:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved resource usage plot to {output_path}")
        else:
            plt.show()
        
        plt.close()
    
    @staticmethod
    def plot_mission_summary(result: MissionResult, output_path: str = None):
        """
        Create comprehensive mission summary with multiple subplots.
        
        Args:
            result: Mission result
            output_path: Optional path to save figure
        """
        fig = plt.figure(figsize=(16, 12))
        
        # 2D trajectory (top left)
        ax1 = plt.subplot(2, 2, 1)
        lats = [s.position[0] for s in result.states]
        lons = [s.position[1] for s in result.states]
        ax1.plot(lons, lats, 'b-', linewidth=2)
        ax1.plot(lons[0], lats[0], 'go', markersize=10, label='Start')
        ax1.plot(lons[-1], lats[-1], 'ro', markersize=10, label='End')
        ax1.set_xlabel('Longitude')
        ax1.set_ylabel('Latitude')
        ax1.set_title('2D Trajectory')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Altitude profile (top right)
        ax2 = plt.subplot(2, 2, 2)
        times = [s.time for s in result.states]
        altitudes = [s.position[2] if len(s.position) > 2 else 0 for s in result.states]
        ax2.plot(times, altitudes, 'b-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Altitude (m)')
        ax2.set_title('Altitude Profile')
        ax2.grid(True, alpha=0.3)
        
        # Speed profile (bottom left)
        ax3 = plt.subplot(2, 2, 3)
        speeds = [np.linalg.norm(s.velocity) if s.velocity is not None else 0 
                 for s in result.states]
        ax3.plot(times, speeds, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Speed (m/s)')
        ax3.set_title('Speed Profile')
        ax3.grid(True, alpha=0.3)
        
        # Metrics summary (bottom right)
        ax4 = plt.subplot(2, 2, 4)
        ax4.axis('off')
        
        summary_text = f"""
        MISSION SUMMARY
        {'='*40}
        
        Mission: {result.mission.name}
        Algorithm: {result.algorithm_used}
        Success: {result.success}
        
        Total Time: {result.total_time:.1f} s ({result.total_time/60:.1f} min)
        Total Distance: {result.total_distance/1000:.2f} km
        Fuel Used: {result.fuel_used:.2f} kg
        
        Planning Time: {result.planning_time:.2f} s
        Violations: {result.constraint_violations}
        
        Objective Value: {result.objective_value:.3f}
        """
        
        ax4.text(0.1, 0.5, summary_text, fontsize=11, family='monospace',
                verticalalignment='center')
        
        plt.suptitle(f'Mission Analysis: {result.mission.name}', 
                    fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        if output_path:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved mission summary to {output_path}")
        else:
            plt.show()
        
        plt.close()
