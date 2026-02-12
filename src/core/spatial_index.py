"""
Spatial indexing for fast constraint checking.

Uses R-tree data structure for O(log n) collision detection
with no-fly zones and obstacles.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import logging

try:
    from rtree import index
    RTREE_AVAILABLE = True
except ImportError:
    RTREE_AVAILABLE = False
    logging.warning("rtree not available - falling back to linear search")

from shapely.geometry import Point, Polygon

logger = logging.getLogger(__name__)


@dataclass
class SpatialZone:
    """Spatial zone with associated constraints."""
    id: str
    polygon: Polygon
    altitude_min: float
    altitude_max: float
    zone_type: str  # 'no_fly', 'restricted', 'warning'
    priority: int = 1


class SpatialConstraintIndex:
    """
    R-tree spatial index for fast no-fly zone collision detection.
    
    Complexity: O(log n) instead of O(n) for n zones.
    
    Benefits:
    - 100x+ speedup for missions with 1000+ zones
    - Memory efficient (only bounding boxes in index)
    - Supports dynamic zone updates
    
    Example:
        >>> zones = [...]  # List of no-fly zones
        >>> index = SpatialConstraintIndex(zones)
        >>> violated, zone = index.check_violation(lat=40.7, lon=-74.0, alt=100)
    """
    
    def __init__(self, zones: List[SpatialZone]):
        """
        Initialize spatial index.
        
        Args:
            zones: List of SpatialZone objects
        """
        self.zones = {}
        self.use_rtree = RTREE_AVAILABLE
        
        if self.use_rtree:
            # R-tree index (2D spatial + 1D altitude handled separately)
            self.idx = index.Index()
            logger.info("Using R-tree spatial index")
        else:
            logger.warning("R-tree not available, using linear search (slower)")
        
        # Build index
        for i, zone in enumerate(zones):
            self._add_zone(i, zone)
        
        logger.info(f"Spatial index built with {len(zones)} zones")
    
    def _add_zone(self, zone_id: int, zone: SpatialZone):
        """Add zone to index."""
        self.zones[zone_id] = zone
        
        if self.use_rtree:
            # Compute bounding box
            bounds = zone.polygon.bounds  # (minx, miny, maxx, maxy)
            
            # Insert into R-tree (2D only, altitude checked separately)
            self.idx.insert(zone_id, bounds)
    
    def check_violation(self, lat: float, lon: float, alt: float) -> Tuple[bool, Optional[SpatialZone]]:
        """
        Fast point-in-polygon check for constraint violation.
        
        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude (meters)
        
        Returns:
            violated: True if point violates a constraint
            zone: The violated zone (if any)
        
        Complexity: O(log n) average case with R-tree
        """
        point = Point(lon, lat)  # Note: Point(x, y) = Point(lon, lat)
        
        if self.use_rtree:
            # Query R-tree for potential intersections (fast)
            candidates = list(self.idx.intersection((lon, lat, lon, lat)))
        else:
            # Fallback: check all zones (slow)
            candidates = list(self.zones.keys())
        
        # Detailed check only for candidates
        for zone_id in candidates:
            zone = self.zones[zone_id]
            
            # Check altitude first (cheap)
            if not (zone.altitude_min <= alt <= zone.altitude_max):
                continue
            
            # Check if point is in polygon (more expensive)
            if zone.polygon.contains(point):
                return True, zone
        
        return False, None
    
    def check_path_violation(self, lat1: float, lon1: float, alt1: float,
                            lat2: float, lon2: float, alt2: float,
                            num_samples: int = 10) -> Tuple[bool, Optional[SpatialZone]]:
        """
        Check if path segment violates any constraints.
        
        Samples path at multiple points for collision detection.
        
        Args:
            lat1, lon1, alt1: Start point
            lat2, lon2, alt2: End point
            num_samples: Number of intermediate points to check
        
        Returns:
            violated: True if path violates a constraint
            zone: The first violated zone (if any)
        """
        # Sample path
        for i in range(num_samples + 1):
            t = i / num_samples
            lat = lat1 + t * (lat2 - lat1)
            lon = lon1 + t * (lon2 - lon1)
            alt = alt1 + t * (alt2 - alt1)
            
            violated, zone = self.check_violation(lat, lon, alt)
            if violated:
                return True, zone
        
        return False, None
    
    def get_closest_approach(self, lat: float, lon: float, alt: float) -> Tuple[float, Optional[SpatialZone]]:
        """
        Compute minimum distance to nearest constraint boundary.
        
        Useful for margin analysis and safety scoring.
        
        Args:
            lat, lon, alt: Point coordinates
        
        Returns:
            distance: Minimum distance to any zone (meters)
            nearest_zone: The nearest zone
        """
        point = Point(lon, lat)
        
        min_distance = float('inf')
        nearest_zone = None
        
        if self.use_rtree:
            # Query nearby zones (within reasonable range)
            search_radius = 0.1  # degrees (~10 km)
            bounds = (lon - search_radius, lat - search_radius,
                     lon + search_radius, lat + search_radius)
            candidates = list(self.idx.intersection(bounds))
        else:
            candidates = list(self.zones.keys())
        
        for zone_id in candidates:
            zone = self.zones[zone_id]
            
            # Skip if altitude doesn't match
            if not (zone.altitude_min <= alt <= zone.altitude_max):
                continue
            
            # Compute distance to boundary
            if zone.polygon.contains(point):
                # Inside zone - distance is negative (violation)
                distance = -zone.polygon.exterior.distance(point)
            else:
                # Outside zone - distance is positive (safe)
                distance = zone.polygon.exterior.distance(point)
            
            # Convert degrees to meters (approximate)
            distance_m = distance * 111000  # 1 degree ≈ 111 km
            
            if abs(distance_m) < abs(min_distance):
                min_distance = distance_m
                nearest_zone = zone
        
        return min_distance, nearest_zone
    
    def get_zones_in_region(self, lat_min: float, lat_max: float,
                           lon_min: float, lon_max: float) -> List[SpatialZone]:
        """
        Get all zones intersecting a bounding box.
        
        Useful for mission planning and visualization.
        
        Args:
            lat_min, lat_max: Latitude bounds (degrees)
            lon_min, lon_max: Longitude bounds (degrees)
        
        Returns:
            List of zones in region
        """
        if self.use_rtree:
            bounds = (lon_min, lat_min, lon_max, lat_max)
            zone_ids = list(self.idx.intersection(bounds))
        else:
            zone_ids = list(self.zones.keys())
        
        return [self.zones[zid] for zid in zone_ids]
    
    def update_zone(self, zone_id: int, new_zone: SpatialZone):
        """
        Dynamically update a zone (for real-time planning).
        
        Args:
            zone_id: ID of zone to update
            new_zone: New zone definition
        """
        if zone_id in self.zones:
            # Remove old zone
            if self.use_rtree:
                old_bounds = self.zones[zone_id].polygon.bounds
                self.idx.delete(zone_id, old_bounds)
        
        # Add new zone
        self._add_zone(zone_id, new_zone)
        
        logger.info(f"Updated zone {zone_id}: {new_zone.id}")
    
    def remove_zone(self, zone_id: int):
        """
        Remove a zone from the index.
        
        Args:
            zone_id: ID of zone to remove
        """
        if zone_id in self.zones:
            if self.use_rtree:
                bounds = self.zones[zone_id].polygon.bounds
                self.idx.delete(zone_id, bounds)
            
            del self.zones[zone_id]
            logger.info(f"Removed zone {zone_id}")


class LinearSearchIndex:
    """
    Fallback implementation using linear search.
    
    Used when rtree is not available. Slower but still functional.
    """
    
    def __init__(self, zones: List[SpatialZone]):
        """Initialize with list of zones."""
        self.zones = {i: zone for i, zone in enumerate(zones)}
    
    def check_violation(self, lat: float, lon: float, alt: float) -> Tuple[bool, Optional[SpatialZone]]:
        """Check violation using linear search."""
        point = Point(lon, lat)
        
        for zone in self.zones.values():
            if zone.altitude_min <= alt <= zone.altitude_max:
                if zone.polygon.contains(point):
                    return True, zone
        
        return False, None


# Convenience function
def create_spatial_index(zones: List[SpatialZone]) -> SpatialConstraintIndex:
    """
    Create appropriate spatial index based on available libraries.
    
    Args:
        zones: List of spatial zones
    
    Returns:
        SpatialConstraintIndex (with R-tree if available)
    """
    return SpatialConstraintIndex(zones)
