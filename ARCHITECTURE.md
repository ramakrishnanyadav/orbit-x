# ORBIT-X System Architecture

**Technical Design Documentation**

---

## Table of Contents

1. [Design Philosophy](#design-philosophy)
2. [Layer Architecture](#layer-architecture)
3. [Core Abstractions](#core-abstractions)
4. [Planning Algorithms](#planning-algorithms)
5. [Domain Implementations](#domain-implementations)
6. [Validation Framework](#validation-framework)
7. [Extensibility](#extensibility)

---

## Design Philosophy

### Unified Framework Principle

ORBIT-X treats **aircraft and spacecraft as specialized instances** of a general vehicle abstraction, not separate systems. This enables:

- **Code Reuse**: Single constraint implementation works for both domains
- **Consistent API**: Same planning interface regardless of vehicle type
- **Algorithmic Flexibility**: Planners work across domains
- **Maintainability**: Changes propagate to all vehicle types

### Domain-Agnostic Core

```
┌─────────────────────────────────────────┐
│   Core Abstractions (Domain-Agnostic)   │
│  - State, Vehicle, Mission, Constraint  │
└───────────────┬─────────────────────────┘
                │
        ┌───────┴────────┐
        │                │
┌───────▼──────┐  ┌─────▼────────┐
│   Aircraft   │  │  Spacecraft  │
│   Dynamics   │  │   Dynamics   │
└──────────────┘  └──────────────┘
```

---

## Layer Architecture

### Layer 1: Data Ingestion & Validation

**Purpose**: Parse mission definitions, validate inputs, normalize data.

**Components**:
- `Mission` data class - JSON/YAML deserializable
- Input sanitization (bounds checking, type validation)
- Coordinate system normalization (lat/lon/alt vs ECI)

**Example**:
```python
mission = Mission.from_json('mission_def.json')
mission.validate()  # Checks feasibility
```

### Layer 2: Constraint Framework

**Purpose**: Unified constraint representation across domains.

**Base Class**:
```python
class Constraint(ABC):
    def check(self, state, action) -> Tuple[bool, float]:
        """Returns (satisfied, margin)"""
        pass
```

**Implementations**:
- **Spatial**: `NoFlyZoneConstraint` (Shapely polygons)
- **Resource**: `ResourceConstraint` (fuel, battery, storage)
- **Temporal**: `TimeWindowConstraint` (activity windows)
- **Kinematic**: `TurnRateConstraint`, `SlewRateConstraint`

**Key Feature**: Domain-agnostic interface allows aircraft turn rate and spacecraft slew rate to use same API.

### Layer 3: Planning Core

**Purpose**: Multi-algorithm optimization with automatic selection.

**Algorithm Dispatch Logic**:
```python
if n_waypoints <= 10 and linear_constraints:
    return MILPPlanner()
elif n_waypoints <= 50:
    return AStarPlanner()
else:
    return GreedyPlanner()
```

**Implemented Algorithms**:

1. **MILP (Mixed-Integer Linear Programming)**
   - Uses: CVXPY + ECOS/MOSEK backend
   - Formulation: TSP with time/fuel objectives
   - Strengths: Globally optimal for small problems
   - Limitations: Exponential complexity (n > 20 intractable)

2. **A* Graph Search**
   - Heuristic: Admissible lower bound on cost-to-go
   - Graph: Discretized state space with lazy collision checking
   - Strengths: Good balance of speed and quality
   - Optimizations: Bidirectional search, adaptive resolution

3. **Greedy Nearest Neighbor**
   - Baseline for comparison
   - Strengths: Fast (O(n²))
   - Weaknesses: No optimality guarantee

### Layer 4: Physics Simulation

**Purpose**: Propagate vehicle state forward in time with accurate dynamics.

**Aircraft Simulator**:
```python
class AircraftDynamics(DynamicsSimulator):
    def propagate(self, state, control, dt):
        # Aerodynamic forces
        L = 0.5 * rho * V^2 * S * C_L
        D = 0.5 * rho * V^2 * S * C_D
        
        # Wind drift
        V_ground = V_air + wind
        
        # Update state (RK4 integration)
        ...
```

**Spacecraft Simulator**:
```python
class SpacecraftDynamics(DynamicsSimulator):
    def propagate(self, state, control, dt):
        # Orbital mechanics
        a = -mu/r^3 * r + a_J2 + a_drag
        
        # Power budget
        P_in = solar_panels * cos(sun_angle)
        P_out = payload_power
        SOC += (P_in - P_out) * dt / capacity
        
        ...
```

**Integration**: RK4 (4th-order Runge-Kutta) for accuracy.

### Layer 5: Validation & Verification

**Purpose**: Ensure plans satisfy constraints and quantify robustness.

**Components**:

1. **Constraint Verification**
   - Forward simulation with timestep constraint checking
   - Margin analysis (distance to violation)

2. **Monte Carlo Analysis**
   - Perturb: wind, mass, drag, fuel
   - Run N=50-1000 simulations
   - Report: success rate, mean/std metrics, failure modes

3. **Baseline Comparison**
   - Compare advanced planners vs greedy
   - Quantify improvement percentage

### Layer 6: Export & Visualization

**Purpose**: Generate outputs for external tools and human review.

**Export Formats**:
- **CSV**: Waypoint table (time, lat, lon, alt, action)
- **JSON**: Full mission result with metadata
- **KML**: Google Earth visualization
- **GMAT Script**: NASA orbital analysis tool
- **STK Scenario**: AGI Systems Tool Kit

**Visualizations**:
- 2D trajectory maps (Matplotlib)
- 3D ground tracks (Plotly)
- Time-series plots (altitude, speed, fuel, battery)
- Monte Carlo distributions

---

## Core Abstractions

### State Representation

**Unified State Space**:
```python
@dataclass
class State:
    time: float
    position: np.ndarray  # [lat, lon, alt] or [x, y, z]
    velocity: np.ndarray
    
    # Domain-specific (optional)
    heading: float         # Aircraft
    fuel: float            # Aircraft
    battery_SOC: float     # Spacecraft
    position_ECI: np.ndarray  # Spacecraft
    quaternion: np.ndarray    # Spacecraft attitude
```

**Design Rationale**: Optional fields allow same class to represent both domains without bloat.

### Vehicle Abstraction

```python
@dataclass
class Vehicle:
    name: str
    vehicle_type: str  # 'aircraft' or 'spacecraft'
    mass: float
    drag_coefficient: float
    
    # Performance limits
    max_speed: float
    max_turn_rate: float  # Aircraft
    max_slew_rate: float  # Spacecraft
    
    # Domain-specific properties dict
    properties: Dict[str, Any]
```

**Extensibility**: `properties` dict allows domain-specific parameters without modifying core class.

---

## Planning Algorithms

### A* Implementation Details

**Graph Construction**:
- Nodes: Discretized states (position + velocity)
- Edges: Dynamically-feasible maneuvers
- Edge weights: time + fuel + risk

**Heuristic Function**:
```python
def heuristic(state, goal):
    dist = ||state.pos - goal.pos||
    min_time = dist / max_speed
    min_fuel = dist * min_fuel_rate
    return w_time * min_time + w_fuel * min_fuel
```

**Admissibility**: Heuristic never overestimates true cost (optimal solution guaranteed).

**Complexity**: O(b^d) where b=branching factor, d=solution depth

### MILP Formulation

**Decision Variables**:
- x[i,j] ∈ {0,1}: Edge from waypoint i to j
- t[i]: Arrival time at waypoint i

**Objective**:
```
minimize: w_time * (t[n] - t[0]) + w_fuel * Σ fuel[i,j] * x[i,j]
```

**Constraints**:
```
# Visit each waypoint exactly once
Σ_j x[i,j] = 1  ∀i
Σ_i x[i,j] = 1  ∀j

# Subtour elimination (Miller-Tucker-Zemlin)
u[i] - u[j] + n*x[i,j] <= n-1  ∀i,j≠0

# Time consistency
t[j] >= t[i] + travel_time[i,j] - M*(1-x[i,j])
```

---

## Domain Implementations

### Aircraft Dynamics

**Coordinate System**: Geographic (lat, lon, alt)

**Forces**:
- Lift: L = 0.5 * ρ * V² * S * C_L
- Drag: D = 0.5 * ρ * V² * S * (C_D0 + k*C_L²)
- Thrust: T (control input)
- Weight: W = m*g

**Wind Model**:
- Spatial: Grid-based interpolation
- Temporal: Sinusoidal variation + noise
- Effect: V_ground = V_air + V_wind

**Turn Dynamics**:
- Coordinated turn: φ = atan(V²/(R*g))
- Turn radius: R = V² / (g * tan(φ))
- Constraint: φ <= φ_max

### Spacecraft Dynamics

**Coordinate System**: Earth-Centered Inertial (ECI)

**Orbital Mechanics**:
```
r̈ = -μ/r³ * r + a_J2 + a_drag

where:
  μ = 398600.4418 km³/s² (Earth GM)
  a_J2 = J2 perturbation (Earth oblateness)
  a_drag = -0.5 * C_d * A/m * ρ * V² * V̂
```

**Power Budget**:
```
P_in = η_solar * A_panel * 1361 W/m² * cos(θ_sun)
P_out = P_payload + P_attitude + P_comms + P_thermal

SOC(t+dt) = SOC(t) + (P_in - P_out) * dt / capacity
```

**Eclipse Detection**:
- Cylindrical shadow model
- Satellite in shadow if: ||r_perp|| < R_Earth AND r·sun < 0

**Ground Target Visibility**:
```
elevation = asin((r_sat - r_target)·ẑ_local / ||r_sat - r_target||)
visible if: elevation >= min_elevation (typically 10-30°)
```

---

## Validation Framework

### Monte Carlo Methodology

**Uncertainty Sources**:
1. Wind (±30% magnitude, ±20° direction)
2. Vehicle mass (±5%)
3. Drag coefficient (±10%)
4. Initial fuel (±2%)

**Process**:
```
for i in 1..N:
    mission_i = perturb(nominal_mission, seed_i)
    result_i = plan(mission_i)
    record(success_i, metrics_i)

report:
    success_rate = Σ success / N
    mean_time = E[time | success]
    95th_percentile_fuel = quantile(fuel, 0.95)
```

**Failure Mode Analysis**:
- Classify failures by constraint type
- Identify critical parameters (sensitivity)

### Baseline Comparison

**Metrics**:
```
improvement = (baseline_cost - advanced_cost) / baseline_cost * 100%
```

**Typical Results**:
- Time: A* 15-25% better than Greedy
- Fuel: A* 20-30% better than Greedy
- MILP: 0-5% better than A* (when tractable)

---

## Extensibility

### Adding New Vehicle Type

1. Create `NewVehicleDynamics(DynamicsSimulator)`
2. Implement `propagate(state, control, dt)`
3. Define vehicle-specific constraints (if needed)
4. Add to mission factory

### Adding New Constraint

1. Subclass `Constraint`
2. Implement `check(state, action) -> (bool, float)`
3. Optionally: `encode_linear()` for MILP support

### Adding New Planner

1. Subclass `Planner`
2. Implement `plan(mission) -> MissionResult`
3. Add to `PlannerSelector` dispatch logic

---

## Performance Optimization Tips

### For Aircraft Missions:
- Use grid resolution = 50-100m for A*
- Enable constraint pre-filtering
- Limit turn rate granularity to 5° steps

### For Spacecraft Missions:
- Precompute access windows (expensive!)
- Cache ground station passes
- Use greedy scheduler for n > 10 targets

### For MILP:
- Limit to n <= 10 waypoints
- Use warm-start from greedy solution
- Set solver time limit (e.g., 60s)

---

## Known Limitations

1. **MILP Scalability**: Intractable for n > 20 waypoints
2. **Wind Model**: Simplified (no turbulence spectrum)
3. **Spacecraft Attitude**: Assumes instant slew (no momentum wheels)
4. **Multi-Vehicle**: Not yet supported
5. **Dynamic Replanning**: Batch planning only (no online updates)

---

## Future Work

- [ ] RRT* for high-dimensional planning
- [ ] Multi-vehicle coordination (collision avoidance)
- [ ] Model Predictive Control (MPC) for real-time replanning
- [ ] Machine learning for heuristic tuning
- [ ] GPU acceleration for Monte Carlo
- [ ] Web-based mission designer UI

---

**Last Updated**: February 2026
