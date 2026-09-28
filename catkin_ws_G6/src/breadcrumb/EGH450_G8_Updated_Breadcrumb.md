# Breadcrumb Route-Planning Changes

## Purpose

Breadcrumb was integrated into the EGH450 Guidance system to provide obstacle-aware routing while preserving the existing mission structure, SPAR motion layer, and pilot-controlled arming/OFFBOARD workflow.

```text
Mission waypoint / ROI / ArUco target
            ↓
        navigate_to()
            ↓
   Breadcrumb RequestPath
            ↓
     Safe route points
            ↓
        fly_to()
            ↓
           SPAR
```

Guidance decides **where the UAV should go**, Breadcrumb decides **how to safely reach it**, and SPAR performs the motion.

## 1. Guidance Integration

Guidance now uses `/breadcrumb/request_path` through:

- `plan_path()` — requests a safe route.
- `navigate_to()` — executes Breadcrumb path points through the existing `fly_to()` / SPAR layer.

If Breadcrumb cannot return a valid path, Guidance fails safely rather than reverting to direct flight.

## 2. Obstacle-Aware Mission Motions

Breadcrumb is used for the main horizontal movements:

- Lawn-mower search navigation.
- ROI / target approach.
- Return to the interrupted search location.
- Final ArUco landing approach.

Direct SPAR movement remains for vertical operations:

- Takeoff.
- Payload descent.
- Payload climb.
- Landing.

## 3. Safe Endpoint Handling

Breadcrumb may adjust a destination if the requested point is inside an obstacle, inside an inflated clearance region, or outside the permitted planning region.

Guidance accepts the safe Breadcrumb endpoint instead of commanding the original unsafe position. For payload operations, the actual safe approach XY is captured before descending so the UAV does not navigate around an obstacle and then descend back toward it.

## 4. Theta* Cost Correction

The original Theta* line-of-sight cost used a different scale from normal A* movement:

```text
Straight step = 10
Diagonal step = 14
Theta* LOS ≈ 1 × distance
```

Theta* was corrected to approximately:

```text
10 × Euclidean distance
```

This made A* and Theta* route costs consistent and improved route quality.

## 5. Movement Policy

General non-axis-aligned navigation can use diagonal movement and Theta* for efficient routing.

Axis-aligned lawn-mower legs use cardinal movement only, with Theta* disabled for that request, producing clearer obstacle bypass behaviour.

Breadcrumb-generated path orientation is ignored by Guidance. The commanded mission yaw remains `0.0 rad`, so obstacle avoidance does not introduce heading changes.

## 6. Sparse Paths

Breadcrumb sparse paths are used where available.

Instead of commanding every grid cell:

```text
→ → → → ↓ ↓ ↓ → → →
```

Guidance receives only meaningful corners and endpoints:

```text
start → corner → corner → destination
```

Guidance uses `path_sparse` when populated and otherwise falls back to the full path.

## 7. Obstacle Clearance

Obstacle inflation was increased from:

```text
0.2 m → 0.4 m
```

The occupancy-grid boundary was separated from internal obstacle inflation so the 0.4 m obstacle padding does not unnecessarily shrink the usable flight area near the map edge. The map boundary itself remains impassable.

## 8. Planning Bounds

Breadcrumb retains the mission limits:

```text
X: -4.0 m to +4.0 m
Y: -3.2 m to +2.0 m
Z: +0.5 m to +4.0 m
```

The lawn-mower search was later extended from `x = -3.2 m` to `x = -3.6 m`, which remains inside the existing X bounds.

## 9. Lawn-Mower Waypoint Refinement

The original long horizontal legs allowed the planner to remain offset from the nominal lawn-mower line after avoiding an obstacle.

Intermediate waypoints were therefore added along each row, typically:

```text
-3.6 → -2.0 → 0.0 → +2.0 → +3.2
```

or the reverse direction.

This breaks long rows into shorter local planning segments and forces the UAV back toward the intended lawn-mower line shortly after an obstacle detour.

## 10. Planner Startup Safety

Breadcrumb was added to the physical `control.launch` stack.

Guidance now waits for `/breadcrumb/request_path` before progressing toward autonomous flight. Breadcrumb only advertises this service after receiving a valid occupancy grid on `/grid`.

```text
environment.launch
      ↓
/grid available
      ↓
Breadcrumb ready
      ↓
/request_path available
      ↓
Guidance continues
      ↓
SPAR / pilot ARM + OFFBOARD
```

If the map or planner is unavailable, Guidance remains blocked rather than starting the mission.

## Final Behaviour

The completed Breadcrumb integration provides:

- Obstacle-aware search navigation.
- Obstacle-aware ROI diversion and return.
- Obstacle-aware ArUco final approach.
- Increased obstacle clearance.
- Safe endpoint adjustment.
- Fixed mission heading during avoidance.
- Sparse route execution.
- No unsafe direct-flight fallback.
- Planner readiness checking before autonomous flight.
- Intermediate lawn-mower waypoints for more predictable path recovery.

The final system preserves the existing **Guidance → SPAR → MAVROS** structure, with Breadcrumb inserted only as the horizontal route-planning layer.
