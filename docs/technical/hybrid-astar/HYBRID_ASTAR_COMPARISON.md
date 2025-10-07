# Hybrid A* Implementation Comparison

**Date:** October 7, 2025  
**Purpose:** Compare existing Hybrid A* code vs MATLAB Navigation Toolbox example

---

## 🔍 Summary

**✅ YES - You already have a Hybrid A* implementation!**

Located in: `matlab/+gik9dof/runStagedTrajectory.m` (lines 347-599)

**Key Finding:** Your existing code uses MATLAB's `plannerHybridAStar` from Navigation Toolbox, which is the **SAME** as the example. However, it's **NOT code-generated** - it's part of the obsolete staged framework that uses MATLAB objects at runtime.

---

## 📊 Comparison Table

| Feature | Your Existing Code | Navigation Toolbox Example | What We Need |
|---------|-------------------|---------------------------|--------------|
| **Algorithm** | `plannerHybridAStar` | `plannerHybridAStar` | ✅ Same |
| **State Space** | SE(2): (x, y, θ) | SE(2): (x, y, θ) | ✅ Same |
| **Map Type** | `occupancyMap` + `validatorOccupancyMap` | `validatorOccupancyMap` or `validatorVehicleCostmap` | ✅ Compatible |
| **Code Generation** | ❌ NO (uses MATLAB objects) | ❌ NO (example only) | 🚨 **Must implement from scratch** |
| **Parameters** | Motion primitive, turning radius, resolution | Same parameters | ✅ Same |
| **Output** | Path states (N×3) | Path object with `.States` | ✅ Compatible |
| **Collision Check** | Circle obstacles on occupancy grid | Generic validator | ✅ Similar |
| **Integration** | Staged framework (obsolete) | Standalone example | 🔄 Need ROS2 integration |

---

## 🧩 Your Existing Implementation Details

### **File Location**
- `matlab/+gik9dof/runStagedTrajectory.m` (function `planStageBHybridAStarPath`, line 347)

### **Key Components**

#### 1. **Map Building** (lines 420-480)
```matlab
function map = buildStageBOccupancyMap(startBase, goalBase, floorDiscs, resolution, safetyMargin)
```
- Creates `occupancyMap` from start/goal + circular obstacles
- Auto-sized map with padding
- Marks disc obstacles with safety margins
- Uses grid-based representation

#### 2. **Hybrid A* Planning** (lines 397-410)
```matlab
ss = stateSpaceSE2;
ss.StateBounds = [occMap.XWorldLimits; occMap.YWorldLimits; -pi pi];

validator = validatorOccupancyMap(ss);
validator.Map = occMap;
validator.ValidationDistance = resolution / 2;

planner = plannerHybridAStar(validator);
planner.MotionPrimitiveLength = options.StageBHybridMotionPrimitiveLength;
planner.MinTurningRadius = max(options.StageBHybridMinTurningRadius, minTurningLowerBound);

pathObj = plan(planner, startState, goalState);
statesRaw = pathObj.States;
```

**🚨 Problem:** Uses MATLAB objects (`plannerHybridAStar`, `validatorOccupancyMap`, `stateSpaceSE2`) which are **NOT code-generatable**!

#### 3. **Path Densification** (lines 510-560)
```matlab
function states = densifyHybridStates(statesIn, maxLinearStep, maxYawStep)
```
- Interpolates waypoints to respect velocity limits
- Inserts intermediate states between coarse Hybrid A* waypoints
- **✅ This is code-generable** (pure math, no objects)

#### 4. **Parameters Used**
From your config:
- `StageBHybridResolution`: Grid resolution (default 0.1m)
- `StageBHybridSafetyMargin`: Obstacle inflation (default 0.15m)
- `StageBHybridMotionPrimitiveLength`: Motion step size (default 0.5m)
- `StageBHybridMinTurningRadius`: Vehicle turning constraint (default 0.5m)
- `StageBMaxLinearSpeed`: Max velocity for densification (default 1.0 m/s)
- `StageBMaxYawRate`: Max angular rate (default 1.0 rad/s)

---

## 🆚 Navigation Toolbox Example

Based on MATLAB help output:

### **Creation Syntax**
```matlab
planner = plannerHybridAStar(validator)
planner = plannerHybridAStar(validator, Name, Value)
```

### **Key Properties**
- `StateValidator`: `validatorOccupancyMap` or `validatorVehicleCostmap`
- `MotionPrimitiveLength`: Length of motion steps
- `MinTurningRadius`: Vehicle kinematic constraint
- `NumMotionPrimitives`: Number of heading directions (default 5)
- `ForwardCost`: Cost multiplier for forward motion (default 1)
- `ReverseCost`: Cost multiplier for reverse (default 3)
- `DirectionSwitchingCost`: Penalty for direction change (default 0)
- `AnalyticExpansionInterval`: Frequency of analytic expansion attempts (default 5)
- `InterpolationDistance`: Spacing in output path (default 1)

### **Object Functions**
- `plan(planner, start, goal)`: Find path
- `show(planner)`: Visualize

---

## 🚨 Critical Issue: Code Generation Incompatibility

### **Why Your Existing Code Won't Code-Generate**

MATLAB Coder **CANNOT** generate C++ code from:
1. ❌ `plannerHybridAStar` object
2. ❌ `validatorOccupancyMap` object
3. ❌ `stateSpaceSE2` object
4. ❌ `occupancyMap` object
5. ❌ Object methods like `.plan()`, `.getTransform()`

These are **System Objects** and **MATLAB classes** that require the MATLAB runtime.

### **What IS Code-Generable**
From your existing code:
1. ✅ `densifyHybridStates()` - Pure math interpolation
2. ✅ Obstacle disc representation (struct arrays)
3. ✅ Pure Pursuit simulation logic
4. ✅ Trajectory integration math

---

## 🎯 What We Need to Build

Since `plannerHybridAStar` is **NOT code-generable**, we have **3 options**:

### **Option 1: Implement Hybrid A* from Scratch** ⭐ RECOMMENDED
**Pros:**
- Full control over algorithm
- Guaranteed code generation compatibility
- Can optimize for embedded systems
- No toolbox dependency at runtime

**Cons:**
- More implementation work
- Need to understand A* + Reeds-Shepp/Dubins curves
- Debugging complexity

**Effort:** 2-3 weeks

---

### **Option 2: Use Simplified Grid-Based A*** 
**Pros:**
- Simpler algorithm (no hybrid states)
- Easier to code-generate
- Still provides obstacle avoidance

**Cons:**
- Paths won't respect vehicle kinematics (will need smoothing)
- Less optimal for car-like robots
- Still requires post-processing

**Effort:** 1 week

---

### **Option 3: Keep MATLAB Object on PC, Use ROS2 Service**
**Pros:**
- Reuse existing code
- Fast development (just add ROS2 wrapper)
- Full Navigation Toolbox features

**Cons:**
- ❌ **Violates your architecture goal** (code generation for Orin)
- Requires MATLAB runtime on PC
- Network latency for planning
- Can't run standalone on Orin

**Effort:** 3 days

---

## 🔧 Hybrid A* Algorithm Components (For Option 1)

To implement from scratch, we need:

### **1. State Space**
- State: `(x, y, θ)` - SE(2) configuration space
- Discretization: Grid cells for (x,y), angle bins for θ

### **2. Motion Primitives**
- Forward/backward arcs
- Different steering angles
- Example: 5 primitives = {left turn, slight left, straight, slight right, right turn}

### **3. Heuristic Function**
- **Non-holonomic distance**: Reeds-Shepp or Dubins path length
- **Euclidean distance**: Fallback for speed
- Guides A* search toward goal

### **4. Collision Checking**
- Footprint: Robot shape (rectangle or circle)
- Check all cells along motion primitive
- Use occupancy grid

### **5. A* Search**
- Priority queue (open set): Nodes ordered by f = g + h
- Closed set: Visited states
- Expand lowest-cost node, generate successors

### **6. Analytic Expansion**
- Try direct Reeds-Shepp connection to goal
- Short-circuit search if collision-free

### **7. Path Extraction**
- Backtrack from goal to start
- Extract waypoints

### **8. Path Smoothing** (Optional)
- Remove redundant waypoints
- Bezier curve fitting

---

## 📋 Implementation Strategy (Option 1 - From Scratch)

### **Phase 1: Core A* Engine**
1. Grid-based A* (ignore orientation first)
2. Priority queue with f-score
3. 8-connected grid search
4. **Test:** Simple 2D obstacle avoidance

### **Phase 2: Add Hybrid States**
1. Extend to (x, y, θ) space
2. Discretize heading into bins (e.g., 72 bins = 5° resolution)
3. 3D grid: grid_x × grid_y × heading_bins
4. **Test:** Find path with orientation constraints

### **Phase 3: Motion Primitives**
1. Generate arc primitives for different steering angles
2. Precompute primitive trajectories
3. Collision check along arcs
4. **Test:** Car-like motion

### **Phase 4: Heuristic**
1. Implement Dubins path length heuristic
2. Or simple Euclidean + heading difference
3. **Test:** Faster search convergence

### **Phase 5: Analytic Expansion**
1. Reeds-Shepp curves (forward + backward)
2. Try direct connection periodically
3. **Test:** Shortcuts in open spaces

### **Phase 6: Code Generation**
1. Wrap in MATLAB function with fixed-size arrays
2. Codegen for ARM64 + x86_64
3. **Test:** C++ output compiles

### **Phase 7: ROS2 Integration**
1. Add to `gik9dof_solver_node`
2. Subscribe to map (`nav_msgs/OccupancyGrid`)
3. Subscribe to goal (`geometry_msgs/PoseStamped`)
4. Publish path to Pure Pursuit
5. **Test:** End-to-end navigation

---

## 💡 My Recommendation

### **Go with Option 1: Implement from Scratch**

**Why:**
1. ✅ Aligns with your code-generation architecture
2. ✅ Your existing code shows you understand the algorithm
3. ✅ Can reuse logic from `densifyHybridStates`, occupancy map building
4. ✅ Full control for optimization on Orin
5. ✅ No MATLAB runtime dependency
6. ✅ Educational value (deep understanding)

**Reusable from Your Existing Code:**
- ✅ Occupancy map building concept (convert to codegen-compatible version)
- ✅ Obstacle disc representation
- ✅ Path densification logic (already works!)
- ✅ Parameter structure (motion primitive length, turning radius, etc.)
- ✅ Integration strategy with Pure Pursuit

**What We Build New:**
- A* search engine (priority queue, open/closed sets)
- Motion primitive generation
- Hybrid state space (x, y, θ)
- Heuristic function (Dubins/Euclidean)
- Collision checking along primitives
- Path extraction and smoothing

---

## 🚀 Next Steps

**If you agree with Option 1 (from scratch), we'll:**

1. **Design phase** (Today)
   - Define data structures (grid, states, nodes)
   - Design motion primitive generation
   - Choose heuristic function
   - Plan memory allocation strategy

2. **Implement core A*** (Days 1-3)
   - Priority queue with fixed-size arrays
   - Grid search in (x, y) first
   - Test with simple 2D maps

3. **Add hybrid states** (Days 4-6)
   - Extend to (x, y, θ)
   - Heading discretization
   - Motion primitives with arcs

4. **Heuristic & expansion** (Days 7-9)
   - Dubins path heuristic
   - Analytic expansion (Reeds-Shepp)

5. **Code generation** (Days 10-12)
   - MATLAB Coder compatibility
   - Fixed-size arrays, static allocation
   - Generate ARM64 + x86_64

6. **ROS2 integration** (Days 13-14)
   - Map subscription
   - Goal subscription
   - Path output to Pure Pursuit

---

## 🤔 Your Decision

**Which option sounds best?**

**A.** 🌟 Option 1: Implement Hybrid A* from scratch (2-3 weeks, full control)  
**B.** Option 2: Simplified grid A* (1 week, simpler but less optimal)  
**C.** Option 3: Keep MATLAB object, use ROS2 service (3 days, violates architecture)  

**Or do you want to see a prototype first?** I can quickly implement a basic grid A* to demonstrate the approach before committing to the full Hybrid A* implementation.

Let me know! 🚀
