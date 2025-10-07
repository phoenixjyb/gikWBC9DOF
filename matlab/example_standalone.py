#!/usr/bin/env python3
"""
Example: How to use the Hybrid A* planner from Python

This shows how to call the MATLAB Hybrid A* planner directly from Python
without ROS2 (useful for testing and standalone applications).
"""

import matlab.engine
import numpy as np
import time

def main():
    # Start MATLAB Engine
    print("Starting MATLAB Engine...")
    eng = matlab.engine.start_matlab()
    
    # Add MATLAB workspace to path (CHANGE THIS PATH!)
    matlab_workspace = r"C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab"
    eng.addpath(eng.genpath(matlab_workspace), nargout=0)
    print(f"✓ MATLAB workspace added: {matlab_workspace}")
    
    # Create start and goal states
    start = eng.gik9dof.HybridState()
    start.x = 2.0
    start.y = 2.0
    start.theta = 0.0
    
    goal = eng.gik9dof.HybridState()
    goal.x = 8.0
    goal.y = 8.0
    goal.theta = 0.0
    
    print(f"\nStart: ({start.x}, {start.y}, {start.theta})")
    print(f"Goal:  ({goal.x}, {goal.y}, {goal.theta})")
    
    # Create occupancy grid (empty 20×20m map, 0.1m resolution)
    grid = eng.gik9dof.OccupancyGrid2D(0.1, 200, 200, 0.0, 0.0)
    
    # Get chassis parameters and inflate obstacles
    params = eng.gik9dof.getChassisParams()
    grid = eng.gik9dof.inflateObstacles(grid, params.inflation_radius)
    
    print(f"\nGrid: 200×200 @ 0.1m resolution")
    print(f"Inflation radius: {params.inflation_radius}m")
    
    # Plan path using FAST codegen version
    print("\n" + "="*60)
    print("Planning path with Hybrid A* (codegen version)...")
    print("="*60)
    
    start_time = time.time()
    path, stats = eng.gik9dof.planHybridAStarCodegen(
        start, goal, grid, nargout=2
    )
    plan_time = time.time() - start_time
    
    # Display results
    print(f"\nResults:")
    print(f"  Success: {stats['success']}")
    
    if stats['success']:
        print(f"  Path length: {stats['path_length']} waypoints")
        print(f"  Path cost: {stats['path_cost']:.2f} m")
        print(f"  Planning time: {plan_time:.4f} sec")
        print(f"  Iterations: {stats['iterations']}")
        
        print(f"\nWaypoints:")
        for i in range(int(stats['path_length'])):
            wp = path[i]
            print(f"  [{i+1}] x={wp['x']:.2f}, y={wp['y']:.2f}, θ={wp['theta']:.2f}")
    else:
        print(f"  Planning failed!")
        print(f"  Iterations: {stats['iterations']}")
        print(f"  Planning time: {plan_time:.4f} sec")
    
    # Compare with original version (slower)
    print("\n" + "="*60)
    print("Planning path with Hybrid A* (original version)...")
    print("="*60)
    
    start_time = time.time()
    path_orig, stats_orig = eng.gik9dof.planHybridAStar(
        start, goal, grid, nargout=2
    )
    plan_time_orig = time.time() - start_time
    
    print(f"\nResults:")
    print(f"  Success: {stats_orig['success']}")
    
    if stats_orig['success']:
        print(f"  Path length: {stats_orig['path_length']} waypoints")
        print(f"  Path cost: {stats_orig['path_cost']:.2f} m")
        print(f"  Planning time: {plan_time_orig:.4f} sec")
        print(f"  Iterations: {stats_orig['iterations']}")
    
    # Comparison
    if stats['success'] and stats_orig['success']:
        speedup = plan_time_orig / plan_time
        print(f"\n" + "="*60)
        print(f"Performance Comparison")
        print("="*60)
        print(f"  Original:  {plan_time_orig:.4f} sec")
        print(f"  Codegen:   {plan_time:.4f} sec")
        print(f"  Speedup:   {speedup:.2f}× faster! ✨")
        print(f"  Cost diff: {abs(stats['path_cost'] - stats_orig['path_cost']):.4f} m")
    
    # Cleanup
    eng.quit()
    print("\n✓ MATLAB Engine stopped")


def example_with_obstacles():
    """Example with obstacles in the environment"""
    print("\n" + "="*60)
    print("Example 2: Planning with Obstacles")
    print("="*60)
    
    # Start MATLAB Engine
    eng = matlab.engine.start_matlab()
    matlab_workspace = r"C:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab"
    eng.addpath(eng.genpath(matlab_workspace), nargout=0)
    
    # Create grid
    grid = eng.gik9dof.OccupancyGrid2D(0.1, 200, 200, 0.0, 0.0)
    
    # Add a vertical wall in the middle (obstacle at x=10m)
    # Convert Python bool array to MATLAB logical
    grid_data = np.zeros((200, 200), dtype=bool)
    grid_data[:, 100] = True  # Wall at x=10m (column 100)
    
    # Set grid data (MATLAB uses column-major, so transpose)
    grid.data = matlab.logical(grid_data.T.tolist())
    
    # Inflate obstacles
    params = eng.gik9dof.getChassisParams()
    grid = eng.gik9dof.inflateObstacles(grid, params.inflation_radius)
    
    # Start on left, goal on right (must go around wall)
    start = eng.gik9dof.HybridState()
    start.x = 5.0
    start.y = 10.0
    start.theta = 0.0
    
    goal = eng.gik9dof.HybridState()
    goal.x = 15.0
    goal.y = 10.0
    goal.theta = 0.0
    
    print(f"\nStart: ({start.x}, {start.y})")
    print(f"Goal:  ({goal.x}, {goal.y})")
    print(f"Obstacle: Vertical wall at x=10m")
    
    # Plan
    start_time = time.time()
    path, stats = eng.gik9dof.planHybridAStarCodegen(
        start, goal, grid, nargout=2
    )
    plan_time = time.time() - start_time
    
    # Results
    print(f"\nResults:")
    print(f"  Success: {stats['success']}")
    
    if stats['success']:
        print(f"  Path length: {stats['path_length']} waypoints")
        print(f"  Path cost: {stats['path_cost']:.2f} m (detour around wall)")
        print(f"  Planning time: {plan_time:.4f} sec")
        print(f"\n  Path goes around the obstacle! ✓")
    else:
        print(f"  No path found (as expected if goal is unreachable)")
    
    eng.quit()


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════════════╗
║         Hybrid A* Path Planner - Python Example                  ║
║         WHEELTEC Mobile Manipulator Navigation                   ║
╚══════════════════════════════════════════════════════════════════╝
    """)
    
    # Run examples
    main()
    
    # Uncomment to run obstacle example:
    # example_with_obstacles()
    
    print("\n" + "="*60)
    print("✓ Examples complete!")
    print("="*60)
    print("\nNext steps:")
    print("  1. Modify matlab_workspace path for your system")
    print("  2. Run: python3 example_standalone.py")
    print("  3. Try adding your own obstacles")
    print("  4. Deploy to ROS2 using hybrid_astar_planner_node.py")
