/**
 * @file validate_cpp_solver.cpp
 * @brief Standalone C++ program to validate MATLAB-generated solver
 * 
 * This program runs the same trajectory through the C++ solver and saves
 * results in JSON format for comparison with MATLAB implementation.
 * 
 * Build: g++ -std=c++17 -O2 validate_cpp_solver.cpp -I../codegen/arm64_realtime -L. -o validate_cpp_solver
 * Run: ./validate_cpp_solver
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>

// MATLAB Coder generated headers
#include "GIKSolver.h"

// JSON handling (simple, no external dependencies)
#include "json_simple.hpp"

using namespace std;
using json = nlohmann::json;

struct Pose {
    double position[3];
    double orientation[4]; // quaternion [w, x, y, z]
};

struct WaypointResult {
    int index;
    vector<double> configuration;
    double solve_time_ms;
    int iterations;
    string status;
    double target_position[3];
    double target_orientation[4];
};

// Convert quaternion to rotation matrix (column-major 3x3)
void quat2rotm(const double q[4], double R[9]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    
    R[0] = 1 - 2*(y*y + z*z);
    R[1] = 2*(x*y + w*z);
    R[2] = 2*(x*z - w*y);
    
    R[3] = 2*(x*y - w*z);
    R[4] = 1 - 2*(x*x + z*z);
    R[5] = 2*(y*z + w*x);
    
    R[6] = 2*(x*z + w*y);
    R[7] = 2*(y*z - w*x);
    R[8] = 1 - 2*(x*x + y*y);
}

// Build 4x4 transformation matrix from position and quaternion
void buildTransform(const double pos[3], const double quat[4], double T[16]) {
    double R[9];
    quat2rotm(quat, R);
    
    // Column-major 4x4 matrix
    T[0] = R[0]; T[4] = R[3]; T[8]  = R[6]; T[12] = pos[0];
    T[1] = R[1]; T[5] = R[4]; T[9]  = R[7]; T[13] = pos[1];
    T[2] = R[2]; T[6] = R[5]; T[10] = R[8]; T[14] = pos[2];
    T[3] = 0;    T[7] = 0;    T[11] = 0;    T[15] = 1;
}

int main() {
    cout << "=== C++ Solver Validation ===" << endl << endl;
    
    // Configuration
    const string TRAJECTORY_FILE = "../1_pull_world_scaled.json";
    const string RESULTS_FILE = "../validation_results_cpp.json";
    
    // Step 1: Load trajectory
    cout << "Step 1: Loading trajectory from " << TRAJECTORY_FILE << "..." << endl;
    
    ifstream trajFile(TRAJECTORY_FILE);
    if (!trajFile.is_open()) {
        cerr << "Error: Could not open " << TRAJECTORY_FILE << endl;
        return 1;
    }
    
    json trajData;
    trajFile >> trajData;
    trajFile.close();
    
    vector<Pose> trajectory;
    for (const auto& pose_json : trajData["poses"]) {
        Pose pose;
        for (int i = 0; i < 3; i++) {
            pose.position[i] = pose_json["position"][i];
        }
        for (int i = 0; i < 4; i++) {
            pose.orientation[i] = pose_json["orientation"][i];
        }
        trajectory.push_back(pose);
    }
    
    int numWaypoints = trajectory.size();
    cout << "  Loaded " << numWaypoints << " waypoints" << endl << endl;
    
    // Step 2: Initialize MATLAB solver
    cout << "Step 2: Initializing MATLAB C++ solver..." << endl;
    
    gik9dof::GIKSolver solver;
    
    cout << "  Solver initialized" << endl << endl;
    
    // Step 3: Solve for initial configuration (waypoint 1)
    cout << "Step 3: Solving for initial configuration (waypoint 1)..." << endl;
    
    double initialTarget[16];
    buildTransform(trajectory[0].position, trajectory[0].orientation, initialTarget);
    
    // Initial guess (home configuration)
    double initialGuess[9] = {
        trajectory[0].position[0],  // base_x
        trajectory[0].position[1],  // base_y
        0.0,                         // base_theta
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0 // arm joints
    };
    
    double initialConfig[9];
    double initialStatus[30]; // Solver status output
    
    auto start = chrono::high_resolution_clock::now();
    solver.solveIK(initialTarget, initialGuess, initialConfig, initialStatus);
    auto end = chrono::high_resolution_clock::now();
    
    double solveTime = chrono::duration<double, milli>(end - start).count();
    
    cout << "  Initial solve: " << solveTime << " ms" << endl;
    cout << "  Initial config: [";
    for (int i = 0; i < 9; i++) {
        cout << fixed << setprecision(4) << initialConfig[i];
        if (i < 8) cout << ", ";
    }
    cout << "]" << endl << endl;
    
    // Step 4: Solve trajectory
    cout << "Step 4: Solving trajectory with C++ solver..." << endl;
    cout << "  Solving waypoints 2 to " << numWaypoints << "..." << endl;
    
    vector<WaypointResult> results;
    double currentConfig[9];
    memcpy(currentConfig, initialConfig, 9 * sizeof(double));
    
    // Store first waypoint
    WaypointResult wp1;
    wp1.index = 1;
    wp1.configuration.assign(initialConfig, initialConfig + 9);
    wp1.solve_time_ms = 0.0;
    wp1.iterations = 0;
    wp1.status = "initial";
    memcpy(wp1.target_position, trajectory[0].position, 3 * sizeof(double));
    memcpy(wp1.target_orientation, trajectory[0].orientation, 4 * sizeof(double));
    results.push_back(wp1);
    
    int successCount = 0;
    double totalSolveTime = 0.0;
    double maxSolveTime = 0.0;
    
    int progressThreshold = 0;
    
    for (int i = 1; i < numWaypoints; i++) {
        // Show progress every 10%
        int progress = (i * 100) / (numWaypoints - 1);
        if (progress >= progressThreshold) {
            cout << "    Progress: " << progress << "% (waypoint " << (i+1) << "/" << numWaypoints << ")" << endl;
            progressThreshold += 10;
        }
        
        // Build target transform
        double targetTform[16];
        buildTransform(trajectory[i].position, trajectory[i].orientation, targetTform);
        
        // Solve IK
        double newConfig[9];
        double status[30];
        
        start = chrono::high_resolution_clock::now();
        solver.solveIK(targetTform, currentConfig, newConfig, status);
        end = chrono::high_resolution_clock::now();
        
        solveTime = chrono::duration<double, milli>(end - start).count();
        
        // Extract status (assuming first element is success flag, second is iterations)
        bool success = (status[0] > 0.5);
        int iterations = static_cast<int>(status[1]);
        
        // Store result
        WaypointResult wp;
        wp.index = i + 1;
        wp.configuration.assign(newConfig, newConfig + 9);
        wp.solve_time_ms = solveTime;
        wp.iterations = iterations;
        wp.status = success ? "success" : "failed";
        memcpy(wp.target_position, trajectory[i].position, 3 * sizeof(double));
        memcpy(wp.target_orientation, trajectory[i].orientation, 4 * sizeof(double));
        results.push_back(wp);
        
        // Update stats
        if (success) successCount++;
        totalSolveTime += solveTime;
        if (solveTime > maxSolveTime) maxSolveTime = solveTime;
        
        // Update current config
        memcpy(currentConfig, newConfig, 9 * sizeof(double));
    }
    
    cout << "  C++ solving complete!" << endl << endl;
    
    // Step 5: Analyze results
    cout << "Step 5: Analyzing C++ results..." << endl;
    
    double successRate = (successCount * 100.0) / (numWaypoints - 1);
    double avgSolveTime = totalSolveTime / (numWaypoints - 1);
    
    cout << "  Success rate: " << fixed << setprecision(1) << successRate << "% (" 
         << successCount << "/" << (numWaypoints-1) << ")" << endl;
    cout << "  Avg solve time: " << setprecision(2) << avgSolveTime << " ms" << endl;
    cout << "  Max solve time: " << maxSolveTime << " ms" << endl << endl;
    
    // Step 6: Save results
    cout << "Step 6: Saving C++ results to " << RESULTS_FILE << "..." << endl;
    
    json output;
    output["metadata"]["solver"] = "C++";
    output["metadata"]["num_waypoints"] = numWaypoints;
    output["metadata"]["success_rate"] = successRate;
    output["metadata"]["avg_solve_time_ms"] = avgSolveTime;
    output["metadata"]["max_solve_time_ms"] = maxSolveTime;
    
    output["waypoints"] = json::array();
    for (const auto& wp : results) {
        json wp_json;
        wp_json["index"] = wp.index;
        wp_json["configuration"] = wp.configuration;
        wp_json["solve_time_ms"] = wp.solve_time_ms;
        wp_json["iterations"] = wp.iterations;
        wp_json["status"] = wp.status;
        wp_json["target_position"] = vector<double>(wp.target_position, wp.target_position + 3);
        wp_json["target_orientation"] = vector<double>(wp.target_orientation, wp.target_orientation + 4);
        output["waypoints"].push_back(wp_json);
    }
    
    ofstream outFile(RESULTS_FILE);
    outFile << output.dump(2);
    outFile.close();
    
    cout << "  C++ results saved" << endl << endl;
    
    cout << "=== C++ VALIDATION COMPLETE ===" << endl;
    cout << "Run validate_cpp_solver.m in MATLAB to compare results" << endl;
    
    return 0;
}
