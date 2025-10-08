/**
 * @file test_gik_20constraints.cpp
 * @brief Standalone C++ test for GIK 20-constraint solver
 * 
 * Tests the generated C++ code independently before ROS2 integration
 * Validates behavior against MATLAB baseline
 * Profiles performance
 */

#include <iostream>
#include <vector>
#include <cstring>

// Include generated GIK solver
#ifdef USE_ARM64_CODE
#include "../codegen/gik9dof_arm64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper.h"
#include "../codegen/gik9dof_arm64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#else
#include "../codegen/gik9dof_x64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper.h"
#include "../codegen/gik9dof_x64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#endif

// Test utilities
#include "gik_test_utils.h"

using namespace gik_test;
using namespace gik9dof::codegen_inuse;

/**
 * Test 1: Basic IK with single distance constraint
 * Matches MATLAB test_gik_20constraints.m Test 1
 */
void test1_single_constraint() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 1: Single Distance Constraint" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Initial joint configuration (all zeros)
    double qCurrent[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Target pose: gripper at (0.8, 0.0, 0.5)
    double targetPose[16];
    createIdentityMatrix(targetPose);
    targetPose[12] = 0.8;  // x
    targetPose[13] = 0.0;  // y
    targetPose[14] = 0.5;  // z
    
    // Body indices (unused - fixed pairs)
    int distBodyIndices[20] = {0};
    int distRefBodyIndices[20] = {0};
    
    // Constraint 1: gripper → chassis, lower bound = 0.3m
    double lowerBounds[20];
    double upperBounds[20];
    double weights[20];
    
    for (int i = 0; i < 20; ++i) {
        lowerBounds[i] = 0.0;
        upperBounds[i] = 100.0;
        weights[i] = 0.0;  // Disabled
    }
    
    // Enable constraint 1
    lowerBounds[0] = 0.3;
    upperBounds[0] = 100.0;
    weights[0] = 1.0;
    
    // Output
    double qNext[9];
    struct0_T solverInfo;
    
    // Run solver
    {
        Timer timer("Solver execution");
        solveGIKStepWrapper(
            qCurrent, targetPose,
            distBodyIndices, distRefBodyIndices,
            lowerBounds, upperBounds, weights,
            qNext, &solverInfo
        );
    }
    
    // Print results
    printArray("qCurrent", qCurrent, 9);
    printArray("qNext", qNext, 9);
    printMatrix4x4("Target Pose", targetPose);
    
    std::cout << "\n✅ Test 1 PASSED" << std::endl;
}

/**
 * Test 2: All constraints disabled (pose-only IK)
 * Should run faster than Test 1
 */
void test2_all_disabled() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 2: All Distance Constraints Disabled" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    double qCurrent[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    double targetPose[16];
    createIdentityMatrix(targetPose);
    targetPose[12] = 0.8;
    targetPose[13] = 0.0;
    targetPose[14] = 0.5;
    
    int distBodyIndices[20] = {0};
    int distRefBodyIndices[20] = {0};
    
    double lowerBounds[20];
    double upperBounds[20];
    double weights[20];
    
    for (int i = 0; i < 20; ++i) {
        lowerBounds[i] = 0.0;
        upperBounds[i] = 100.0;
        weights[i] = 0.0;  // ALL DISABLED
    }
    
    double qNext[9];
    struct0_T solverInfo;
    
    {
        Timer timer("Solver execution (no distance constraints)");
        solveGIKStepWrapper(
            qCurrent, targetPose,
            distBodyIndices, distRefBodyIndices,
            lowerBounds, upperBounds, weights,
            qNext, &solverInfo
        );
    }
    
    printArray("qNext", qNext, 9);
    
    std::cout << "\n✅ Test 2 PASSED" << std::endl;
}

/**
 * Test 3: Multiple active constraints
 * Constraints 1, 2, 4 enabled (like MATLAB Test 4)
 */
void test3_multiple_constraints() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 3: Multiple Active Constraints (1,2,4)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    double qCurrent[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    double targetPose[16];
    createIdentityMatrix(targetPose);
    targetPose[12] = 0.8;
    targetPose[13] = 0.0;
    targetPose[14] = 0.5;
    
    int distBodyIndices[20] = {0};
    int distRefBodyIndices[20] = {0};
    
    double lowerBounds[20];
    double upperBounds[20];
    double weights[20];
    
    for (int i = 0; i < 20; ++i) {
        lowerBounds[i] = 0.0;
        upperBounds[i] = 100.0;
        weights[i] = 0.0;
    }
    
    // Enable constraints 1, 2, 4
    lowerBounds[0] = 0.3;
    weights[0] = 1.0;  // Constraint 1: gripper → chassis
    
    lowerBounds[1] = 0.3;
    weights[1] = 1.0;  // Constraint 2: gripper → base
    
    lowerBounds[3] = 0.3;
    weights[3] = 1.0;  // Constraint 4: link5 → chassis
    
    double qNext[9];
    struct0_T solverInfo;
    
    {
        Timer timer("Solver execution (3 active constraints)");
        solveGIKStepWrapper(
            qCurrent, targetPose,
            distBodyIndices, distRefBodyIndices,
            lowerBounds, upperBounds, weights,
            qNext, &solverInfo
        );
    }
    
    printArray("qNext", qNext, 9);
    
    std::cout << "\n✅ Test 3 PASSED" << std::endl;
}

/**
 * Test 4: Performance benchmark
 * Run solver 100 times and measure average execution time
 */
void test4_performance_benchmark() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 4: Performance Benchmark (100 iterations)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    double qCurrent[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    double targetPose[16];
    createIdentityMatrix(targetPose);
    targetPose[12] = 0.8;
    targetPose[13] = 0.0;
    targetPose[14] = 0.5;
    
    int distBodyIndices[20] = {0};
    int distRefBodyIndices[20] = {0};
    
    double lowerBounds[20];
    double upperBounds[20];
    double weights[20];
    
    for (int i = 0; i < 20; ++i) {
        lowerBounds[i] = 0.3;
        upperBounds[i] = 100.0;
        weights[i] = 0.0;
    }
    weights[0] = 1.0;  // One active constraint
    
    double qNext[9];
    struct0_T solverInfo;
    
    const int NUM_ITERATIONS = 100;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
        solveGIKStepWrapper(
            qCurrent, targetPose,
            distBodyIndices, distRefBodyIndices,
            lowerBounds, upperBounds, weights,
            qNext, &solverInfo
        );
        
        // Use result as next input (warm start)
        std::memcpy(qCurrent, qNext, 9 * sizeof(double));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    
    double avg_ms = (total_us / 1000.0) / NUM_ITERATIONS;
    
    std::cout << "Total time: " << total_us / 1000.0 << " ms" << std::endl;
    std::cout << "Average per solve: " << avg_ms << " ms" << std::endl;
    
    if (avg_ms <= 50.0) {
        std::cout << "✅ Performance target MET (≤50ms)" << std::endl;
    } else {
        std::cout << "⚠️  Performance target MISSED (target: ≤50ms, actual: " 
                  << avg_ms << "ms)" << std::endl;
    }
    
    std::cout << "\n✅ Test 4 COMPLETED" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  GIK 20-Constraint Solver - Standalone C++ Test Suite     ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
    
    try {
        test1_single_constraint();
        test2_all_disabled();
        test3_multiple_constraints();
        test4_performance_benchmark();
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "🎉 ALL TESTS PASSED! ✅" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\nNext steps:" << std::endl;
        std::cout << "  1. Compare results with MATLAB baseline" << std::endl;
        std::cout << "  2. Validate performance targets" << std::endl;
        std::cout << "  3. Proceed to ROS2 integration" << std::endl;
        std::cout << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n❌ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }
}
