/*
 * GIK Debug Validator with Detailed Output
 * 
 * This version adds extensive debug output to understand convergence differences
 * between MATLAB and C++ solvers.
 *
 * Features:
 * - Iteration-by-iteration cost tracking (would need solver modification)
 * - Input/output comparison
 * - Solver reset testing
 * - Sequential vs independent solving comparison
 * 
 * Compile: See build_validation_wsl.sh
 * Usage: ./validate_gik_debug <test_cases.json> <output.json> [options]
 * 
 * Options:
 *   --reset-per-test    Reset solver between each test
 *   --sequential        Use previous solution as initial guess
 *   --verbose           Print detailed per-test info
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstring>

// Generated GIK solver includes
#include "../codegen/gik9dof_arm64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper.h"
#include "../codegen/gik9dof_arm64_20constraints/gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"

// JSON parsing (simple manual parsing for this specific format)
#include <sstream>

// Simple JSON value extractor
std::string extractJsonString(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    
    pos = json.find("\"", pos + key.length() + 2);
    if (pos == std::string::npos) return "";
    pos++;
    
    size_t end = json.find("\"", pos);
    if (end == std::string::npos) return "";
    
    return json.substr(pos, end - pos);
}

int extractJsonInt(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0;
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) return 0;
    pos++;
    
    // Skip whitespace
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\n' || json[pos] == '\t')) pos++;
    
    std::string numStr;
    while (pos < json.length() && (isdigit(json[pos]) || json[pos] == '-' || json[pos] == '+')) {
        numStr += json[pos++];
    }
    
    if (numStr.empty()) return 0;
    return std::stoi(numStr);
}

// Extract array from JSON
std::vector<double> extractJsonArray(const std::string& json, const std::string& key) {
    std::vector<double> result;
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return result;
    
    pos = json.find("[", pos);
    if (pos == std::string::npos) return result;
    pos++;
    
    size_t end = json.find("]", pos);
    if (end == std::string::npos) return result;
    
    std::string arrayStr = json.substr(pos, end - pos);
    std::istringstream iss(arrayStr);
    double val;
    char comma;
    
    while (iss >> val) {
        result.push_back(val);
        iss >> comma; // Skip comma
    }
    
    return result;
}

struct TestCase {
    int id;
    int waypointIndex;
    std::string sourceFile;
    int totalWaypoints;
    
    double qCurrent[9];
    double targetPose[16];
    int distBodyIndices[20];
    int distRefBodyIndices[20];
    double distBoundsLower[20];
    double distBoundsUpper[20];
    double distWeights[20];
    
    // MATLAB reference
    double matlab_qNext[9];
    int matlab_iterations;
    double matlab_solveTime;
    std::string matlab_status;
};

struct TestResult {
    int testId;
    int waypointIndex;
    double qNext[9];
    double solveTime_ms;
    int iterations;
    std::string status;
    
    double L2_diff;
    double max_diff;
    bool passed;
    
    // Debug info
    double initial_error;
    double final_error;
    std::vector<double> cost_history;  // Would need solver modification
};

// Compute L2 norm of difference
double computeL2Diff(const double* a, const double* b, int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// Compute max absolute difference
double computeMaxDiff(const double* a, const double* b, int n) {
    double maxDiff = 0.0;
    for (int i = 0; i < n; i++) {
        double diff = std::abs(a[i] - b[i]);
        if (diff > maxDiff) {
            maxDiff = diff;
        }
    }
    return maxDiff;
}

// Print joint configuration
void printJointConfig(const std::string& label, const double* q) {
    std::cout << label << ": [";
    for (int i = 0; i < 9; i++) {
        std::cout << std::fixed << std::setprecision(4) << q[i];
        if (i < 8) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// Print target pose matrix
void printPose(const std::string& label, const double* pose) {
    std::cout << label << ":" << std::endl;
    for (int i = 0; i < 4; i++) {
        std::cout << "  [";
        for (int j = 0; j < 4; j++) {
            std::cout << std::fixed << std::setprecision(4) << std::setw(8) << pose[i * 4 + j];
            if (j < 3) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

// Parse test cases from JSON file
std::vector<TestCase> loadTestCases(const std::string& filename) {
    std::vector<TestCase> tests;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return tests;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    // Simple JSON parsing for this specific format
    // Extract test_cases array
    size_t pos = content.find("\"testCases\"");
    if (pos == std::string::npos) {
        std::cerr << "Error: Cannot find testCases in JSON" << std::endl;
        return tests;
    }
    
    pos = content.find("[", pos);
    if (pos == std::string::npos) return tests;
    
    // Parse each test case
    size_t testPos = pos + 1;
    while (true) {
        testPos = content.find("{", testPos);
        if (testPos == std::string::npos || testPos > content.find("]", pos)) break;
        
        size_t testEnd = content.find("}", testPos);
        if (testEnd == std::string::npos) break;
        
        // Find the actual end (account for nested objects)
        int braceCount = 1;
        testEnd = testPos + 1;
        while (braceCount > 0 && testEnd < content.length()) {
            if (content[testEnd] == '{') braceCount++;
            if (content[testEnd] == '}') braceCount--;
            testEnd++;
        }
        
        std::string testJson = content.substr(testPos, testEnd - testPos);
        
        TestCase tc;
        tc.id = extractJsonInt(testJson, "id");
        tc.waypointIndex = extractJsonInt(testJson, "waypointIndex");
        tc.sourceFile = extractJsonString(testJson, "sourceFile");
        tc.totalWaypoints = extractJsonInt(testJson, "totalWaypoints");
        
        // Extract arrays
        auto qCurr = extractJsonArray(testJson, "qCurrent");
        auto tPose = extractJsonArray(testJson, "targetPose");
        auto distBI = extractJsonArray(testJson, "distBodyIndices");
        auto distRBI = extractJsonArray(testJson, "distRefBodyIndices");
        auto distBL = extractJsonArray(testJson, "distBoundsLower");
        auto distBU = extractJsonArray(testJson, "distBoundsUpper");
        auto distW = extractJsonArray(testJson, "distWeights");
        
        if (qCurr.size() == 9) std::copy(qCurr.begin(), qCurr.end(), tc.qCurrent);
        if (tPose.size() == 16) std::copy(tPose.begin(), tPose.end(), tc.targetPose);
        if (distBI.size() == 20) {
            for (int i = 0; i < 20; i++) tc.distBodyIndices[i] = (int)distBI[i];
        }
        if (distRBI.size() == 20) {
            for (int i = 0; i < 20; i++) tc.distRefBodyIndices[i] = (int)distRBI[i];
        }
        if (distBL.size() == 20) std::copy(distBL.begin(), distBL.end(), tc.distBoundsLower);
        if (distBU.size() == 20) std::copy(distBU.begin(), distBU.end(), tc.distBoundsUpper);
        if (distW.size() == 20) std::copy(distW.begin(), distW.end(), tc.distWeights);
        
        // Extract MATLAB reference
        auto matlab_qNext = extractJsonArray(testJson, "qNext");
        if (matlab_qNext.size() == 9) std::copy(matlab_qNext.begin(), matlab_qNext.end(), tc.matlab_qNext);
        tc.matlab_iterations = extractJsonInt(testJson, "iterations");
        
        tests.push_back(tc);
        testPos = testEnd;
    }
    
    return tests;
}

// Run single test
TestResult runTest(const TestCase& tc, bool resetSolver, double* warmStart, bool verbose) {
    TestResult result;
    result.testId = tc.id;
    result.waypointIndex = tc.waypointIndex;
    result.passed = false;
    
    // If reset requested, delete and recreate solver
    if (resetSolver) {
        solveGIKStepWrapper_delete();
        solveGIKStepWrapper_new();
        solveGIKStepWrapper_init();
    }
    
    // Use warmstart if provided, otherwise use qCurrent
    double initialGuess[9];
    if (warmStart != nullptr) {
        std::copy(warmStart, warmStart + 9, initialGuess);
        if (verbose) {
            std::cout << "  Using warm-start from previous solution" << std::endl;
        }
    } else {
        std::copy(tc.qCurrent, tc.qCurrent + 9, initialGuess);
    }
    
    if (verbose) {
        std::cout << "========================================" << std::endl;
        std::cout << "Test " << tc.id << " (waypoint " << tc.waypointIndex << ")" << std::endl;
        std::cout << "========================================" << std::endl;
        printJointConfig("Initial guess", initialGuess);
        printPose("Target pose", tc.targetPose);
        
        // Print active constraints
        int activeConstraints = 0;
        for (int i = 0; i < 20; i++) {
            if (tc.distWeights[i] > 0) {
                std::cout << "  Constraint " << (i+1) << ": weight=" << tc.distWeights[i]
                          << ", bounds=[" << tc.distBoundsLower[i] << ", " << tc.distBoundsUpper[i] << "]" << std::endl;
                activeConstraints++;
            }
        }
        std::cout << "Active distance constraints: " << activeConstraints << std::endl;
    }
    
    // Prepare solver inputs
    double qNext[9];
    struct0_T solverInfo;
    
    // Measure solve time
    auto start = std::chrono::high_resolution_clock::now();
    
    // Call C++ solver
    gik9dof::codegen_inuse::solveGIKStepWrapper(
        initialGuess,
        tc.targetPose,
        tc.distBodyIndices,
        tc.distRefBodyIndices,
        tc.distBoundsLower,
        tc.distBoundsUpper,
        tc.distWeights,
        qNext,
        &solverInfo
    );
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    
    // Store results
    std::copy(qNext, qNext + 9, result.qNext);
    result.solveTime_ms = duration.count();
    result.iterations = (int)solverInfo.Iterations;
    
    // Extract status string
    std::string statusStr(solverInfo.Status.data, 
                         std::min((int)solverInfo.Status.size[0], 14));
    result.status = statusStr;
    
    // Compute differences from MATLAB reference
    result.L2_diff = computeL2Diff(qNext, tc.matlab_qNext, 9);
    result.max_diff = computeMaxDiff(qNext, tc.matlab_qNext, 9);
    
    // Check pass/fail (tolerances)
    const double L2_TOLERANCE = 0.01;   // rad
    const double MAX_TOLERANCE = 0.02;  // rad
    result.passed = (result.L2_diff < L2_TOLERANCE) && (result.max_diff < MAX_TOLERANCE);
    
    if (verbose) {
        printJointConfig("C++ solution", qNext);
        printJointConfig("MATLAB reference", tc.matlab_qNext);
        std::cout << "Solve time: " << std::fixed << std::setprecision(2) << result.solveTime_ms << " ms" << std::endl;
        std::cout << "Iterations: " << result.iterations << " (MATLAB: " << tc.matlab_iterations << ")" << std::endl;
        std::cout << "Status: " << result.status << " (MATLAB: " << tc.matlab_status << ")" << std::endl;
        std::cout << "L2 difference: " << std::scientific << std::setprecision(6) << result.L2_diff << " rad" << std::endl;
        std::cout << "Max difference: " << result.max_diff << " rad (" 
                  << (result.max_diff * 180.0 / M_PI) << " deg)" << std::endl;
        std::cout << "Result: " << (result.passed ? "✓ PASS" : "✗ FAIL") << std::endl;
        std::cout << std::endl;
    }
    
    return result;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <test_cases.json> <output.json> [--reset-per-test] [--sequential] [--verbose]" << std::endl;
        return 1;
    }
    
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    
    bool resetPerTest = false;
    bool sequential = false;
    bool verbose = false;
    
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--reset-per-test") resetPerTest = true;
        if (arg == "--sequential") sequential = true;
        if (arg == "--verbose") verbose = true;
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "GIK Debug Validator with Detailed Output" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Options:" << std::endl;
    std::cout << "  Reset per test: " << (resetPerTest ? "YES" : "NO") << std::endl;
    std::cout << "  Sequential solving: " << (sequential ? "YES (warm-start)" : "NO (independent)") << std::endl;
    std::cout << "  Verbose output: " << (verbose ? "YES" : "NO") << std::endl;
    std::cout << std::endl;
    
    // Load test cases
    std::cout << "Loading test cases from: " << inputFile << std::endl;
    auto testCases = loadTestCases(inputFile);
    
    if (testCases.empty()) {
        std::cerr << "Error: No test cases loaded" << std::endl;
        return 1;
    }
    
    std::cout << "  Loaded " << testCases.size() << " test cases" << std::endl;
    std::cout << std::endl;
    
    // Initialize solver
    std::cout << "Initializing C++ GIK solver..." << std::endl;
    solveGIKStepWrapper_new();
    solveGIKStepWrapper_init();
    std::cout << "  Solver initialized" << std::endl;
    std::cout << std::endl;
    
    // Run tests
    std::cout << "Running " << testCases.size() << " test cases..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    std::vector<TestResult> results;
    double* warmStart = nullptr;
    double prevSolution[9];
    
    int passCount = 0;
    int failCount = 0;
    
    for (size_t i = 0; i < testCases.size(); i++) {
        const auto& tc = testCases[i];
        
        if (!verbose) {
            std::cout << "Test " << tc.id << " (waypoint " << tc.waypointIndex << ")..." << std::flush;
        }
        
        TestResult result = runTest(tc, resetPerTest, warmStart, verbose);
        results.push_back(result);
        
        if (!verbose) {
            std::cout << " " << (result.passed ? "✓ PASS" : "✗ FAIL")
                      << " | L2=" << std::fixed << std::setprecision(5) << result.L2_diff
                      << " | max=" << std::fixed << std::setprecision(5) << result.max_diff
                      << " | " << std::fixed << std::setprecision(2) << result.solveTime_ms << " ms"
                      << " | " << result.iterations << " iters"
                      << std::endl;
        }
        
        if (result.passed) passCount++;
        else failCount++;
        
        // Update warm-start for next test if sequential
        if (sequential) {
            std::copy(result.qNext, result.qNext + 9, prevSolution);
            warmStart = prevSolution;
        }
    }
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << std::endl;
    
    // Summary statistics
    std::cout << "Summary:" << std::endl;
    std::cout << "  Total tests: " << results.size() << std::endl;
    std::cout << "  Passed: " << passCount << " ✓" << std::endl;
    std::cout << "  Failed: " << failCount << " ✗" << std::endl;
    std::cout << std::endl;
    
    double sumL2 = 0.0, sumMax = 0.0, sumTime = 0.0, sumIters = 0.0;
    for (const auto& r : results) {
        sumL2 += r.L2_diff;
        sumMax += r.max_diff;
        sumTime += r.solveTime_ms;
        sumIters += r.iterations;
    }
    
    std::cout << "Average metrics:" << std::endl;
    std::cout << "  L2 diff: " << std::fixed << std::setprecision(6) << (sumL2 / results.size()) << " rad" << std::endl;
    std::cout << "  Max diff: " << std::fixed << std::setprecision(6) << (sumMax / results.size()) << " rad" << std::endl;
    std::cout << "  Solve time: " << std::fixed << std::setprecision(2) << (sumTime / results.size()) << " ms" << std::endl;
    std::cout << "  Iterations: " << std::fixed << std::setprecision(1) << (sumIters / results.size()) << std::endl;
    std::cout << std::endl;
    
    // Cleanup
    solveGIKStepWrapper_delete();
    
    // Write results to JSON
    std::ofstream outFile(outputFile);
    outFile << "{" << std::endl;
    outFile << "  \"test_run_info\": {" << std::endl;
    outFile << "    \"date\": \"2025-10-08\"," << std::endl;
    outFile << "    \"reset_per_test\": " << (resetPerTest ? "true" : "false") << "," << std::endl;
    outFile << "    \"sequential\": " << (sequential ? "true" : "false") << "," << std::endl;
    outFile << "    \"total_tests\": " << results.size() << "," << std::endl;
    outFile << "    \"passed\": " << passCount << "," << std::endl;
    outFile << "    \"failed\": " << failCount << std::endl;
    outFile << "  }," << std::endl;
    outFile << "  \"results\": [" << std::endl;
    
    for (size_t i = 0; i < results.size(); i++) {
        const auto& r = results[i];
        outFile << "    {" << std::endl;
        outFile << "      \"test_id\": " << r.testId << "," << std::endl;
        outFile << "      \"waypoint_index\": " << r.waypointIndex << "," << std::endl;
        outFile << "      \"passed\": " << (r.passed ? "true" : "false") << "," << std::endl;
        outFile << "      \"L2_diff\": " << std::setprecision(8) << r.L2_diff << "," << std::endl;
        outFile << "      \"max_diff\": " << std::setprecision(8) << r.max_diff << "," << std::endl;
        outFile << "      \"solve_time_ms\": " << std::setprecision(4) << r.solveTime_ms << "," << std::endl;
        outFile << "      \"iterations\": " << r.iterations << "," << std::endl;
        outFile << "      \"status\": \"" << r.status << "\"" << std::endl;
        outFile << "    }" << (i < results.size() - 1 ? "," : "") << std::endl;
    }
    
    outFile << "  ]" << std::endl;
    outFile << "}" << std::endl;
    outFile.close();
    
    std::cout << "Results saved to: " << outputFile << std::endl;
    std::cout << std::endl;
    
    if (failCount > 0) {
        std::cout << "⚠ Some tests FAILED!" << std::endl;
        return 1;
    }
    
    std::cout << "✓ All tests PASSED!" << std::endl;
    return 0;
}
