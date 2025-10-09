#!/usr/bin/env python3
"""
compare_gik_results.py
Compare MATLAB vs C++ GIK solver results

This script:
1. Loads validation results JSON from C++ program
2. Compares with MATLAB reference in the test cases JSON
3. Generates detailed comparison report
4. Creates visualizations (optional)

Usage:
    python compare_gik_results.py gik_validation_results.json
"""

import json
import sys
import numpy as np
from pathlib import Path
from datetime import datetime

# ============================================================================
# Analysis Functions
# ============================================================================

def load_results(results_file):
    """Load validation results JSON"""
    with open(results_file, 'r') as f:
        return json.load(f)

def compute_statistics(results):
    """Compute statistical summary of results"""
    num_tests = len(results)
    
    qDiff_L2 = [r['qDiff_L2'] for r in results]
    qDiff_max = [r['qDiff_max'] for r in results]
    solve_times = [r['cpp_solveTime_ms'] for r in results]
    iterations = [r['cpp_iterations'] for r in results]
    
    stats = {
        'num_tests': num_tests,
        'num_passed': sum(1 for r in results if r['withinTolerance']),
        'num_failed': sum(1 for r in results if not r['withinTolerance']),
        'qDiff_L2': {
            'mean': np.mean(qDiff_L2),
            'std': np.std(qDiff_L2),
            'min': np.min(qDiff_L2),
            'max': np.max(qDiff_L2),
            'median': np.median(qDiff_L2),
        },
        'qDiff_max': {
            'mean': np.mean(qDiff_max),
            'std': np.std(qDiff_max),
            'min': np.min(qDiff_max),
            'max': np.max(qDiff_max),
            'median': np.median(qDiff_max),
        },
        'solve_time_ms': {
            'mean': np.mean(solve_times),
            'std': np.std(solve_times),
            'min': np.min(solve_times),
            'max': np.max(solve_times),
            'median': np.median(solve_times),
        },
        'iterations': {
            'mean': np.mean(iterations),
            'std': np.std(iterations),
            'min': int(np.min(iterations)),
            'max': int(np.max(iterations)),
            'median': np.median(iterations),
        }
    }
    
    return stats

def print_report(data, stats):
    """Print detailed text report"""
    meta = data['metadata']
    results = data['results']
    
    print("=" * 80)
    print("GIK VALIDATION REPORT")
    print("=" * 80)
    print()
    
    # Metadata
    print("Test Information:")
    print(f"  Date: {meta.get('testDate', 'unknown')}")
    print(f"  Total tests: {meta['numTests']}")
    print(f"  Tolerances: L2={meta['jointToleranceL2']}, Max={meta['jointToleranceMax']}")
    print()
    
    # Summary
    print("Results Summary:")
    print(f"  ✅ Passed: {stats['num_passed']}/{stats['num_tests']} ({100*stats['num_passed']/stats['num_tests']:.1f}%)")
    print(f"  ❌ Failed: {stats['num_failed']}/{stats['num_tests']} ({100*stats['num_failed']/stats['num_tests']:.1f}%)")
    print()
    
    # Statistics
    print("Joint Difference Statistics (L2 norm):")
    print(f"  Mean:   {stats['qDiff_L2']['mean']:.6f} rad")
    print(f"  Median: {stats['qDiff_L2']['median']:.6f} rad")
    print(f"  Std:    {stats['qDiff_L2']['std']:.6f} rad")
    print(f"  Min:    {stats['qDiff_L2']['min']:.6f} rad")
    print(f"  Max:    {stats['qDiff_L2']['max']:.6f} rad")
    print()
    
    print("Joint Difference Statistics (Max absolute):")
    print(f"  Mean:   {stats['qDiff_max']['mean']:.6f} rad ({np.rad2deg(stats['qDiff_max']['mean']):.3f} deg)")
    print(f"  Median: {stats['qDiff_max']['median']:.6f} rad ({np.rad2deg(stats['qDiff_max']['median']):.3f} deg)")
    print(f"  Max:    {stats['qDiff_max']['max']:.6f} rad ({np.rad2deg(stats['qDiff_max']['max']):.3f} deg)")
    print()
    
    print("Solve Time Statistics (C++):")
    print(f"  Mean:   {stats['solve_time_ms']['mean']:.3f} ms")
    print(f"  Median: {stats['solve_time_ms']['median']:.3f} ms")
    print(f"  Min:    {stats['solve_time_ms']['min']:.3f} ms")
    print(f"  Max:    {stats['solve_time_ms']['max']:.3f} ms")
    print()
    
    print("Iteration Statistics (C++):")
    print(f"  Mean:   {stats['iterations']['mean']:.1f}")
    print(f"  Median: {stats['iterations']['median']:.1f}")
    print(f"  Min:    {stats['iterations']['min']}")
    print(f"  Max:    {stats['iterations']['max']}")
    print()
    
    # Failed cases detail
    if stats['num_failed'] > 0:
        print("Failed Test Cases:")
        print("-" * 80)
        for r in results:
            if not r['withinTolerance']:
                print(f"  Test {r['id']} (waypoint {r['waypointIndex']}):")
                print(f"    L2 diff:  {r['qDiff_L2']:.6f} rad")
                print(f"    Max diff: {r['qDiff_max']:.6f} rad ({np.rad2deg(r['qDiff_max']):.3f} deg)")
                print(f"    Status:   {r['cpp_status']}")
                print(f"    Time:     {r['cpp_solveTime_ms']:.2f} ms")
        print()
    
    # Overall verdict
    print("=" * 80)
    if stats['num_failed'] == 0:
        print("✅ VALIDATION PASSED - All tests within tolerance!")
    else:
        print(f"❌ VALIDATION FAILED - {stats['num_failed']} test(s) failed!")
    print("=" * 80)
    print()

def save_summary(output_file, data, stats):
    """Save summary to JSON file"""
    summary = {
        'metadata': data['metadata'],
        'statistics': stats,
        'generatedAt': datetime.now().isoformat()
    }
    
    with open(output_file, 'w') as f:
        json.dump(summary, f, indent=2)
    
    print(f"Summary saved to: {output_file}")

# ============================================================================
# Main
# ============================================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python compare_gik_results.py <validation_results.json>")
        print()
        print("Example:")
        print("  python compare_gik_results.py gik_validation_results.json")
        sys.exit(1)
    
    results_file = sys.argv[1]
    
    # Check file exists
    if not Path(results_file).exists():
        print(f"❌ Error: File not found: {results_file}")
        sys.exit(1)
    
    # Load results
    print(f"Loading results from: {results_file}")
    data = load_results(results_file)
    
    # Compute statistics
    stats = compute_statistics(data['results'])
    
    # Print report
    print()
    print_report(data, stats)
    
    # Save summary
    summary_file = Path(results_file).stem + "_summary.json"
    save_summary(summary_file, data, stats)
    
    # Exit code
    sys.exit(0 if stats['num_failed'] == 0 else 1)

if __name__ == '__main__':
    main()
