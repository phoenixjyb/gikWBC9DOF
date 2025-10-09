#!/bin/bash
# run_gik_validation.sh
# Automated GIK validation workflow
# 
# This script orchestrates the full validation process:
# 1. Extract test cases from MAT file (MATLAB on Windows)
# 2. Run C++ validation (WSL/Linux)
# 3. Compare results and generate report

set -e

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VALIDATION_DIR="$SCRIPT_DIR"

# File paths
MAT_FILE="crossCheckMatVsCpp/log_matfile/log_holistic_iter0150.mat"
TEST_CASES_JSON="gik_test_cases.json"
RESULTS_JSON="gik_validation_results.json"
SUMMARY_JSON="gik_validation_results_summary.json"

# Number of test cases to extract
NUM_TEST_CASES=10

# ============================================================================
# Colors for output
# ============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ============================================================================
# Helper functions
# ============================================================================

print_header() {
    echo ""
    echo "========================================================================"
    echo -e "${BLUE}$1${NC}"
    echo "========================================================================"
    echo ""
}

print_step() {
    echo -e "${GREEN}▶${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} Error: $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} Warning: $1"
}

# ============================================================================
# Validation Steps
# ============================================================================

step1_extract_test_cases() {
    print_header "STEP 1: Extract Test Cases from MAT File"
    
    print_step "Running MATLAB extraction script..."
    
    # Check if running in Windows or WSL
    if command -v matlab.exe &> /dev/null; then
        # Windows with MATLAB
        MATLAB_CMD="matlab.exe"
    elif command -v matlab &> /dev/null; then
        # Linux/WSL with MATLAB
        MATLAB_CMD="matlab"
    else
        print_error "MATLAB not found in PATH"
        echo "Please ensure MATLAB is installed and available"
        exit 1
    fi
    
    # Run MATLAB extraction
    cd "$VALIDATION_DIR/.."
    
    $MATLAB_CMD -batch "cd('$VALIDATION_DIR/..'); \
        addpath(genpath('matlab')); \
        extract_test_cases_from_mat('$MAT_FILE', 'validation/$TEST_CASES_JSON', $NUM_TEST_CASES); \
        exit;"
    
    cd "$VALIDATION_DIR"
    
    # Check if file was created
    if [ ! -f "$TEST_CASES_JSON" ]; then
        print_error "Test cases JSON not created: $TEST_CASES_JSON"
        exit 1
    fi
    
    FILE_SIZE=$(du -h "$TEST_CASES_JSON" | cut -f1)
    print_success "Test cases extracted: $TEST_CASES_JSON ($FILE_SIZE)"
    echo ""
}

step2_build_cpp_validator() {
    print_header "STEP 2: Build C++ Validation Program"
    
    print_step "Building standalone validator..."
    
    if [ ! -f "$VALIDATION_DIR/build_validation_wsl.sh" ]; then
        print_error "Build script not found: build_validation_wsl.sh"
        exit 1
    fi
    
    chmod +x "$VALIDATION_DIR/build_validation_wsl.sh"
    
    if "$VALIDATION_DIR/build_validation_wsl.sh"; then
        print_success "Build completed"
    else
        print_error "Build failed"
        exit 1
    fi
    
    echo ""
}

step3_run_cpp_validation() {
    print_header "STEP 3: Run C++ Validation"
    
    print_step "Running validation tests..."
    
    VALIDATOR="$VALIDATION_DIR/validate_gik_standalone"
    
    if [ ! -f "$VALIDATOR" ]; then
        print_error "Validator executable not found: $VALIDATOR"
        print_warning "Run step 2 to build the validator"
        exit 1
    fi
    
    if "$VALIDATOR" "$TEST_CASES_JSON" "$RESULTS_JSON"; then
        print_success "Validation completed"
    else
        print_warning "Validation completed with failures (see results)"
    fi
    
    echo ""
}

step4_compare_results() {
    print_header "STEP 4: Compare Results and Generate Report"
    
    print_step "Running comparison analysis..."
    
    if [ ! -f "$RESULTS_JSON" ]; then
        print_error "Results file not found: $RESULTS_JSON"
        exit 1
    fi
    
    # Check if Python is available
    if command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
    elif command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        print_error "Python not found"
        print_warning "Skipping automated report generation"
        echo "You can manually inspect: $RESULTS_JSON"
        return
    fi
    
    # Run comparison script
    if $PYTHON_CMD "$VALIDATION_DIR/compare_gik_results.py" "$RESULTS_JSON"; then
        print_success "All tests PASSED! ✅"
        VALIDATION_STATUS=0
    else
        print_warning "Some tests FAILED! See report above"
        VALIDATION_STATUS=1
    fi
    
    echo ""
    
    return $VALIDATION_STATUS
}

# ============================================================================
# Main Workflow
# ============================================================================

main() {
    print_header "GIK MATLAB vs C++ Validation Workflow"
    
    echo "Configuration:"
    echo "  MAT file: $MAT_FILE"
    echo "  Test cases: $NUM_TEST_CASES"
    echo "  Output: $TEST_CASES_JSON -> $RESULTS_JSON"
    echo ""
    
    # Parse command line arguments
    SKIP_EXTRACT=false
    SKIP_BUILD=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-extract)
                SKIP_EXTRACT=true
                shift
                ;;
            --skip-build)
                SKIP_BUILD=true
                shift
                ;;
            --num-tests)
                NUM_TEST_CASES="$2"
                shift 2
                ;;
            *)
                echo "Unknown option: $1"
                echo "Usage: $0 [--skip-extract] [--skip-build] [--num-tests N]"
                exit 1
                ;;
        esac
    done
    
    # Execute workflow
    if [ "$SKIP_EXTRACT" = false ]; then
        step1_extract_test_cases
    else
        print_warning "Skipping test case extraction (using existing $TEST_CASES_JSON)"
    fi
    
    if [ "$SKIP_BUILD" = false ]; then
        step2_build_cpp_validator
    else
        print_warning "Skipping build (using existing validator)"
    fi
    
    step3_run_cpp_validation
    
    if step4_compare_results; then
        FINAL_STATUS=0
    else
        FINAL_STATUS=1
    fi
    
    # Final summary
    print_header "VALIDATION COMPLETE"
    
    echo "Generated files:"
    echo "  Test cases:  $TEST_CASES_JSON"
    echo "  Results:     $RESULTS_JSON"
    if [ -f "$SUMMARY_JSON" ]; then
        echo "  Summary:     $SUMMARY_JSON"
    fi
    echo ""
    
    if [ $FINAL_STATUS -eq 0 ]; then
        print_success "🎉 Validation PASSED - C++ solver matches MATLAB reference!"
    else
        print_warning "⚠️  Validation FAILED - Check results for details"
    fi
    
    echo ""
    
    exit $FINAL_STATUS
}

# Run main
main "$@"
