#ifndef GIK_TEST_UTILS_H
#define GIK_TEST_UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <iomanip>

namespace gik_test {

// Helper to print arrays
template<typename T>
void printArray(const char* name, const T* arr, size_t size) {
    std::cout << name << ": [";
    for (size_t i = 0; i < size; ++i) {
        std::cout << arr[i];
        if (i < size - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// Helper to print 4x4 matrix (column-major)
void printMatrix4x4(const char* name, const double* mat) {
    std::cout << name << ":" << std::endl;
    for (int row = 0; row < 4; ++row) {
        std::cout << "  [";
        for (int col = 0; col < 4; ++col) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) 
                      << mat[col * 4 + row];  // Column-major
            if (col < 3) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

// Timer class for performance measurement
class Timer {
public:
    Timer(const char* name) : name_(name) {
        start_ = std::chrono::high_resolution_clock::now();
    }
    
    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        std::cout << "⏱️  " << name_ << ": " << duration.count() / 1000.0 << " ms" << std::endl;
    }
    
private:
    const char* name_;
    std::chrono::high_resolution_clock::time_point start_;
};

// Helper to create identity matrix (column-major)
void createIdentityMatrix(double* mat) {
    for (int i = 0; i < 16; ++i) {
        mat[i] = 0.0;
    }
    mat[0] = 1.0;  // (0,0)
    mat[5] = 1.0;  // (1,1)
    mat[10] = 1.0; // (2,2)
    mat[15] = 1.0; // (3,3)
}

// Helper to create transformation matrix (column-major)
// R = rotation (3x3), p = position (3x1)
void createTransformMatrix(double* mat, const double* R, const double* p) {
    // First 3 columns: rotation
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            mat[col * 4 + row] = R[col * 3 + row];
        }
        mat[col * 4 + 3] = 0.0;
    }
    
    // Fourth column: position + homogeneous coordinate
    mat[12] = p[0];
    mat[13] = p[1];
    mat[14] = p[2];
    mat[15] = 1.0;
}

// Compute RMS difference between two joint configurations
double computeRMS(const double* q1, const double* q2, size_t size) {
    double sum = 0.0;
    for (size_t i = 0; i < size; ++i) {
        double diff = q1[i] - q2[i];
        sum += diff * diff;
    }
    return std::sqrt(sum / size);
}

// Check if two arrays are approximately equal
bool areClose(const double* a, const double* b, size_t size, double tol = 1e-6) {
    for (size_t i = 0; i < size; ++i) {
        if (std::abs(a[i] - b[i]) > tol) {
            return false;
        }
    }
    return true;
}

} // namespace gik_test

#endif // GIK_TEST_UTILS_H
