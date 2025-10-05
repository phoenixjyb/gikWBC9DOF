//
// gik9dof_codegen_followTrajectory_data.cpp
//
// Code generation for function 'gik9dof_codegen_followTrajectory_data'
//

// Include files
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
coder::generalizedInverseKinematics solver;

bool solver_not_empty;

coder::rigidBodyTree robot;

unsigned int state[625];

double freq;

bool freq_not_empty;

omp_nest_lock_t gik9dof_codegen_followTrajectory_nestLockGlobal;

const signed char iv[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

const char cv1[9]{'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c'};

const char cv2[11]{'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k', '_', 'x'};

const char cv3[11]{'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k', '_', 'y'};

const char cv4[21]{'a', 'b', 's', 't', 'r', 'a', 'c', 't', '_', 'c', 'h',
                   'a', 's', 's', 'i', 's', '_', 'l', 'i', 'n', 'k'};

const char cv5[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                   'b', 'a', 's', 'e', '_', 'l', 'i', 'n', 'k'};

const char cv6[14]{'l', 'e', 'f', 't', '_', 'a', 'r',
                   'm', '_', 'l', 'i', 'n', 'k', '1'};

const char cv7[14]{'l', 'e', 'f', 't', '_', 'a', 'r',
                   'm', '_', 'l', 'i', 'n', 'k', '2'};

const char cv8[14]{'l', 'e', 'f', 't', '_', 'a', 'r',
                   'm', '_', 'l', 'i', 'n', 'k', '3'};

const char cv9[14]{'l', 'e', 'f', 't', '_', 'a', 'r',
                   'm', '_', 'l', 'i', 'n', 'k', '4'};

const char cv10[14]{'l', 'e', 'f', 't', '_', 'a', 'r',
                    'm', '_', 'l', 'i', 'n', 'k', '5'};

const char cv11[14]{'l', 'e', 'f', 't', '_', 'a', 'r',
                    'm', '_', 'l', 'i', 'n', 'k', '6'};

const char cv12[17]{'l', 'e', 'f', 't', '_', 'g', 'r', 'i', 'p',
                    'p', 'e', 'r', '_', 'l', 'i', 'n', 'k'};

bool isInitialized_gik9dof_codegen_followTrajectory{false};

// End of code generation (gik9dof_codegen_followTrajectory_data.cpp)
