//
// File: structConstructorHelper.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

// Include Files
#include "structConstructorHelper.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
namespace gik9dof {
void binary_expand_op_10(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[19].Violation.set_size(1, in1[19].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[19].Violation.set_size(in1[19].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[19].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_11(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[18].Violation.set_size(1, in1[18].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[18].Violation.set_size(in1[18].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[18].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_12(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[17].Violation.set_size(1, in1[17].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[17].Violation.set_size(in1[17].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[17].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_13(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[16].Violation.set_size(1, in1[16].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[16].Violation.set_size(in1[16].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[16].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_14(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[15].Violation.set_size(1, in1[15].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[15].Violation.set_size(in1[15].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[15].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_15(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[14].Violation.set_size(1, in1[14].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[14].Violation.set_size(in1[14].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[14].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_16(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[13].Violation.set_size(1, in1[13].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[13].Violation.set_size(in1[13].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[13].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_17(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[12].Violation.set_size(1, in1[12].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[12].Violation.set_size(in1[12].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[12].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_18(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[11].Violation.set_size(1, in1[11].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[11].Violation.set_size(in1[11].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[11].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_19(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[10].Violation.set_size(1, in1[10].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[10].Violation.set_size(in1[10].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[10].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_20(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[9].Violation.set_size(1, in1[9].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[9].Violation.set_size(in1[9].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[9].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_21(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[8].Violation.set_size(1, in1[8].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[8].Violation.set_size(in1[8].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[8].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_22(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[7].Violation.set_size(1, in1[7].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[7].Violation.set_size(in1[7].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[7].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_23(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[6].Violation.set_size(1, in1[6].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[6].Violation.set_size(in1[6].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[6].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_24(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[5].Violation.set_size(1, in1[5].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[5].Violation.set_size(in1[5].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[5].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_25(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[4].Violation.set_size(1, in1[4].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[4].Violation.set_size(in1[4].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[4].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_26(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[3].Violation.set_size(1, in1[3].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[3].Violation.set_size(in1[3].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[3].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_27(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3,
                         const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[2].Violation.set_size(1, in1[2].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[2].Violation.set_size(in1[2].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[2].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                int in4
//                const ::coder::array<double, 1U> &in5
//                const ::coder::array<double, 1U> &in7
// Return Type  : void
//
void binary_expand_op_28(struct1_T in1[22],
                         const ::coder::array<double, 1U> &in3, int in4,
                         const ::coder::array<double, 1U> &in5,
                         const ::coder::array<double, 1U> &in7)
{
  ::coder::array<double, 1U> b_in3;
  ::coder::array<double, 1U> c_in3;
  int loop_ub;
  int stride_0_0_tmp;
  int stride_1_0;
  if (in5.size(0) == 1) {
    loop_ub = in4 + 1;
  } else {
    loop_ub = in5.size(0);
  }
  b_in3.set_size(loop_ub);
  stride_0_0_tmp = (in4 + 1 != 1);
  stride_1_0 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    b_in3[i] = in3[i * stride_0_0_tmp] - in5[i * stride_1_0];
  }
  if (in7.size(0) == 1) {
    loop_ub = in4 + 1;
  } else {
    loop_ub = in7.size(0);
  }
  c_in3.set_size(loop_ub);
  stride_1_0 = (in7.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    c_in3[i] = in3[i * stride_0_0_tmp] - in7[i * stride_1_0];
  }
  in1[1].Violation.set_size(1, in1[1].Violation.size(1));
  if (c_in3.size(0) == 1) {
    loop_ub = b_in3.size(0);
  } else {
    loop_ub = c_in3.size(0);
  }
  in1[1].Violation.set_size(in1[1].Violation.size(0), loop_ub);
  stride_0_0_tmp = (b_in3.size(0) != 1);
  stride_1_0 = (c_in3.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = b_in3[i * stride_0_0_tmp];
    b_varargin_2 = c_in3[i * stride_1_0];
    in1[1].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_8(struct1_T in1[22],
                        const ::coder::array<double, 1U> &in3,
                        const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[21].Violation.set_size(1, in1[21].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[21].Violation.set_size(in1[21].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[21].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

//
// Arguments    : struct1_T in1[22]
//                const ::coder::array<double, 1U> &in3
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
void binary_expand_op_9(struct1_T in1[22],
                        const ::coder::array<double, 1U> &in3,
                        const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1[20].Violation.set_size(1, in1[20].Violation.size(1));
  if (in5.size(0) == 1) {
    loop_ub = in3.size(0);
  } else {
    loop_ub = in5.size(0);
  }
  in1[20].Violation.set_size(in1[20].Violation.size(0), loop_ub);
  stride_0_1 = (in3.size(0) != 1);
  stride_1_1 = (in5.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    double b_varargin_2;
    double varargin_2;
    varargin_2 = in3[i * stride_0_1];
    b_varargin_2 = in5[i * stride_1_1];
    in1[20].Violation[i] =
        std::fmin(0.0, varargin_2) + std::fmax(0.0, b_varargin_2);
  }
}

} // namespace gik9dof

//
// File trailer for structConstructorHelper.cpp
//
// [EOF]
//
