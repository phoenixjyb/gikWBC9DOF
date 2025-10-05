//
// structConstructorHelper.cpp
//
// Code generation for function 'structConstructorHelper'
//

// Include files
#include "structConstructorHelper.h"
#include "gik9dof_codegen_followTrajectory_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
void binary_expand_op_8(struct_T in1[3], const coder::array<double, 1U> &in3,
                        const coder::array<double, 1U> &in5)
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

void binary_expand_op_9(struct_T in1[3], const coder::array<double, 1U> &in3,
                        int in5, const coder::array<double, 1U> &in6,
                        const coder::array<double, 1U> &in8)
{
  coder::array<double, 1U> b_in3;
  coder::array<double, 1U> c_in3;
  int loop_ub;
  int stride_0_0_tmp;
  int stride_1_0;
  if (in6.size(0) == 1) {
    loop_ub = in5 + 1;
  } else {
    loop_ub = in6.size(0);
  }
  b_in3.set_size(loop_ub);
  stride_0_0_tmp = (in5 + 1 != 1);
  stride_1_0 = (in6.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    b_in3[i] = in3[i * stride_0_0_tmp] - in6[i * stride_1_0];
  }
  if (in8.size(0) == 1) {
    loop_ub = in5 + 1;
  } else {
    loop_ub = in8.size(0);
  }
  c_in3.set_size(loop_ub);
  stride_1_0 = (in8.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    c_in3[i] = in3[i * stride_0_0_tmp] - in8[i * stride_1_0];
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

// End of code generation (structConstructorHelper.cpp)
