//
// File: rigidBodyJoint.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 19:17:46
//

#ifndef RIGIDBODYJOINT_H
#define RIGIDBODYJOINT_H

// Include Files
#include "CharacterVector.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class GIKSolver;

}

// Type Definitions
namespace gik9dof {
namespace coder {
class rigidBodyJoint {
public:
  rigidBodyJoint *init();
  void set_MotionSubspace(const double msubspace_data[]);
  rigidBodyJoint *init(const char jname[14]);
  rigidBodyJoint *b_init();
  void set_JointAxis();
  void b_set_MotionSubspace(const double msubspace[6]);
  void set_PositionLimits();
  void resetHomePosition();
  rigidBodyJoint *copy(rigidBodyJoint &iobj_0) const;
  void get_MotionSubspace(double msubspace_data[], int msubspace_size[2]) const;
  rigidBodyJoint *c_init();
  void b_set_JointAxis();
  rigidBodyJoint *d_init();
  void c_set_JointAxis();
  void set_PositionLimits(GIKSolver *aInstancePtr);
  rigidBodyJoint *e_init();
  rigidBodyJoint *f_init();
  void b_set_PositionLimits();
  rigidBodyJoint *g_init();
  void d_set_JointAxis();
  void c_set_PositionLimits();
  rigidBodyJoint *h_init();
  void d_set_PositionLimits();
  rigidBodyJoint *i_init();
  rigidBodyJoint *j_init();
  void e_set_PositionLimits();
  rigidBodyJoint *k_init();
  rigidBodyJoint *l_init();
  void get_JointAxis(double ax[3]) const;
  void transformBodyToParent(const ::coder::array<double, 1U> &q,
                             double T[16]) const;
  rigidBodyJoint();
  ~rigidBodyJoint();
  double VelocityNumber;
  double PositionNumber;
  bool InTree;
  double JointToParentTransform[16];
  double ChildToJointTransform[16];
  robotics::manip::internal::CharacterVector NameInternal;
  double PositionLimitsInternal[14];
  double HomePositionInternal[7];
  double JointAxisInternal[3];
  double MotionSubspaceInternal[36];
  robotics::manip::internal::CharacterVector TypeInternal;
};

} // namespace coder
} // namespace gik9dof

// Function Declarations
namespace gik9dof {
void binary_expand_op_5(::coder::array<double, 1U> &in1,
                        const double in2_data[], const int in2_size[2]);

void binary_expand_op_6(::coder::array<double, 1U> &in1,
                        const double in2_data[], const int in2_size[2],
                        const ::coder::array<double, 1U> &in3);

void binary_expand_op_7(::coder::array<double, 1U> &in1,
                        const double in2_data[], const int in2_size[2],
                        const ::coder::array<double, 1U> &in3);

} // namespace gik9dof

#endif
//
// File trailer for rigidBodyJoint.h
//
// [EOF]
//
