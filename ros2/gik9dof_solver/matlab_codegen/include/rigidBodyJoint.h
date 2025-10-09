//
// File: rigidBodyJoint.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
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

namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem;

}
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

// Type Definitions
namespace gik9dof {
namespace coder {
class rigidBodyJoint {
public:
  rigidBodyJoint *init();
  void set_MotionSubspace(const double msubspace_data[]);
  rigidBodyJoint *init(const char jname[14]);
  rigidBodyJoint *b_init();
  void setFixedTransform();
  void set_JointAxis();
  void b_set_MotionSubspace(const double msubspace[6]);
  void set_PositionLimits();
  void resetHomePosition();
  void set_HomePosition();
  rigidBodyJoint *copy(rigidBodyJoint &iobj_0) const;
  void get_MotionSubspace(double msubspace_data[], int msubspace_size[2]) const;
  rigidBodyJoint *c_init();
  void b_set_JointAxis();
  rigidBodyJoint *d_init();
  void c_set_JointAxis();
  void set_PositionLimits(GIKSolver *aInstancePtr);
  rigidBodyJoint *e_init();
  void b_setFixedTransform();
  rigidBodyJoint *f_init();
  void c_setFixedTransform();
  void b_set_PositionLimits();
  rigidBodyJoint *g_init();
  void d_setFixedTransform();
  void d_set_JointAxis();
  void c_set_PositionLimits();
  rigidBodyJoint *h_init();
  void e_setFixedTransform();
  void d_set_PositionLimits();
  rigidBodyJoint *i_init();
  void f_setFixedTransform();
  rigidBodyJoint *j_init();
  void g_setFixedTransform();
  void e_set_PositionLimits();
  rigidBodyJoint *k_init();
  void h_setFixedTransform();
  rigidBodyJoint *l_init();
  void i_setFixedTransform();
  void get_JointAxis(double ax[3]) const;
  void transformBodyToParent(const ::coder::array<double, 1U> &q,
                             double T[16]) const;
  static void randomConfig(GIKSolver *aInstancePtr,
                           robotics::manip::internal::GIKProblem *problem,
                           ::coder::array<double, 1U> &rc);
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

#endif
//
// File trailer for rigidBodyJoint.h
//
// [EOF]
//
