//
// rigidBodyJoint.h
//
// Code generation for function 'rigidBodyJoint'
//

#ifndef RIGIDBODYJOINT_H
#define RIGIDBODYJOINT_H

// Include files
#include "CharacterVector.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class GIKProblem;

}
} // namespace manip
} // namespace robotics
} // namespace coder

// Type Definitions
namespace coder {
class rigidBodyJoint {
public:
  rigidBodyJoint *init();
  void set_MotionSubspace(const double msubspace_data[]);
  rigidBodyJoint *init(const char jname[14]);
  rigidBodyJoint *b_init();
  void setFixedTransform(const double input[16]);
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
  void b_set_PositionLimits();
  rigidBodyJoint *e_init();
  rigidBodyJoint *f_init();
  void c_set_PositionLimits();
  rigidBodyJoint *g_init();
  void d_set_JointAxis();
  void d_set_PositionLimits();
  rigidBodyJoint *h_init();
  void e_set_PositionLimits();
  rigidBodyJoint *i_init();
  rigidBodyJoint *j_init();
  void f_set_PositionLimits();
  rigidBodyJoint *k_init();
  rigidBodyJoint *l_init();
  void get_JointAxis(double ax[3]) const;
  void transformBodyToParent(const array<double, 1U> &q, double T[16]) const;
  static void randomConfig(robotics::manip::internal::GIKProblem *problem,
                           array<double, 1U> &rc);
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

#endif
// End of code generation (rigidBodyJoint.h)
