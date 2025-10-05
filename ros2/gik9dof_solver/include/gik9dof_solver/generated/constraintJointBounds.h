//
// constraintJointBounds.h
//
// Code generation for function 'constraintJointBounds'
//

#ifndef CONSTRAINTJOINTBOUNDS_H
#define CONSTRAINTJOINTBOUNDS_H

// Include files
#include "gik9dof_codegen_followTrajectory_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class rigidBodyTree;

}

// Type Definitions
namespace coder {
class constraintJointBounds {
public:
  constraintJointBounds *init(rigidBodyTree &rigidbodytree);
  constraintJointBounds();
  ~constraintJointBounds();
  array<double, 2U> BoundsInternal;
  array<double, 2U> WeightsInternal;

protected:
  cell_12 ConstructorPropertyDefaultValues;

private:
  double NumElements;
};

} // namespace coder

#endif
// End of code generation (constraintJointBounds.h)
