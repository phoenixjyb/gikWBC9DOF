//
// loadRobotForCodegen.h
//
// Code generation for function 'loadRobotForCodegen'
//

#ifndef LOADROBOTFORCODEGEN_H
#define LOADROBOTFORCODEGEN_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class rigidBodyTree;

}

// Function Declarations
namespace gik9dof {
namespace codegen {
coder::rigidBodyTree *loadRobotForCodegen();

}
} // namespace gik9dof
void loadRobotForCodegen_delete();

void loadRobotForCodegen_init();

void loadRobotForCodegen_new();

#endif
// End of code generation (loadRobotForCodegen.h)
