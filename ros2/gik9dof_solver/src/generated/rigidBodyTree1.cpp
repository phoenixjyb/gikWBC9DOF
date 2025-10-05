//
// rigidBodyTree1.cpp
//
// Code generation for function 'rigidBodyTree1'
//

// Include files
#include "rigidBodyTree1.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rigidBody1.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
namespace coder {
void rigidBodyTree::addBody(rigidBody &bodyin,
                            robotics::manip::internal::CollisionSet &iobj_0,
                            rigidBodyJoint &iobj_1,
                            robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv[4]{'b', 'a', 's', 'e'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 4) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 4) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 4) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 4) {
            if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv1[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::b_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 11) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 11) {
        if (obj.Vector[loop_ub] != cv2[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 11) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 11) {
            if (obj.Vector[loop_ub] != cv2[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::c_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 11) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 11) {
        if (obj.Vector[loop_ub] != cv3[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 11) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 11) {
            if (obj.Vector[loop_ub] != cv3[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::d_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 21) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 21) {
        if (obj.Vector[loop_ub] != cv4[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 21) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 21) {
            if (obj.Vector[loop_ub] != cv4[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::e_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 18) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 18) {
        if (obj.Vector[loop_ub] != cv5[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 18) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 18) {
            if (obj.Vector[loop_ub] != cv5[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::f_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 14) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 14) {
        if (obj.Vector[loop_ub] != cv6[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 14) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 14) {
            if (obj.Vector[loop_ub] != cv6[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::g_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 14) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 14) {
        if (obj.Vector[loop_ub] != cv7[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 14) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 14) {
            if (obj.Vector[loop_ub] != cv7[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::get_BaseName(char basename_data[],
                                 int basename_size[2]) const
{
  int loop_ub;
  if (TreeInternal.Base.NameInternal.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(TreeInternal.Base.NameInternal.Length);
  }
  basename_size[0] = 1;
  basename_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&TreeInternal.Base.NameInternal.Vector[0],
                &TreeInternal.Base.NameInternal.Vector[loop_ub],
                &basename_data[0]);
  }
}

void rigidBodyTree::h_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 14) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 14) {
        if (obj.Vector[loop_ub] != cv8[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 14) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 14) {
            if (obj.Vector[loop_ub] != cv8[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::i_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 14) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 14) {
        if (obj.Vector[loop_ub] != cv9[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 14) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 14) {
            if (obj.Vector[loop_ub] != cv9[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

rigidBodyTree *rigidBodyTree::init()
{
  rigidBodyTree *obj;
  obj = this;
  obj->TreeInternal.init();
  obj->TreeInternal.Base.CollisionsInternal = obj->_pobj0.init(10.0);
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

void rigidBodyTree::j_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 14) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 14) {
        if (obj.Vector[loop_ub] != cv10[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 14) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 14) {
            if (obj.Vector[loop_ub] != cv10[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

void rigidBodyTree::k_addBody(rigidBody &bodyin,
                              robotics::manip::internal::CollisionSet &iobj_0,
                              rigidBodyJoint &iobj_1,
                              robotics::manip::internal::RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *jnt;
  robotics::manip::internal::CharacterVector obj;
  robotics::manip::internal::RigidBody *b_bodyin;
  robotics::manip::internal::RigidBody *body;
  double b_index;
  int obj_size[2];
  int exitg1;
  int loop_ub;
  int pid;
  char obj_data[200];
  bool b_bool;
  b_bodyin = bodyin.BodyInternal;
  obj = b_bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByName(obj_data, obj_size);
  pid = -1;
  obj = TreeInternal.Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 14) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 14) {
        if (obj.Vector[loop_ub] != cv11[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (b_bool) {
    pid = 0;
  } else {
    int i;
    bool exitg2;
    b_index = TreeInternal.NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(b_index) - 1)) {
      body = TreeInternal.Bodies[i];
      obj = body->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (loop_ub == 14) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 14) {
            if (obj.Vector[loop_ub] != cv11[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        pid = i + 1;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  jnt = b_bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  TreeInternal.findBodyIndexByJointName(obj_data, obj_size);
  b_index = TreeInternal.NumBodies + 1.0;
  body = b_bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  TreeInternal.Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  TreeInternal.NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    TreeInternal.NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.PositionDoFMap[loop_ub] = TreeInternal.PositionNumber + 1.0;
    TreeInternal.PositionDoFMap[loop_ub + 12] =
        TreeInternal.PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    TreeInternal.VelocityDoFMap[loop_ub] = TreeInternal.VelocityNumber + 1.0;
    TreeInternal.VelocityDoFMap[loop_ub + 12] =
        TreeInternal.VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.PositionDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.PositionDoFMap[loop_ub + 11] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    TreeInternal.VelocityDoFMap[loop_ub - 1] = 0.0;
    TreeInternal.VelocityDoFMap[loop_ub + 11] = -1.0;
  }
  jnt = body->JointInternal;
  TreeInternal.PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  TreeInternal.VelocityNumber += jnt->VelocityNumber;
}

rigidBodyTree::rigidBodyTree() = default;

rigidBodyTree::~rigidBodyTree() = default;

} // namespace coder

// End of code generation (rigidBodyTree1.cpp)
