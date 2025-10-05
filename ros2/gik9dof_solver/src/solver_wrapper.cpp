#include "gik9dof_solver/solver_wrapper.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

#ifdef GIK9DOF_CODEGEN_AVAILABLE
#include "gik9dof_solver/generated/gik9dof_codegen_followTrajectory.h"
#include "gik9dof_solver/generated/gik9dof_codegen_followTrajectory_initialize.h"
#include "gik9dof_solver/generated/gik9dof_codegen_followTrajectory_terminate.h"
#endif

namespace gik9dof_solver
{

namespace
{
constexpr std::array<double, 16> identityPoseArray()
{
  std::array<double, 16> pose {};
  pose[0] = 1.0;
  pose[5] = 1.0;
  pose[10] = 1.0;
  pose[15] = 1.0;
  return pose;
}

#ifdef GIK9DOF_CODEGEN_AVAILABLE
inline void toColumnMajor(const std::array<double, 16> & row_major, double (&column_major)[16])
{
  for (std::size_t row = 0; row < 4; ++row)
  {
    for (std::size_t col = 0; col < 4; ++col)
    {
      column_major[col * 4 + row] = row_major[row * 4 + col];
    }
  }
}
#endif

}  // namespace

SolverWrapper::SolverWrapper()
{
  initializeIfNeeded();
}

SolverWrapper::~SolverWrapper()
{
  terminateIfNeeded();
}

void SolverWrapper::initializeIfNeeded()
{
#ifdef GIK9DOF_CODEGEN_AVAILABLE
  if (!initialized_)
  {
    gik9dof_codegen_followTrajectory_initialize();
    initialized_ = true;
  }
#endif
}

void SolverWrapper::terminateIfNeeded()
{
#ifdef GIK9DOF_CODEGEN_AVAILABLE
  if (initialized_)
  {
    gik9dof_codegen_followTrajectory_terminate();
    initialized_ = false;
  }
#endif
}

SolverResult SolverWrapper::solveHolisticStep(const std::array<double, 9> & current,
                                              const std::array<double, 16> & target_pose,
                                              const SolverOptions & options)
{
  SolverResult result;
  result.joint_positions = current;

#ifdef GIK9DOF_CODEGEN_AVAILABLE
  initializeIfNeeded();

  double q_current[9];
  std::copy(current.begin(), current.end(), std::begin(q_current));

  double poses_data[16];
  toColumnMajor(target_pose, poses_data);
  int poses_size[3] = {4, 4, 1};

  double q_next[9] {};
  gik9dof::codegen::followTrajectory(q_current, poses_data, poses_size,
                                     options.distance_lower_bound,
                                     options.distance_weight,
                                     q_next);
  std::copy(std::begin(q_next), std::end(q_next), result.joint_positions.begin());
  result.success = true;
  result.exit_flag = 1;
  result.iterations = 1;
  result.constraint_violation = 0.0;
#else
  (void)target_pose;
  (void)options;
  result.success = true;
  result.exit_flag = 1;
  result.iterations = 0;
  result.constraint_violation = 0.0;
#endif

  return result;
}

SolverResult SolverWrapper::solveStagedStep(const std::array<double, 9> & current,
                                            const std::array<double, 16> & target_pose,
                                            const SolverOptions & options,
                                            bool arm_active,
                                            bool base_active)
{
  SolverResult result = solveHolisticStep(current, target_pose, options);

  if (!arm_active)
  {
    for (std::size_t i = 3; i < result.joint_positions.size(); ++i)
    {
      result.joint_positions[i] = current[i];
    }
  }

  if (!base_active)
  {
    for (std::size_t i = 0; i < 3; ++i)
    {
      result.joint_positions[i] = current[i];
    }
  }

  return result;
}

std::array<double, 16> identityPose()
{
  return identityPoseArray();
}

}  // namespace gik9dof_solver