#pragma once

#include <array>
#include <vector>

namespace gik9dof_ros
{

struct SolverOptions
{
  double distance_lower_bound {0.0};
  double distance_weight {0.0};
};

struct SolverResult
{
  std::array<double, 9> joint_positions{};
  bool success {false};
  int exit_flag {0};
  int iterations {0};
  double constraint_violation {0.0};
};

inline SolverResult solveHolisticStep(const std::array<double, 9>& current, const std::array<double, 16>& target_pose,
                                      const SolverOptions& options)
{
  SolverResult result;
  result.joint_positions = current;  // placeholder pass-through
  (void)target_pose;
  (void)options;
  result.success = true;
  result.exit_flag = 1;
  result.iterations = 0;
  result.constraint_violation = 0.0;
  return result;
}

inline SolverResult solveStagedStep(const std::array<double, 9>& current,
                                    const std::array<double, 16>& target_pose,
                                    const SolverOptions& options,
                                    bool arm_active, bool base_active)
{
  SolverResult result = solveHolisticStep(current, target_pose, options);
  if (!arm_active)
  {
    for (size_t i = 3; i < 9; ++i)
      result.joint_positions[i] = current[i];
  }
  if (!base_active)
  {
    for (size_t i = 0; i < 3; ++i)
      result.joint_positions[i] = current[i];
  }
  return result;
}

inline std::array<double, 16> identityPose()
{
  std::array<double, 16> pose{};
  pose[0] = pose[5] = pose[10] = pose[15] = 1.0;
  return pose;
}

}  // namespace gik9dof_ros
