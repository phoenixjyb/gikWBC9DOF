#pragma once

#include <array>

namespace gik9dof_solver
{

struct SolverOptions
{
  double distance_lower_bound {0.0};
  double distance_weight {0.0};
};

struct SolverResult
{
  std::array<double, 9> joint_positions {};
  bool success {false};
  int exit_flag {0};
  int iterations {0};
  double constraint_violation {0.0};
};

class SolverWrapper
{
public:
  SolverWrapper();
  ~SolverWrapper();

  SolverResult solveHolisticStep(const std::array<double, 9> & current,
                                 const std::array<double, 16> & target_pose,
                                 const SolverOptions & options);

  SolverResult solveStagedStep(const std::array<double, 9> & current,
                               const std::array<double, 16> & target_pose,
                               const SolverOptions & options,
                               bool arm_active,
                               bool base_active);

private:
  void initializeIfNeeded();
  void terminateIfNeeded();

  bool initialized_ {false};
};

std::array<double, 16> identityPose();

}  // namespace gik9dof_solver
