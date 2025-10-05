#pragma once

#include <array>

#include "gik9dof_solver/solver_wrapper.hpp"

namespace gik9dof_ros
{

using SolverOptions = gik9dof_solver::SolverOptions;
using SolverResult = gik9dof_solver::SolverResult;

inline gik9dof_solver::SolverWrapper & solverInstance()
{
  static gik9dof_solver::SolverWrapper solver;
  return solver;
}

inline SolverResult solveHolisticStep(const std::array<double, 9> & current,
                                      const std::array<double, 16> & target_pose,
                                      const SolverOptions & options)
{
  return solverInstance().solveHolisticStep(current, target_pose, options);
}

inline SolverResult solveStagedStep(const std::array<double, 9> & current,
                                    const std::array<double, 16> & target_pose,
                                    const SolverOptions & options,
                                    bool arm_active,
                                    bool base_active)
{
  return solverInstance().solveStagedStep(current, target_pose, options, arm_active, base_active);
}

inline std::array<double, 16> identityPose()
{
  return gik9dof_solver::identityPose();
}

}  // namespace gik9dof_ros
