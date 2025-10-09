//
// File: ErrorDampedLevenbergMarquardt.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "SystemTimeProvider.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_internal_types.h"
#include "ixfun.h"
#include "mldivide.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Declarations
static void binary_expand_op_3(const coder::array<double, 2U> &in1, double in2,
                               const coder::array<double, 2U> &in3,
                               coder::array<double, 1U> &in4);

static void minus(coder::array<double, 1U> &in1,
                  const coder::array<double, 1U> &in2);

static void plus(coder::array<double, 1U> &in1,
                 const coder::array<double, 1U> &in2);

static void plus(coder::array<double, 1U> &in1,
                 const coder::array<double, 1U> &in2,
                 const coder::array<double, 1U> &in3);

// Function Definitions
//
// Arguments    : const coder::array<double, 2U> &in1
//                double in2
//                const coder::array<double, 2U> &in3
//                coder::array<double, 1U> &in4
// Return Type  : void
//
static void binary_expand_op_3(const coder::array<double, 2U> &in1, double in2,
                               const coder::array<double, 2U> &in3,
                               coder::array<double, 1U> &in4)
{
  coder::array<double, 2U> b_in1;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  if (in3.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in3.size(0);
  }
  if (in3.size(1) == 1) {
    b_loop_ub = in1.size(1);
  } else {
    b_loop_ub = in3.size(1);
  }
  b_in1.set_size(loop_ub, b_loop_ub);
  stride_0_0 = (in1.size(0) != 1);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_0 = (in3.size(0) != 1);
  stride_1_1 = (in3.size(1) != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (int i{0}; i < b_loop_ub; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      b_in1[i1 + b_in1.size(0) * i] =
          -(in1[i1 * stride_0_0 + in1.size(0) * aux_0_1] +
            in2 * in3[i1 * stride_1_0 + in3.size(0) * aux_1_1]);
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  coder::mldivide(b_in1, in4);
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
// Return Type  : void
//
static void minus(coder::array<double, 1U> &in1,
                  const coder::array<double, 1U> &in2)
{
  coder::array<double, 1U> b_in2;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in1.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in1.size(0);
  }
  b_in2.set_size(loop_ub);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in1.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    b_in2[i] = in2[i * stride_0_0] - in1[i * stride_1_0];
  }
  in1.set_size(loop_ub);
  for (int i{0}; i < loop_ub; i++) {
    in1[i] = b_in2[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
// Return Type  : void
//
static void plus(coder::array<double, 1U> &in1,
                 const coder::array<double, 1U> &in2)
{
  coder::array<double, 1U> b_in1;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in2.size(0) == 1) {
    loop_ub = in1.size(0);
  } else {
    loop_ub = in2.size(0);
  }
  b_in1.set_size(loop_ub);
  stride_0_0 = (in1.size(0) != 1);
  stride_1_0 = (in2.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    b_in1[i] = in1[i * stride_0_0] + in2[i * stride_1_0];
  }
  in1.set_size(loop_ub);
  for (int i{0}; i < loop_ub; i++) {
    in1[i] = b_in1[i];
  }
}

//
// Arguments    : coder::array<double, 1U> &in1
//                const coder::array<double, 1U> &in2
//                const coder::array<double, 1U> &in3
// Return Type  : void
//
static void plus(coder::array<double, 1U> &in1,
                 const coder::array<double, 1U> &in2,
                 const coder::array<double, 1U> &in3)
{
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in3.size(0) == 1) {
    loop_ub = in2.size(0);
  } else {
    loop_ub = in3.size(0);
  }
  in1.set_size(loop_ub);
  stride_0_0 = (in2.size(0) != 1);
  stride_1_0 = (in3.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    in1[i] = in2[i * stride_0_0] + in3[i * stride_1_0];
  }
}

//
// Arguments    : char params_Name[18]
//                double &params_MaxTime
//                double &params_GradientTolerance
//                double &params_SolutionTolerance
//                boolean_T &params_ConstraintsOn
//                boolean_T &params_RandomRestart
//                double &params_StepTolerance
//                double &params_ErrorChangeTolerance
//                double &params_DampingBias
//                boolean_T &params_UseErrorDamping
// Return Type  : double
//
namespace coder {
namespace robotics {
namespace core {
namespace internal {
double ErrorDampedLevenbergMarquardt::getSolverParams(
    char params_Name[18], double &params_MaxTime,
    double &params_GradientTolerance, double &params_SolutionTolerance,
    boolean_T &params_ConstraintsOn, boolean_T &params_RandomRestart,
    double &params_StepTolerance, double &params_ErrorChangeTolerance,
    double &params_DampingBias, boolean_T &params_UseErrorDamping) const
{
  double params_MaxNumIteration;
  for (int i{0}; i < 18; i++) {
    params_Name[i] = Name[i];
  }
  params_MaxNumIteration = MaxNumIteration;
  params_MaxTime = MaxTime;
  params_GradientTolerance = GradientTolerance;
  params_SolutionTolerance = SolutionTolerance;
  params_ConstraintsOn = ConstraintsOn;
  params_RandomRestart = RandomRestart;
  params_StepTolerance = StepTolerance;
  params_ErrorChangeTolerance = ErrorChangeTolerance;
  params_DampingBias = DampingBias;
  params_UseErrorDamping = UseErrorDamping;
  return params_MaxNumIteration;
}

//
// Arguments    : array<double, 1U> &xSol
//                double &en
//                double &iter
// Return Type  : NLPSolverExitFlags
//
NLPSolverExitFlags
ErrorDampedLevenbergMarquardt::solveInternal(array<double, 1U> &xSol,
                                             double &en, double &iter)
{
  manip::internal::GIKProblem *problem;
  array<double, 2U> A;
  array<double, 2U> C;
  array<double, 2U> H0;
  array<double, 2U> Jac;
  array<double, 2U> W;
  array<double, 2U> b_W;
  array<double, 2U> b_problem;
  array<double, 2U> d_problem;
  array<double, 2U> e_problem;
  array<double, 2U> f_problem;
  array<double, 2U> r2;
  array<double, 2U> r3;
  array<double, 1U> a__1;
  array<double, 1U> c_problem;
  array<double, 1U> c_x;
  array<double, 1U> evprev;
  array<double, 1U> f;
  array<double, 1U> grad;
  array<double, 1U> step;
  array<double, 1U> x;
  array<double, 1U> xprev;
  array<boolean_T, 1U> b_x;
  double newcost;
  int b_i;
  int i;
  int loop_ub;
  int n;
  NLPSolverExitFlags exitFlag;
  n = SeedInternal.size(0);
  x.set_size(n);
  loop_ub = SeedInternal.size(0);
  for (i = 0; i < loop_ub; i++) {
    x[i] = SeedInternal[i];
  }
  TimeObjInternal.StartTime.tv_sec = tic(TimeObjInternal.StartTime.tv_nsec);
  xSol.set_size(n);
  xprev.set_size(n);
  for (i = 0; i < n; i++) {
    xSol[i] = x[i];
    xprev[i] = x[i];
  }
  problem = ExtraArgs;
  problem->residuals(x, a__1, Jac);
  problem->get_WeightMatrix(b_problem);
  problem->residuals(x, c_problem);
  problem->get_WeightMatrix(d_problem);
  ExtraArgs = problem;
  problem = ExtraArgs;
  problem->evaluateSolution(x, evprev);
  newcost = MaxNumIterationInternal;
  i = static_cast<int>(newcost);
  b_i = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (b_i <= i - 1) {
      __m128d r;
      double b_C;
      int aoffset;
      int b_nc_tmp;
      int boffset;
      int i1;
      int nc_tmp;
      int scalarLB;
      int vectorUB;
      boolean_T flag;
      problem = ExtraArgs;
      problem->residuals(x, a__1, Jac);
      problem->get_WeightMatrix(W);
      problem->residuals(x, f);
      loop_ub = f.size(0);
      A.set_size(1, f.size(0));
      scalarLB = (f.size(0) / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i1 = 0; i1 <= vectorUB; i1 += 2) {
        r = _mm_loadu_pd(&f[i1]);
        _mm_storeu_pd(&A[i1], _mm_mul_pd(_mm_set1_pd(0.5), r));
      }
      for (i1 = scalarLB; i1 < loop_ub; i1++) {
        A[i1] = 0.5 * f[i1];
      }
      aoffset = A.size(1);
      nc_tmp = W.size(1);
      C.set_size(1, W.size(1));
      for (int j{0}; j < nc_tmp; j++) {
        boffset = j * W.size(0);
        C[j] = 0.0;
        for (int k{0}; k < aoffset; k++) {
          C[j] = C[j] + A[k] * W[boffset + k];
        }
      }
      b_C = 0.0;
      for (i1 = 0; i1 < nc_tmp; i1++) {
        b_C += C[i1] * f[i1];
      }
      problem->get_WeightMatrix(W);
      ExtraArgs = problem;
      problem = ExtraArgs;
      problem->get_WeightMatrix(b_W);
      problem->residuals(x, f, H0);
      aoffset = f.size(0);
      nc_tmp = b_W.size(1);
      A.set_size(1, b_W.size(1));
      for (int j{0}; j < nc_tmp; j++) {
        boffset = j * b_W.size(0);
        A[j] = 0.0;
        for (int k{0}; k < aoffset; k++) {
          A[j] = A[j] + f[k] * b_W[boffset + k];
        }
      }
      b_nc_tmp = H0.size(1);
      C.set_size(1, H0.size(1));
      for (int j{0}; j < b_nc_tmp; j++) {
        boffset = j * H0.size(0);
        C[j] = 0.0;
        for (int k{0}; k < nc_tmp; k++) {
          C[j] = C[j] + A[k] * H0[boffset + k];
        }
      }
      grad.set_size(H0.size(1));
      for (i1 = 0; i1 < b_nc_tmp; i1++) {
        grad[i1] = C[i1];
      }
      problem = ExtraArgs;
      en = problem->evaluateSolution(x, a__1);
      loop_ub = x.size(0);
      xSol.set_size(x.size(0));
      for (i1 = 0; i1 < loop_ub; i1++) {
        xSol[i1] = x[i1];
      }
      iter = static_cast<double>(b_i) + 1.0;
      flag = (b_norm(grad) < GradientTolerance);
      if (flag) {
        exitFlag = NLPSolverExitFlags::LocalMinimumFound;
        exitg1 = 1;
      } else {
        __m128d r1;
        boolean_T exitg2;
        boolean_T guard1;
        boolean_T guard2;
        guard1 = false;
        guard2 = false;
        if (static_cast<double>(b_i) + 1.0 > 1.0) {
          if (x.size(0) == xprev.size(0)) {
            xprev.set_size(x.size(0));
            scalarLB = (x.size(0) / 2) << 1;
            vectorUB = scalarLB - 2;
            for (i1 = 0; i1 <= vectorUB; i1 += 2) {
              r = _mm_loadu_pd(&x[i1]);
              r1 = _mm_loadu_pd(&xprev[i1]);
              _mm_storeu_pd(&xprev[i1], _mm_sub_pd(r, r1));
            }
            for (i1 = scalarLB; i1 < loop_ub; i1++) {
              xprev[i1] = x[i1] - xprev[i1];
            }
          } else {
            minus(xprev, x);
          }
          aoffset = xprev.size(0);
          f.set_size(xprev.size(0));
          for (int k{0}; k < aoffset; k++) {
            f[k] = std::abs(xprev[k]);
          }
          b_x.set_size(xprev.size(0));
          for (i1 = 0; i1 < aoffset; i1++) {
            b_x[i1] = (f[i1] < StepTolerance);
          }
          flag = true;
          aoffset = 1;
          exitg2 = false;
          while ((!exitg2) && (aoffset <= b_x.size(0))) {
            if (!b_x[aoffset - 1]) {
              flag = false;
              exitg2 = true;
            } else {
              aoffset++;
            }
          }
          if (flag) {
            exitFlag = NLPSolverExitFlags::StepSizeBelowMinimum;
            exitg1 = 1;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          if (static_cast<double>(b_i) + 1.0 > 1.0) {
            if (a__1.size(0) == evprev.size(0)) {
              loop_ub = a__1.size(0);
              evprev.set_size(a__1.size(0));
              scalarLB = (a__1.size(0) / 2) << 1;
              vectorUB = scalarLB - 2;
              for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                r = _mm_loadu_pd(&a__1[i1]);
                r1 = _mm_loadu_pd(&evprev[i1]);
                _mm_storeu_pd(&evprev[i1], _mm_sub_pd(r, r1));
              }
              for (i1 = scalarLB; i1 < loop_ub; i1++) {
                evprev[i1] = a__1[i1] - evprev[i1];
              }
            } else {
              minus(evprev, a__1);
            }
            aoffset = evprev.size(0);
            f.set_size(evprev.size(0));
            for (int k{0}; k < aoffset; k++) {
              f[k] = std::abs(evprev[k]);
            }
            b_x.set_size(evprev.size(0));
            for (i1 = 0; i1 < aoffset; i1++) {
              b_x[i1] = (f[i1] < ErrorChangeTolerance);
            }
            flag = true;
            aoffset = 1;
            exitg2 = false;
            while ((!exitg2) && (aoffset <= b_x.size(0))) {
              if (!b_x[aoffset - 1]) {
                flag = false;
                exitg2 = true;
              } else {
                aoffset++;
              }
            }
            if (flag) {
              exitFlag = NLPSolverExitFlags::ChangeInErrorBelowMinimum;
              exitg1 = 1;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }
        if (guard1) {
          newcost = toc(TimeObjInternal.StartTime.tv_sec,
                        TimeObjInternal.StartTime.tv_nsec);
          flag = (newcost > MaxTimeInternal);
          if (flag) {
            exitFlag = NLPSolverExitFlags::TimeLimitExceeded;
            exitg1 = 1;
          } else {
            double bkj;
            double cc;
            int coffset;
            int mc_tmp;
            loop_ub = a__1.size(0);
            evprev.set_size(a__1.size(0));
            for (i1 = 0; i1 < loop_ub; i1++) {
              evprev[i1] = a__1[i1];
            }
            loop_ub = x.size(0);
            xprev.set_size(x.size(0));
            for (i1 = 0; i1 < loop_ub; i1++) {
              xprev[i1] = x[i1];
            }
            flag = UseErrorDamping;
            cc = static_cast<double>(flag) * b_C;
            newcost = cc + DampingBias;
            mc_tmp = Jac.size(1);
            aoffset = Jac.size(0);
            nc_tmp = W.size(1);
            b_W.set_size(Jac.size(1), W.size(1));
            for (int j{0}; j < nc_tmp; j++) {
              coffset = j * mc_tmp;
              boffset = j * W.size(0);
              for (int c_i{0}; c_i < mc_tmp; c_i++) {
                b_W[coffset + c_i] = 0.0;
              }
              for (int k{0}; k < aoffset; k++) {
                bkj = W[boffset + k];
                for (int c_i{0}; c_i < mc_tmp; c_i++) {
                  i1 = coffset + c_i;
                  b_W[i1] = b_W[i1] + Jac[c_i * Jac.size(0) + k] * bkj;
                }
              }
            }
            H0.set_size(Jac.size(1), Jac.size(1));
            for (int j{0}; j < mc_tmp; j++) {
              coffset = j * mc_tmp;
              boffset = j * Jac.size(0);
              for (int c_i{0}; c_i < mc_tmp; c_i++) {
                H0[coffset + c_i] = 0.0;
              }
              for (int k{0}; k < nc_tmp; k++) {
                aoffset = k * b_W.size(0);
                bkj = Jac[boffset + k];
                scalarLB = (mc_tmp / 2) << 1;
                vectorUB = scalarLB - 2;
                for (int c_i{0}; c_i <= vectorUB; c_i += 2) {
                  r = _mm_loadu_pd(&b_W[aoffset + c_i]);
                  i1 = coffset + c_i;
                  r1 = _mm_loadu_pd(&H0[i1]);
                  _mm_storeu_pd(
                      &H0[i1], _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(bkj))));
                }
                for (int c_i{scalarLB}; c_i < mc_tmp; c_i++) {
                  i1 = coffset + c_i;
                  H0[i1] = H0[i1] + b_W[aoffset + c_i] * bkj;
                }
              }
            }
            b_W.set_size(n, n);
            mc_tmp = n * n;
            for (i1 = 0; i1 < mc_tmp; i1++) {
              b_W[i1] = 0.0;
            }
            if (n > 0) {
              for (int k{0}; k < n; k++) {
                b_W[k + b_W.size(0) * k] = 1.0;
              }
            }
            step.set_size(b_nc_tmp);
            for (i1 = 0; i1 < b_nc_tmp; i1++) {
              step[i1] = grad[i1];
            }
            if ((H0.size(0) == b_W.size(0)) && (H0.size(1) == b_W.size(1))) {
              Jac.set_size(H0.size(0), H0.size(1));
              aoffset = H0.size(0) * H0.size(1);
              scalarLB = (aoffset / 2) << 1;
              vectorUB = scalarLB - 2;
              for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                r = _mm_loadu_pd(&b_W[i1]);
                r1 = _mm_loadu_pd(&H0[i1]);
                _mm_storeu_pd(
                    &Jac[i1],
                    _mm_mul_pd(
                        _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(newcost), r)),
                        _mm_set1_pd(-1.0)));
              }
              for (i1 = scalarLB; i1 < aoffset; i1++) {
                Jac[i1] = -(H0[i1] + newcost * b_W[i1]);
              }
              mldivide(Jac, step);
            } else {
              binary_expand_op_3(H0, newcost, b_W, step);
            }
            if (x.size(0) == step.size(0)) {
              c_x.set_size(x.size(0));
              scalarLB = (x.size(0) / 2) << 1;
              vectorUB = scalarLB - 2;
              for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                r = _mm_loadu_pd(&x[i1]);
                r1 = _mm_loadu_pd(&step[i1]);
                _mm_storeu_pd(&c_x[i1], _mm_add_pd(r, r1));
              }
              for (i1 = scalarLB; i1 < loop_ub; i1++) {
                c_x[i1] = x[i1] + step[i1];
              }
            } else {
              plus(c_x, x, step);
            }
            problem = ExtraArgs;
            problem->residuals(c_x, a__1, Jac);
            problem->get_WeightMatrix(W);
            problem->residuals(c_x, f);
            loop_ub = f.size(0);
            A.set_size(1, f.size(0));
            scalarLB = (f.size(0) / 2) << 1;
            vectorUB = scalarLB - 2;
            for (i1 = 0; i1 <= vectorUB; i1 += 2) {
              r = _mm_loadu_pd(&f[i1]);
              _mm_storeu_pd(&A[i1], _mm_mul_pd(_mm_set1_pd(0.5), r));
            }
            for (i1 = scalarLB; i1 < loop_ub; i1++) {
              A[i1] = 0.5 * f[i1];
            }
            aoffset = A.size(1);
            nc_tmp = W.size(1);
            C.set_size(1, W.size(1));
            for (int j{0}; j < nc_tmp; j++) {
              boffset = j * W.size(0);
              C[j] = 0.0;
              for (int k{0}; k < aoffset; k++) {
                C[j] = C[j] + A[k] * W[boffset + k];
              }
            }
            newcost = 0.0;
            for (i1 = 0; i1 < nc_tmp; i1++) {
              newcost += C[i1] * f[i1];
            }
            problem->get_WeightMatrix(e_problem);
            bkj = 1.0;
            while (newcost > b_C) {
              bkj *= 2.5;
              newcost = cc + bkj * DampingBias;
              b_W.set_size(n, n);
              for (i1 = 0; i1 < mc_tmp; i1++) {
                b_W[i1] = 0.0;
              }
              if (n > 0) {
                for (int k{0}; k < n; k++) {
                  b_W[k + b_W.size(0) * k] = 1.0;
                }
              }
              step.set_size(b_nc_tmp);
              for (i1 = 0; i1 < b_nc_tmp; i1++) {
                step[i1] = grad[i1];
              }
              if ((H0.size(0) == b_W.size(0)) && (H0.size(1) == b_W.size(1))) {
                Jac.set_size(H0.size(0), H0.size(1));
                aoffset = H0.size(0) * H0.size(1);
                scalarLB = (aoffset / 2) << 1;
                vectorUB = scalarLB - 2;
                for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                  r = _mm_loadu_pd(&b_W[i1]);
                  r1 = _mm_loadu_pd(&H0[i1]);
                  _mm_storeu_pd(
                      &Jac[i1],
                      _mm_mul_pd(
                          _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(newcost), r)),
                          _mm_set1_pd(-1.0)));
                }
                for (i1 = scalarLB; i1 < aoffset; i1++) {
                  Jac[i1] = -(H0[i1] + newcost * b_W[i1]);
                }
                mldivide(Jac, step);
              } else {
                binary_expand_op_3(H0, newcost, b_W, step);
              }
              if (x.size(0) == step.size(0)) {
                loop_ub = x.size(0);
                c_x.set_size(x.size(0));
                scalarLB = (x.size(0) / 2) << 1;
                vectorUB = scalarLB - 2;
                for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                  r = _mm_loadu_pd(&x[i1]);
                  r1 = _mm_loadu_pd(&step[i1]);
                  _mm_storeu_pd(&c_x[i1], _mm_add_pd(r, r1));
                }
                for (i1 = scalarLB; i1 < loop_ub; i1++) {
                  c_x[i1] = x[i1] + step[i1];
                }
              } else {
                plus(c_x, x, step);
              }
              problem = ExtraArgs;
              problem->residuals(c_x, a__1, Jac);
              problem->get_WeightMatrix(W);
              problem->residuals(c_x, f);
              loop_ub = f.size(0);
              A.set_size(1, f.size(0));
              scalarLB = (f.size(0) / 2) << 1;
              vectorUB = scalarLB - 2;
              for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                r = _mm_loadu_pd(&f[i1]);
                _mm_storeu_pd(&A[i1], _mm_mul_pd(_mm_set1_pd(0.5), r));
              }
              for (i1 = scalarLB; i1 < loop_ub; i1++) {
                A[i1] = 0.5 * f[i1];
              }
              aoffset = A.size(1);
              nc_tmp = W.size(1);
              C.set_size(1, W.size(1));
              for (int j{0}; j < nc_tmp; j++) {
                boffset = j * W.size(0);
                C[j] = 0.0;
                for (int k{0}; k < aoffset; k++) {
                  C[j] = C[j] + A[k] * W[boffset + k];
                }
              }
              newcost = 0.0;
              for (i1 = 0; i1 < nc_tmp; i1++) {
                newcost += C[i1] * f[i1];
              }
              problem->get_WeightMatrix(f_problem);
            }
            if (x.size(0) == step.size(0)) {
              loop_ub = x.size(0);
              scalarLB = (x.size(0) / 2) << 1;
              vectorUB = scalarLB - 2;
              for (i1 = 0; i1 <= vectorUB; i1 += 2) {
                r = _mm_loadu_pd(&x[i1]);
                r1 = _mm_loadu_pd(&step[i1]);
                _mm_storeu_pd(&x[i1], _mm_add_pd(r, r1));
              }
              for (i1 = scalarLB; i1 < loop_ub; i1++) {
                x[i1] = x[i1] + step[i1];
              }
            } else {
              plus(x, step);
            }
            if (ConstraintsOn) {
              problem = ExtraArgs;
              loop_ub = problem->DesignVariableBoundsInternal.size(0);
              r2.set_size(loop_ub, 2);
              aoffset = problem->DesignVariableBoundsInternal.size(0) << 1;
              for (i1 = 0; i1 < aoffset; i1++) {
                r2[i1] = problem->DesignVariableBoundsInternal[i1];
              }
              aoffset = problem->DesignVariableBoundsInternal.size(0);
              r3.set_size(aoffset, 2);
              mc_tmp = problem->DesignVariableBoundsInternal.size(0) << 1;
              for (i1 = 0; i1 < mc_tmp; i1++) {
                r3[i1] = problem->DesignVariableBoundsInternal[i1];
              }
              if (r3.size(0) == x.size(0)) {
                a__1.set_size(aoffset);
                for (i1 = 0; i1 < aoffset; i1++) {
                  newcost = r3[i1];
                  bkj = x[i1];
                  a__1[i1] = std::fmax(newcost, bkj);
                }
              } else {
                f.set_size(aoffset);
                for (i1 = 0; i1 < aoffset; i1++) {
                  f[i1] = r3[i1];
                }
                ::coder::internal::expand_max(f, x, a__1);
              }
              if (r2.size(0) == a__1.size(0)) {
                x.set_size(loop_ub);
                for (i1 = 0; i1 < loop_ub; i1++) {
                  newcost = r2[i1 + r2.size(0)];
                  bkj = a__1[i1];
                  x[i1] = std::fmin(newcost, bkj);
                }
              } else {
                f.set_size(loop_ub);
                for (i1 = 0; i1 < loop_ub; i1++) {
                  f[i1] = r2[i1 + r2.size(0)];
                }
                ::coder::internal::expand_min(f, a__1, x);
              }
            }
            b_i++;
          }
        }
      }
    } else {
      problem = ExtraArgs;
      en = problem->evaluateSolution(xSol, a__1);
      loop_ub = x.size(0);
      xSol.set_size(x.size(0));
      for (i = 0; i < loop_ub; i++) {
        xSol[i] = x[i];
      }
      iter = MaxNumIterationInternal;
      exitFlag = NLPSolverExitFlags::IterationLimitExceeded;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return exitFlag;
}

} // namespace internal
} // namespace core
} // namespace robotics
} // namespace coder

//
// File trailer for ErrorDampedLevenbergMarquardt.cpp
//
// [EOF]
//
