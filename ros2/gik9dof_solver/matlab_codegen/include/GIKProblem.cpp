//
// File: GIKProblem.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

// Include Files
#include "GIKProblem.h"
#include "CharacterVector.h"
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "KinematicConstraint.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "eye.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "structConstructorHelper.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>

// Type Definitions
namespace gik9dof
{
  struct cell_wrap_67
  {
    ::coder::array<double, 2U> f1;
  };
}

// Function Declarations
namespace gik9dof
{
  static void binary_expand_op_1(::coder::array<double, 1U> &in1, const ::coder::
    array<int, 1U> &in2, const ::coder::array<double, 1U> &in3, int in4, const ::
    coder::array<double, 1U> &in5);
  static void binary_expand_op_2(::coder::array<double, 1U> &in1, const ::coder::
    array<int, 1U> &in2, const double in3[2], const ::coder::array<double, 1U>
    &in4);
}

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
namespace gik9dof
{
  namespace coder
  {
    namespace robotics
    {
      namespace manip
      {
        namespace internal
        {
          void GIKProblem::b_set_EnforceJointLimits()
          {
            ::coder::array<double, 2U> A;
            ::coder::array<double, 2U> r;
            ::coder::array<double, 2U> r1;
            ::coder::array<double, 1U> b;
            EnforceJointLimitsInternal = true;
            if (EnforceJointLimitsInternal) {
              double d;
              int b_loop_ub;
              int c_loop_ub;
              int loop_ub;
              loop_ub = DesignVariableBoundsInternal.size(0);
              r.set_size(loop_ub, 2);
              b_loop_ub = DesignVariableBoundsInternal.size(0) << 1;
              for (int i{0}; i < b_loop_ub; i++) {
                r[i] = DesignVariableBoundsInternal[i];
              }

              d = NumPositions;
              Tree->get_JointPositionLimits(r1);
              if (d < 1.0) {
                b_loop_ub = 0;
              } else {
                b_loop_ub = static_cast<int>(d);
              }

              for (int i{0}; i < 2; i++) {
                for (c_loop_ub = 0; c_loop_ub < b_loop_ub; c_loop_ub++) {
                  r[c_loop_ub + r.size(0) * i] = r1[c_loop_ub + r1.size(0) * i];
                }
              }

              DesignVariableBoundsInternal.set_size(loop_ub, 2);
              c_loop_ub = r.size(0) << 1;
              for (int i{0}; i < c_loop_ub; i++) {
                DesignVariableBoundsInternal[i] = r[i];
              }

              b_loop_ub = 2 * r.size(0);
              A.set_size(b_loop_ub, loop_ub);
              c_loop_ub = b_loop_ub * r.size(0);
              for (int i{0}; i < c_loop_ub; i++) {
                A[i] = 0.0;
              }

              b.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                b[i] = 0.0;
              }

              for (int i{0}; i < loop_ub; i++) {
                c_loop_ub = static_cast<int>(static_cast<unsigned int>(i + 1) <<
                  1);
                A[(c_loop_ub + A.size(0) * i) - 2] = -1.0;
                A[(c_loop_ub + A.size(0) * i) - 1] = 1.0;
                b[c_loop_ub - 2] = -r[i];
                b[c_loop_ub - 1] = r[i + r.size(0)];
              }

              ConstraintMatrixInternal.set_size(loop_ub, b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                for (c_loop_ub = 0; c_loop_ub < loop_ub; c_loop_ub++) {
                  ConstraintMatrixInternal[c_loop_ub +
                    ConstraintMatrixInternal.size(0) * i] = A[i + A.size(0) *
                    c_loop_ub];
                }
              }

              ConstraintBoundInternal.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                ConstraintBoundInternal[i] = b[i];
              }
            } else {
              int b_loop_ub;
              int c_loop_ub;
              int loop_ub;
              loop_ub = DesignVariableBoundsInternal.size(0);
              r.set_size(loop_ub, 2);
              b_loop_ub = DesignVariableBoundsInternal.size(0) << 1;
              for (int i{0}; i < b_loop_ub; i++) {
                r[i] = DesignVariableBoundsInternal[i];
              }

              b_loop_ub = static_cast<int>(NumPositions);
              c_loop_ub = static_cast<int>(NumPositions);
              for (int i{0}; i < b_loop_ub; i++) {
                r[i] = rtMinusInf;
              }

              for (int i{0}; i < c_loop_ub; i++) {
                r[i + r.size(0)] = rtInf;
              }

              DesignVariableBoundsInternal.set_size(loop_ub, 2);
              c_loop_ub = r.size(0) << 1;
              for (int i{0}; i < c_loop_ub; i++) {
                DesignVariableBoundsInternal[i] = r[i];
              }

              b_loop_ub = 2 * r.size(0);
              A.set_size(b_loop_ub, loop_ub);
              c_loop_ub = b_loop_ub * r.size(0);
              for (int i{0}; i < c_loop_ub; i++) {
                A[i] = 0.0;
              }

              b.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                b[i] = 0.0;
              }

              for (int i{0}; i < loop_ub; i++) {
                c_loop_ub = static_cast<int>(static_cast<unsigned int>(i + 1) <<
                  1);
                A[(c_loop_ub + A.size(0) * i) - 2] = -1.0;
                A[(c_loop_ub + A.size(0) * i) - 1] = 1.0;
                b[c_loop_ub - 2] = -r[i];
                b[c_loop_ub - 1] = r[i + r.size(0)];
              }

              ConstraintMatrixInternal.set_size(loop_ub, b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                for (c_loop_ub = 0; c_loop_ub < loop_ub; c_loop_ub++) {
                  ConstraintMatrixInternal[c_loop_ub +
                    ConstraintMatrixInternal.size(0) * i] = A[i + A.size(0) *
                    c_loop_ub];
                }
              }

              ConstraintBoundInternal.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                ConstraintBoundInternal[i] = b[i];
              }
            }
          }

          //
          // Arguments    : const ::coder::array<double, 1U> &x
          //                ::coder::array<double, 1U> &f
          //                ::coder::array<double, 2U> &J
          // Return Type  : void
          //
          void GIKProblem::residualsInternal(const ::coder::array<double, 1U> &x,
            ::coder::array<double, 1U> &f, ::coder::array<double, 2U> &J)
          {
            JointPositionBounds *obj;
            ::coder::array<double, 2U> cols;
            ::coder::array<double, 2U> r3;
            ::coder::array<double, 2U> rows;
            ::coder::array<double, 1U> q;
            ::coder::array<double, 1U> slacks;
            ::coder::array<int, 2U> r1;
            ::coder::array<int, 1U> r;
            ::coder::array<int, 1U> r10;
            ::coder::array<int, 1U> r11;
            ::coder::array<int, 1U> r12;
            ::coder::array<int, 1U> r13;
            ::coder::array<int, 1U> r14;
            ::coder::array<int, 1U> r15;
            ::coder::array<int, 1U> r16;
            ::coder::array<int, 1U> r17;
            ::coder::array<int, 1U> r18;
            ::coder::array<int, 1U> r19;
            ::coder::array<int, 1U> r20;
            ::coder::array<int, 1U> r21;
            ::coder::array<int, 1U> r22;
            ::coder::array<int, 1U> r23;
            ::coder::array<int, 1U> r24;
            ::coder::array<int, 1U> r25;
            ::coder::array<int, 1U> r26;
            ::coder::array<int, 1U> r27;
            ::coder::array<int, 1U> r28;
            ::coder::array<int, 1U> r29;
            ::coder::array<int, 1U> r30;
            ::coder::array<int, 1U> r31;
            ::coder::array<int, 1U> r32;
            ::coder::array<int, 1U> r33;
            ::coder::array<int, 1U> r34;
            ::coder::array<int, 1U> r35;
            ::coder::array<int, 1U> r36;
            ::coder::array<int, 1U> r37;
            ::coder::array<int, 1U> r38;
            ::coder::array<int, 1U> r39;
            ::coder::array<int, 1U> r4;
            ::coder::array<int, 1U> r40;
            ::coder::array<int, 1U> r41;
            ::coder::array<int, 1U> r42;
            ::coder::array<int, 1U> r43;
            ::coder::array<int, 1U> r44;
            ::coder::array<int, 1U> r45;
            ::coder::array<int, 1U> r46;
            ::coder::array<int, 1U> r47;
            ::coder::array<int, 1U> r5;
            ::coder::array<int, 1U> r6;
            ::coder::array<int, 1U> r7;
            ::coder::array<int, 1U> r8;
            ::coder::array<int, 1U> r9;
            ::coder::array<signed char, 2U> r2;
            cell_wrap_67 JCell[22];
            double b_tmp_data[66];
            double c_tmp_data[66];
            double d_tmp_data[66];
            double e_tmp_data[66];
            double f_tmp_data[66];
            double g_tmp_data[66];
            double h_tmp_data[66];
            double i_tmp_data[66];
            double j_tmp_data[66];
            double k_tmp_data[66];
            double l_tmp_data[66];
            double m_tmp_data[66];
            double n_tmp_data[66];
            double o_tmp_data[66];
            double p_tmp_data[66];
            double q_tmp_data[66];
            double r_tmp_data[66];
            double s_tmp_data[66];
            double t_tmp_data[66];
            double tmp_data[66];
            double u_tmp_data[66];
            double gCell_f1[2];
            double d;
            double gCell_f10;
            double gCell_f11;
            double gCell_f12;
            double gCell_f13;
            double gCell_f14;
            double gCell_f15;
            double gCell_f16;
            double gCell_f17;
            double gCell_f18;
            double gCell_f19;
            double gCell_f20;
            double gCell_f21;
            double gCell_f22;
            double gCell_f3;
            double gCell_f4;
            double gCell_f5;
            double gCell_f6;
            double gCell_f7;
            double gCell_f8;
            double gCell_f9;
            int b_loop_ub;
            int c_loop_ub;
            int d_loop_ub;
            int loop_ub;
            int ndx_tmp_idx_0;
            d = NumPositions;
            if (d < 1.0) {
              loop_ub = 0;
            } else {
              loop_ub = static_cast<int>(d);
            }

            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = x[i];
            }

            int tmp_size[2];
            Constraints.f1->evaluate(q, gCell_f1, tmp_data, tmp_size);
            obj = Constraints.f2;
            eye(obj->NumElements, JCell[1].f1);
            gCell_f3 = Constraints.f3->evaluate(q, b_tmp_data, tmp_size);
            gCell_f4 = Constraints.f4->evaluate(q, c_tmp_data, tmp_size);
            gCell_f5 = Constraints.f5->evaluate(q, d_tmp_data, tmp_size);
            gCell_f6 = Constraints.f6->evaluate(q, e_tmp_data, tmp_size);
            gCell_f7 = Constraints.f7->evaluate(q, f_tmp_data, tmp_size);
            gCell_f8 = Constraints.f8->evaluate(q, g_tmp_data, tmp_size);
            gCell_f9 = Constraints.f9->evaluate(q, h_tmp_data, tmp_size);
            gCell_f10 = Constraints.f10->evaluate(q, i_tmp_data, tmp_size);
            gCell_f11 = Constraints.f11->evaluate(q, j_tmp_data, tmp_size);
            gCell_f12 = Constraints.f12->evaluate(q, k_tmp_data, tmp_size);
            gCell_f13 = Constraints.f13->evaluate(q, l_tmp_data, tmp_size);
            gCell_f14 = Constraints.f14->evaluate(q, m_tmp_data, tmp_size);
            gCell_f15 = Constraints.f15->evaluate(q, n_tmp_data, tmp_size);
            gCell_f16 = Constraints.f16->evaluate(q, o_tmp_data, tmp_size);
            gCell_f17 = Constraints.f17->evaluate(q, p_tmp_data, tmp_size);
            gCell_f18 = Constraints.f18->evaluate(q, q_tmp_data, tmp_size);
            gCell_f19 = Constraints.f19->evaluate(q, r_tmp_data, tmp_size);
            gCell_f20 = Constraints.f20->evaluate(q, s_tmp_data, tmp_size);
            gCell_f21 = Constraints.f21->evaluate(q, t_tmp_data, tmp_size);
            gCell_f22 = Constraints.f22->evaluate(q, u_tmp_data, tmp_size);
            f.set_size(static_cast<int>(NumResiduals));
            b_loop_ub = static_cast<int>(NumResiduals);
            for (int i{0}; i < b_loop_ub; i++) {
              f[i] = 0.0;
            }

            ndx_tmp_idx_0 = static_cast<int>(NumResiduals);
            J.set_size(ndx_tmp_idx_0, static_cast<int>(NumVariables));
            b_loop_ub = static_cast<int>(NumResiduals) * static_cast<int>
              (NumVariables);
            for (int i{0}; i < b_loop_ub; i++) {
              J[i] = 0.0;
            }

            b_loop_ub = ResidualIndices[0].f1.size(1);
            rows.set_size(1, b_loop_ub);
            c_loop_ub = ResidualIndices[0].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              rows[i] = ResidualIndices[0].f1[i];
            }

            c_loop_ub = SlackIndices[0].f1.size(1);
            cols.set_size(1, c_loop_ub);
            d_loop_ub = SlackIndices[0].f1.size(1);
            for (int i{0}; i < d_loop_ub; i++) {
              cols[i] = SlackIndices[0].f1[i];
            }

            d = NumPositions;
            q.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < b_loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = tmp_data[b_i + b_loop_ub * i];
              }
            }

            slacks.set_size(c_loop_ub);
            for (int i{0}; i < c_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[0].f1.size(0), EqualityFlags[0].f1.size(1));
            c_loop_ub = EqualityFlags[0].f1.size(0) * EqualityFlags[0].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[0].f1[i]);
            }

            for (int i{0}; i < b_loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f1->BoundsInternal.size(0), 2);
            c_loop_ub = Constraints.f1->BoundsInternal.size(0) << 1;
            for (int i{0}; i < c_loop_ub; i++) {
              r3[i] = Constraints.f1->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[0].f1.size(0) * EqualityFlags[0].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[0].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r4.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[0].f1[b_i]) {
                r4[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[0].f1.size(0) * EqualityFlags[0].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[0].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r5.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[0].f1[b_i]) {
                r5[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = r5.size(0);
            for (int i{0}; i < c_loop_ub; i++) {
              slacks[r4[i]] = r3[r5[i]];
            }

            r.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (slacks.size(0) == 2) {
              for (int i{0}; i < b_loop_ub; i++) {
                f[r[i]] = gCell_f1[i] - slacks[i];
              }
            } else {
              binary_expand_op_2(f, r, gCell_f1, slacks);
            }

            b_loop_ub = ResidualIndices[1].f1.size(1);
            rows.set_size(1, b_loop_ub);
            c_loop_ub = ResidualIndices[1].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              rows[i] = ResidualIndices[1].f1[i];
            }

            c_loop_ub = SlackIndices[1].f1.size(1);
            cols.set_size(1, c_loop_ub);
            d_loop_ub = SlackIndices[1].f1.size(1);
            for (int i{0}; i < d_loop_ub; i++) {
              cols[i] = SlackIndices[1].f1[i];
            }

            d = NumPositions;
            q.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < b_loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = JCell[1].f1[b_i + b_loop_ub * i];
              }
            }

            slacks.set_size(c_loop_ub);
            for (int i{0}; i < c_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[1].f1.size(0), EqualityFlags[1].f1.size(1));
            c_loop_ub = EqualityFlags[1].f1.size(0) * EqualityFlags[1].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[1].f1[i]);
            }

            for (int i{0}; i < b_loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f2->BoundsInternal.size(0), 2);
            c_loop_ub = Constraints.f2->BoundsInternal.size(0) << 1;
            for (int i{0}; i < c_loop_ub; i++) {
              r3[i] = Constraints.f2->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[1].f1.size(0) * EqualityFlags[1].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[1].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r6.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[1].f1[b_i]) {
                r6[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[1].f1.size(0) * EqualityFlags[1].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[1].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r7.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[1].f1[b_i]) {
                r7[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = r7.size(0);
            for (int i{0}; i < c_loop_ub; i++) {
              slacks[r6[i]] = r3[r7[i]];
            }

            r.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (loop_ub == slacks.size(0)) {
              for (int i{0}; i < b_loop_ub; i++) {
                f[r[i]] = x[i] - slacks[i];
              }
            } else {
              binary_expand_op_1(f, r, x, loop_ub - 1, slacks);
            }

            loop_ub = ResidualIndices[2].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[2].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[2].f1[i];
            }

            b_loop_ub = SlackIndices[2].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[2].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[2].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = b_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[2].f1.size(0), EqualityFlags[2].f1.size(1));
            b_loop_ub = EqualityFlags[2].f1.size(0) * EqualityFlags[2].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[2].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f3->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f3->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f3->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[2].f1.size(0) * EqualityFlags[2].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[2].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r8.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[2].f1[b_i]) {
                r8[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[2].f1.size(0) * EqualityFlags[2].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[2].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r9.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[2].f1[b_i]) {
                r9[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r9.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r8[i]] = r3[r9[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f3 - slacks[i];
            }

            loop_ub = ResidualIndices[3].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[3].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[3].f1[i];
            }

            b_loop_ub = SlackIndices[3].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[3].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[3].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = c_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[3].f1.size(0), EqualityFlags[3].f1.size(1));
            b_loop_ub = EqualityFlags[3].f1.size(0) * EqualityFlags[3].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[3].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f4->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f4->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f4->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[3].f1.size(0) * EqualityFlags[3].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[3].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r10.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[3].f1[b_i]) {
                r10[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[3].f1.size(0) * EqualityFlags[3].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[3].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r11.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[3].f1[b_i]) {
                r11[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r11.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r10[i]] = r3[r11[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f4 - slacks[i];
            }

            loop_ub = ResidualIndices[4].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[4].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[4].f1[i];
            }

            b_loop_ub = SlackIndices[4].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[4].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[4].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = d_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[4].f1.size(0), EqualityFlags[4].f1.size(1));
            b_loop_ub = EqualityFlags[4].f1.size(0) * EqualityFlags[4].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[4].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f5->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f5->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f5->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[4].f1.size(0) * EqualityFlags[4].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[4].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r12.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[4].f1[b_i]) {
                r12[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[4].f1.size(0) * EqualityFlags[4].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[4].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r13.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[4].f1[b_i]) {
                r13[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r13.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r12[i]] = r3[r13[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f5 - slacks[i];
            }

            loop_ub = ResidualIndices[5].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[5].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[5].f1[i];
            }

            b_loop_ub = SlackIndices[5].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[5].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[5].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = e_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[5].f1.size(0), EqualityFlags[5].f1.size(1));
            b_loop_ub = EqualityFlags[5].f1.size(0) * EqualityFlags[5].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[5].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f6->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f6->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f6->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[5].f1.size(0) * EqualityFlags[5].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[5].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r14.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[5].f1[b_i]) {
                r14[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[5].f1.size(0) * EqualityFlags[5].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[5].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r15.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[5].f1[b_i]) {
                r15[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r15.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r14[i]] = r3[r15[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f6 - slacks[i];
            }

            loop_ub = ResidualIndices[6].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[6].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[6].f1[i];
            }

            b_loop_ub = SlackIndices[6].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[6].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[6].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = f_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[6].f1.size(0), EqualityFlags[6].f1.size(1));
            b_loop_ub = EqualityFlags[6].f1.size(0) * EqualityFlags[6].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[6].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f7->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f7->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f7->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[6].f1.size(0) * EqualityFlags[6].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[6].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r16.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[6].f1[b_i]) {
                r16[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[6].f1.size(0) * EqualityFlags[6].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[6].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r17.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[6].f1[b_i]) {
                r17[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r17.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r16[i]] = r3[r17[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f7 - slacks[i];
            }

            loop_ub = ResidualIndices[7].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[7].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[7].f1[i];
            }

            b_loop_ub = SlackIndices[7].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[7].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[7].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = g_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[7].f1.size(0), EqualityFlags[7].f1.size(1));
            b_loop_ub = EqualityFlags[7].f1.size(0) * EqualityFlags[7].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[7].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f8->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f8->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f8->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[7].f1.size(0) * EqualityFlags[7].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[7].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r18.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[7].f1[b_i]) {
                r18[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[7].f1.size(0) * EqualityFlags[7].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[7].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r19.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[7].f1[b_i]) {
                r19[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r19.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r18[i]] = r3[r19[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f8 - slacks[i];
            }

            loop_ub = ResidualIndices[8].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[8].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[8].f1[i];
            }

            b_loop_ub = SlackIndices[8].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[8].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[8].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = h_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[8].f1.size(0), EqualityFlags[8].f1.size(1));
            b_loop_ub = EqualityFlags[8].f1.size(0) * EqualityFlags[8].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[8].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f9->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f9->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f9->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[8].f1.size(0) * EqualityFlags[8].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[8].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r20.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[8].f1[b_i]) {
                r20[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[8].f1.size(0) * EqualityFlags[8].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[8].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r21.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[8].f1[b_i]) {
                r21[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r21.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r20[i]] = r3[r21[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f9 - slacks[i];
            }

            loop_ub = ResidualIndices[9].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[9].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[9].f1[i];
            }

            b_loop_ub = SlackIndices[9].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[9].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[9].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = i_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[9].f1.size(0), EqualityFlags[9].f1.size(1));
            b_loop_ub = EqualityFlags[9].f1.size(0) * EqualityFlags[9].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[9].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f10->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f10->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f10->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[9].f1.size(0) * EqualityFlags[9].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[9].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r22.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[9].f1[b_i]) {
                r22[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[9].f1.size(0) * EqualityFlags[9].f1.size(1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[9].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r23.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[9].f1[b_i]) {
                r23[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r23.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r22[i]] = r3[r23[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f10 - slacks[i];
            }

            loop_ub = ResidualIndices[10].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[10].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[10].f1[i];
            }

            b_loop_ub = SlackIndices[10].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[10].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[10].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = j_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[10].f1.size(0), EqualityFlags[10].f1.size
                        (1));
            b_loop_ub = EqualityFlags[10].f1.size(0) * EqualityFlags[10].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[10].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f11->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f11->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f11->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[10].f1.size(0) * EqualityFlags[10].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[10].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r24.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[10].f1[b_i]) {
                r24[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[10].f1.size(0) * EqualityFlags[10].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[10].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r25.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[10].f1[b_i]) {
                r25[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r25.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r24[i]] = r3[r25[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f11 - slacks[i];
            }

            loop_ub = ResidualIndices[11].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[11].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[11].f1[i];
            }

            b_loop_ub = SlackIndices[11].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[11].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[11].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = k_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[11].f1.size(0), EqualityFlags[11].f1.size
                        (1));
            b_loop_ub = EqualityFlags[11].f1.size(0) * EqualityFlags[11].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[11].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f12->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f12->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f12->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[11].f1.size(0) * EqualityFlags[11].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[11].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r26.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[11].f1[b_i]) {
                r26[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[11].f1.size(0) * EqualityFlags[11].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[11].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r27.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[11].f1[b_i]) {
                r27[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r27.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r26[i]] = r3[r27[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f12 - slacks[i];
            }

            loop_ub = ResidualIndices[12].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[12].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[12].f1[i];
            }

            b_loop_ub = SlackIndices[12].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[12].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[12].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = l_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[12].f1.size(0), EqualityFlags[12].f1.size
                        (1));
            b_loop_ub = EqualityFlags[12].f1.size(0) * EqualityFlags[12].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[12].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f13->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f13->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f13->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[12].f1.size(0) * EqualityFlags[12].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[12].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r28.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[12].f1[b_i]) {
                r28[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[12].f1.size(0) * EqualityFlags[12].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[12].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r29.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[12].f1[b_i]) {
                r29[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r29.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r28[i]] = r3[r29[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f13 - slacks[i];
            }

            loop_ub = ResidualIndices[13].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[13].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[13].f1[i];
            }

            b_loop_ub = SlackIndices[13].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[13].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[13].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = m_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[13].f1.size(0), EqualityFlags[13].f1.size
                        (1));
            b_loop_ub = EqualityFlags[13].f1.size(0) * EqualityFlags[13].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[13].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f14->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f14->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f14->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[13].f1.size(0) * EqualityFlags[13].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[13].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r30.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[13].f1[b_i]) {
                r30[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[13].f1.size(0) * EqualityFlags[13].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[13].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r31.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[13].f1[b_i]) {
                r31[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r31.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r30[i]] = r3[r31[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f14 - slacks[i];
            }

            loop_ub = ResidualIndices[14].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[14].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[14].f1[i];
            }

            b_loop_ub = SlackIndices[14].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[14].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[14].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = n_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[14].f1.size(0), EqualityFlags[14].f1.size
                        (1));
            b_loop_ub = EqualityFlags[14].f1.size(0) * EqualityFlags[14].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[14].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f15->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f15->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f15->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[14].f1.size(0) * EqualityFlags[14].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[14].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r32.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[14].f1[b_i]) {
                r32[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[14].f1.size(0) * EqualityFlags[14].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[14].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r33.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[14].f1[b_i]) {
                r33[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r33.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r32[i]] = r3[r33[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f15 - slacks[i];
            }

            loop_ub = ResidualIndices[15].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[15].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[15].f1[i];
            }

            b_loop_ub = SlackIndices[15].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[15].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[15].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = o_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[15].f1.size(0), EqualityFlags[15].f1.size
                        (1));
            b_loop_ub = EqualityFlags[15].f1.size(0) * EqualityFlags[15].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[15].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f16->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f16->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f16->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[15].f1.size(0) * EqualityFlags[15].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[15].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r34.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[15].f1[b_i]) {
                r34[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[15].f1.size(0) * EqualityFlags[15].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[15].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r35.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[15].f1[b_i]) {
                r35[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r35.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r34[i]] = r3[r35[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f16 - slacks[i];
            }

            loop_ub = ResidualIndices[16].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[16].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[16].f1[i];
            }

            b_loop_ub = SlackIndices[16].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[16].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[16].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = p_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[16].f1.size(0), EqualityFlags[16].f1.size
                        (1));
            b_loop_ub = EqualityFlags[16].f1.size(0) * EqualityFlags[16].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[16].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f17->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f17->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f17->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[16].f1.size(0) * EqualityFlags[16].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[16].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r36.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[16].f1[b_i]) {
                r36[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[16].f1.size(0) * EqualityFlags[16].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[16].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r37.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[16].f1[b_i]) {
                r37[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r37.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r36[i]] = r3[r37[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f17 - slacks[i];
            }

            loop_ub = ResidualIndices[17].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[17].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[17].f1[i];
            }

            b_loop_ub = SlackIndices[17].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[17].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[17].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = q_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[17].f1.size(0), EqualityFlags[17].f1.size
                        (1));
            b_loop_ub = EqualityFlags[17].f1.size(0) * EqualityFlags[17].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[17].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f18->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f18->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f18->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[17].f1.size(0) * EqualityFlags[17].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[17].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r38.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[17].f1[b_i]) {
                r38[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[17].f1.size(0) * EqualityFlags[17].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[17].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r39.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[17].f1[b_i]) {
                r39[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r39.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r38[i]] = r3[r39[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f18 - slacks[i];
            }

            loop_ub = ResidualIndices[18].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[18].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[18].f1[i];
            }

            b_loop_ub = SlackIndices[18].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[18].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[18].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = r_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[18].f1.size(0), EqualityFlags[18].f1.size
                        (1));
            b_loop_ub = EqualityFlags[18].f1.size(0) * EqualityFlags[18].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[18].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f19->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f19->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f19->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[18].f1.size(0) * EqualityFlags[18].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[18].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r40.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[18].f1[b_i]) {
                r40[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[18].f1.size(0) * EqualityFlags[18].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[18].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r41.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[18].f1[b_i]) {
                r41[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r41.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r40[i]] = r3[r41[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f19 - slacks[i];
            }

            loop_ub = ResidualIndices[19].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[19].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[19].f1[i];
            }

            b_loop_ub = SlackIndices[19].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[19].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[19].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = s_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[19].f1.size(0), EqualityFlags[19].f1.size
                        (1));
            b_loop_ub = EqualityFlags[19].f1.size(0) * EqualityFlags[19].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[19].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f20->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f20->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f20->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[19].f1.size(0) * EqualityFlags[19].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[19].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r42.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[19].f1[b_i]) {
                r42[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[19].f1.size(0) * EqualityFlags[19].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[19].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r43.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[19].f1[b_i]) {
                r43[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r43.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r42[i]] = r3[r43[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f20 - slacks[i];
            }

            loop_ub = ResidualIndices[20].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[20].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[20].f1[i];
            }

            b_loop_ub = SlackIndices[20].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[20].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[20].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = t_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[20].f1.size(0), EqualityFlags[20].f1.size
                        (1));
            b_loop_ub = EqualityFlags[20].f1.size(0) * EqualityFlags[20].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[20].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f21->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f21->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f21->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[20].f1.size(0) * EqualityFlags[20].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[20].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r44.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[20].f1[b_i]) {
                r44[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[20].f1.size(0) * EqualityFlags[20].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[20].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r45.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[20].f1[b_i]) {
                r45[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r45.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r44[i]] = r3[r45[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f21 - slacks[i];
            }

            loop_ub = ResidualIndices[21].f1.size(1);
            rows.set_size(1, loop_ub);
            b_loop_ub = ResidualIndices[21].f1.size(1);
            for (int i{0}; i < b_loop_ub; i++) {
              rows[i] = ResidualIndices[21].f1[i];
            }

            b_loop_ub = SlackIndices[21].f1.size(1);
            cols.set_size(1, b_loop_ub);
            c_loop_ub = SlackIndices[21].f1.size(1);
            for (int i{0}; i < c_loop_ub; i++) {
              cols[i] = SlackIndices[21].f1[i];
            }

            d = NumPositions;
            q.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              q[i] = rows[i];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            if (d < 1.0) {
              d_loop_ub = 0;
            } else {
              d_loop_ub = static_cast<int>(d);
            }

            for (int i{0}; i < d_loop_ub; i++) {
              for (int b_i{0}; b_i < loop_ub; b_i++) {
                J[r[b_i] + J.size(0) * i] = u_tmp_data[b_i + loop_ub * i];
              }
            }

            slacks.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[i] = x[static_cast<int>(cols[i]) - 1];
            }

            r1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r1[i] = static_cast<int>(rows[i]) + ndx_tmp_idx_0 * (static_cast<
                int>(cols[i]) - 1);
            }

            r2.set_size(EqualityFlags[21].f1.size(0), EqualityFlags[21].f1.size
                        (1));
            b_loop_ub = EqualityFlags[21].f1.size(0) * EqualityFlags[21].f1.size
              (1);
            for (int i{0}; i < b_loop_ub; i++) {
              r2[i] = static_cast<signed char>(-!EqualityFlags[21].f1[i]);
            }

            for (int i{0}; i < loop_ub; i++) {
              J[r1[i] - 1] = r2[i];
            }

            r3.set_size(Constraints.f22->BoundsInternal.size(0), 2);
            b_loop_ub = Constraints.f22->BoundsInternal.size(0) << 1;
            for (int i{0}; i < b_loop_ub; i++) {
              r3[i] = Constraints.f22->BoundsInternal[i];
            }

            c_loop_ub = EqualityFlags[21].f1.size(0) * EqualityFlags[21].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[21].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r46.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[21].f1[b_i]) {
                r46[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            c_loop_ub = EqualityFlags[21].f1.size(0) * EqualityFlags[21].f1.size
              (1);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[21].f1[b_i]) {
                d_loop_ub++;
              }
            }

            r47.set_size(d_loop_ub);
            d_loop_ub = 0;
            for (int b_i{0}; b_i < c_loop_ub; b_i++) {
              if (EqualityFlags[21].f1[b_i]) {
                r47[d_loop_ub] = b_i;
                d_loop_ub++;
              }
            }

            b_loop_ub = r47.size(0);
            for (int i{0}; i < b_loop_ub; i++) {
              slacks[r46[i]] = r3[r47[i]];
            }

            r.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              r[i] = static_cast<int>(q[i]) - 1;
            }

            for (int i{0}; i < loop_ub; i++) {
              f[r[i]] = gCell_f22 - slacks[i];
            }
          }

          //
          // Arguments    : const ::coder::array<double, 2U> &b_value
          // Return Type  : void
          //
          void GIKProblem::set_DesignVariableBounds(const ::coder::array<double,
            2U> &b_value)
          {
            ::coder::array<double, 2U> A;
            int b_loop_ub;
            int loop_ub;
            int loop_ub_tmp;
            loop_ub = b_value.size(0);
            DesignVariableBoundsInternal.set_size(b_value.size(0), 2);
            loop_ub_tmp = b_value.size(0) << 1;
            for (int i{0}; i < loop_ub_tmp; i++) {
              DesignVariableBoundsInternal[i] = b_value[i];
            }

            b_loop_ub = 2 * b_value.size(0);
            A.set_size(b_loop_ub, b_value.size(0));
            loop_ub_tmp = b_loop_ub * b_value.size(0);
            for (int i{0}; i < loop_ub_tmp; i++) {
              A[i] = 0.0;
            }

            ConstraintBoundInternal.set_size(b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              ConstraintBoundInternal[i] = 0.0;
            }

            for (int i{0}; i < loop_ub; i++) {
              loop_ub_tmp = static_cast<int>(static_cast<unsigned int>(i + 1) <<
                1);
              A[(loop_ub_tmp + A.size(0) * i) - 2] = -1.0;
              A[(loop_ub_tmp + A.size(0) * i) - 1] = 1.0;
              ConstraintBoundInternal[loop_ub_tmp - 2] = -b_value[i];
              ConstraintBoundInternal[loop_ub_tmp - 1] = b_value[i +
                b_value.size(0)];
            }

            ConstraintMatrixInternal.set_size(b_value.size(0), b_loop_ub);
            for (int i{0}; i < b_loop_ub; i++) {
              for (loop_ub_tmp = 0; loop_ub_tmp < loop_ub; loop_ub_tmp++) {
                ConstraintMatrixInternal[loop_ub_tmp +
                  ConstraintMatrixInternal.size(0) * i] = A[i + A.size(0) *
                  loop_ub_tmp];
              }
            }
          }

          //
          // Arguments    : void
          // Return Type  : void
          //
          void GIKProblem::updateDesignVariableBounds()
          {
            ::coder::array<double, 2U> bounds;
            ::coder::array<double, 2U> r1;
            ::coder::array<double, 1U> x;
            ::coder::array<double, 1U> y;
            ::coder::array<int, 1U> r;
            ::coder::array<int, 1U> r10;
            ::coder::array<int, 1U> r11;
            ::coder::array<int, 1U> r12;
            ::coder::array<int, 1U> r13;
            ::coder::array<int, 1U> r14;
            ::coder::array<int, 1U> r15;
            ::coder::array<int, 1U> r16;
            ::coder::array<int, 1U> r17;
            ::coder::array<int, 1U> r18;
            ::coder::array<int, 1U> r19;
            ::coder::array<int, 1U> r2;
            ::coder::array<int, 1U> r20;
            ::coder::array<int, 1U> r21;
            ::coder::array<int, 1U> r22;
            ::coder::array<int, 1U> r3;
            ::coder::array<int, 1U> r4;
            ::coder::array<int, 1U> r5;
            ::coder::array<int, 1U> r6;
            ::coder::array<int, 1U> r7;
            ::coder::array<int, 1U> r8;
            ::coder::array<int, 1U> r9;
            int i;
            int loop_ub;
            int nx_tmp;
            int s;
            bounds.set_size(Constraints.f1->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f1->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f1->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r[i]] = rtMinusInf;
              bounds[r[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[0].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[0].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[0].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[0].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[0].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f2->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f2->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f2->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r2.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r2[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r2.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r2[i]] = rtMinusInf;
              bounds[r2[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[1].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[1].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[1].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[1].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[1].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f3->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f3->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f3->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r3.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r3[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r3.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r3[i]] = rtMinusInf;
              bounds[r3[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[2].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[2].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[2].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[2].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[2].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f4->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f4->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f4->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r4.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r4[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r4.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r4[i]] = rtMinusInf;
              bounds[r4[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[3].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[3].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[3].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[3].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[3].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f5->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f5->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f5->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r5.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r5[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r5.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r5[i]] = rtMinusInf;
              bounds[r5[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[4].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[4].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[4].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[4].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[4].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f6->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f6->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f6->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r6.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r6[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r6.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r6[i]] = rtMinusInf;
              bounds[r6[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[5].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[5].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[5].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[5].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[5].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f7->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f7->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f7->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r7.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r7[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r7.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r7[i]] = rtMinusInf;
              bounds[r7[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[6].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[6].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[6].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[6].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[6].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f8->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f8->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f8->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r8.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r8[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r8.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r8[i]] = rtMinusInf;
              bounds[r8[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[7].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[7].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[7].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[7].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[7].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f9->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f9->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f9->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r9.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r9[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r9.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r9[i]] = rtMinusInf;
              bounds[r9[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[8].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[8].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[8].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[8].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[8].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f10->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f10->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f10->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r10.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r10[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r10.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r10[i]] = rtMinusInf;
              bounds[r10[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[9].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[9].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[9].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[9].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[9].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f11->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f11->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f11->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r11.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r11[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r11.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r11[i]] = rtMinusInf;
              bounds[r11[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[10].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[10].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[10].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[10].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[10].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f12->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f12->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f12->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r12.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r12[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r12.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r12[i]] = rtMinusInf;
              bounds[r12[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[11].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[11].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[11].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[11].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[11].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f13->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f13->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f13->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r13.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r13[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r13.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r13[i]] = rtMinusInf;
              bounds[r13[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[12].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[12].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[12].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[12].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[12].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f14->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f14->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f14->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r14.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r14[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r14.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r14[i]] = rtMinusInf;
              bounds[r14[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[13].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[13].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[13].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[13].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[13].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f15->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f15->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f15->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r15.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r15[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r15.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r15[i]] = rtMinusInf;
              bounds[r15[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[14].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[14].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[14].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[14].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[14].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f16->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f16->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f16->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r16.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r16[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r16.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r16[i]] = rtMinusInf;
              bounds[r16[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[15].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[15].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[15].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[15].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[15].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f17->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f17->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f17->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r17.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r17[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r17.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r17[i]] = rtMinusInf;
              bounds[r17[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[16].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[16].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[16].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[16].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[16].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f18->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f18->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f18->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r18.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r18[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r18.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r18[i]] = rtMinusInf;
              bounds[r18[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[17].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[17].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[17].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[17].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[17].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f19->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f19->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f19->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r19.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r19[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r19.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r19[i]] = rtMinusInf;
              bounds[r19[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[18].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[18].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[18].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[18].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[18].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f20->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f20->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f20->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r20.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r20[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r20.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r20[i]] = rtMinusInf;
              bounds[r20[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[19].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[19].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[19].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[19].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[19].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f21->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f21->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f21->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r21.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r21[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r21.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r21[i]] = rtMinusInf;
              bounds[r21[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[20].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[20].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[20].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[20].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[20].f1[i] = (y[i] < 2.2204460492503131E-16);
            }

            bounds.set_size(Constraints.f22->BoundsInternal.size(0), 2);
            loop_ub = Constraints.f22->BoundsInternal.size(0) << 1;
            for (i = 0; i < loop_ub; i++) {
              bounds[i] = Constraints.f22->BoundsInternal[i];
            }

            i = bounds.size(0);
            x.set_size(bounds.size(0));
            if (bounds.size(0) != 0) {
              for (s = 0; s < i; s++) {
                x[s] = bounds[s + i] - bounds[s];
              }
            }

            nx_tmp = x.size(0);
            y.set_size(x.size(0));
            for (s = 0; s < nx_tmp; s++) {
              y[s] = std::abs(x[s]);
            }

            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                s++;
              }
            }

            r22.set_size(s);
            s = 0;
            for (loop_ub = 0; loop_ub < nx_tmp; loop_ub++) {
              if (y[loop_ub] < 2.2204460492503131E-16) {
                r22[s] = loop_ub;
                s++;
              }
            }

            loop_ub = r22.size(0);
            for (i = 0; i < loop_ub; i++) {
              bounds[r22[i]] = rtMinusInf;
              bounds[r22[i] + bounds.size(0)] = rtInf;
            }

            r1.set_size(DesignVariableBoundsInternal.size(0), 2);
            s = DesignVariableBoundsInternal.size(0) << 1;
            for (i = 0; i < s; i++) {
              r1[i] = DesignVariableBoundsInternal[i];
            }

            loop_ub = SlackIndices[21].f1.size(1);
            r.set_size(loop_ub);
            s = SlackIndices[21].f1.size(1);
            for (i = 0; i < s; i++) {
              r[i] = static_cast<int>(SlackIndices[21].f1[i]) - 1;
            }

            for (i = 0; i < 2; i++) {
              for (s = 0; s < loop_ub; s++) {
                r1[r[s] + r1.size(0) * i] = bounds[s + bounds.size(0) * i];
              }
            }

            set_DesignVariableBounds(r1);
            EqualityFlags[21].f1.set_size(x.size(0), 1);
            for (i = 0; i < nx_tmp; i++) {
              EqualityFlags[21].f1[i] = (y[i] < 2.2204460492503131E-16);
            }
          }

          //
          // Arguments    : ::coder::array<double, 1U> &in1
          //                const ::coder::array<int, 1U> &in2
          //                const ::coder::array<double, 1U> &in3
          //                int in4
          //                const ::coder::array<double, 1U> &in5
          // Return Type  : void
          //
        }
      }
    }
  }

  static void binary_expand_op_1(::coder::array<double, 1U> &in1, const ::coder::
    array<int, 1U> &in2, const ::coder::array<double, 1U> &in3, int in4, const ::
    coder::array<double, 1U> &in5)
  {
    int loop_ub;
    int stride_0_0;
    int stride_1_0;
    stride_0_0 = (in4 + 1 != 1);
    stride_1_0 = (in5.size(0) != 1);
    loop_ub = in2.size(0);
    for (int i{0}; i < loop_ub; i++) {
      in1[in2[i]] = in3[i * stride_0_0] - in5[i * stride_1_0];
    }
  }

  //
  // Arguments    : ::coder::array<double, 1U> &in1
  //                const ::coder::array<int, 1U> &in2
  //                const double in3[2]
  //                const ::coder::array<double, 1U> &in4
  // Return Type  : void
  //
  static void binary_expand_op_2(::coder::array<double, 1U> &in1, const ::coder::
    array<int, 1U> &in2, const double in3[2], const ::coder::array<double, 1U>
    &in4)
  {
    int loop_ub;
    int stride_0_0;
    stride_0_0 = (in4.size(0) != 1);
    loop_ub = in2.size(0);
    for (int i{0}; i < loop_ub; i++) {
      in1[in2[i]] = in3[i] - in4[i * stride_0_0];
    }
  }

  //
  // Arguments    : void
  // Return Type  : GIKProblem
  //
  namespace coder
  {
    namespace robotics
    {
      namespace manip
      {
        namespace internal
        {
          GIKProblem::GIKProblem()
            = default;

          //
          // Arguments    : void
          // Return Type  : void
          //
          GIKProblem::~GIKProblem()
            = default;

          //
          // Arguments    : const ::coder::array<double, 1U> &x
          //                struct1_T violations[22]
          // Return Type  : void
          //
          void GIKProblem::constraintViolations(const ::coder::array<double, 1U>
            &x, struct1_T violations[22])
          {
            static const char b_cv1[8]{ 'd', 'i', 's', 't', 'a', 'n', 'c', 'e' };

            static const char b_cv[5]{ 'j', 'o', 'i', 'n', 't' };

            DistanceBoundsConstraint *c_obj;
            JointPositionBounds *b_obj;
            PoseTarget *obj;
            ::coder::array<double, 2U> Jrobot;
            ::coder::array<double, 1U> ab_varargin_2;
            ::coder::array<double, 1U> b_varargin_2;
            ::coder::array<double, 1U> bb_varargin_2;
            ::coder::array<double, 1U> c_varargin_2;
            ::coder::array<double, 1U> cb_varargin_2;
            ::coder::array<double, 1U> d_varargin_2;
            ::coder::array<double, 1U> db_varargin_2;
            ::coder::array<double, 1U> e_varargin_2;
            ::coder::array<double, 1U> eb_varargin_2;
            ::coder::array<double, 1U> f_varargin_2;
            ::coder::array<double, 1U> fb_varargin_2;
            ::coder::array<double, 1U> g_varargin_2;
            ::coder::array<double, 1U> gb_varargin_2;
            ::coder::array<double, 1U> h_varargin_2;
            ::coder::array<double, 1U> hb_varargin_2;
            ::coder::array<double, 1U> i_varargin_2;
            ::coder::array<double, 1U> ib_varargin_2;
            ::coder::array<double, 1U> j_varargin_2;
            ::coder::array<double, 1U> jb_varargin_2;
            ::coder::array<double, 1U> k_varargin_2;
            ::coder::array<double, 1U> kb_varargin_2;
            ::coder::array<double, 1U> l_varargin_2;
            ::coder::array<double, 1U> lb_varargin_2;
            ::coder::array<double, 1U> m_varargin_2;
            ::coder::array<double, 1U> mb_varargin_2;
            ::coder::array<double, 1U> n_varargin_2;
            ::coder::array<double, 1U> nb_varargin_2;
            ::coder::array<double, 1U> o_varargin_2;
            ::coder::array<double, 1U> ob_varargin_2;
            ::coder::array<double, 1U> p_varargin_2;
            ::coder::array<double, 1U> q;
            ::coder::array<double, 1U> q_varargin_2;
            ::coder::array<double, 1U> r;
            ::coder::array<double, 1U> r1;
            ::coder::array<double, 1U> r_varargin_2;
            ::coder::array<double, 1U> s_varargin_2;
            ::coder::array<double, 1U> t_varargin_2;
            ::coder::array<double, 1U> u_varargin_2;
            ::coder::array<double, 1U> v_varargin_2;
            ::coder::array<double, 1U> w_varargin_2;
            ::coder::array<double, 1U> x_varargin_2;
            ::coder::array<double, 1U> y_varargin_2;
            double T_data[16];
            double cv_max[2];
            double g[2];
            double b_r;
            double cv_min_idx_0;
            double cv_min_idx_1;
            double varargin_2;
            int T_size[2];
            int b_loop_ub;
            int c_loop_ub;
            int d_loop_ub;
            int e_loop_ub;
            int f_loop_ub;
            int g_loop_ub;
            int h_loop_ub;
            int i;
            int i1;
            int i_loop_ub;
            int j_loop_ub;
            int k_loop_ub;
            int l_loop_ub;
            int loop_ub;
            int m_loop_ub;
            int n_loop_ub;
            int o_loop_ub;
            int p_loop_ub;
            int q_loop_ub;
            int r_loop_ub;
            int s_loop_ub;
            int t_loop_ub;
            int u_loop_ub;
            varargin_2 = NumPositions;
            if (varargin_2 < 1.0) {
              loop_ub = 0;
            } else {
              loop_ub = static_cast<int>(varargin_2);
            }

            q.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              q[i] = x[i];
            }

            double JTwist[12];
            obj = Constraints.f1;
            obj->Tree->efficientFKAndJacobianForIK(q, obj->EndEffectorIndex,
              obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            obj->evaluateFromTransform(T_data, T_size, g, JTwist);
            r.set_size(obj->BoundsInternal.size(0));
            b_loop_ub = obj->BoundsInternal.size(0);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = obj->BoundsInternal[i];
            }

            if (r.size(0) == 2) {
              cv_max[0] = g[0] - r[0];
              cv_max[1] = g[1] - r[1];
            } else {
              minus(cv_max, g, r);
            }

            cv_min_idx_0 = std::fmin(0.0, cv_max[0]);
            cv_min_idx_1 = std::fmin(0.0, cv_max[1]);
            r.set_size(obj->BoundsInternal.size(0));
            b_loop_ub = obj->BoundsInternal.size(0);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = obj->BoundsInternal[i + obj->BoundsInternal.size(0)];
            }

            if (r.size(0) == 2) {
              g[0] -= r[0];
              g[1] -= r[1];
            } else {
              minus(g, r);
            }

            b_obj = Constraints.f2;
            r.set_size(b_obj->BoundsInternal.size(0));
            b_loop_ub = b_obj->BoundsInternal.size(0);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = b_obj->BoundsInternal[i];
            }

            r1.set_size(b_obj->BoundsInternal.size(0));
            b_loop_ub = b_obj->BoundsInternal.size(0);
            for (i = 0; i < b_loop_ub; i++) {
              r1[i] = b_obj->BoundsInternal[i + b_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f3;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            b_loop_ub = c_obj->BoundsInternal.size(0);
            b_varargin_2.set_size(b_loop_ub);
            c_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < c_loop_ub; i++) {
              b_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            c_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            c_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < c_loop_ub; i++) {
              c_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f4;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            c_loop_ub = c_obj->BoundsInternal.size(0);
            d_varargin_2.set_size(c_loop_ub);
            d_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < d_loop_ub; i++) {
              d_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            e_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            d_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < d_loop_ub; i++) {
              e_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f5;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            d_loop_ub = c_obj->BoundsInternal.size(0);
            f_varargin_2.set_size(d_loop_ub);
            e_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < e_loop_ub; i++) {
              f_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            g_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            e_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < e_loop_ub; i++) {
              g_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f6;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            e_loop_ub = c_obj->BoundsInternal.size(0);
            h_varargin_2.set_size(e_loop_ub);
            f_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < f_loop_ub; i++) {
              h_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            i_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            f_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < f_loop_ub; i++) {
              i_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f7;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            f_loop_ub = c_obj->BoundsInternal.size(0);
            j_varargin_2.set_size(f_loop_ub);
            g_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < g_loop_ub; i++) {
              j_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            k_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            g_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < g_loop_ub; i++) {
              k_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f8;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            g_loop_ub = c_obj->BoundsInternal.size(0);
            l_varargin_2.set_size(g_loop_ub);
            h_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < h_loop_ub; i++) {
              l_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            m_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            h_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < h_loop_ub; i++) {
              m_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f9;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            h_loop_ub = c_obj->BoundsInternal.size(0);
            n_varargin_2.set_size(h_loop_ub);
            i_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < i_loop_ub; i++) {
              n_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            o_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            i_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < i_loop_ub; i++) {
              o_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f10;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            i_loop_ub = c_obj->BoundsInternal.size(0);
            p_varargin_2.set_size(i_loop_ub);
            j_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < j_loop_ub; i++) {
              p_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            q_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            j_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < j_loop_ub; i++) {
              q_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f11;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            j_loop_ub = c_obj->BoundsInternal.size(0);
            r_varargin_2.set_size(j_loop_ub);
            k_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < k_loop_ub; i++) {
              r_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            s_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            k_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < k_loop_ub; i++) {
              s_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f12;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            k_loop_ub = c_obj->BoundsInternal.size(0);
            t_varargin_2.set_size(k_loop_ub);
            l_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < l_loop_ub; i++) {
              t_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            u_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            l_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < l_loop_ub; i++) {
              u_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f13;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            l_loop_ub = c_obj->BoundsInternal.size(0);
            v_varargin_2.set_size(l_loop_ub);
            m_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < m_loop_ub; i++) {
              v_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            w_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            m_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < m_loop_ub; i++) {
              w_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f14;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            m_loop_ub = c_obj->BoundsInternal.size(0);
            x_varargin_2.set_size(m_loop_ub);
            n_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < n_loop_ub; i++) {
              x_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            y_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            n_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < n_loop_ub; i++) {
              y_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f15;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            n_loop_ub = c_obj->BoundsInternal.size(0);
            ab_varargin_2.set_size(n_loop_ub);
            o_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < o_loop_ub; i++) {
              ab_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            bb_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            o_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < o_loop_ub; i++) {
              bb_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f16;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            o_loop_ub = c_obj->BoundsInternal.size(0);
            cb_varargin_2.set_size(o_loop_ub);
            p_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < p_loop_ub; i++) {
              cb_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            db_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            p_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < p_loop_ub; i++) {
              db_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f17;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            p_loop_ub = c_obj->BoundsInternal.size(0);
            eb_varargin_2.set_size(p_loop_ub);
            q_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < q_loop_ub; i++) {
              eb_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            fb_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            q_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < q_loop_ub; i++) {
              fb_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f18;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            q_loop_ub = c_obj->BoundsInternal.size(0);
            gb_varargin_2.set_size(q_loop_ub);
            r_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < r_loop_ub; i++) {
              gb_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            hb_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            r_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < r_loop_ub; i++) {
              hb_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f19;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            r_loop_ub = c_obj->BoundsInternal.size(0);
            ib_varargin_2.set_size(r_loop_ub);
            s_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < s_loop_ub; i++) {
              ib_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            jb_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            s_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < s_loop_ub; i++) {
              jb_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f20;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            s_loop_ub = c_obj->BoundsInternal.size(0);
            kb_varargin_2.set_size(s_loop_ub);
            t_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < t_loop_ub; i++) {
              kb_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            lb_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            t_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < t_loop_ub; i++) {
              lb_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f21;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            t_loop_ub = c_obj->BoundsInternal.size(0);
            mb_varargin_2.set_size(t_loop_ub);
            u_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < u_loop_ub; i++) {
              mb_varargin_2[i] = b_r - c_obj->BoundsInternal[i];
            }

            nb_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            u_loop_ub = c_obj->BoundsInternal.size(0);
            for (i = 0; i < u_loop_ub; i++) {
              nb_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            c_obj = Constraints.f22;
            c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
              c_obj->ReferenceBodyIndex, T_data, T_size, Jrobot);
            i = T_size[0] * (T_size[1] - 1);
            varargin_2 = T_data[i];
            b_r = varargin_2 * varargin_2;
            varargin_2 = T_data[i + 1];
            b_r += varargin_2 * varargin_2;
            varargin_2 = T_data[i + 2];
            b_r += varargin_2 * varargin_2;
            b_r = std::sqrt(b_r + 2.2204460492503131E-16);
            u_loop_ub = c_obj->BoundsInternal.size(0);
            q.set_size(u_loop_ub);
            i1 = c_obj->BoundsInternal.size(0);
            for (i = 0; i < i1; i++) {
              q[i] = b_r - c_obj->BoundsInternal[i];
            }

            ob_varargin_2.set_size(c_obj->BoundsInternal.size(0));
            i1 = c_obj->BoundsInternal.size(0);
            for (i = 0; i < i1; i++) {
              ob_varargin_2[i] = b_r - c_obj->BoundsInternal[i +
                c_obj->BoundsInternal.size(0)];
            }

            violations[0].Type.size[0] = 1;
            violations[0].Type.size[1] = 4;
            violations[0].Type.data[0] = 'p';
            violations[0].Type.data[1] = 'o';
            violations[0].Type.data[2] = 's';
            violations[0].Type.data[3] = 'e';
            violations[1].Type.size[0] = 1;
            violations[1].Type.size[1] = 5;
            for (i = 0; i < 5; i++) {
              violations[1].Type.data[i] = b_cv[i];
            }

            violations[2].Type.size[0] = 1;
            violations[2].Type.size[1] = 8;
            violations[3].Type.size[0] = 1;
            violations[3].Type.size[1] = 8;
            violations[4].Type.size[0] = 1;
            violations[4].Type.size[1] = 8;
            violations[5].Type.size[0] = 1;
            violations[5].Type.size[1] = 8;
            violations[6].Type.size[0] = 1;
            violations[6].Type.size[1] = 8;
            violations[7].Type.size[0] = 1;
            violations[7].Type.size[1] = 8;
            violations[8].Type.size[0] = 1;
            violations[8].Type.size[1] = 8;
            violations[9].Type.size[0] = 1;
            violations[9].Type.size[1] = 8;
            violations[10].Type.size[0] = 1;
            violations[10].Type.size[1] = 8;
            violations[11].Type.size[0] = 1;
            violations[11].Type.size[1] = 8;
            violations[12].Type.size[0] = 1;
            violations[12].Type.size[1] = 8;
            violations[13].Type.size[0] = 1;
            violations[13].Type.size[1] = 8;
            violations[14].Type.size[0] = 1;
            violations[14].Type.size[1] = 8;
            violations[15].Type.size[0] = 1;
            violations[15].Type.size[1] = 8;
            violations[16].Type.size[0] = 1;
            violations[16].Type.size[1] = 8;
            violations[17].Type.size[0] = 1;
            violations[17].Type.size[1] = 8;
            violations[18].Type.size[0] = 1;
            violations[18].Type.size[1] = 8;
            violations[19].Type.size[0] = 1;
            violations[19].Type.size[1] = 8;
            violations[20].Type.size[0] = 1;
            violations[20].Type.size[1] = 8;
            violations[21].Type.size[0] = 1;
            violations[21].Type.size[1] = 8;
            for (i = 0; i < 8; i++) {
              violations[2].Type.data[i] = b_cv1[i];
              violations[3].Type.data[i] = b_cv1[i];
              violations[4].Type.data[i] = b_cv1[i];
              violations[5].Type.data[i] = b_cv1[i];
              violations[6].Type.data[i] = b_cv1[i];
              violations[7].Type.data[i] = b_cv1[i];
              violations[8].Type.data[i] = b_cv1[i];
              violations[9].Type.data[i] = b_cv1[i];
              violations[10].Type.data[i] = b_cv1[i];
              violations[11].Type.data[i] = b_cv1[i];
              violations[12].Type.data[i] = b_cv1[i];
              violations[13].Type.data[i] = b_cv1[i];
              violations[14].Type.data[i] = b_cv1[i];
              violations[15].Type.data[i] = b_cv1[i];
              violations[16].Type.data[i] = b_cv1[i];
              violations[17].Type.data[i] = b_cv1[i];
              violations[18].Type.data[i] = b_cv1[i];
              violations[19].Type.data[i] = b_cv1[i];
              violations[20].Type.data[i] = b_cv1[i];
              violations[21].Type.data[i] = b_cv1[i];
            }

            violations[0].Violation.set_size(1, 2);
            violations[0].Violation[0] = cv_min_idx_0 + std::fmax(0.0, g[0]);
            violations[0].Violation[1] = cv_min_idx_1 + std::fmax(0.0, g[1]);
            if (loop_ub == 1) {
              i = r.size(0);
              i1 = r1.size(0);
            } else {
              i = loop_ub;
              i1 = loop_ub;
            }

            if ((loop_ub == r.size(0)) && (loop_ub == r1.size(0)) && (i == i1))
            {
              violations[1].Violation.set_size(1, loop_ub);
              for (i = 0; i < loop_ub; i++) {
                b_r = x[i] - r[i];
                varargin_2 = x[i] - r1[i];
                violations[1].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_28(violations, x, loop_ub - 1, r, r1);
            }

            if (b_varargin_2.size(0) == c_varargin_2.size(0)) {
              violations[2].Violation.set_size(1, b_loop_ub);
              for (i = 0; i < b_loop_ub; i++) {
                b_r = b_varargin_2[i];
                varargin_2 = c_varargin_2[i];
                violations[2].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_27(violations, b_varargin_2, c_varargin_2);
            }

            if (d_varargin_2.size(0) == e_varargin_2.size(0)) {
              violations[3].Violation.set_size(1, c_loop_ub);
              for (i = 0; i < c_loop_ub; i++) {
                b_r = d_varargin_2[i];
                varargin_2 = e_varargin_2[i];
                violations[3].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_26(violations, d_varargin_2, e_varargin_2);
            }

            if (f_varargin_2.size(0) == g_varargin_2.size(0)) {
              violations[4].Violation.set_size(1, d_loop_ub);
              for (i = 0; i < d_loop_ub; i++) {
                b_r = f_varargin_2[i];
                varargin_2 = g_varargin_2[i];
                violations[4].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_25(violations, f_varargin_2, g_varargin_2);
            }

            if (h_varargin_2.size(0) == i_varargin_2.size(0)) {
              violations[5].Violation.set_size(1, e_loop_ub);
              for (i = 0; i < e_loop_ub; i++) {
                b_r = h_varargin_2[i];
                varargin_2 = i_varargin_2[i];
                violations[5].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_24(violations, h_varargin_2, i_varargin_2);
            }

            if (j_varargin_2.size(0) == k_varargin_2.size(0)) {
              violations[6].Violation.set_size(1, f_loop_ub);
              for (i = 0; i < f_loop_ub; i++) {
                b_r = j_varargin_2[i];
                varargin_2 = k_varargin_2[i];
                violations[6].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_23(violations, j_varargin_2, k_varargin_2);
            }

            if (l_varargin_2.size(0) == m_varargin_2.size(0)) {
              violations[7].Violation.set_size(1, g_loop_ub);
              for (i = 0; i < g_loop_ub; i++) {
                b_r = l_varargin_2[i];
                varargin_2 = m_varargin_2[i];
                violations[7].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_22(violations, l_varargin_2, m_varargin_2);
            }

            if (n_varargin_2.size(0) == o_varargin_2.size(0)) {
              violations[8].Violation.set_size(1, h_loop_ub);
              for (i = 0; i < h_loop_ub; i++) {
                b_r = n_varargin_2[i];
                varargin_2 = o_varargin_2[i];
                violations[8].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_21(violations, n_varargin_2, o_varargin_2);
            }

            if (p_varargin_2.size(0) == q_varargin_2.size(0)) {
              violations[9].Violation.set_size(1, i_loop_ub);
              for (i = 0; i < i_loop_ub; i++) {
                b_r = p_varargin_2[i];
                varargin_2 = q_varargin_2[i];
                violations[9].Violation[i] = std::fmin(0.0, b_r) + std::fmax(0.0,
                  varargin_2);
              }
            } else {
              binary_expand_op_20(violations, p_varargin_2, q_varargin_2);
            }

            if (r_varargin_2.size(0) == s_varargin_2.size(0)) {
              violations[10].Violation.set_size(1, j_loop_ub);
              for (i = 0; i < j_loop_ub; i++) {
                b_r = r_varargin_2[i];
                varargin_2 = s_varargin_2[i];
                violations[10].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_19(violations, r_varargin_2, s_varargin_2);
            }

            if (t_varargin_2.size(0) == u_varargin_2.size(0)) {
              violations[11].Violation.set_size(1, k_loop_ub);
              for (i = 0; i < k_loop_ub; i++) {
                b_r = t_varargin_2[i];
                varargin_2 = u_varargin_2[i];
                violations[11].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_18(violations, t_varargin_2, u_varargin_2);
            }

            if (v_varargin_2.size(0) == w_varargin_2.size(0)) {
              violations[12].Violation.set_size(1, l_loop_ub);
              for (i = 0; i < l_loop_ub; i++) {
                b_r = v_varargin_2[i];
                varargin_2 = w_varargin_2[i];
                violations[12].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_17(violations, v_varargin_2, w_varargin_2);
            }

            if (x_varargin_2.size(0) == y_varargin_2.size(0)) {
              violations[13].Violation.set_size(1, m_loop_ub);
              for (i = 0; i < m_loop_ub; i++) {
                b_r = x_varargin_2[i];
                varargin_2 = y_varargin_2[i];
                violations[13].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_16(violations, x_varargin_2, y_varargin_2);
            }

            if (ab_varargin_2.size(0) == bb_varargin_2.size(0)) {
              violations[14].Violation.set_size(1, n_loop_ub);
              for (i = 0; i < n_loop_ub; i++) {
                b_r = ab_varargin_2[i];
                varargin_2 = bb_varargin_2[i];
                violations[14].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_15(violations, ab_varargin_2, bb_varargin_2);
            }

            if (cb_varargin_2.size(0) == db_varargin_2.size(0)) {
              violations[15].Violation.set_size(1, o_loop_ub);
              for (i = 0; i < o_loop_ub; i++) {
                b_r = cb_varargin_2[i];
                varargin_2 = db_varargin_2[i];
                violations[15].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_14(violations, cb_varargin_2, db_varargin_2);
            }

            if (eb_varargin_2.size(0) == fb_varargin_2.size(0)) {
              violations[16].Violation.set_size(1, p_loop_ub);
              for (i = 0; i < p_loop_ub; i++) {
                b_r = eb_varargin_2[i];
                varargin_2 = fb_varargin_2[i];
                violations[16].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_13(violations, eb_varargin_2, fb_varargin_2);
            }

            if (gb_varargin_2.size(0) == hb_varargin_2.size(0)) {
              violations[17].Violation.set_size(1, q_loop_ub);
              for (i = 0; i < q_loop_ub; i++) {
                b_r = gb_varargin_2[i];
                varargin_2 = hb_varargin_2[i];
                violations[17].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_12(violations, gb_varargin_2, hb_varargin_2);
            }

            if (ib_varargin_2.size(0) == jb_varargin_2.size(0)) {
              violations[18].Violation.set_size(1, r_loop_ub);
              for (i = 0; i < r_loop_ub; i++) {
                b_r = ib_varargin_2[i];
                varargin_2 = jb_varargin_2[i];
                violations[18].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_11(violations, ib_varargin_2, jb_varargin_2);
            }

            if (kb_varargin_2.size(0) == lb_varargin_2.size(0)) {
              violations[19].Violation.set_size(1, s_loop_ub);
              for (i = 0; i < s_loop_ub; i++) {
                b_r = kb_varargin_2[i];
                varargin_2 = lb_varargin_2[i];
                violations[19].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_10(violations, kb_varargin_2, lb_varargin_2);
            }

            if (mb_varargin_2.size(0) == nb_varargin_2.size(0)) {
              violations[20].Violation.set_size(1, t_loop_ub);
              for (i = 0; i < t_loop_ub; i++) {
                b_r = mb_varargin_2[i];
                varargin_2 = nb_varargin_2[i];
                violations[20].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_9(violations, mb_varargin_2, nb_varargin_2);
            }

            if (q.size(0) == ob_varargin_2.size(0)) {
              violations[21].Violation.set_size(1, u_loop_ub);
              for (i = 0; i < u_loop_ub; i++) {
                b_r = q[i];
                varargin_2 = ob_varargin_2[i];
                violations[21].Violation[i] = std::fmin(0.0, b_r) + std::fmax
                  (0.0, varargin_2);
              }
            } else {
              binary_expand_op_8(violations, q, ob_varargin_2);
            }
          }

          //
          // Arguments    : const ::coder::array<double, 1U> &x
          //                ::coder::array<double, 1U> &ev
          // Return Type  : double
          //
          double GIKProblem::evaluateSolution(const ::coder::array<double, 1U>
            &x, ::coder::array<double, 1U> &ev)
          {
            ::coder::array<double, 2U> a;
            ::coder::array<double, 1U> y;
            int inner;
            int mc_tmp;
            residuals(x, ev);
            get_WeightMatrix(a);
            mc_tmp = a.size(0);
            inner = a.size(1);
            y.set_size(a.size(0));
            for (int i{0}; i < mc_tmp; i++) {
              y[i] = 0.0;
            }

            for (int k{0}; k < inner; k++) {
              int aoffset;
              aoffset = k * a.size(0);
              for (int i{0}; i < mc_tmp; i++) {
                y[i] = y[i] + a[aoffset + i] * ev[k];
              }
            }

            return b_norm(y);
          }

          //
          // Arguments    : double value_data[]
          //                int value_size[2]
          // Return Type  : void
          //
          void GIKProblem::get_KinematicPath(double value_data[], int
            value_size[2])
          {
            DistanceBoundsConstraint *b_obj;
            PoseTarget *obj;
            ::coder::array<double, 2U> b_value;
            ::coder::array<double, 2U> c_value;
            ::coder::array<double, 2U> d_value;
            ::coder::array<double, 2U> e_value;
            ::coder::array<double, 2U> f_value;
            ::coder::array<double, 2U> g_value;
            ::coder::array<double, 2U> h_value;
            ::coder::array<double, 2U> i_value;
            ::coder::array<double, 2U> j_value;
            ::coder::array<double, 2U> k_value;
            ::coder::array<double, 2U> l_value;
            ::coder::array<double, 2U> m_value;
            ::coder::array<double, 2U> n_value;
            ::coder::array<double, 2U> o_value;
            ::coder::array<double, 2U> p_value;
            ::coder::array<double, 2U> q_value;
            ::coder::array<double, 2U> r_value;
            ::coder::array<double, 2U> s_value;
            ::coder::array<double, 2U> t_value;
            ::coder::array<double, 2U> u_value;
            ::coder::array<double, 2U> v_value;
            double y_data[264];
            double xtmp;
            int idx_data[264];
            int iwork_data[264];
            int tmp_size[2];
            int y_size[2];
            int b_i;
            int i;
            int i1;
            int i2;
            int i3;
            int i4;
            int j;
            int k;
            int kEnd;
            int loop_ub;
            int n;
            int nb;
            int nd2;
            int p;
            int pEnd;
            int q;
            int qEnd;
            char b_tmp_data[200];
            char tmp_data[200];
            bool exitg1;
            obj = Constraints.f1;
            obj->get_EndEffector(tmp_data, y_size);
            obj->get_ReferenceBody(b_tmp_data, tmp_size);
            obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              b_value);
            Constraints.f2->get_KinematicPath(c_value);
            loop_ub = b_value.size(1) + c_value.size(1);
            d_value.set_size(1, loop_ub);
            nd2 = b_value.size(1);
            for (i = 0; i < nd2; i++) {
              d_value[i] = b_value[i];
            }

            nd2 = c_value.size(1);
            for (i = 0; i < nd2; i++) {
              d_value[i + b_value.size(1)] = c_value[i];
            }

            b_obj = Constraints.f3;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              b_value);
            b_obj = Constraints.f4;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              c_value);
            b_obj = Constraints.f5;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              e_value);
            b_obj = Constraints.f6;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              f_value);
            b_obj = Constraints.f7;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              g_value);
            b_obj = Constraints.f8;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              h_value);
            b_obj = Constraints.f9;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              i_value);
            b_obj = Constraints.f10;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              j_value);
            b_obj = Constraints.f11;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              k_value);
            b_obj = Constraints.f12;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              l_value);
            b_obj = Constraints.f13;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              m_value);
            b_obj = Constraints.f14;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              n_value);
            b_obj = Constraints.f15;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              o_value);
            b_obj = Constraints.f16;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              p_value);
            b_obj = Constraints.f17;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              q_value);
            b_obj = Constraints.f18;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              r_value);
            b_obj = Constraints.f19;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              s_value);
            b_obj = Constraints.f20;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              t_value);
            b_obj = Constraints.f21;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              u_value);
            b_obj = Constraints.f22;
            b_obj->get_EndEffector(tmp_data, y_size);
            b_obj->get_ReferenceBody(b_tmp_data, tmp_size);
            b_obj->Tree->kinematicPath(tmp_data, y_size, b_tmp_data, tmp_size,
              v_value);
            y_size[0] = 1;
            i = d_value.size(1) + b_value.size(1);
            nd2 = i + c_value.size(1);
            nb = nd2 + e_value.size(1);
            n = nb + f_value.size(1);
            b_i = n + g_value.size(1);
            pEnd = b_i + h_value.size(1);
            p = pEnd + i_value.size(1);
            q = p + j_value.size(1);
            qEnd = q + k_value.size(1);
            kEnd = qEnd + l_value.size(1);
            j = kEnd + m_value.size(1);
            k = j + n_value.size(1);
            i1 = k + o_value.size(1);
            i2 = i1 + p_value.size(1);
            i3 = i2 + q_value.size(1);
            i4 = i3 + r_value.size(1);
            y_size[1] = (((i4 + s_value.size(1)) + t_value.size(1)) +
                         u_value.size(1)) + v_value.size(1);
            for (int i5{0}; i5 < loop_ub; i5++) {
              y_data[i5] = d_value[i5];
            }

            loop_ub = b_value.size(1);
            for (int i5{0}; i5 < loop_ub; i5++) {
              y_data[i5 + d_value.size(1)] = b_value[i5];
            }

            loop_ub = c_value.size(1);
            for (int i5{0}; i5 < loop_ub; i5++) {
              y_data[(i5 + d_value.size(1)) + b_value.size(1)] = c_value[i5];
            }

            loop_ub = e_value.size(1);
            for (int i5{0}; i5 < loop_ub; i5++) {
              y_data[((i5 + d_value.size(1)) + b_value.size(1)) + c_value.size(1)]
                = e_value[i5];
            }

            loop_ub = f_value.size(1);
            for (int i5{0}; i5 < loop_ub; i5++) {
              y_data[(((i5 + d_value.size(1)) + b_value.size(1)) + c_value.size
                      (1)) + e_value.size(1)] = f_value[i5];
            }

            loop_ub = g_value.size(1);
            for (int i5{0}; i5 < loop_ub; i5++) {
              y_data[(((i5 + i) + c_value.size(1)) + e_value.size(1)) +
                f_value.size(1)] = g_value[i5];
            }

            loop_ub = h_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + nd2) + e_value.size(1)) + f_value.size(1)) +
                g_value.size(1)] = h_value[i];
            }

            loop_ub = i_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + nb) + f_value.size(1)) + g_value.size(1)) +
                h_value.size(1)] = i_value[i];
            }

            loop_ub = j_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + n) + g_value.size(1)) + h_value.size(1)) +
                i_value.size(1)] = j_value[i];
            }

            loop_ub = k_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + b_i) + h_value.size(1)) + i_value.size(1)) +
                j_value.size(1)] = k_value[i];
            }

            loop_ub = l_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + pEnd) + i_value.size(1)) + j_value.size(1)) +
                k_value.size(1)] = l_value[i];
            }

            loop_ub = m_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + p) + j_value.size(1)) + k_value.size(1)) +
                l_value.size(1)] = m_value[i];
            }

            loop_ub = n_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + q) + k_value.size(1)) + l_value.size(1)) +
                m_value.size(1)] = n_value[i];
            }

            loop_ub = o_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + qEnd) + l_value.size(1)) + m_value.size(1)) +
                n_value.size(1)] = o_value[i];
            }

            loop_ub = p_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + kEnd) + m_value.size(1)) + n_value.size(1)) +
                o_value.size(1)] = p_value[i];
            }

            loop_ub = q_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + j) + n_value.size(1)) + o_value.size(1)) +
                p_value.size(1)] = q_value[i];
            }

            loop_ub = r_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + k) + o_value.size(1)) + p_value.size(1)) +
                q_value.size(1)] = r_value[i];
            }

            loop_ub = s_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + i1) + p_value.size(1)) + q_value.size(1)) +
                r_value.size(1)] = s_value[i];
            }

            loop_ub = t_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + i2) + q_value.size(1)) + r_value.size(1)) +
                s_value.size(1)] = t_value[i];
            }

            loop_ub = u_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + i3) + r_value.size(1)) + s_value.size(1)) +
                t_value.size(1)] = u_value[i];
            }

            loop_ub = v_value.size(1);
            for (i = 0; i < loop_ub; i++) {
              y_data[(((i + i4) + s_value.size(1)) + t_value.size(1)) +
                u_value.size(1)] = v_value[i];
            }

            ::gik9dof::coder::internal::b_sort(y_data, y_size);
            nd2 = y_size[1];
            n = y_size[1] + 1;
            if (nd2 - 1 >= 0) {
              std::memset(&idx_data[0], 0, static_cast<unsigned int>(nd2) *
                          sizeof(int));
            }

            if (y_size[1] != 0) {
              i = y_size[1] - 1;
              for (k = 1; k <= i; k += 2) {
                xtmp = y_data[k];
                if ((y_data[k - 1] <= xtmp) || std::isnan(xtmp)) {
                  idx_data[k - 1] = k;
                  idx_data[k] = k + 1;
                } else {
                  idx_data[k - 1] = k + 1;
                  idx_data[k] = k;
                }
              }

              if ((static_cast<unsigned int>(y_size[1]) & 1U) != 0U) {
                idx_data[y_size[1] - 1] = y_size[1];
              }

              b_i = 2;
              while (b_i < n - 1) {
                nb = b_i << 1;
                j = 1;
                for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
                  p = j;
                  q = pEnd - 1;
                  qEnd = j + nb;
                  if (qEnd > n) {
                    qEnd = n;
                  }

                  k = 0;
                  kEnd = qEnd - j;
                  while (k + 1 <= kEnd) {
                    xtmp = y_data[idx_data[q] - 1];
                    i = idx_data[p - 1];
                    if ((y_data[i - 1] <= xtmp) || std::isnan(xtmp)) {
                      iwork_data[k] = i;
                      p++;
                      if (p == pEnd) {
                        while (q + 1 < qEnd) {
                          k++;
                          iwork_data[k] = idx_data[q];
                          q++;
                        }
                      }
                    } else {
                      iwork_data[k] = idx_data[q];
                      q++;
                      if (q + 1 == qEnd) {
                        while (p < pEnd) {
                          k++;
                          iwork_data[k] = idx_data[p - 1];
                          p++;
                        }
                      }
                    }

                    k++;
                  }

                  for (k = 0; k < kEnd; k++) {
                    idx_data[(j + k) - 1] = iwork_data[k];
                  }

                  j = qEnd;
                }

                b_i = nb;
              }
            }

            c_value.set_size(1, y_size[1]);
            for (k = 0; k < nd2; k++) {
              c_value[k] = y_data[idx_data[k] - 1];
            }

            k = 0;
            while ((k + 1 <= nd2) && std::isinf(c_value[k]) && (c_value[k] < 0.0))
            {
              k++;
            }

            n = k;
            k = y_size[1];
            while ((k >= 1) && std::isnan(c_value[k - 1])) {
              k--;
            }

            b_i = y_size[1] - k;
            exitg1 = false;
            while ((!exitg1) && (k >= 1)) {
              xtmp = c_value[k - 1];
              if (std::isinf(xtmp) && (xtmp > 0.0)) {
                k--;
              } else {
                exitg1 = true;
              }
            }

            nd2 = (y_size[1] - k) - b_i;
            nb = -1;
            if (n > 0) {
              nb = 0;
            }

            while (n + 1 <= k) {
              xtmp = c_value[n];
              do {
                n++;
              } while (!((n + 1 > k) || (c_value[n] != xtmp)));

              nb++;
              c_value[nb] = xtmp;
            }

            if (nd2 > 0) {
              nb++;
              c_value[nb] = c_value[k];
            }

            n = k + nd2;
            for (j = 0; j < b_i; j++) {
              c_value[(nb + j) + 1] = c_value[n + j];
            }

            if (b_i - 1 >= 0) {
              nb += b_i;
            }

            if (nb + 1 < 1) {
              loop_ub = 0;
            } else {
              loop_ub = nb + 1;
            }

            c_value.set_size(c_value.size(0), loop_ub);
            nd2 = c_value.size(1) >> 1;
            for (b_i = 0; b_i < nd2; b_i++) {
              n = (c_value.size(1) - b_i) - 1;
              xtmp = c_value[b_i];
              c_value[b_i] = c_value[n];
              c_value[n] = xtmp;
            }

            value_size[0] = 1;
            value_size[1] = loop_ub;
            for (i = 0; i < loop_ub; i++) {
              value_data[i] = c_value[i];
            }
          }

          //
          // Arguments    : ::coder::array<double, 2U> &b_value
          // Return Type  : void
          //
          void GIKProblem::get_WeightMatrix(::coder::array<double, 2U> &b_value)
            const
          {
            ::coder::array<double, 2U> r3;
            ::coder::array<double, 2U> v;
            ::coder::array<double, 1U> r;
            ::coder::array<int, 1U> r1;
            ::coder::array<int, 1U> r2;
            int b_loop_ub;
            int i;
            int i1;
            int loop_ub;
            eye(NumResiduals, b_value);
            loop_ub = ResidualIndices[0].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[0].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[0].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f1->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f1->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f1->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[1].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[1].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[1].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f2->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f2->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f2->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[2].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[2].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[2].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f3->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f3->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f3->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[3].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[3].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[3].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f4->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f4->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f4->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[4].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[4].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[4].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f5->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f5->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f5->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[5].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[5].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[5].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f6->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f6->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f6->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[6].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[6].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[6].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f7->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f7->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f7->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[7].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[7].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[7].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f8->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f8->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f8->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[8].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[8].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[8].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f9->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f9->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f9->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[9].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[9].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[9].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f10->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f10->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f10->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[10].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[10].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[10].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f11->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f11->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f11->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[11].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[11].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[11].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f12->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f12->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f12->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[12].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[12].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[12].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f13->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f13->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f13->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[13].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[13].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[13].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f14->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f14->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f14->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[14].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[14].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[14].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f15->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f15->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f15->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[15].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[15].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[15].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f16->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f16->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f16->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[16].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[16].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[16].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f17->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f17->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f17->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[17].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[17].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[17].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f18->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f18->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f18->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[18].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[18].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[18].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f19->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f19->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f19->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[19].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[19].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[19].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f20->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f20->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f20->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[20].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[20].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[20].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f21->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f21->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f21->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }

            loop_ub = ResidualIndices[21].f1.size(1);
            r.set_size(loop_ub);
            b_loop_ub = ResidualIndices[21].f1.size(1);
            for (i = 0; i < b_loop_ub; i++) {
              r[i] = ResidualIndices[21].f1[i];
            }

            r1.set_size(loop_ub);
            r2.set_size(loop_ub);
            for (i = 0; i < loop_ub; i++) {
              i1 = static_cast<int>(r[i]) - 1;
              r1[i] = i1;
              r2[i] = i1;
            }

            i = Constraints.f22->Weights.size(1);
            v.set_size(1, i);
            b_loop_ub = Constraints.f22->Weights.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              v[i1] = Constraints.f22->Weights[i1];
            }

            r3.set_size(i, i);
            b_loop_ub = v.size(1) * v.size(1);
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              r3[i1] = 0.0;
            }

            for (b_loop_ub = 0; b_loop_ub < i; b_loop_ub++) {
              r3[b_loop_ub + r3.size(0) * b_loop_ub] = v[b_loop_ub];
            }

            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
              }
            }
          }

          //
          // Arguments    : b_RigidBodyTree *tree
          // Return Type  : GIKProblem *
          //
          GIKProblem *GIKProblem::init(b_RigidBodyTree *tree)
          {
            GIKProblem *obj;
            ::coder::array<double, 2U> r;
            ::coder::array<double, 2U> varargin_1;
            ::coder::array<double, 2U> y;
            ::coder::array<double, 1U> b_obj;
            ::coder::array<double, 1U> b_varargin_1;
            cell_wrap_7 residualIndices[22];
            cell_wrap_7 slackIndices[22];
            cell_wrap_8 equalityFlags[22];
            double numResiduals;
            double numResidualsTotal;
            int b_loop_ub;
            int loop_ub;
            obj = this;
            obj->Tree = tree;
            obj->NumPositions = obj->Tree->PositionNumber;
            obj->Constraints.f1 = obj->_pobj2.init(obj->Tree);
            numResiduals = obj->Constraints.f1->NumElements;
            if (std::isnan(numResiduals)) {
              residualIndices[0].f1.set_size(1, 1);
              residualIndices[0].f1[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              residualIndices[0].f1.set_size(1, 0);
            } else {
              residualIndices[0].f1.set_size(1, static_cast<int>(numResiduals -
                1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                residualIndices[0].f1[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = residualIndices[0].f1.size(1);
            slackIndices[0].f1.set_size(1, residualIndices[0].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[0].f1[i] = obj->NumPositions + residualIndices[0]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[0].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[0].f1[i] = false;
            }

            numResidualsTotal = numResiduals;
            obj->Constraints.f2 = obj->_pobj1.init(obj->Tree);
            numResiduals = obj->Constraints.f2->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[1].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[1].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[1].f1.size(1);
            slackIndices[1].f1.set_size(1, residualIndices[1].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[1].f1[i] = obj->NumPositions + residualIndices[1]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[1].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[1].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f3 = obj->_pobj0[0].init(obj->Tree);
            numResiduals = obj->Constraints.f3->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[2].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[2].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[2].f1.size(1);
            slackIndices[2].f1.set_size(1, residualIndices[2].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[2].f1[i] = obj->NumPositions + residualIndices[2]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[2].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[2].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f4 = obj->_pobj0[1].init(obj->Tree);
            numResiduals = obj->Constraints.f4->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[3].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[3].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[3].f1.size(1);
            slackIndices[3].f1.set_size(1, residualIndices[3].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[3].f1[i] = obj->NumPositions + residualIndices[3]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[3].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[3].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f5 = obj->_pobj0[2].init(obj->Tree);
            numResiduals = obj->Constraints.f5->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[4].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[4].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[4].f1.size(1);
            slackIndices[4].f1.set_size(1, residualIndices[4].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[4].f1[i] = obj->NumPositions + residualIndices[4]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[4].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[4].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f6 = obj->_pobj0[3].init(obj->Tree);
            numResiduals = obj->Constraints.f6->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[5].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[5].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[5].f1.size(1);
            slackIndices[5].f1.set_size(1, residualIndices[5].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[5].f1[i] = obj->NumPositions + residualIndices[5]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[5].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[5].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f7 = obj->_pobj0[4].init(obj->Tree);
            numResiduals = obj->Constraints.f7->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[6].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[6].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[6].f1.size(1);
            slackIndices[6].f1.set_size(1, residualIndices[6].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[6].f1[i] = obj->NumPositions + residualIndices[6]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[6].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[6].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f8 = obj->_pobj0[5].init(obj->Tree);
            numResiduals = obj->Constraints.f8->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[7].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[7].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[7].f1.size(1);
            slackIndices[7].f1.set_size(1, residualIndices[7].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[7].f1[i] = obj->NumPositions + residualIndices[7]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[7].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[7].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f9 = obj->_pobj0[6].init(obj->Tree);
            numResiduals = obj->Constraints.f9->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[8].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[8].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[8].f1.size(1);
            slackIndices[8].f1.set_size(1, residualIndices[8].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[8].f1[i] = obj->NumPositions + residualIndices[8]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[8].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[8].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f10 = obj->_pobj0[7].init(obj->Tree);
            numResiduals = obj->Constraints.f10->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[9].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[9].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[9].f1.size(1);
            slackIndices[9].f1.set_size(1, residualIndices[9].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[9].f1[i] = obj->NumPositions + residualIndices[9]
                .f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[9].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[9].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f11 = obj->_pobj0[8].init(obj->Tree);
            numResiduals = obj->Constraints.f11->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[10].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[10].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[10].f1.size(1);
            slackIndices[10].f1.set_size(1, residualIndices[10].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[10].f1[i] = obj->NumPositions + residualIndices[10].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[10].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[10].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f12 = obj->_pobj0[9].init(obj->Tree);
            numResiduals = obj->Constraints.f12->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[11].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[11].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[11].f1.size(1);
            slackIndices[11].f1.set_size(1, residualIndices[11].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[11].f1[i] = obj->NumPositions + residualIndices[11].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[11].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[11].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f13 = obj->_pobj0[10].init(obj->Tree);
            numResiduals = obj->Constraints.f13->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[12].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[12].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[12].f1.size(1);
            slackIndices[12].f1.set_size(1, residualIndices[12].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[12].f1[i] = obj->NumPositions + residualIndices[12].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[12].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[12].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f14 = obj->_pobj0[11].init(obj->Tree);
            numResiduals = obj->Constraints.f14->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[13].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[13].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[13].f1.size(1);
            slackIndices[13].f1.set_size(1, residualIndices[13].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[13].f1[i] = obj->NumPositions + residualIndices[13].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[13].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[13].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f15 = obj->_pobj0[12].init(obj->Tree);
            numResiduals = obj->Constraints.f15->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[14].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[14].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[14].f1.size(1);
            slackIndices[14].f1.set_size(1, residualIndices[14].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[14].f1[i] = obj->NumPositions + residualIndices[14].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[14].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[14].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f16 = obj->_pobj0[13].init(obj->Tree);
            numResiduals = obj->Constraints.f16->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[15].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[15].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[15].f1.size(1);
            slackIndices[15].f1.set_size(1, residualIndices[15].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[15].f1[i] = obj->NumPositions + residualIndices[15].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[15].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[15].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f17 = obj->_pobj0[14].init(obj->Tree);
            numResiduals = obj->Constraints.f17->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[16].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[16].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[16].f1.size(1);
            slackIndices[16].f1.set_size(1, residualIndices[16].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[16].f1[i] = obj->NumPositions + residualIndices[16].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[16].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[16].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f18 = obj->_pobj0[15].init(obj->Tree);
            numResiduals = obj->Constraints.f18->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[17].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[17].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[17].f1.size(1);
            slackIndices[17].f1.set_size(1, residualIndices[17].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[17].f1[i] = obj->NumPositions + residualIndices[17].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[17].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[17].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f19 = obj->_pobj0[16].init(obj->Tree);
            numResiduals = obj->Constraints.f19->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[18].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[18].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[18].f1.size(1);
            slackIndices[18].f1.set_size(1, residualIndices[18].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[18].f1[i] = obj->NumPositions + residualIndices[18].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[18].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[18].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f20 = obj->_pobj0[17].init(obj->Tree);
            numResiduals = obj->Constraints.f20->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[19].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[19].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[19].f1.size(1);
            slackIndices[19].f1.set_size(1, residualIndices[19].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[19].f1[i] = obj->NumPositions + residualIndices[19].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[19].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[19].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f21 = obj->_pobj0[18].init(obj->Tree);
            numResiduals = obj->Constraints.f21->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[20].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[20].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[20].f1.size(1);
            slackIndices[20].f1.set_size(1, residualIndices[20].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[20].f1[i] = obj->NumPositions + residualIndices[20].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[20].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[20].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            obj->Constraints.f22 = obj->_pobj0[19].init(obj->Tree);
            numResiduals = obj->Constraints.f22->NumElements;
            if (std::isnan(numResiduals)) {
              y.set_size(1, 1);
              y[0] = rtNaN;
            } else if (numResiduals < 1.0) {
              y.set_size(1, 0);
            } else {
              y.set_size(1, static_cast<int>(numResiduals - 1.0) + 1);
              loop_ub = static_cast<int>(numResiduals - 1.0);
              for (int i{0}; i <= loop_ub; i++) {
                y[i] = static_cast<double>(i) + 1.0;
              }
            }

            loop_ub = y.size(1);
            residualIndices[21].f1.set_size(1, y.size(1));
            for (int i{0}; i < loop_ub; i++) {
              residualIndices[21].f1[i] = numResidualsTotal + y[i];
            }

            loop_ub = residualIndices[21].f1.size(1);
            slackIndices[21].f1.set_size(1, residualIndices[21].f1.size(1));
            for (int i{0}; i < loop_ub; i++) {
              slackIndices[21].f1[i] = obj->NumPositions + residualIndices[21].
                f1[i];
            }

            loop_ub = static_cast<int>(numResiduals);
            equalityFlags[21].f1.set_size(1, loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              equalityFlags[21].f1[i] = false;
            }

            numResidualsTotal += numResiduals;
            for (int i{0}; i < 22; i++) {
              obj->ResidualIndices[i] = residualIndices[i];
            }

            for (int i{0}; i < 22; i++) {
              obj->SlackIndices[i] = slackIndices[i];
            }

            for (int i{0}; i < 22; i++) {
              obj->EqualityFlags[i] = equalityFlags[i];
            }

            obj->NumResiduals = numResidualsTotal;
            obj->NumSlacks = numResidualsTotal;
            obj->NumVariables = obj->NumPositions + obj->NumSlacks;
            loop_ub = static_cast<int>(obj->NumVariables);
            b_loop_ub = static_cast<int>(obj->NumVariables);
            varargin_1.set_size(loop_ub, 2);
            for (int i{0}; i < loop_ub; i++) {
              varargin_1[i] = rtMinusInf;
            }

            for (int i{0}; i < b_loop_ub; i++) {
              varargin_1[i + varargin_1.size(0)] = rtInf;
            }

            obj->set_DesignVariableBounds(varargin_1);
            obj->b_set_EnforceJointLimits();
            obj->updateDesignVariableBounds();
            loop_ub = static_cast<int>(obj->NumVariables);
            obj->LastX.set_size(loop_ub);
            for (int i{0}; i < loop_ub; i++) {
              obj->LastX[i] = 0.0;
            }

            b_obj.set_size(obj->LastX.size(0));
            loop_ub = obj->LastX.size(0) - 1;
            for (int i{0}; i <= loop_ub; i++) {
              b_obj[i] = obj->LastX[i];
            }

            obj->residualsInternal(b_obj, b_varargin_1, r);
            loop_ub = b_varargin_1.size(0);
            obj->LastF.set_size(b_varargin_1.size(0));
            for (int i{0}; i < loop_ub; i++) {
              obj->LastF[i] = b_varargin_1[i];
            }

            obj->LastJ.set_size(r.size(0), r.size(1));
            loop_ub = r.size(0) * r.size(1);
            for (int i{0}; i < loop_ub; i++) {
              obj->LastJ[i] = r[i];
            }

            obj->matlabCodegenIsDeleted = false;
            return obj;
          }

          //
          // Arguments    : const ::coder::array<double, 1U> &x
          //                ::coder::array<double, 1U> &f
          //                ::coder::array<double, 2U> &J
          // Return Type  : void
          //
          void GIKProblem::residuals(const ::coder::array<double, 1U> &x, ::
            coder::array<double, 1U> &f, ::coder::array<double, 2U> &J)
          {
            double d;
            int i;
            bool flag;
            flag = true;
            d = NumVariables;
            i = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (i <= static_cast<int>(d) - 1) {
                if (x[i] != LastX[i]) {
                  exitg1 = 1;
                } else {
                  i++;
                }
              } else {
                flag = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);

            if (flag) {
              residualsInternal(x, f, J);
              i = x.size(0);
              LastX.set_size(x.size(0));
              for (int b_i{0}; b_i < i; b_i++) {
                LastX[b_i] = x[b_i];
              }

              i = f.size(0);
              LastF.set_size(f.size(0));
              for (int b_i{0}; b_i < i; b_i++) {
                LastF[b_i] = f[b_i];
              }

              LastJ.set_size(J.size(0), J.size(1));
              i = J.size(0) * J.size(1);
              for (int b_i{0}; b_i < i; b_i++) {
                LastJ[b_i] = J[b_i];
              }
            } else {
              f.set_size(LastF.size(0));
              i = LastF.size(0);
              for (int b_i{0}; b_i < i; b_i++) {
                f[b_i] = LastF[b_i];
              }

              J.set_size(LastJ.size(0), LastJ.size(1));
              i = LastJ.size(0) * LastJ.size(1);
              for (int b_i{0}; b_i < i; b_i++) {
                J[b_i] = LastJ[b_i];
              }
            }
          }

          //
          // Arguments    : const ::coder::array<double, 1U> &x
          //                ::coder::array<double, 1U> &f
          // Return Type  : void
          //
          void GIKProblem::residuals(const ::coder::array<double, 1U> &x, ::
            coder::array<double, 1U> &f)
          {
            ::coder::array<double, 2U> J;
            double d;
            int i;
            bool flag;
            flag = true;
            d = NumVariables;
            i = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (i <= static_cast<int>(d) - 1) {
                if (x[i] != LastX[i]) {
                  exitg1 = 1;
                } else {
                  i++;
                }
              } else {
                flag = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);

            if (flag) {
              residualsInternal(x, f, J);
              i = x.size(0);
              LastX.set_size(x.size(0));
              for (int b_i{0}; b_i < i; b_i++) {
                LastX[b_i] = x[b_i];
              }

              i = f.size(0);
              LastF.set_size(f.size(0));
              for (int b_i{0}; b_i < i; b_i++) {
                LastF[b_i] = f[b_i];
              }

              LastJ.set_size(J.size(0), J.size(1));
              i = J.size(0) * J.size(1);
              for (int b_i{0}; b_i < i; b_i++) {
                LastJ[b_i] = J[b_i];
              }
            } else {
              f.set_size(LastF.size(0));
              i = LastF.size(0);
              for (int b_i{0}; b_i < i; b_i++) {
                f[b_i] = LastF[b_i];
              }
            }
          }

          //
          // Arguments    : bool b_value
          // Return Type  : void
          //
          void GIKProblem::set_EnforceJointLimits(bool b_value)
          {
            ::coder::array<double, 2U> A;
            ::coder::array<double, 2U> r;
            ::coder::array<double, 2U> r1;
            ::coder::array<double, 1U> b;
            EnforceJointLimitsInternal = b_value;
            if (EnforceJointLimitsInternal) {
              double d;
              int b_loop_ub;
              int c_loop_ub;
              int loop_ub;
              loop_ub = DesignVariableBoundsInternal.size(0);
              r.set_size(loop_ub, 2);
              b_loop_ub = DesignVariableBoundsInternal.size(0) << 1;
              for (int i{0}; i < b_loop_ub; i++) {
                r[i] = DesignVariableBoundsInternal[i];
              }

              d = NumPositions;
              Tree->get_JointPositionLimits(r1);
              if (d < 1.0) {
                b_loop_ub = 0;
              } else {
                b_loop_ub = static_cast<int>(d);
              }

              for (int i{0}; i < 2; i++) {
                for (c_loop_ub = 0; c_loop_ub < b_loop_ub; c_loop_ub++) {
                  r[c_loop_ub + r.size(0) * i] = r1[c_loop_ub + r1.size(0) * i];
                }
              }

              DesignVariableBoundsInternal.set_size(loop_ub, 2);
              c_loop_ub = r.size(0) << 1;
              for (int i{0}; i < c_loop_ub; i++) {
                DesignVariableBoundsInternal[i] = r[i];
              }

              b_loop_ub = 2 * r.size(0);
              A.set_size(b_loop_ub, loop_ub);
              c_loop_ub = b_loop_ub * r.size(0);
              for (int i{0}; i < c_loop_ub; i++) {
                A[i] = 0.0;
              }

              b.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                b[i] = 0.0;
              }

              for (int i{0}; i < loop_ub; i++) {
                c_loop_ub = static_cast<int>(static_cast<unsigned int>(i + 1) <<
                  1);
                A[(c_loop_ub + A.size(0) * i) - 2] = -1.0;
                A[(c_loop_ub + A.size(0) * i) - 1] = 1.0;
                b[c_loop_ub - 2] = -r[i];
                b[c_loop_ub - 1] = r[i + r.size(0)];
              }

              ConstraintMatrixInternal.set_size(loop_ub, b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                for (c_loop_ub = 0; c_loop_ub < loop_ub; c_loop_ub++) {
                  ConstraintMatrixInternal[c_loop_ub +
                    ConstraintMatrixInternal.size(0) * i] = A[i + A.size(0) *
                    c_loop_ub];
                }
              }

              ConstraintBoundInternal.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                ConstraintBoundInternal[i] = b[i];
              }
            } else {
              int b_loop_ub;
              int c_loop_ub;
              int loop_ub;
              loop_ub = DesignVariableBoundsInternal.size(0);
              r.set_size(loop_ub, 2);
              b_loop_ub = DesignVariableBoundsInternal.size(0) << 1;
              for (int i{0}; i < b_loop_ub; i++) {
                r[i] = DesignVariableBoundsInternal[i];
              }

              b_loop_ub = static_cast<int>(NumPositions);
              c_loop_ub = static_cast<int>(NumPositions);
              for (int i{0}; i < b_loop_ub; i++) {
                r[i] = rtMinusInf;
              }

              for (int i{0}; i < c_loop_ub; i++) {
                r[i + r.size(0)] = rtInf;
              }

              DesignVariableBoundsInternal.set_size(loop_ub, 2);
              c_loop_ub = r.size(0) << 1;
              for (int i{0}; i < c_loop_ub; i++) {
                DesignVariableBoundsInternal[i] = r[i];
              }

              b_loop_ub = 2 * r.size(0);
              A.set_size(b_loop_ub, loop_ub);
              c_loop_ub = b_loop_ub * r.size(0);
              for (int i{0}; i < c_loop_ub; i++) {
                A[i] = 0.0;
              }

              b.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                b[i] = 0.0;
              }

              for (int i{0}; i < loop_ub; i++) {
                c_loop_ub = static_cast<int>(static_cast<unsigned int>(i + 1) <<
                  1);
                A[(c_loop_ub + A.size(0) * i) - 2] = -1.0;
                A[(c_loop_ub + A.size(0) * i) - 1] = 1.0;
                b[c_loop_ub - 2] = -r[i];
                b[c_loop_ub - 1] = r[i + r.size(0)];
              }

              ConstraintMatrixInternal.set_size(loop_ub, b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                for (c_loop_ub = 0; c_loop_ub < loop_ub; c_loop_ub++) {
                  ConstraintMatrixInternal[c_loop_ub +
                    ConstraintMatrixInternal.size(0) * i] = A[i + A.size(0) *
                    c_loop_ub];
                }
              }

              ConstraintBoundInternal.set_size(b_loop_ub);
              for (int i{0}; i < b_loop_ub; i++) {
                ConstraintBoundInternal[i] = b[i];
              }
            }
          }

          //
          // Arguments    : const constraintPoseTarget &varargin_1
          //                const constraintJointBounds &varargin_2
          //                const constraintDistanceBounds *varargin_3
          //                const constraintDistanceBounds *varargin_4
          //                const constraintDistanceBounds *varargin_5
          //                const constraintDistanceBounds *varargin_6
          //                const constraintDistanceBounds *varargin_7
          //                const constraintDistanceBounds *varargin_8
          //                const constraintDistanceBounds *varargin_9
          //                const constraintDistanceBounds *varargin_10
          //                const constraintDistanceBounds *varargin_11
          //                const constraintDistanceBounds *varargin_12
          //                const constraintDistanceBounds *varargin_13
          //                const constraintDistanceBounds *varargin_14
          //                const constraintDistanceBounds *varargin_15
          //                const constraintDistanceBounds *varargin_16
          //                const constraintDistanceBounds *varargin_17
          //                const constraintDistanceBounds *varargin_18
          //                const constraintDistanceBounds *varargin_19
          //                const constraintDistanceBounds *varargin_20
          //                const constraintDistanceBounds *varargin_21
          //                const constraintDistanceBounds *varargin_22
          // Return Type  : void
          //
          void GIKProblem::update(const constraintPoseTarget &varargin_1, const
            constraintJointBounds &varargin_2, const constraintDistanceBounds
            *varargin_3, const constraintDistanceBounds *varargin_4, const
            constraintDistanceBounds *varargin_5, const constraintDistanceBounds
            *varargin_6, const constraintDistanceBounds *varargin_7, const
            constraintDistanceBounds *varargin_8, const constraintDistanceBounds
            *varargin_9, const constraintDistanceBounds *varargin_10, const
            constraintDistanceBounds *varargin_11, const
            constraintDistanceBounds *varargin_12, const
            constraintDistanceBounds *varargin_13, const
            constraintDistanceBounds *varargin_14, const
            constraintDistanceBounds *varargin_15, const
            constraintDistanceBounds *varargin_16, const
            constraintDistanceBounds *varargin_17, const
            constraintDistanceBounds *varargin_18, const
            constraintDistanceBounds *varargin_19, const
            constraintDistanceBounds *varargin_20, const
            constraintDistanceBounds *varargin_21, const
            constraintDistanceBounds *varargin_22)
          {
            JointPositionBounds *d_obj;
            PoseTarget *obj;
            RigidBody *e_obj;
            b_RigidBodyTree *b_obj;
            ::coder::array<double, 2U> r1;
            ::coder::array<double, 1U> f_obj;
            ::coder::array<double, 1U> r;
            int a_size[2];
            int exitg1;
            int i;
            int kstr;
            char a_data[200];
            bool b_bool;
            obj = Constraints.f1;
            obj->Weights.set_size(1, 2);
            obj->Weights[0] = varargin_1.Weights[0];
            obj->Weights[1] = varargin_1.Weights[1];
            obj->get_EndEffector(a_data, a_size);
            b_bool = false;
            if (a_size[1] == 17) {
              kstr = 0;
              do {
                exitg1 = 0;
                if (kstr < 17) {
                  if (a_data[kstr] != varargin_1.EndEffector[kstr]) {
                    exitg1 = 1;
                  } else {
                    kstr++;
                  }
                } else {
                  b_bool = true;
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }

            if (!b_bool) {
              CharacterVector c_obj;
              int bid;
              b_obj = obj->Tree;
              bid = -1;
              c_obj = b_obj->Base.NameInternal;
              if (c_obj.Length < 1.0) {
                i = 0;
              } else {
                i = static_cast<int>(c_obj.Length);
              }

              b_bool = false;
              if (i == 17) {
                kstr = 0;
                do {
                  exitg1 = 0;
                  if (kstr < 17) {
                    if (c_obj.Vector[kstr] != varargin_1.EndEffector[kstr]) {
                      exitg1 = 1;
                    } else {
                      kstr++;
                    }
                  } else {
                    b_bool = true;
                    exitg1 = 1;
                  }
                } while (exitg1 == 0);
              }

              if (b_bool) {
                bid = 0;
              } else {
                double d;
                int b_i;
                bool exitg2;
                d = b_obj->NumBodies;
                b_i = 0;
                exitg2 = false;
                while ((!exitg2) && (b_i <= static_cast<int>(d) - 1)) {
                  e_obj = b_obj->Bodies[b_i];
                  c_obj = e_obj->NameInternal;
                  if (c_obj.Length < 1.0) {
                    i = 0;
                  } else {
                    i = static_cast<int>(c_obj.Length);
                  }

                  b_bool = false;
                  if (i == 17) {
                    kstr = 0;
                    do {
                      exitg1 = 0;
                      if (kstr < 17) {
                        if (c_obj.Vector[kstr] != varargin_1.EndEffector[kstr])
                        {
                          exitg1 = 1;
                        } else {
                          kstr++;
                        }
                      } else {
                        b_bool = true;
                        exitg1 = 1;
                      }
                    } while (exitg1 == 0);
                  }

                  if (b_bool) {
                    bid = b_i + 1;
                    exitg2 = true;
                  } else {
                    b_i++;
                  }
                }
              }

              obj->EndEffectorIndex = bid;
            }

            obj->get_ReferenceBody(a_data, a_size);
            if (static_cast<unsigned char>(a_size[1]) != 0) {
              obj->ReferenceBodyIndex = 0.0;
            }

            for (i = 0; i < 16; i++) {
              obj->TargetTransform[i] = varargin_1.TargetTransform[i];
            }

            obj->BoundsInternal[obj->BoundsInternal.size(0)] =
              varargin_1.OrientationTolerance;
            obj->BoundsInternal[obj->BoundsInternal.size(0) + 1] =
              varargin_1.PositionTolerance;
            d_obj = Constraints.f2;
            d_obj->Weights.set_size(1, d_obj->Weights.size(1));
            kstr = varargin_2.WeightsInternal.size(1);
            d_obj->Weights.set_size(d_obj->Weights.size(0), kstr);
            for (i = 0; i < kstr; i++) {
              d_obj->Weights[i] = varargin_2.WeightsInternal[i];
            }

            d_obj->BoundsInternal.set_size(varargin_2.BoundsInternal.size(0), 2);
            kstr = varargin_2.BoundsInternal.size(0) << 1;
            for (i = 0; i < kstr; i++) {
              d_obj->BoundsInternal[i] = varargin_2.BoundsInternal[i];
            }

            Constraints.f3->update(varargin_3);
            Constraints.f4->update(varargin_4);
            Constraints.f5->update(varargin_5);
            Constraints.f6->update(varargin_6);
            Constraints.f7->update(varargin_7);
            Constraints.f8->update(varargin_8);
            Constraints.f9->update(varargin_9);
            Constraints.f10->update(varargin_10);
            Constraints.f11->update(varargin_11);
            Constraints.f12->update(varargin_12);
            Constraints.f13->update(varargin_13);
            Constraints.f14->update(varargin_14);
            Constraints.f15->update(varargin_15);
            Constraints.f16->update(varargin_16);
            Constraints.f17->update(varargin_17);
            Constraints.f18->update(varargin_18);
            Constraints.f19->update(varargin_19);
            Constraints.f20->update(varargin_20);
            Constraints.f21->update(varargin_21);
            Constraints.f22->update(varargin_22);
            updateDesignVariableBounds();
            kstr = static_cast<int>(NumVariables);
            LastX.set_size(kstr);
            for (i = 0; i < kstr; i++) {
              LastX[i] = 0.0;
            }

            f_obj.set_size(LastX.size(0));
            kstr = LastX.size(0) - 1;
            for (i = 0; i <= kstr; i++) {
              f_obj[i] = LastX[i];
            }

            residualsInternal(f_obj, r, r1);
            kstr = r.size(0);
            LastF.set_size(r.size(0));
            for (i = 0; i < kstr; i++) {
              LastF[i] = r[i];
            }

            LastJ.set_size(r1.size(0), r1.size(1));
            kstr = r1.size(0) * r1.size(1);
            for (i = 0; i < kstr; i++) {
              LastJ[i] = r1[i];
            }
          }
        }
      }
    }
  }
}

//
// File trailer for GIKProblem.cpp
//
// [EOF]
//
