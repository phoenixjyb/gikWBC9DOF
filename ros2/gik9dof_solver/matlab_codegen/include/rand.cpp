//
// File: rand.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "rand.h"
#include "GIKSolver.h"
#include "eml_rand_mt19937ar.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
//                double varargin_1
//                double r_data[]
// Return Type  : int
//
namespace gik9dof {
namespace coder {
int b_rand(GIKSolver *aInstancePtr, double varargin_1, double r_data[])
{
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData *localSD;
  int i;
  int r_size;
  localSD = aInstancePtr->getStackData();
  i = static_cast<int>(varargin_1);
  r_size = static_cast<int>(varargin_1);
  for (int k{0}; k < i; k++) {
    unsigned int u[2];
    // ========================= COPYRIGHT NOTICE ============================
    //  This is a uniform (0,1) pseudorandom number generator based on:
    //
    //  A C-program for MT19937, with initialization improved 2002/1/26.
    //  Coded by Takuji Nishimura and Makoto Matsumoto.
    //
    //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
    //  All rights reserved.
    //
    //  Redistribution and use in source and binary forms, with or without
    //  modification, are permitted provided that the following conditions
    //  are met:
    //
    //    1. Redistributions of source code must retain the above copyright
    //       notice, this list of conditions and the following disclaimer.
    //
    //    2. Redistributions in binary form must reproduce the above copyright
    //       notice, this list of conditions and the following disclaimer
    //       in the documentation and/or other materials provided with the
    //       distribution.
    //
    //    3. The names of its contributors may not be used to endorse or
    //       promote products derived from this software without specific
    //       prior written permission.
    //
    //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
    //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    //
    // =============================   END   =================================
    do {
      internal::randfun::genrand_uint32_vector(localSD->pd->state, u);
      u[0] >>= 5U;
      u[1] >>= 6U;
    } while ((u[0] == 0U) && (u[1] == 0U));
    r_data[k] =
        1.1102230246251565E-16 *
        (static_cast<double>(u[0]) * 6.7108864E+7 + static_cast<double>(u[1]));
  }
  return r_size;
}

//
// Arguments    : GIKSolver *aInstancePtr
//                double r[5]
// Return Type  : void
//
void b_rand(GIKSolver *aInstancePtr, double r[5])
{
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  for (int k{0}; k < 5; k++) {
    unsigned int u[2];
    // ========================= COPYRIGHT NOTICE ============================
    //  This is a uniform (0,1) pseudorandom number generator based on:
    //
    //  A C-program for MT19937, with initialization improved 2002/1/26.
    //  Coded by Takuji Nishimura and Makoto Matsumoto.
    //
    //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
    //  All rights reserved.
    //
    //  Redistribution and use in source and binary forms, with or without
    //  modification, are permitted provided that the following conditions
    //  are met:
    //
    //    1. Redistributions of source code must retain the above copyright
    //       notice, this list of conditions and the following disclaimer.
    //
    //    2. Redistributions in binary form must reproduce the above copyright
    //       notice, this list of conditions and the following disclaimer
    //       in the documentation and/or other materials provided with the
    //       distribution.
    //
    //    3. The names of its contributors may not be used to endorse or
    //       promote products derived from this software without specific
    //       prior written permission.
    //
    //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
    //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    //
    // =============================   END   =================================
    do {
      internal::randfun::genrand_uint32_vector(localSD->pd->state, u);
      u[0] >>= 5U;
      u[1] >>= 6U;
    } while ((u[0] == 0U) && (u[1] == 0U));
    r[k] = 1.1102230246251565E-16 * (static_cast<double>(u[0]) * 6.7108864E+7 +
                                     static_cast<double>(u[1]));
  }
}

//
// Arguments    : GIKSolver *aInstancePtr
//                double r[3]
// Return Type  : void
//
void c_rand(GIKSolver *aInstancePtr, double r[3])
{
  gik9dof_codegen_realtime_solveGIKStepWrapperStackData *localSD;
  localSD = aInstancePtr->getStackData();
  for (int k{0}; k < 3; k++) {
    unsigned int u[2];
    // ========================= COPYRIGHT NOTICE ============================
    //  This is a uniform (0,1) pseudorandom number generator based on:
    //
    //  A C-program for MT19937, with initialization improved 2002/1/26.
    //  Coded by Takuji Nishimura and Makoto Matsumoto.
    //
    //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
    //  All rights reserved.
    //
    //  Redistribution and use in source and binary forms, with or without
    //  modification, are permitted provided that the following conditions
    //  are met:
    //
    //    1. Redistributions of source code must retain the above copyright
    //       notice, this list of conditions and the following disclaimer.
    //
    //    2. Redistributions in binary form must reproduce the above copyright
    //       notice, this list of conditions and the following disclaimer
    //       in the documentation and/or other materials provided with the
    //       distribution.
    //
    //    3. The names of its contributors may not be used to endorse or
    //       promote products derived from this software without specific
    //       prior written permission.
    //
    //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
    //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    //
    // =============================   END   =================================
    do {
      internal::randfun::genrand_uint32_vector(localSD->pd->state, u);
      u[0] >>= 5U;
      u[1] >>= 6U;
    } while ((u[0] == 0U) && (u[1] == 0U));
    r[k] = 1.1102230246251565E-16 * (static_cast<double>(u[0]) * 6.7108864E+7 +
                                     static_cast<double>(u[1]));
  }
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for rand.cpp
//
// [EOF]
//
