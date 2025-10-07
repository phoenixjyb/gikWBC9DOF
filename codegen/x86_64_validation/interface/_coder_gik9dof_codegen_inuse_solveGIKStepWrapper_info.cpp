//
// File: _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_info.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "_coder_gik9dof_codegen_inuse_solveGIKStepWrapper_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
//
// Arguments    : void
// Return Type  : const mxArray *
//
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[7]{
      "789ced57c16ed34010dd408bb8042221101fd00b448a804394706c9ab481d42d71aa48ed"
      "56b08927ce366bafbb765297137f009fc32770e4d21bffc1012161c7"
      "de3431318e68306a95b9ac675f76e7cd6ce6798d32f5dd0c42e81e0aacf43818b3a19f0b"
      "c75b68d6a278261cd722beb4f510c946f08fe1d8e5a603ae13382631",
      "60b252e3063589e9b4ce2d40026cce46a08d911e65d0a206a8d38ee27b466d0a9a383ee4"
      "3f57fad01da8430389be7dc9904d3b937a7c89c9770dcd5a5c3dba31"
      "f5c845f0a3eaf1ce4b7ca65aa40bb8cb35d0c1dcaebfb6896131c02d4109c33a1db4372b"
      "e5adbd1a3688c34807e7bda9b2c67b381f2e794bcda10d785c246fb9",
      "ea80d516c4b240148c99bc3e5d31afa70979493c245898e157e80c29d39abcc39d1a1795"
      "004233fcdefd25bfa8c5f193b6ac73d612e2497cb9e73caf8ec14127"
      "d5f1fe827945c7cbdfdf1d8fdf8bdf3269c6cbfebcfd23cd78d2fe573c3766bf45ff978f"
      "62e2e522b85ad5dedb35c33a544e956770babd596a37f6ab973cf613",
      "e224f140317e5afbaf747c7e5e57d3f1697e4d20cca1f2c52df95957e417bd2f44f9495c"
      "509d6a9b5c3b6f090074dd757d5e5d6fb6ae9f8deeac741d2d5fd715"
      "b2e332b77e60524b6fb42b2555d1cbd5cacdd1f5af31eb17ada311b37f2e821fd52bc71b"
      "41d70ace9d0dec70ce3adcc5769f08d0b0f02f61b46b070f4387321b",
      "e727935e7b0bc079ea7de308d35382aa105c6c79da005a0346607640e8bb449c0e89d09c"
      "71a32f4bdf9f24e4277149b5e0332d48a205a5b1affa5a24aa2e756a"
      "8ce8f21be9baded33b09f1247e54579678debf97317c8da7a63f179f2f52d5f30fbd07eb"
      "69c693765df5fc614cbc5c047f71c2faaa0b8783a2faaa543c78d37e",
      "dedcd9433747cfd3ebef65eaf9a4bfebfe4ccfbb0346fb7ba5e3b3f1563a3e3faf958e07"
      "b6d2f13fc749e28162fc7fbdff2fc64cf649",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 6224U, &nameCaptureInfo);
  return nameCaptureInfo;
}

//
// Arguments    : void
// Return Type  : mxArray *
//
mxArray *emlrtMexFcnProperties()
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9]{"Version",
                                 "ResolvedFunctions",
                                 "Checksum",
                                 "EntryPoints",
                                 "CoverageInfo",
                                 "IsPolymorphic",
                                 "PropertyList",
                                 "UUID",
                                 "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8]{
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 4);
  emlrtSetField(
      xEntryPoints, 0, "QualifiedName",
      emlrtMxCreateString("gik9dof.codegen_inuse.solveGIKStepWrapper"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString(
                    "H:\\wSpace\\codegenGIKsample\\Trial\\gikWBC9DOF\\matlab\\+"
                    "gik9dof\\+codegen_inuse\\solveGIKStepWrapper.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739897.1476967592));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2712019 (R2024b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("QUqfYPKIByPlGPjMf4hkCH"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_info.cpp
//
// [EOF]
//
