//
// File: _coder_holisticVelocityController_info.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:20:37
//

// Include Files
#include "_coder_holisticVelocityController_info.h"
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
  const char_T *data[5]{
      "789cdd54cb8ed3301475d08058f0e88ac52c58a0d98d543112148a5835f368c53c805694"
      "8741759ddbc6d48e2bdba1d33f981dbfc3723e84ff60c1867692b44d"
      "c04a454641c3dddc5c1d3be7dc73938b9cd6918310ba83a2787b2bcab7e3ba12e76b281d"
      "59dcc99c73d2c7d175b491ba97e05fe34c6560e0d4444540042c6e7a",
      "52b08004a6331d0352a025ff02de0532601c3a4c407bb5389e57627f055a147368feecfa"
      "4047ed5020e5eba542be5a2cfcf866e977634d3f5e5bfca864f00f7b"
      "1f9bcff0a43d261430951e0c213868bdd0448c39e08e6284e3211b751b6e7df7641f0b62"
      "38e9635f72a60da36f804bcaccd49db9a824e7a0aa22ddc759c13e1e",
      "e4f491e0338d754f0eaa3452520d033660e0b93ed19a69d7289ed6d52ba8eb8655578468"
      "a3426a967ce705f9b0952f8dffdd3cb763fbf076ec1ffeddbf68b279"
      "fedd5db39f6c5e9ebf7991eb4fbe3b65f2f53671a74cbe24fe15dfa9e57deb7e8ff72c7c"
      "950cde0abbdecece94e987afdcc74df6b4361c7d568da58e97393c79",
      "3a90a52eebfde796fb57f5bf2ebaafb772fa49f0ecbea67cd6c93b32694cbb3e003f6482"
      "99555dbd82baf2f6b527c33e87cb9beb272b5f1abfacb9fec9bff964"
      "cbda2f3f6ae5eeebfbcf377f96c997c4ffbeaff74e1e35274153d5760ffbb5c9f07d77c4"
      "8e957bf5f7f52f765ef7a8",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3184U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 9);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("holisticVelocityController"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(9.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString("H:"
                          "\\wSpace\\codegenGIKsample\\Trial\\gikWBC9DOF\\matla"
                          "b\\holisticVelocityController.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739897.47254629631));
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
                emlrtMxCreateString("PgVQmh8belc63hBdITJJCF"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_holisticVelocityController_info.cpp
//
// [EOF]
//
