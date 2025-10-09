//
// File: _coder_solveGIKStepWrapper_info.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "_coder_solveGIKStepWrapper_info.h"
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
  const char_T *data[6]{
      "789ced56416fd3301476c790b8001502248e48bb40a518384cdb755d3b3abaae2c4515ab"
      "2ae634af8dd7384e1d772b5cf80970e6c2dfe1c88fe07f40d2c4691b"
      "16a5a25527a0efe23c7fb1bff79ef33e07e52a473984d01d14dad9bd70bc1df9f968dc40"
      "b396c473d17823e12bbb893667d629fc533476b823612443c7210ce2",
      "952667d4218e6cbc770109f0b87d01e618e9521b1a94813eedd4028f95a7a0d809a0e0b9"
      "6841a7af0f1912963789d09e76e27a7c4bc97773ce7af452ea914fe0"
      "ad521b3347620b5fea2ee900ee70137ae01c545e7984b936e086a0c4c63dda6fee1577f7"
      "8fcb981169130317fca95d937771215af28e3a430ff0b850fe725d82",
      "db14c47541682cceebf382793dcdc84be15170da4c6c9a31a4b679c20d2ecb5c1443287c"
      "3ffe0eff30bea4a5c5a76c59e76c65f0297cf9e77c552d359655c7bb"
      "73e6951c27efdf1a8f6f1ffd18a7b62abefb5fbf6cac924fd975f18d52f69bf7bb7c98c2"
      "974fe07ac9fce095997b5a1bd49ec1e0606fa759ad972671d43378b2",
      "e24029feaaf6ff9eb27ede3ab294fdf309bc5529b6b7c28e159ccb2d2c39b70d3ec29e45"
      "049858048d4a3b5ef83094d4f670219ef45b5b002e50ff2e148eaf02"
      "2521b8d8f77501cc2a5c806380e81d11311812614a8d4de5b7a89e3fc9c84fe12a542d88"
      "5453816ab56a5d0fee1b511a5159b6494fdda57fab9e1b197c0a6f55",
      "6a4b3cefdfcba8b1d9bcce52e25e96fe3cfef8b3bd4a3e65ff0bdfa27afe20852f9fc05f"
      "9cdb963e82d3feb67eb8b3fde675f3f9c9cb63f4efe8f9eafa7b997a"
      "1ef7772598e9faff7fc9fe5eebf82cdf5ac7afce6bade3d7cbb7d6f1c5f6ff05927476b"
      "6",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 4728U, &nameCaptureInfo);
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
      "Name",     "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "FullPath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 7);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("solveGIKStepWrapper"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(7.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString(
                    "/mnt/h/wSpace/codegenGIKsample/Trial/gikWBC9DOF/matlab/"
                    "+gik9dof/+codegen_inuse/solveGIKStepWrapper.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739899.41825231479));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.1.0.2537033 (R2024a)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("E0DvxDzpYIRnv1M8VqU3CE"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_solveGIKStepWrapper_info.cpp
//
// [EOF]
//
