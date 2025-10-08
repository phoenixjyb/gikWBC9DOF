//
// File: _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_info.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
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
  const char_T *data[6]{
      "789ced56c16ed34010ddd056e2528840f005bd40240b38442dc7a6711b48d35007456ab6"
      "a26b7b926c637b9df5a6044efc017c0e9fc0914b6ffc070784841d7b"
      "93d8d43882605494b98cc72fbbf36636f3d6a8503b2c20846ea1d0ceee847e338a8b91bf"
      "81e296c40b915f4bc4d236d07a6c9dc4df47de608e80b1080387d830",
      "5d69329b3ac411ad372e200e1eb32ec09c205d6a418bdaa0cd078d20b2d539681a0450f0"
      "5ce98331d04636e27d6fc6d09a0fa6fdf89452effa82fd3052fa514c"
      "e09deae9c153fc5a738901d86026f4c0d9af3df788ed5a805b9c120bf7e8a0bd5bd9d93b"
      "52b14d8445745cf25fed98ac8b4bd19257d4197980274df2976b02dc",
      "3627ae0b5cb163757df8c3ba1e66d425f188a012e3a7e8236a99c74c674265bc124228c6"
      "efec37f9252d8d9fb4659db399914fe2cb3de7abfa181e74561f6f2f"
      "5857d2cf7e7f73e2bf96bf14f2ccb7f97ded5b9ef9a4fdab7ce394fd16fd5fde4fc9574c"
      "e05ad57ceba9b67bd218361ec1707f77bb5d6f56673c9a1979b278a0",
      "9438affd3fa7ac5fb48f76cafec504dea9554eb7c2a9e58c892d2c18b37436c65e9f7030"
      "310f86951a5ef83012d4f27069fad21f6f0eb844fdbb903bbe125439"
      "677ccfd70630eb70018e0ebc7748f87044b8292683be2c3d7f90519fc425552560aa48a2"
      "4aa3ded4823b8757c754a816e9c9bbf4baeab99e914fe29d5a6389e7",
      "fd731ba36b3b37fdb9fc7899ab9ebfebdeddc8339fb4ebaae7f752f21513f89373abaf8d"
      "e16450d69e6d975fbe683f3e3e3842ff8f9ee737dfcbd4f3e97cd782"
      "375dff1b3039df2b1d8fe75be9f8d575ad743cb4958eff3a4f160f9412ffedfd7f009cf4"
      "7fbf",
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
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 7);
  emlrtSetField(
      xEntryPoints, 0, "QualifiedName",
      emlrtMxCreateString("gik9dof.codegen_inuse.solveGIKStepWrapper"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(7.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString(
                    "H:\\wSpace\\codegenGIKsample\\Trial\\gikWBC9DOF\\matlab\\+"
                    "gik9dof\\+codegen_inuse\\solveGIKStepWrapper.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739898.50916666666));
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
                emlrtMxCreateString("E0DvxDzpYIRnv1M8VqU3CE"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_info.cpp
//
// [EOF]
//
