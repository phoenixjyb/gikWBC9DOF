//
// _coder_gik9dof_codegen_followTrajectory_info.cpp
//
// Code generation for function 'gik9dof.codegen.followTrajectory'
//

// Include files
#include "_coder_gik9dof_codegen_followTrajectory_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[8]{
      "789ced58bf6fd3401476508b90101009c104135d50a4a84534346ce45793d224a54e5bd1"
      "baa8e7f892b839fb92b393b899bac146ff013676d68e1d1919e8c682"
      "c43f80f80788e35c7e981c8eb07195286f39bffb7cf7bdf72eefb3632e90c906388ebbcd"
      "5976f6d01a6ff5fc606fbcc68d9a1d0ff4c6459bcff5e71746d651fc",
      "7d6f2c625587866e392a50607fa584155905aa5e38a9418e400da32694ba484946b0202b"
      "901f7672a6a7a486a0be6342e675bc028b55bea170a4a20d2244c34e"
      "bf1e178c7c1726acc73ea31e411b7e903c4c3f175a7c0d14a150c4122c43753df352034a"
      "0d41a140648084b25cdd8bc5a3897c4a50808e8028843a5351099784",
      "506f8950c208e15681806358d43139092ba3f99cbacce781433e14ef0516eec515ee1e5b"
      "27215e87b5e1788e5cc6739d198f8548b8212238e03b77c9b7cbe41b"
      "c5bd3acfe1ba9967e954b73b13e6611f07f7dfe88e3f9e7eeb427ef145be7ffde8271fb5"
      "abe23318fb4dfa3bbccfe00bdaf0e86a24a71bed178ddd8db2d8aea7",
      "97f3daeb567c10c796038f531c1cc3f76bff59e9e7772ef378e49007c5edba8c3090b6b1"
      "88f51426716bae7b9f5b7db61b2b2e6a5e3d6f0f1df828eed5798ea9"
      "5ff791eb97aefcf259a7031fa49b7ef2519b569dbec7e00bdaf067d9f57495afe79b6d50"
      "496eafecc4a2f565839b1d9dbe60ac9fd6be76abd74b0ef950dcaed7",
      "624346e3047b5af5fa8d031fc5bd3ad771f5f3f3bdfaa7cf7addfe547feb271fb569d5eb"
      "49dfabe3593e95588d929363496b26a269a5b51649c76647afbf30d6"
      "4f5a4785b17fd0861f64e2874b56b7128cf52541c71889d810b40a20501288d9aa7251b3"
      "2e1aba8c3421d49fecb435814248567548d48e022409c124d1d10428",
      "6dc226544548ca5940ea0d40247d44bfcf5ce6f7d8213f8ad350c366a4611a6838b7b9c5"
      "9bff0448d290f5140265faede9aa74fcf33ff2d1fd45073e8a1f6472"
      "1e9ef79f65b47fef3a62c4ed95fe5c9e5ffaaae7a7a5bb8b7ef2519b563d9ff4fdfbc931"
      "aaf006dcaf46f88db5c8ceabbd95ed749e9b1d3df7afbfbdd4f37e7f",
      "67cc9952e7ddcfdedf731d1fe59bebf8f8bce63a6ed95cc7ffcee31407c7f0fff7febf01"
      "99c141e9",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 7592U, &nameCaptureInfo);
  return nameCaptureInfo;
}

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
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("gik9dof.codegen.followTrajectory"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString(
                    "H:\\wSpace\\codegenGIKsample\\Trial\\gikWBC9DOF\\matlab\\+"
                    "gik9dof\\+codegen\\followTrajectory.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739894.50640046294));
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
                emlrtMxCreateString("8mTZukjiY3Ga5AgGIop0yC"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

// End of code generation (_coder_gik9dof_codegen_followTrajectory_info.cpp)
