# Code Generation Namespaces Explained

## TL;DR

| Namespace | Status | Purpose | Used By |
|-----------|--------|---------|---------|
| **`+codegen_inuse`** | ✅ **ACTIVE** | Current deployment code | `generate_code_arm64.m`, `generate_code_x86_64.m` |
| **`+codegen`** | ⚠️ **LEGACY** | Old approach (not used) | Old/archived trajectory planning |
| **`codegen/`** (directory) | ✅ **OUTPUT** | Generated C++ code | Build outputs |

---

## Detailed Breakdown

### 1. **`matlab/+gik9dof/+codegen_inuse/`** ✅ ACTIVE

**Purpose**: Modern, optimized approach for real-time deployment

**Key Features:**
- ✅ Builds robot **procedurally** (no file I/O)
- ✅ Code generation compatible
- ✅ Optimized for real-time (MaxTime=50ms, MaxIterations=50)
- ✅ **Currently used** for ARM64 and x86_64 code generation

**Files:**
```
+codegen_inuse/
├── buildRobotForCodegen.m       # Procedural robot builder (no URDF)
├── solveGIKStepWrapper.m        # Main entry point with persistent solver
├── solveGIKStepRealtime.m       # Core IK solve function
├── validate_robot_builder.m     # Validation script
└── generateCodeARM64.m          # Old ARM64 codegen (superseded)
```

**Used By:**
- `generate_code_arm64.m` → Generates ARM NEON code
- `generate_code_x86_64.m` → Generates x86_64 SSE/AVX code
- `RUN_CODEGEN.m` → Validation and codegen workflow
- `RUN_VALIDATION.m` → Pre-codegen validation

**Entry Point:**
```matlab
gik9dof.codegen_inuse.solveGIKStepWrapper(qCurrent, targetPose, distanceLower, distanceWeight)
```

---

### 2. **`matlab/+gik9dof/+codegen/`** ⚠️ LEGACY (NOT USED)

**Purpose**: Old approach using pre-generated robot data

**Why It's Legacy:**
- ❌ Loads robot from **MAT file** (`robotModel.mat`)
- ❌ File I/O incompatible with embedded deployment
- ❌ More complex (requires pre-generation step)
- ❌ Includes trajectory planning features we don't use
- ❌ **Not optimized** for real-time

**Files:**
```
+codegen/
├── loadRobotForCodegen.m        # Loads from robotModel.mat ❌
├── robotModel.mat               # Pre-generated robot data ❌
├── generateRobotModelData.m     # Creates robotModel.mat
├── solveGIKStep.m               # Basic IK solver
├── solveGIKStepWithLock.m       # IK with joint locking
├── stagedFollowTrajectory.m     # Complex trajectory planning ❌
├── stageBPlanPath.m             # Path planning (not used)
├── followTrajectory.m           # Simple trajectory following
└── generateCode.m               # Old codegen script ❌
```

**Used By:**
- `stagedFollowTrajectory.m` - Old trajectory planner (archived)
- `followTrajectory.m` - Old trajectory follower (archived)
- **Nothing in current active deployment**

**Why It Exists:**
- Historical approach from early development
- Used for MATLAB-only simulations
- Kept for reference but not actively maintained

---

### 3. **`codegen/`** (Directory) ✅ OUTPUT DIRECTORY

**Purpose**: Output directory for generated C++ code

**Not a namespace**, just a folder structure:

```
codegen/
├── arm64_realtime/          # ✅ ARM NEON code (ACTIVE - REAL-TIME)
│   ├── GIKSolver.h/cpp
│   └── [~200 generated files]
│
├── x86_64_validation/       # ✅ x86_64 SSE/AVX code (ACTIVE - VALIDATION)
│   ├── GIKSolver.h/cpp
│   └── [~200 generated files]
│
└── html/                    # Code generation reports
    └── report.mldatx
```

---

## Why Two Namespaces?

### Evolution Timeline:

**Phase 1: Early Development** (`+codegen`)
- Used URDF → MAT file → Load in code
- Good for quick iteration in MATLAB
- Not suitable for embedded deployment

**Phase 2: Production** (`+codegen_inuse`)
- Procedural robot building (no files)
- Optimized for real-time constraints
- Fully code-generation compatible
- **This is what we use now**

---

## What Should You Delete?

### ❌ Can Safely Delete (if desired):

```
matlab/+gik9dof/+codegen/        # Entire legacy namespace
```

**But keep for now because:**
- Historical reference
- Possible future trajectory planning features
- Documentation/examples
- Doesn't interfere with active code

### ⚠️ DO NOT DELETE:

```
matlab/+gik9dof/+codegen_inuse/   # ✅ ACTIVE - Used for codegen
codegen/arm64_realtime/              # ✅ Generated code for Orin
codegen/x86_64_realtime/             # ✅ Generated code for WSL
```

---

## Current Active Flow

```
┌─────────────────────────────────────────────────┐
│  generate_code_arm64.m                          │
│  generate_code_x86_64.m                         │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  MATLAB Coder calls:                            │
│  gik9dof.codegen_inuse.solveGIKStepWrapper   │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  buildRobotForCodegen() - Procedural robot      │
│  solveGIKStepRealtime() - IK solver             │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  Generated C++ code:                            │
│  codegen/arm64_realtime/GIKSolver.cpp           │ ← REAL-TIME (Orin)
│  codegen/x86_64_validation/GIKSolver.cpp        │ ← VALIDATION (WSL)
└─────────────────────────────────────────────────┘
```

---

## Legacy Flow (NOT USED)

```
┌─────────────────────────────────────────────────┐
│  generateRobotModelData.m                       │
│  Creates: robotModel.mat                        │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  gik9dof.codegen.loadRobotForCodegen()          │
│  Loads: robotModel.mat ❌ (File I/O)            │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│  gik9dof.codegen.solveGIKStep()                 │
│  No real-time optimizations ❌                  │
└─────────────────────────────────────────────────┘
```

---

## Recommendations

### Keep As-Is ✅
**Don't delete `+codegen`** because:
1. Doesn't interfere with anything
2. Good reference for alternative approaches
3. May be useful for future features
4. Minimal disk space (~50KB)

### Active Maintenance ✅
Focus on **`+codegen_inuse`**:
- ✅ Already optimized (MaxTime=50ms, MaxIterations=50)
- ✅ ARM NEON configured
- ✅ x86_64 SSE/AVX configured
- ✅ Warm-start enabled
- ✅ This is what generates your deployment code

### Documentation ✅
Add a README to clarify:

```matlab
% Create: matlab/+gik9dof/+codegen/README.txt
% 
% LEGACY CODE - NOT USED IN CURRENT DEPLOYMENT
% 
% This namespace uses MAT file loading which is incompatible
% with embedded deployment. Use +codegen_inuse instead.
% 
% For current code generation, see:
%   - generate_code_arm64.m
%   - generate_code_x86_64.m
%   - +codegen_inuse/
```

---

## Summary Table

| Feature | `+codegen` (OLD) | `+codegen_inuse` (NEW) |
|---------|------------------|---------------------------|
| **Robot Loading** | MAT file ❌ | Procedural ✅ |
| **File I/O** | Yes ❌ | No ✅ |
| **Real-time Optimized** | No ❌ | Yes (50ms) ✅ |
| **SIMD Optimized** | Generic ❌ | ARM NEON / x86 SSE ✅ |
| **Warm-start** | No ❌ | Yes ✅ |
| **Currently Used** | No ❌ | **YES** ✅ |
| **Code Generation** | Old method ❌ | **Active** ✅ |

---

**Bottom Line**: Use `+codegen_inuse` for everything. Ignore `+codegen` (it's just legacy code).
