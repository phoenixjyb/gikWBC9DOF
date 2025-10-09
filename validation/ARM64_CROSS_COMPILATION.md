# ARM64 Cross-Compilation Guide for Orin

This guide explains how to cross-compile the GIK validation tools for NVIDIA Jetson Orin (ARM64 architecture).

---

## Prerequisites

### On Development Machine (x86-64)

1. **Install ARM64 cross-compiler**:
```bash
sudo apt-get update
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

2. **Verify installation**:
```bash
aarch64-linux-gnu-gcc --version
aarch64-linux-gnu-g++ --version
```

### On Jetson Orin (Target)

1. **Install required libraries**:
```bash
sudo apt-get update
sudo apt-get install libgomp1 libccd-dev
```

2. **Verify libraries**:
```bash
ldconfig -p | grep gomp
ldconfig -p | grep ccd
```

---

## Cross-Compilation Script

### `build_validation_arm64.sh`

```bash
#!/bin/bash
# Cross-compile GIK validator for ARM64 (Jetson Orin)

set -e

echo "=========================================="
echo "Cross-Compiling GIK Validator for ARM64"
echo "=========================================="
echo ""

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CODEGEN_DIR="${SCRIPT_DIR}/../codegen/gik9dof_arm64_20constraints"
SRC_FILE="${SCRIPT_DIR}/validate_gik_standalone.cpp"
OUTPUT="${SCRIPT_DIR}/validate_gik_standalone_arm64"

# Cross-compiler
CC="aarch64-linux-gnu-gcc"
CXX="aarch64-linux-gnu-g++"

# Check cross-compiler
if ! command -v $CXX &> /dev/null; then
    echo "❌ Error: ARM64 cross-compiler not found"
    echo "   Install: sudo apt-get install g++-aarch64-linux-gnu"
    exit 1
fi

echo "Cross-compiler: $($CXX --version | head -1)"
echo ""

# Find source files
echo "Finding source files..."
CPP_FILES=$(find "$CODEGEN_DIR" -name "*.cpp" \
    -not -path "*/interface/*" \
    -not -path "*/html/*" \
    -not -path "*/examples/*" \
    2>/dev/null)

C_FILES=$(find "$CODEGEN_DIR" -name "*.c" \
    -not -path "*/interface/*" \
    -not -path "*/html/*" \
    -not -path "*/examples/*" \
    2>/dev/null)

CPP_COUNT=$(echo "$CPP_FILES" | wc -l)
C_COUNT=$(echo "$C_FILES" | wc -l)

echo "  Found $CPP_COUNT .cpp files"
echo "  Found $C_COUNT .c files"
echo ""

# Compile
echo "Cross-compiling for ARM64..."
$CXX -o "$OUTPUT" \
    "$SRC_FILE" \
    $CPP_FILES \
    $C_FILES \
    -I"$CODEGEN_DIR" \
    -std=c++14 \
    -O2 \
    -march=armv8-a \
    -mtune=cortex-a78 \
    -Wall \
    -Wno-unused-variable \
    -Wno-unused-but-set-variable \
    -lm \
    -lstdc++ \
    -lpthread \
    -lgomp \
    -lccd \
    -lrt

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    file "$OUTPUT"
    ls -lh "$OUTPUT"
    echo ""
    echo "Deploy to Orin:"
    echo "  scp $OUTPUT orin@<ORIN_IP>:/home/orin/"
    echo "  scp gik_test_cases_20.json orin@<ORIN_IP>:/home/orin/"
    echo ""
    echo "Run on Orin:"
    echo "  ssh orin@<ORIN_IP>"
    echo "  ./validate_gik_standalone_arm64 gik_test_cases_20.json results.json"
else
    echo "❌ Build failed!"
    exit 1
fi
```

---

## Build Process

### 1. Cross-Compile on Development Machine

```bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/validation
chmod +x build_validation_arm64.sh
./build_validation_arm64.sh
```

Expected output:
```
✅ Build successful!
validate_gik_standalone_arm64: ELF 64-bit LSB executable, ARM aarch64, ...
```

### 2. Deploy to Orin

```bash
# Set Orin IP address
ORIN_IP="192.168.1.100"  # Change to your Orin's IP

# Copy binary and test data
scp validate_gik_standalone_arm64 orin@${ORIN_IP}:/home/orin/
scp gik_test_cases_20.json orin@${ORIN_IP}:/home/orin/

# Copy Python analysis script
scp compare_gik_results.py orin@${ORIN_IP}:/home/orin/
```

### 3. Run on Orin

```bash
# SSH to Orin
ssh orin@${ORIN_IP}

# Make executable
chmod +x validate_gik_standalone_arm64

# Run validation
./validate_gik_standalone_arm64 gik_test_cases_20.json gik_results_orin.json

# Analyze results
python3 compare_gik_results.py gik_results_orin.json

# Copy results back
exit
scp orin@${ORIN_IP}:/home/orin/gik_results_orin.json ./
```

---

## Performance Comparison

### Expected Results

| Platform | Architecture | Solve Time | Notes |
|----------|--------------|------------|-------|
| WSL | x86-64 (Intel) | ~20 ms | Development baseline |
| Orin | ARM64 (Cortex-A78) | ~15-25 ms | Production target |

### Factors Affecting Performance

1. **Clock Speed**:
   - Orin: 8-core ARM Cortex-A78 @ 2.2 GHz
   - Development: Varies by CPU

2. **Memory Bandwidth**:
   - Orin: LPDDR5 @ 204.8 GB/s
   - Critical for matrix operations

3. **Cache**:
   - L1: 64KB per core (I), 64KB per core (D)
   - L2: 256KB per core
   - L3: 4MB shared

4. **SIMD**:
   - ARM NEON (128-bit SIMD)
   - Auto-vectorization by compiler

---

## Optimization Flags

### For Speed (-O3 + tuning)

```bash
$CXX -o "$OUTPUT" \
    ... \
    -O3 \
    -march=armv8.2-a \
    -mtune=cortex-a78 \
    -ffast-math \
    -funroll-loops \
    -ftree-vectorize \
    ...
```

### For Size (-Os)

```bash
$CXX -o "$OUTPUT" \
    ... \
    -Os \
    -march=armv8-a \
    ...
```

### For Debug (-O0 + symbols)

```bash
$CXX -o "$OUTPUT" \
    ... \
    -O0 \
    -g \
    -march=armv8-a \
    ...
```

---

## Troubleshooting

### Issue: Cross-compiler not found

```bash
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

### Issue: Missing libraries on Orin

```bash
# On Orin
sudo apt-get install libgomp1 libccd-dev
```

### Issue: Illegal instruction on Orin

Cause: Used too-new ARM features  
Solution: Change `-march=armv8.2-a` to `-march=armv8-a`

### Issue: Segmentation fault on Orin

Possible causes:
1. Stack size too small: `ulimit -s unlimited`
2. Different library versions
3. Alignment issues (rare with ARM64)

Debug:
```bash
# On Orin
gdb ./validate_gik_standalone_arm64
run gik_test_cases_20.json results.json
bt  # backtrace when crash
```

### Issue: Performance worse than expected

Check:
```bash
# CPU frequency scaling
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
# Should be "performance", not "powersave"

# Set to performance mode
sudo nvpmodel -m 0  # Max performance mode
sudo jetson_clocks   # Lock clocks to max
```

---

## Native Compilation on Orin (Alternative)

If cross-compilation has issues, compile natively on Orin:

```bash
# On Orin
cd /home/orin
git clone <your_repo> gikWBC9DOF
cd gikWBC9DOF/validation

# Install build tools
sudo apt-get install build-essential cmake

# Build natively
g++ -o validate_gik_standalone \
    validate_gik_standalone.cpp \
    $(find ../codegen/gik9dof_arm64_20constraints -name "*.cpp" -not -path "*/interface/*" -not -path "*/html/*" -not -path "*/examples/*") \
    $(find ../codegen/gik9dof_arm64_20constraints -name "*.c" -not -path "*/interface/*" -not -path "*/html/*" -not -path "*/examples/*") \
    -I../codegen/gik9dof_arm64_20constraints \
    -std=c++14 -O2 -lm -lstdc++ -lpthread -lgomp -lccd -lrt

# Run
./validate_gik_standalone gik_test_cases_20.json results.json
```

---

## Automated Deployment Script

### `deploy_to_orin.sh`

```bash
#!/bin/bash
# Automated build and deploy to Orin

set -e

ORIN_IP="${1:-192.168.1.100}"
ORIN_USER="${2:-orin}"

echo "Building for ARM64..."
./build_validation_arm64.sh

echo "Deploying to ${ORIN_USER}@${ORIN_IP}..."
scp validate_gik_standalone_arm64 ${ORIN_USER}@${ORIN_IP}:/home/${ORIN_USER}/
scp gik_test_cases_20.json ${ORIN_USER}@${ORIN_IP}:/home/${ORIN_USER}/
scp compare_gik_results.py ${ORIN_USER}@${ORIN_IP}:/home/${ORIN_USER}/

echo "Running tests on Orin..."
ssh ${ORIN_USER}@${ORIN_IP} << 'EOF'
chmod +x validate_gik_standalone_arm64
./validate_gik_standalone_arm64 gik_test_cases_20.json gik_results_orin.json
python3 compare_gik_results.py gik_results_orin.json
EOF

echo "Fetching results..."
scp ${ORIN_USER}@${ORIN_IP}:/home/${ORIN_USER}/gik_results_orin.json ./

echo "✅ Complete! Results: gik_results_orin.json"
```

Usage:
```bash
./deploy_to_orin.sh 192.168.1.100 orin
```

---

## Next Steps

1. **Build ARM64 validator**:
   ```bash
   ./build_validation_arm64.sh
   ```

2. **Deploy to Orin**:
   ```bash
   ./deploy_to_orin.sh <ORIN_IP> <USERNAME>
   ```

3. **Compare results**:
   - x86-64 WSL vs ARM64 Orin
   - Same code, different architecture
   - Check for numerical differences

4. **Optimize if needed**:
   - Profile on Orin
   - Tune compiler flags
   - Consider ARM NEON optimizations

---

**Status**: Ready for implementation  
**Effort**: ~1 hour for first deployment  
**Risk**: Low (standard cross-compilation)
