# Quick Commands for Orin - Fixed Line Endings

**New Package**: `orin_chassis_follower_20251011_144336.zip`  
**Fix Applied**: Unix line endings (LF) for bash scripts

---

## Commands to Run on Orin

```bash
# 1. SSH to Orin
ssh cr@192.168.100.150

# 2. Clean up old package (if exists)
rm -rf orin_chassis_follower_20251011_132300*

# 3. Extract new package
cd ~
unzip -o orin_chassis_follower_20251011_144336.zip
cd orin_chassis_follower_20251011_144336

# 4. Verify line endings are correct (should show "ASCII text")
file build_on_orin.sh

# 5. Make scripts executable
chmod +x build_on_orin.sh test_on_orin.sh

# 6. Build (takes ~5-6 minutes)
./build_on_orin.sh

# 7. Test
./test_on_orin.sh
```

---

## Troubleshooting

If you still see the `^M` error:

```bash
# Convert line endings manually using dos2unix
sudo apt-get install dos2unix
dos2unix build_on_orin.sh test_on_orin.sh

# Or use sed
sed -i 's/\r$//' build_on_orin.sh
sed -i 's/\r$//' test_on_orin.sh

# Then make executable and run
chmod +x build_on_orin.sh test_on_orin.sh
./build_on_orin.sh
```

---

## Verify Success

After `./build_on_orin.sh` completes, you should see:

```
✅ Build successful!

To use the workspace:
  source install/setup.bash

To launch chassis path follower:
  ros2 launch gik9dof_controllers chassis_path_follower_launch.py
```

---

**Package transferred**: ✅  
**Ready to build on Orin**: ✅
