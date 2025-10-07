# Safely Stop Hung Processes on Orin

## Safe Method (Recommended)

Just press **Ctrl+C** in each terminal window where the process is running:

**Terminal 1** (solver node):
```
Press Ctrl+C
```

**Terminal 2** (test script):
```
Press Ctrl+C
```

This only affects processes you started, not other users.

---

## If Ctrl+C Doesn't Work

### Method 1: Kill Only Your Own Processes

```bash
# List only YOUR processes
ps -u $USER | grep gik9dof_solver_node
ps -u $USER | grep "python3.*run_cpp_test"

# Kill only the specific ones (replace PID with actual number from above)
kill <PID_of_solver>
kill <PID_of_python_test>
```

### Method 2: Use Process Name with Your Username

```bash
# Kill only gik9dof_solver_node (safe - you're the only one running it)
pkill -u $USER gik9dof_solver_node

# For Python, be more specific to avoid killing other users' Python processes
pkill -u $USER -f "run_cpp_test_arm64.py"
```

The `-u $USER` flag ensures you only kill **your own** processes, not other users'.

---

## Even Safer: Just Close the Terminals

If you're still concerned:
1. Simply close the two SSH terminal windows
2. The processes will be terminated when the connection drops
3. Open fresh SSH sessions

---

## What's Actually Running

To see what would be affected:
```bash
# See YOUR processes only
ps -u $USER -f | grep -E "gik9dof|python3"
```

This shows you exactly what would be killed before you do it.

---

## Why It's Safe to Kill These

1. **gik9dof_solver_node** - This is your test ROS2 node, no one else is using it
2. **run_cpp_test_arm64.py** - Your validation script, specific filename makes it safe

The `-f "run_cpp_test_arm64.py"` option matches the **full command line**, so it only kills the Python process running that specific script, not other Python processes.

---

## Recommended Action

**Safest approach**:
```bash
# In Terminal 1 and Terminal 2, just press:
Ctrl+C
```

**If that doesn't work**:
```bash
# Kill only the solver (definitely yours, safe)
pkill -u $USER gik9dof_solver_node

# Kill only your specific Python script (safe, won't affect others)
pkill -u $USER -f "run_cpp_test_arm64.py"
```

**If you want to be extra cautious**:
```bash
# Just close the terminal windows and open new ones
# Or disconnect SSH and reconnect
```

All of these methods are safe and won't affect other users' processes! 👍
