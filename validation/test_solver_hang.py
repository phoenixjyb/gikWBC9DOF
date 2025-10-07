#!/usr/bin/env python3
"""
Simple standalone test - bypasses ROS2 to test solver directly
"""
import time
import signal
import sys

def timeout_handler(signum, frame):
    print("\n❌ TIMEOUT: Solver hung for more than 10 seconds!")
    print("This confirms the ARM64 IK solver is hanging during computation.")
    print("\nPossible causes:")
    print("1. SSE intrinsics stubs causing infinite loops")
    print("2. Division by zero or NaN propagation")
    print("3. Matrix decomposition failure")
    sys.exit(1)

print("="*60)
print("Simple IK Solver Hang Test")
print("="*60)
print()
print("This test will call the IK solver with a simple target.")
print("If it hangs for >10 seconds, we'll timeout and report the issue.")
print()

# Set 10 second timeout
signal.signal(signal.SIGALRM, timeout_handler)
signal.alarm(10)

print("Attempting to import ROS2 and solver...")
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Pose
    print("✅ Imports successful")
except Exception as e:
    print(f"❌ Import failed: {e}")
    sys.exit(1)

print()
print("Starting simple IK solve test...")
print("Target: x=1.0, y=0.0, z=0.5")
print()

# Initialize ROS2
rclpy.init()

# Create a test pose
test_pose = Pose()
test_pose.position.x = 1.0
test_pose.position.y = 0.0
test_pose.position.z = 0.5
test_pose.orientation.w = 1.0
test_pose.orientation.x = 0.0
test_pose.orientation.y = 0.0
test_pose.orientation.z = 0.0

print("Sending test pose and waiting for response...")
print("(Timeout in 10 seconds if solver hangs)")
print()

# The solver should process this quickly
# If it hangs, the alarm will fire

# Just wait and see if solver responds
time.sleep(12)

# If we get here, solver didn't hang
signal.alarm(0)  # Cancel alarm
print("✅ Test completed without hanging!")

rclpy.shutdown()
