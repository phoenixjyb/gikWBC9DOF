#!/bin/bash
# Run solver and test in tmux with split panes

SESSION="gik_test"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Create new session
tmux new-session -d -s $SESSION

# Set up environment in the session
tmux send-keys -t $SESSION "cd ~/gikWBC9DOF/ros2" C-m
tmux send-keys -t $SESSION "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION "source install/setup.bash" C-m

# Split window horizontally
tmux split-window -h -t $SESSION

# Set up environment in second pane
tmux send-keys -t $SESSION:0.1 "cd ~/gikWBC9DOF/ros2" C-m
tmux send-keys -t $SESSION:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "source install/setup.bash" C-m

# Left pane: Run solver node
tmux select-pane -t $SESSION:0.0
tmux send-keys -t $SESSION:0.0 "echo '=== SOLVER NODE ===' && ros2 run gik9dof_solver gik9dof_solver_node" C-m

# Wait for solver to start
sleep 3

# Right pane: Run test
tmux select-pane -t $SESSION:0.1
tmux send-keys -t $SESSION:0.1 "cd ~/gikWBC9DOF/validation" C-m
tmux send-keys -t $SESSION:0.1 "echo '=== TEST SCRIPT ===' && sleep 2 && python3 simple_test.py" C-m

# Attach to session
tmux attach-session -t $SESSION
