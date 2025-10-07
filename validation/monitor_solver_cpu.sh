#!/bin/bash
# Monitor solver CPU usage while it's running

echo "Monitoring solver CPU usage for 30 seconds..."
echo "If CPU is high (~100%), the solver is actively iterating"
echo "If CPU is low (<5%), the solver is truly hung/deadlocked"
echo ""

for i in {1..30}; do
    echo -n "[$i/30] "
    ps aux | grep "gik9dof_solver_node" | grep -v grep | awk '{print "CPU: " $3 "%, MEM: " $4 "%, TIME: " $10}'
    sleep 1
done

echo ""
echo "Done monitoring"
