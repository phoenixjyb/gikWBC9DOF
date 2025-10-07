#!/bin/bash
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do 
    echo -n "[$i] "
    ps aux | grep gik9dof_solver_node | grep -v grep | awk '{print "CPU: " $3 "%, MEM: " $4 "%"}'
    sleep 1
done
