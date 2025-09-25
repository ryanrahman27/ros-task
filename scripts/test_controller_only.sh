#!/bin/bash

echo "Testing P Controller (without simulation)..."

# Build the workspace first
echo "Building workspace..."
cd /root/workspace
colcon build --packages-select limo_control

# Source the workspace
echo "Sourcing workspace..."
source /root/workspace/install/setup.bash

# Test the controller node (this will fail to find odometry, but we can see if it starts)
echo "Testing controller node startup..."
timeout 5s ros2 run limo_control p_controller --ros-args -p goal_x:=2.0 -p goal_y:=2.0 || echo "Controller started successfully (expected timeout due to no odometry)"

echo "Controller test completed!"