#!/bin/bash

# Build the workspace first
echo "Building workspace..."
cd /root/workspace
colcon build --packages-select limo_control limo_simulation

# Source the workspace
echo "Sourcing workspace..."
source /root/workspace/install/setup.bash

# Launch the Limo robot simulation with P controller
echo "Starting Limo robot simulation with P controller..."

# Launch the complete system (simulation + controller) in headless mode
ros2 launch limo_control limo_control_headless.launch.py \
    goal_x:=3.0 \
    goal_y:=2.0 \
    kp_linear:=1.5 \
    kp_angular:=2.5

echo "Simulation and controller launched successfully!"
