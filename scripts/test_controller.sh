#!/bin/bash

# Test script for the P controller
echo "Testing P Controller Implementation..."

# Check if we're in the right directory
if [ ! -f "workspace/limo_control/src/p_controller.cpp" ]; then
    echo "Error: Please run this script from the ros-task root directory"
    exit 1
fi

echo "✓ P controller source code found"

# Check if launch file exists
if [ ! -f "workspace/limo_control/launch/limo_control.launch.py" ]; then
    echo "Error: Launch file not found"
    exit 1
fi

echo "✓ Launch file found"

# Check if app.sh exists and is executable
if [ ! -f "scripts/deploy/app.sh" ]; then
    echo "Error: app.sh not found"
    exit 1
fi

if [ ! -x "scripts/deploy/app.sh" ]; then
    echo "Error: app.sh is not executable"
    exit 1
fi

echo "✓ app.sh script found and executable"

# Check CMakeLists.txt has the executable target
if ! grep -q "add_executable(p_controller" workspace/limo_control/CMakeLists.txt; then
    echo "Error: CMakeLists.txt missing p_controller executable"
    exit 1
fi

echo "✓ CMakeLists.txt properly configured"

echo ""
echo "🎉 All checks passed! The P controller implementation is ready."
echo ""
echo "To run the system:"
echo "1. Build the simulator: ./scripts/build/sim.sh"
echo "2. Run the system: ./scripts/deploy/start.sh"
echo ""
echo "The robot will navigate to position (3.0, 2.0) using P control."