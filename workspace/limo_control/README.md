# Limo Control Package

This package contains a Proportional (P) controller for the Limo robot.

## Features

- **P Controller**: Implements a proportional controller for robot navigation
- **Goal Tracking**: Navigates the robot to a specified goal position
- **Parameter Tuning**: Configurable gains and limits via ROS parameters
- **Safety Limits**: Maximum velocity limits to prevent unsafe behavior

## Usage

### Launch the complete system (simulation + controller):
```bash
ros2 launch limo_control limo_control.launch.py
```

### Launch with custom parameters:
```bash
ros2 launch limo_control limo_control.launch.py \
    goal_x:=3.0 \
    goal_y:=2.0 \
    kp_linear:=1.5 \
    kp_angular:=2.5
```

## Parameters

- `goal_x` (double, default: 2.0): Target X position
- `goal_y` (double, default: 2.0): Target Y position  
- `kp_linear` (double, default: 1.0): Proportional gain for linear velocity
- `kp_angular` (double, default: 2.0): Proportional gain for angular velocity
- `max_linear_vel` (double, default: 0.5): Maximum linear velocity (m/s)
- `max_angular_vel` (double, default: 1.0): Maximum angular velocity (rad/s)
- `goal_tolerance` (double, default: 0.1): Goal tolerance distance (m)

## Topics

### Subscribed
- `/odom` (nav_msgs/Odometry): Robot odometry data

### Published  
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to the robot

## Algorithm

The P controller calculates:
1. **Distance Error**: Euclidean distance to goal
2. **Angle Error**: Difference between current heading and direction to goal
3. **Control Output**: 
   - Linear velocity = Kp_linear × distance_error
   - Angular velocity = Kp_angular × angle_error

The controller stops when the robot reaches within `goal_tolerance` distance of the goal.