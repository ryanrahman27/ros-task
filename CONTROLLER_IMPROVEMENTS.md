# P Controller Improvements - Staged Approach

## Overview
The P controller has been updated to implement a two-stage approach that ensures the robot first turns to face the goal before moving forward, preventing large cross-track errors and creating more predictable trajectories.

## Problem with Previous Implementation
The previous controller applied both linear and angular velocities simultaneously, causing the robot to move in a potentially wrong direction initially. This resulted in:
- Large cross-track errors
- Inefficient paths
- XY trajectory plots showing diagonal movement instead of turn-then-drive behavior

## New Staged Approach

### Stage 1: Turn-in-Place
**Purpose:** Rotate the robot until it is aligned with the goal

**Behavior:**
- **Linear velocity:** 0 (robot doesn't move forward)
- **Angular velocity:** Proportional to angle error with gain `kp_angular`
- **Condition:** Continues until `|angle_error| <= angular_tolerance`

**Why this works:**
- Ensures the robot is facing the correct direction before moving
- Prevents the robot from moving in the wrong direction initially
- Creates minimal cross-track error

### Stage 2: Drive-to-Goal
**Purpose:** Move forward to the goal while maintaining orientation

**Behavior:**
- **Linear velocity:** Proportional to distance error with gain `kp_linear`
- **Angular velocity:** Small proportional correction with gain `kp_angular_trim` (smaller than `kp_angular`)
- **Condition:** Active once `|angle_error| <= angular_tolerance`

**Why this works:**
- The smaller `kp_angular_trim` ensures smooth orientation adjustments
- Linear motion dominates, creating efficient forward movement
- Small angular corrections prevent orientation drift

## New Parameters

### `kp_angular_trim` (default: 0.5)
- Smaller gain used during drive-to-goal phase
- Purpose: Gentle orientation corrections without disrupting forward motion
- Typical value: 25-50% of `kp_angular`

### `angular_tolerance` (default: 0.05 rad ≈ 2.9°)
- Threshold angle error for switching from turn-in-place to drive-to-goal
- Lower values: More precise initial alignment, longer turn-in-place phase
- Higher values: Faster transition, but may have more cross-track error

## Parameter Tuning Guide

### For Faster Response
```
kp_angular: 3.0-4.0  (faster turning)
angular_tolerance: 0.08-0.1 rad  (earlier transition to driving)
```

### For More Precise Trajectories
```
kp_angular: 2.0  (moderate turning)
angular_tolerance: 0.03-0.05 rad  (more precise alignment before driving)
kp_angular_trim: 0.3-0.5  (gentler orientation corrections)
```

### For Aggressive Driving
```
kp_linear: 1.5-2.0  (faster forward motion)
kp_angular_trim: 0.2-0.3  (minimal orientation trimming)
```

## Expected Behavior

### XY Trajectory
The robot should now exhibit a clear turn-then-drive pattern:
1. **Initial rotation:** Position stays roughly constant while robot rotates
2. **Forward motion:** Nearly straight line toward goal with minimal deviation
3. **Final approach:** Small adjustments to reach goal precisely

### Velocity Commands
- **Turn-in-place phase:** `linear_vel ≈ 0`, `angular_vel = max` initially, decreasing
- **Transition:** Clear log message: "Switching to DRIVE-TO-GOAL phase"
- **Drive-to-goal phase:** `linear_vel` proportional to distance, `angular_vel` small corrections

## Code Changes Summary

### New Member Variables
```cpp
double kp_angular_trim_;       // Trimming gain for drive phase
double angular_tolerance_;     // Threshold for phase switching
bool in_drive_phase_;          // Tracks current phase
```

### Control Logic (lines 101-142)
```cpp
// Stage 1: Turn-in-place until aligned
if (std::abs(angle_error) > angular_tolerance_) {
    angular_vel = kp_angular_ * angle_error;
    linear_vel = 0.0;  // No forward motion
    in_drive_phase_ = false;
}
// Stage 2: Drive to goal with orientation trimming
else {
    linear_vel = kp_linear_ * distance_error;
    angular_vel = kp_angular_trim_ * angle_error;  // Smaller gain
    in_drive_phase_ = true;
}
```

## Testing Recommendations

1. **Visual Verification:**
   - Plot XY trajectory - should show turn-in-place followed by nearly straight drive
   - Observe robot in simulator - should clearly rotate first, then drive

2. **Performance Metrics:**
   - Measure max cross-track error - should be significantly reduced
   - Measure total path length - should be closer to optimal (straight line after rotation)
   - Measure time to goal - may be slightly longer due to complete rotation first

3. **Parameter Tuning:**
   - Start with default values
   - Adjust `angular_tolerance` based on desired alignment precision
   - Tune `kp_angular_trim` to balance stability and responsiveness

## Additional Notes

- The controller now properly sets `has_odom_ = true` in the odometry callback (line 63)
- Informative logging shows phase transitions and current phase
- Debug logging provides detailed information for each phase

## Next Steps for Testing

1. Rebuild the ROS2 workspace:
   ```bash
   cd workspace
   colcon build --packages-select limo_control
   ```

2. Run simulation with new controller:
   ```bash
   # Terminal 1: Start simulation
   ros2 launch limo_simulation limo_headless.launch.py
   
   # Terminal 2: Start controller
   ros2 launch limo_control limo_control_headless.launch.py
   ```

3. Plot the trajectory:
   ```bash
   python3 scripts/plot_performance.py
   ```

4. Observe the clear turn-then-drive behavior in the XY trajectory plot!

## Expected Improvements

- ✅ Clear two-stage behavior: turn-in-place then drive
- ✅ Minimal cross-track error
- ✅ More predictable and efficient trajectories
- ✅ Better alignment before forward motion
- ✅ Smooth transition between phases

