#!/usr/bin/env python3
"""
Plot the staged P controller performance to visualize the turn-in-place
followed by drive-to-goal behavior.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def plot_staged_controller(csv_file):
    """Plot XY trajectory highlighting the two-stage behavior"""
    
    # Read the CSV file
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: Could not find {csv_file}")
        return
    
    # Remove rows where robot hasn't moved yet
    df = df[(df['robot_x'] != 0) | (df['robot_y'] != 0) | (df['robot_yaw'] != 0)]
    
    if len(df) == 0:
        print("No movement data found in CSV")
        return
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('Staged P Controller Performance Analysis', fontsize=16, fontweight='bold')
    
    # Plot 1: XY Trajectory with phase annotations
    ax1 = axes[0, 0]
    
    # Calculate where the phase transition likely occurred
    # Transition happens when angular_vel becomes small and linear_vel becomes significant
    if 'linear_vel' in df.columns and 'angular_vel' in df.columns:
        # Find transition point (where linear velocity starts increasing)
        transition_idx = None
        for i in range(len(df)):
            if abs(df['linear_vel'].iloc[i]) > 0.1:  # Linear velocity threshold
                transition_idx = i
                break
        
        if transition_idx and transition_idx > 0:
            # Turn-in-place phase
            ax1.plot(df['robot_x'].iloc[:transition_idx], 
                    df['robot_y'].iloc[:transition_idx],
                    'ro-', linewidth=2, markersize=4, label='Turn-in-Place', alpha=0.7)
            
            # Drive-to-goal phase
            ax1.plot(df['robot_x'].iloc[transition_idx:], 
                    df['robot_y'].iloc[transition_idx:],
                    'b^-', linewidth=2, markersize=4, label='Drive-to-Goal', alpha=0.7)
            
            # Mark transition point
            ax1.plot(df['robot_x'].iloc[transition_idx], 
                    df['robot_y'].iloc[transition_idx],
                    'g*', markersize=20, label='Phase Transition', zorder=5)
        else:
            ax1.plot(df['robot_x'], df['robot_y'], 'b-', linewidth=2, label='Trajectory')
    else:
        ax1.plot(df['robot_x'], df['robot_y'], 'b-', linewidth=2, label='Trajectory')
    
    # Mark start and goal
    ax1.plot(df['robot_x'].iloc[0], df['robot_y'].iloc[0], 'go', markersize=15, 
            label='Start', zorder=5)
    if 'goal_x' in df.columns and 'goal_y' in df.columns:
        goal_x = df['goal_x'].iloc[0]
        goal_y = df['goal_y'].iloc[0]
        ax1.plot(goal_x, goal_y, 'r*', markersize=20, label='Goal', zorder=5)
        
        # Draw circle for goal tolerance
        circle = plt.Circle((goal_x, goal_y), 0.1, color='r', fill=False, 
                          linestyle='--', linewidth=2, alpha=0.5)
        ax1.add_patch(circle)
    
    ax1.set_xlabel('X Position (m)', fontsize=12)
    ax1.set_ylabel('Y Position (m)', fontsize=12)
    ax1.set_title('Robot Trajectory (XY Plot)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')
    ax1.axis('equal')
    
    # Plot 2: Position vs Time
    ax2 = axes[0, 1]
    # Handle both Unix timestamp and relative time formats
    if df['timestamp'].iloc[0] > 1e9:  # Unix timestamp
        time_diff = (df['timestamp'] - df['timestamp'].iloc[0])
        # If timestamps are all the same or not varying much, use row index * 0.1s (10 Hz sampling)
        if time_diff.max() < 0.1:
            time = np.arange(len(df)) * 0.1
        else:
            time = time_diff
    else:
        time = df['timestamp']
    ax2.plot(time, df['robot_x'], 'b-', linewidth=2, label='X Position')
    ax2.plot(time, df['robot_y'], 'r-', linewidth=2, label='Y Position')
    if 'goal_x' in df.columns:
        ax2.axhline(y=df['goal_x'].iloc[0], color='b', linestyle='--', 
                   linewidth=1, alpha=0.5, label='Goal X')
        ax2.axhline(y=df['goal_y'].iloc[0], color='r', linestyle='--', 
                   linewidth=1, alpha=0.5, label='Goal Y')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Position (m)', fontsize=12)
    ax2.set_title('Position vs Time', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best')
    
    # Plot 3: Velocities vs Time
    ax3 = axes[1, 0]
    if 'linear_vel' in df.columns and 'angular_vel' in df.columns:
        ax3.plot(time, df['linear_vel'], 'b-', linewidth=2, label='Linear Velocity')
        ax3.plot(time, df['angular_vel'], 'r-', linewidth=2, label='Angular Velocity')
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        
        # Highlight phase transition
        if transition_idx:
            trans_x = time.iloc[transition_idx] if isinstance(time, pd.Series) else time[transition_idx]
            ax3.axvline(x=trans_x, color='g', linestyle='--', 
                       linewidth=2, alpha=0.7, label='Phase Transition')
    
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Velocity (m/s or rad/s)', fontsize=12)
    ax3.set_title('Control Velocities vs Time', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best')
    
    # Plot 4: Errors vs Time
    ax4 = axes[1, 1]
    if 'euclidean_error' in df.columns and 'orientation_error' in df.columns:
        ax4.plot(time, df['euclidean_error'], 'b-', linewidth=2, 
                label='Distance Error')
        ax4_twin = ax4.twinx()
        ax4_twin.plot(time, np.abs(df['orientation_error']) * 180/np.pi, 'r-', 
                     linewidth=2, label='|Angular Error| (deg)')
        
        # Angular tolerance line
        if transition_idx:
            trans_x = time.iloc[transition_idx] if isinstance(time, pd.Series) else time[transition_idx]
            ax4_twin.axhline(y=2.9, color='orange', linestyle='--', linewidth=2, 
                           alpha=0.7, label='Angular Tolerance (2.9°)')
            ax4.axvline(x=trans_x, color='g', linestyle='--', 
                       linewidth=2, alpha=0.7, label='Phase Transition')
        
        ax4.set_xlabel('Time (s)', fontsize=12)
        ax4.set_ylabel('Distance Error (m)', fontsize=12, color='b')
        ax4_twin.set_ylabel('Angular Error (deg)', fontsize=12, color='r')
        ax4.tick_params(axis='y', labelcolor='b')
        ax4_twin.tick_params(axis='y', labelcolor='r')
    
    ax4.set_title('Tracking Errors vs Time', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    
    # Combine legends
    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4_twin.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    plt.tight_layout()
    
    # Save the plot
    output_file = 'plots/staged_controller_analysis.png'
    os.makedirs('plots', exist_ok=True)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\n✅ Plot saved to: {output_file}")
    
    # Print statistics
    print("\n📊 Performance Statistics:")
    if isinstance(time, pd.Series):
        total_time = time.iloc[-1]
        trans_time = time.iloc[transition_idx] if transition_idx else 0
    else:
        total_time = time[-1]
        trans_time = time[transition_idx] if transition_idx else 0
    
    print(f"   Total runtime: {total_time:.2f} seconds")
    if 'euclidean_error' in df.columns:
        final_error = df['euclidean_error'].iloc[-1]
        print(f"   Final distance error: {final_error:.3f} m")
    if transition_idx:
        print(f"   Phase transition at: {trans_time:.2f} seconds")
        turn_time = trans_time
        drive_time = total_time - turn_time
        print(f"   Turn-in-place duration: {turn_time:.2f} seconds")
        print(f"   Drive-to-goal duration: {drive_time:.2f} seconds")
    
    plt.show()

if __name__ == '__main__':
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'workspace/staged_performance_data.csv'
    plot_staged_controller(csv_file)

