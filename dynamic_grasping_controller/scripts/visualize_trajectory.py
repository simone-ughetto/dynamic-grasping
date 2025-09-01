#!/usr/bin/env python3
# filepath: /home/sughetto/my_robot_ws/src/dynamic_grasping_controller/scripts/visualize_trajectory.py

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
from matplotlib.animation import FuncAnimation
from matplotlib import rcParams
from matplotlib.gridspec import GridSpec

# Set smaller font sizes for better readability
rcParams['axes.labelsize'] = 9
rcParams['xtick.labelsize'] = 8
rcParams['ytick.labelsize'] = 8
rcParams['legend.fontsize'] = 8
rcParams['figure.titlesize'] = 12

def set_equal_aspect_3d(ax, X, Y, Z):
    """
    Adjust 3D plot limits to have equal aspect ratio.
    """
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max()
    mid_x = (X.max()+X.min()) * 0.5
    mid_y = (Y.max()+Y.min()) * 0.5
    mid_z = (Z.max()+Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

def draw_orientation_frame(ax, pos, orient, scale=0.03):
    """
    Draw coordinate frame representing orientation at specified position.
    
    Args:
        ax: Matplotlib 3D axes
        pos: Position (x, y, z)
        orient: Orientation as (roll, pitch, yaw) in radians
        scale: Size of the coordinate frame arrows
        
    Returns:
        List of quiver objects representing the orientation frame
    """
    # Convert Euler angles to rotation matrix
    roll, pitch, yaw = orient
    
    # Calculate rotation matrix (using simplistic Euler sequence for visualization)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    R = Rz @ Ry @ Rx  # Combined rotation matrix
    
    # Create axis vectors
    x_axis = R @ np.array([1, 0, 0]) * scale
    y_axis = R @ np.array([0, 1, 0]) * scale
    z_axis = R @ np.array([0, 0, 1]) * scale
    
    # Draw arrows and store the returned objects
    arrows = []
    arrows.append(ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], 
                            color='r', arrow_length_ratio=0.15))
    arrows.append(ax.quiver(pos[0], pos[1], pos[2], y_axis[0], y_axis[1], y_axis[2], 
                            color='g', arrow_length_ratio=0.15))
    arrows.append(ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], 
                            color='b', arrow_length_ratio=0.15))
    
    # Return the arrow objects so they can be removed later
    return arrows

def visualize_trajectory(csv_file):
    """
    Create an interactive visualization of the robot trajectory and object motion.
    """
    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} does not exist!")
        return
    
    # Load data
    print(f"Loading trajectory data from {csv_file}...")
    df = pd.read_csv(csv_file)
    
    # Calculate distance to object and velocity magnitudes
    df['distance'] = np.sqrt((df['x'] - df['obj_x'])**2 + 
                            (df['y'] - df['obj_y'])**2 + 
                            (df['z'] - df['obj_z'])**2)
    df['ee_vel_mag'] = np.sqrt(df['vx']**2 + df['vy']**2 + df['vz']**2)
    df['obj_vel_mag'] = np.sqrt(df['obj_vx']**2 + df['obj_vy']**2 + df['obj_vz']**2)

    # Extract object velocity for constant motion model - use the first non-zero velocity
    # (assuming the object starts with a relatively constant velocity)
    obj_velocities = df[['obj_vx', 'obj_vy', 'obj_vz']].values
    
    # Find the first significant velocity to use for prediction
    obj_vel_magnitudes = np.sqrt(np.sum(obj_velocities**2, axis=1))
    significant_vel_idx = np.argmax(obj_vel_magnitudes > 1e-4)
    
    # Get the constant velocity for our prediction model
    constant_obj_vel = obj_velocities[significant_vel_idx]
    
    # Get initial object position 
    initial_obj_pos = df[['obj_x', 'obj_y', 'obj_z']].iloc[0].values
    
    # Pre-calculate all object positions based on constant velocity model
    # This will replace recorded values or extend beyond them
    obj_positions = np.zeros((len(df), 3))
    
    for i in range(len(df)):
        time = df['time'].iloc[i]
        # Calculate position with constant velocity model: pos = initial_pos + vel * time
        obj_positions[i] = initial_obj_pos + constant_obj_vel * time
    
    # Update the DataFrame with calculated positions
    df['obj_x_model'] = obj_positions[:, 0]
    df['obj_y_model'] = obj_positions[:, 1]
    df['obj_z_model'] = obj_positions[:, 2]
    
    # Calculate model-based distance
    df['distance_model'] = np.sqrt((df['x'] - df['obj_x_model'])**2 + 
                                  (df['y'] - df['obj_y_model'])**2 + 
                                  (df['z'] - df['obj_z_model'])**2)

    #---------------------------------------------------------------------------
    # FIGURE 1: 3D TRAJECTORY PLOT
    #---------------------------------------------------------------------------
    fig_3d = plt.figure(figsize=(10, 8))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    
    # Plot 3D Trajectories - including both actual and modeled object paths
    ax_3d.plot(df['x'], df['y'], df['z'], 'b-', label='End Effector')
    ax_3d.plot(df['obj_x'], df['obj_y'], df['obj_z'], 'r--', alpha=0.5, 
               label='Object (Recorded)')
    ax_3d.plot(df['obj_x_model'], df['obj_y_model'], df['obj_z_model'], 'r-', 
               label='Object (Model)')
    
    # Add velocity vectors at regular intervals
    step = max(1, len(df) // 8)  # Show arrows at about 8 points
    for i in range(0, len(df), step):
        # End effector velocity vector
        ax_3d.quiver(df['x'].iloc[i], df['y'].iloc[i], df['z'].iloc[i],
                   df['vx'].iloc[i]/5, df['vy'].iloc[i]/5, df['vz'].iloc[i]/5,
                   color='blue', length=0.05)
        # Object velocity vector
        ax_3d.quiver(df['obj_x'].iloc[i], df['obj_y'].iloc[i], df['obj_z'].iloc[i],
                   df['obj_vx'].iloc[i]/5, df['obj_vy'].iloc[i]/5, df['obj_vz'].iloc[i]/5,
                   color='red', length=0.05)
        
        # Add orientation frames for the end effector
        if all(col in df.columns for col in ['roll', 'pitch', 'yaw']):
            pos = [df['x'].iloc[i], df['y'].iloc[i], df['z'].iloc[i]]
            orient = [df['roll'].iloc[i], df['pitch'].iloc[i], df['yaw'].iloc[i]]
            draw_orientation_frame(ax_3d, pos, orient)
    
    # Add markers for start and end points
    ax_3d.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], 
                c='green', marker='o', s=100, label='Start')
    ax_3d.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], 
                c='blue', marker='*', s=100, label='End')
    ax_3d.scatter(df['obj_x'].iloc[0], df['obj_y'].iloc[0], df['obj_z'].iloc[0], 
                c='orange', marker='o', s=100)
    ax_3d.scatter(df['obj_x'].iloc[-1], df['obj_y'].iloc[-1], df['obj_z'].iloc[-1], 
                c='red', marker='*', s=100)
    
    ax_3d.set_xlabel('X (m)')
    ax_3d.set_ylabel('Y (m)')
    ax_3d.set_zlabel('Z (m)')
    ax_3d.set_title('3D Trajectory')
    ax_3d.legend(loc='upper right')
    
    # Set equal aspect ratio for the 3D plot
    set_equal_aspect_3d(ax_3d, df['x'], df['y'], df['z'])
    
    # Save 3D plot figure
    trajectory_plot_file = os.path.splitext(csv_file)[0] + '_3d_trajectory.png'
    fig_3d.savefig(trajectory_plot_file)
    print(f"3D trajectory plot saved to {trajectory_plot_file}")

    #---------------------------------------------------------------------------
    # FIGURE 2: DATA PLOTS (Velocity, Orientation, Joints)
    #---------------------------------------------------------------------------
    fig_data = plt.figure(figsize=(16, 12))
    
    # Define grid with 2 rows and 2 columns (removed distance plot)
    gs = GridSpec(2, 2, figure=fig_data, wspace=0.3, hspace=0.3)
    
    # Create subplot axes for data plots (removed distance plot)
    ax_vel = fig_data.add_subplot(gs[0, 0])     # Velocity plot
    ax_orient = fig_data.add_subplot(gs[0, 1])  # Orientation plot
    ax_jpos = fig_data.add_subplot(gs[1, 0])    # Joint positions
    ax_jvel = fig_data.add_subplot(gs[1, 1])    # Joint velocities
    
    # Plot velocity magnitudes
    ax_vel.plot(df['time'], df['ee_vel_mag'], 'g-', label='EE Velocity')
    ax_vel.plot(df['time'], df['obj_vel_mag'], 'r-', label='Object Velocity')
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_title('Velocity Magnitudes')
    ax_vel.legend()
    ax_vel.grid(True)
    
    # Plot orientation data (roll, pitch, yaw)
    if all(col in df.columns for col in ['roll', 'pitch', 'yaw']):
        ax_orient.plot(df['time'], df['roll'], 'r-', label='Roll')
        ax_orient.plot(df['time'], df['pitch'], 'g-', label='Pitch')
        ax_orient.plot(df['time'], df['yaw'], 'b-', label='Yaw')
        ax_orient.set_xlabel('Time (s)')
        ax_orient.set_ylabel('Angle (rad)')
        ax_orient.set_title('End Effector Orientation')
        ax_orient.legend(ncol=3)
        ax_orient.grid(True)
        
        # Show degrees on second y-axis
        ax_orient_deg = ax_orient.twinx()
        ax_orient_deg.plot(df['time'], np.degrees(df['roll']), 'r-', alpha=0)
        ax_orient_deg.plot(df['time'], np.degrees(df['pitch']), 'g-', alpha=0)
        ax_orient_deg.plot(df['time'], np.degrees(df['yaw']), 'b-', alpha=0)
        ax_orient_deg.set_ylabel('Angle (deg)')
    else:
        ax_orient.text(0.5, 0.5, "Orientation data not available in CSV", 
                      horizontalalignment='center', verticalalignment='center',
                      transform=ax_orient.transAxes)
    
    # Plot joint positions
    joint_pos_columns = [col for col in df.columns if col.endswith('_pos')]
    for joint_col in joint_pos_columns:
        joint_name = joint_col.replace('_pos', '')
        ax_jpos.plot(df['time'], df[joint_col], label=joint_name)
    ax_jpos.set_xlabel('Time (s)')
    ax_jpos.set_ylabel('Joint Position (rad)')
    ax_jpos.set_title('Joint Positions')
    ax_jpos.legend(ncol=3, fontsize='small', loc='upper right')
    ax_jpos.grid(True)
    
    # Plot joint velocities
    joint_vel_columns = [col for col in df.columns if col.endswith('_vel') and col not in 
                        ['vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'ee_vel_mag', 'obj_vel_mag']]
    for joint_col in joint_vel_columns:
        joint_name = joint_col.replace('_vel', '')
        ax_jvel.plot(df['time'], df[joint_col], label=joint_name)
    ax_jvel.set_xlabel('Time (s)')
    ax_jvel.set_ylabel('Joint Velocity (rad/s)')
    ax_jvel.set_title('Joint Velocities')
    ax_jvel.legend(ncol=3, fontsize='small', loc='upper right')
    ax_jvel.grid(True)
    
    # Adjust figure spacing
    fig_data.subplots_adjust(left=0.08, right=0.92, bottom=0.08, top=0.92)
    
    # Save data plots figure
    data_plot_file = os.path.splitext(csv_file)[0] + '_data_plots.png'
    fig_data.savefig(data_plot_file)
    print(f"Data plots saved to {data_plot_file}")
    
    #---------------------------------------------------------------------------
    # FIGURE 3: DEDICATED DISTANCE ANALYSIS (model only)
    #---------------------------------------------------------------------------
    fig_dist = plt.figure(figsize=(10, 6))
    ax = fig_dist.add_subplot(111)
    
    # Plot only the model distance (both raw and smoothed)
    if len(df) > 5:  # Only smooth if enough data points
        smooth_dist_model = df['distance_model'].rolling(window=5, center=True).mean()
        ax.plot(df['time'], df['distance_model'], 'r-', alpha=0.4, label='Raw Model Distance')
        ax.plot(df['time'], smooth_dist_model, 'r-', linewidth=2, label='Smoothed Model Distance')
    else:
        ax.plot(df['time'], df['distance_model'], 'r-', linewidth=2, label='Model Distance')
    
    # Add annotations for model distance
    min_model_dist = df['distance_model'].min()
    min_model_dist_time = df['time'][df['distance_model'].idxmin()]
    ax.axhline(y=min_model_dist, color='r', linestyle='--', alpha=0.7)
    ax.scatter(min_model_dist_time, min_model_dist, color='r', s=100, zorder=5)
    ax.annotate(f'Model Min: {min_model_dist:.4f}m at t={min_model_dist_time:.2f}s',
                xy=(min_model_dist_time, min_model_dist), 
                xytext=(min_model_dist_time, min_model_dist*1.4),
                arrowprops=dict(arrowstyle="->", color='r'), ha='center')
    
    # Add start/end annotations
    start_dist = df['distance_model'].iloc[0]
    ax.scatter(0, start_dist, color='k', s=100, zorder=5)
    ax.annotate(f'Start: {start_dist:.4f}m',
                xy=(0, start_dist), xytext=(0, start_dist*1.2),
                arrowprops=dict(arrowstyle="->", color='k'), ha='left')
    
    end_dist = df['distance_model'].iloc[-1]
    end_time = df['time'].iloc[-1]
    ax.scatter(end_time, end_dist, color='k', s=100, zorder=5)
    ax.annotate(f'End: {end_dist:.4f}m',
                xy=(end_time, end_dist), xytext=(end_time, end_dist*1.2),
                arrowprops=dict(arrowstyle="->", color='k'), ha='right')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Distance Between End Effector and Modeled Object Over Time')
    ax.grid(True)
    ax.legend()
    
    # Save distance analysis figure
    dist_analysis_file = os.path.splitext(csv_file)[0] + '_distance_analysis.png'
    fig_dist.savefig(dist_analysis_file)
    print(f"Distance analysis saved to {dist_analysis_file}")
    
    #---------------------------------------------------------------------------
    # FIGURE 4: ANIMATION
    #---------------------------------------------------------------------------
    fig_anim = plt.figure(figsize=(10, 8))
    ax_anim = fig_anim.add_subplot(111, projection='3d')

    ax_anim.set_xlabel('X (m)')
    ax_anim.set_ylabel('Y (m)')
    ax_anim.set_zlabel('Z (m)')
    ax_anim.set_title('Trajectory Animation')

    # Set axis limits with a small margin and equal aspect
    margin = 0.05
    x_all = np.concatenate([df['x'], df['obj_x']])
    y_all = np.concatenate([df['y'], df['obj_y']])
    z_all = np.concatenate([df['z'], df['obj_z']])
    ax_anim.set_xlim([x_all.min()-margin, x_all.max()+margin])
    ax_anim.set_ylim([y_all.min()-margin, y_all.max()+margin])
    ax_anim.set_zlim([z_all.min()-margin, z_all.max()+margin])
    set_equal_aspect_3d(ax_anim, x_all, y_all, z_all)

    # Create point objects for animation
    ee_point, = ax_anim.plot([], [], [], 'bo', markersize=10, label='End Effector')
    obj_point, = ax_anim.plot([], [], [], 'ro', markersize=10, label='Object')
    ee_trail, = ax_anim.plot([], [], [], 'b-', alpha=0.5)
    obj_trail, = ax_anim.plot([], [], [], 'r-', alpha=0.5)

    # Initial quiver objects (using empty arrays)
    initial_pos = np.array([[0, 0, 0]])
    initial_dir = np.array([[0, 0, 0]])
    ee_vel_quiver = ax_anim.quiver(initial_pos[:, 0], initial_pos[:, 1], initial_pos[:, 2],
                                   initial_dir[:, 0], initial_dir[:, 1], initial_dir[:, 2],
                                   length=0.1, color='blue', arrow_length_ratio=0.3)
    obj_vel_quiver = ax_anim.quiver(initial_pos[:, 0], initial_pos[:, 1], initial_pos[:, 2],
                                    initial_dir[:, 0], initial_dir[:, 1], initial_dir[:, 2],
                                    length=0.1, color='red', arrow_length_ratio=0.3)

    # For storing orientation frame arrows
    orientation_arrows = []

    ax_anim.legend()

    def init():
        ee_point.set_data([], [])
        ee_point.set_3d_properties([])
        obj_point.set_data([], [])
        obj_point.set_3d_properties([])
        ee_trail.set_data([], [])
        ee_trail.set_3d_properties([])
        obj_trail.set_data([], [])
        obj_trail.set_3d_properties([])
        # Return all animated elements that need resetting
        return ee_point, obj_point, ee_trail, obj_trail

    def animate(i):
        nonlocal ee_vel_quiver, obj_vel_quiver, orientation_arrows

        # Use frame index 'i' directly, corresponding to the data row
        if i >= len(df):
            # Prevent index out of bounds if animation runs too long
            return ee_point, obj_point, ee_trail, obj_trail, ee_vel_quiver, obj_vel_quiver, *orientation_arrows

        # --- Update End Effector ---
        ee_pos_x = df['x'].iloc[i]
        ee_pos_y = df['y'].iloc[i]
        ee_pos_z = df['z'].iloc[i]
        ee_vel_x = df['vx'].iloc[i]
        ee_vel_y = df['vy'].iloc[i]
        ee_vel_z = df['vz'].iloc[i]

        ee_point.set_data([ee_pos_x], [ee_pos_y])
        ee_point.set_3d_properties([ee_pos_z])
        ee_trail.set_data(df['x'].iloc[:i+1], df['y'].iloc[:i+1])
        ee_trail.set_3d_properties(df['z'].iloc[:i+1])

        # --- Update Object using constant velocity model ---
        obj_pos_x = df['obj_x_model'].iloc[i]
        obj_pos_y = df['obj_y_model'].iloc[i]
        obj_pos_z = df['obj_z_model'].iloc[i]
        
        # Use constant velocity from our model
        obj_vel_x = constant_obj_vel[0]
        obj_vel_y = constant_obj_vel[1]
        obj_vel_z = constant_obj_vel[2]
        
        obj_point.set_data([obj_pos_x], [obj_pos_y])
        obj_point.set_3d_properties([obj_pos_z])
        
        # Use modeled positions for the trail
        obj_trail.set_data(df['obj_x_model'].iloc[:i+1], df['obj_y_model'].iloc[:i+1])
        obj_trail.set_3d_properties(df['obj_z_model'].iloc[:i+1])

        # --- Update Velocity and Orientation Visuals ---
        # Remove and recreate quiver objects to update velocity vectors
        if ee_vel_quiver:
            ee_vel_quiver.remove()
        if obj_vel_quiver:
            obj_vel_quiver.remove()

        # Remove previous orientation arrows
        for arrow_list in orientation_arrows:
            for arrow in arrow_list:
                 if arrow:
                     arrow.remove()
        orientation_arrows = []

        # Recreate EE velocity quiver
        ee_vel_quiver = ax_anim.quiver(
            ee_pos_x, ee_pos_y, ee_pos_z,
            ee_vel_x, ee_vel_y, ee_vel_z,
            color='blue', length=0.1, arrow_length_ratio=0.3, normalize=False
        )

        # Recreate Object velocity quiver
        obj_vel_quiver = ax_anim.quiver(
            obj_pos_x, obj_pos_y, obj_pos_z,
            obj_vel_x, obj_vel_y, obj_vel_z,
            color='red', length=0.1, arrow_length_ratio=0.3, normalize=False
        )

        # Add orientation visualization if available
        if all(col in df.columns for col in ['roll', 'pitch', 'yaw']):
            pos = [ee_pos_x, ee_pos_y, ee_pos_z]
            orient = [df['roll'].iloc[i], df['pitch'].iloc[i], df['yaw'].iloc[i]]
            arrows = draw_orientation_frame(ax_anim, pos, orient, scale=0.05)
            orientation_arrows.append(arrows)

        # Return all animated elements
        flat_orientation_arrows = [item for sublist in orientation_arrows for item in sublist]
        return ee_point, obj_point, ee_trail, obj_trail, ee_vel_quiver, obj_vel_quiver, *flat_orientation_arrows

    try:
        # Use number of data points as frames and dt (0.05s = 50ms) as interval
        dt_ms = 50
        ani = FuncAnimation(fig_anim, animate, frames=len(df),
                           init_func=init, interval=dt_ms, blit=False) # blit=False is often needed for quiver/complex updates
        animation_file = os.path.splitext(csv_file)[0] + '_animation.mp4'
        try:
            ani.save(animation_file, writer='ffmpeg', fps=1000/dt_ms) # Set FPS based on interval
            print(f"Animation saved to {animation_file}")
        except Exception as e:
            print(f"Could not save animation: {e}")
            print("Install ffmpeg to save animations: sudo apt install ffmpeg")
    except Exception as e:
        print(f"Error creating animation: {e}")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize robot trajectory')
    parser.add_argument('csv_file', nargs='?', 
                        default='/home/sughetto/my_robot_ws/trajectory_data.csv',
                        help='Path to trajectory CSV file (default: /home/sughetto/my_robot_ws/trajectory_data.csv)')
    args = parser.parse_args()
    
    visualize_trajectory(args.csv_file)
