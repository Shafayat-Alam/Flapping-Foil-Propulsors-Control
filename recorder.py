#!/usr/bin/env python3
"""
record_session.py - Record ROS2 bag, extract to CSV, log power, and generate plots
Usage: python3 record_session.py <session_name>
"""

import sys
import os
import subprocess
import json
from pathlib import Path
from datetime import datetime
import csv
import threading
import time
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def log_tegrastats(output_file, stop_event):
    """Log tegrastats output to CSV (Jetson only)"""
    try:
        # Check if tegrastats exists (Jetson)
        result = subprocess.run(['which', 'tegrastats'], capture_output=True)
        if result.returncode != 0:
            print("tegrastats not found - skipping power logging (not on Jetson)")
            return
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'raw_tegrastats'])
            
            # Start tegrastats process
            proc = subprocess.Popen(
                ['tegrastats', '--interval', '100'],  # 100ms interval
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print("Logging tegrastats (power/thermal)...")
            
            while not stop_event.is_set():
                line = proc.stdout.readline()
                if not line:
                    break
                
                timestamp_s = time.time()
                dt = datetime.fromtimestamp(timestamp_s)
                human_time = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                timestamp_ms = int(timestamp_s * 1000)
                
                writer.writerow([human_time, timestamp_ms, line.strip()])
                f.flush()  # Ensure data is written
            
            proc.terminate()
            proc.wait()
    
    except Exception as e:
        print(f"Tegrastats logging error: {e}")

def record_bag(bag_path, topics, session_dir):
    """Record ROS2 bag with tegrastats logging"""
    cmd = ["ros2", "bag", "record", "-o", str(bag_path)] + topics
    
    print(f"Recording to: {bag_path}")
    print(f"Topics: {', '.join(topics)}")
    print("Press Ctrl+C to stop recording...")
    
    # Start tegrastats logging thread
    stop_event = threading.Event()
    csv_dir = session_dir / "csv"
    csv_dir.mkdir(exist_ok=True)
    tegra_file = csv_dir / "tegrastats.csv"
    tegra_thread = threading.Thread(
        target=log_tegrastats, 
        args=(tegra_file, stop_event),
        daemon=True
    )
    tegra_thread.start()
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nRecording stopped. Processing bag...")
    finally:
        stop_event.set()
        tegra_thread.join(timeout=2.0)

def extract_bag_to_csv(bag_path, csv_dir):
    """Extract ROS2 bag data to CSV files"""
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from std_msgs.msg import String, Float32MultiArray
    
    # Try mcap first (default in newer ROS2), then sqlite3
    reader = SequentialReader()
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    # Try to open with mcap first
    try:
        storage_options = StorageOptions(uri=str(bag_path), storage_id='mcap')
        reader.open(storage_options, converter_options)
        print("Opened bag with mcap storage")
    except RuntimeError:
        # Fall back to sqlite3
        try:
            storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
            reader.open(storage_options, converter_options)
            print("Opened bag with sqlite3 storage")
        except RuntimeError as e:
            print(f"Error: Could not open bag with mcap or sqlite3: {e}")
            return
    
    # Organize messages by topic
    topic_data = {
        '/mission_input': [],
        '/mission_cmd': [],
        '/mission_status': [],
        '/apriltag_detections': [],
        '/joint_cmd': [],
        '/joint_feedback': [],
        '/telemetry': []
    }
    
    type_map = reader.get_all_topics_and_types()
    type_dict = {t.name: t.type for t in type_map}
    
    print("Extracting messages from bag...")
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        
        if topic not in topic_data:
            continue
        
        # Deserialize message
        msg_type = type_dict[topic]
        if 'String' in msg_type:
            msg = deserialize_message(data, String)
        elif 'Float32MultiArray' in msg_type:
            msg = deserialize_message(data, Float32MultiArray)
        else:
            continue
        
        # Convert timestamp to human readable and ms
        timestamp_s = timestamp / 1e9
        dt = datetime.fromtimestamp(timestamp_s)
        human_time = dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        timestamp_ms = int(timestamp / 1e6)
        
        topic_data[topic].append({
            'timestamp_human': human_time,
            'timestamp_ms': timestamp_ms,
            'msg': msg
        })
    
    # Write CSVs
    print("Writing CSV files...")
    
    # mission_input CSV (raw mission lines fed from the terminal/bash)
    if topic_data['/mission_input']:
        with open(csv_dir / 'mission_input.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'mission'])
            for entry in topic_data['/mission_input']:
                writer.writerow([
                    entry['timestamp_human'],
                    entry['timestamp_ms'],
                    entry['msg'].data
                ])

    # mission_cmd CSV (mission goals dispatched to the controller)
    if topic_data['/mission_cmd']:
        with open(csv_dir / 'mission_cmd.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'label', 'target_tag_id', 'max_retries'])
            for entry in topic_data['/mission_cmd']:
                try:
                    m = json.loads(entry['msg'].data)
                except (json.JSONDecodeError, TypeError):
                    m = {}
                writer.writerow([
                    entry['timestamp_human'],
                    entry['timestamp_ms'],
                    m.get('label', ''),
                    m.get('target_tag_id', ''),
                    m.get('max_retries', ''),
                ])

    # mission_status CSV (interpreted status/events back from the controller)
    if topic_data['/mission_status']:
        with open(csv_dir / 'mission_status.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'label', 'state',
                             'progress', 'roll', 'pitch', 'yaw', 'event'])
            for entry in topic_data['/mission_status']:
                try:
                    s = json.loads(entry['msg'].data)
                except (json.JSONDecodeError, TypeError):
                    s = {}
                ori = s.get('orientation', [None, None, None]) or [None, None, None]
                writer.writerow([
                    entry['timestamp_human'],
                    entry['timestamp_ms'],
                    s.get('label', ''),
                    s.get('state', ''),
                    s.get('progress', ''),
                    ori[0], ori[1], ori[2],
                    s.get('event', ''),
                ])

    # apriltag_detections CSV (one row per detected tag per message)
    if topic_data['/apriltag_detections']:
        with open(csv_dir / 'apriltag_detections.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'tag_id',
                             'distance_m', 'bearing_rad', 'elevation_rad', 'valid'])
            for entry in topic_data['/apriltag_detections']:
                data = entry['msg'].data
                # Format: [tag_id, dist, bearing, elev, valid] per tag
                for i in range(0, len(data), 5):
                    if i + 4 < len(data):
                        writer.writerow([
                            entry['timestamp_human'],
                            entry['timestamp_ms'],
                            data[i], data[i+1], data[i+2], data[i+3], data[i+4]
                        ])
    
    # joint_cmd CSV
    if topic_data['/joint_cmd']:
        with open(csv_dir / 'joint_cmd.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'data'])
            for entry in topic_data['/joint_cmd']:
                writer.writerow([
                    entry['timestamp_human'],
                    entry['timestamp_ms'],
                    ','.join(map(str, entry['msg'].data))
                ])
    
    # joint_feedback CSV
    if topic_data['/joint_feedback']:
        with open(csv_dir / 'joint_feedback.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'servo_id', 'mode', 'pos_rad', 'vel_rad_s', 'curr_A', 'volt_V'])
            for entry in topic_data['/joint_feedback']:
                data = entry['msg'].data
                # Format: [id, mode, pos, vel, curr, volt] per servo
                for i in range(0, len(data), 6):
                    if i + 5 < len(data):
                        writer.writerow([
                            entry['timestamp_human'],
                            entry['timestamp_ms'],
                            data[i],      # servo_id
                            data[i+1],    # mode
                            data[i+2],    # pos_rad
                            data[i+3],    # vel_rad_s
                            data[i+4],    # curr_A
                            data[i+5]     # volt_V
                        ])
    
    # telemetry CSV
    if topic_data['/telemetry']:
        with open(csv_dir / 'telemetry.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_human', 'timestamp_ms', 'seq', 'sample', 'goal', 'pos_rad', 'vel_rad_s', 'curr_A', 'volt_V'])
            for entry in topic_data['/telemetry']:
                data = entry['msg'].data
                if len(data) < 2:
                    continue
                seq = data[0]
                sample = data[1]
                # Format: [seq, sample, goal, pos, vel, curr, volt] per servo
                for i in range(2, len(data), 5):
                    if i + 4 < len(data):
                        writer.writerow([
                            entry['timestamp_human'],
                            entry['timestamp_ms'],
                            seq,
                            sample,
                            data[i],      # goal
                            data[i+1],    # pos_rad
                            data[i+2],    # vel_rad_s
                            data[i+3],    # curr_A
                            data[i+4]     # volt_V
                        ])
    
    print(f"CSV files written to: {csv_dir}")

def generate_plots(csv_dir, plots_dir):
    """Generate master plots and per-command plots from all CSV files"""
    print("Generating plots...")
    
    # Plot telemetry (goal + actual positions, velocities, currents, voltages)
    _generate_telemetry_plots(csv_dir, plots_dir)
    
    # Plot joint_feedback (actual servo data)
    _generate_joint_feedback_plots(csv_dir, plots_dir)
    
    # Plot joint_cmd (final commands sent to servos)
    _generate_joint_cmd_plots(csv_dir, plots_dir)

    # Plot mission status (progress + orientation over the run)
    _generate_mission_status_plots(csv_dir, plots_dir)

    # Plot AprilTag detections (distance + bearing to target over the run)
    _generate_apriltag_plots(csv_dir, plots_dir)

    print(f"Plots saved to: {plots_dir}")

def _generate_telemetry_plots(csv_dir, plots_dir):
    """Generate plots from telemetry.csv"""
    telemetry_file = csv_dir / 'telemetry.csv'
    if not telemetry_file.exists():
        print("  No telemetry.csv found - skipping telemetry plots")
        return
    
    df = pd.read_csv(telemetry_file)
    if df.empty:
        print("  Telemetry CSV is empty - skipping plots")
        return
    
    # Normalize timestamps
    df['time_ms'] = df['timestamp_ms'] - df['timestamp_ms'].min()
    
    # Master plots
    master_dir = plots_dir / 'master'
    master_dir.mkdir(parents=True, exist_ok=True)
    print("  Creating telemetry master plots...")
    _create_telemetry_plot_set(df, master_dir, "Telemetry - Master - All Commands")
    
    # Per-mission plots (grouped by mission sequence number)
    seqs = df['seq'].unique()
    for seq in seqs:
        seq_df = df[df['seq'] == seq]
        seq_dir = plots_dir / f'mission_{int(seq)}'
        seq_dir.mkdir(parents=True, exist_ok=True)
        print(f"  Creating telemetry plots for mission seq={int(seq)}...")
        _create_telemetry_plot_set(seq_df, seq_dir, f"Telemetry - Mission {int(seq)}")

def _generate_joint_feedback_plots(csv_dir, plots_dir):
    """Generate plots from joint_feedback.csv"""
    feedback_file = csv_dir / 'joint_feedback.csv'
    if not feedback_file.exists():
        print("  No joint_feedback.csv found - skipping feedback plots")
        return
    
    df = pd.read_csv(feedback_file)
    if df.empty:
        print("  Joint feedback CSV is empty - skipping plots")
        return
    
    # Normalize timestamps
    df['time_ms'] = df['timestamp_ms'] - df['timestamp_ms'].min()
    
    # Master plots
    master_dir = plots_dir / 'master'
    master_dir.mkdir(parents=True, exist_ok=True)
    print("  Creating joint_feedback master plots...")
    _create_feedback_plot_set(df, master_dir, "Joint Feedback - Master - All Data")

def _generate_joint_cmd_plots(csv_dir, plots_dir):
    """Generate plots from joint_cmd.csv"""
    cmd_file = csv_dir / 'joint_cmd.csv'
    if not cmd_file.exists():
        print("  No joint_cmd.csv found - skipping joint_cmd plots")
        return
    
    df = pd.read_csv(cmd_file)
    if df.empty:
        print("  Joint cmd CSV is empty - skipping plots")
        return
    
    # Normalize timestamps
    df['time_ms'] = df['timestamp_ms'] - df['timestamp_ms'].min()
    
    # Parse data column (comma-separated values)
    # Format: [id0, id1, ..., mode0, mode1, ..., val0, val1, ...]
    print("  Creating joint_cmd master plots...")
    _create_joint_cmd_plot(df, plots_dir / 'master', "Joint Cmd - Master - All Commands")

def _generate_mission_status_plots(csv_dir, plots_dir):
    """Generate plots from mission_status.csv (progress + orientation)."""
    status_file = csv_dir / 'mission_status.csv'
    if not status_file.exists():
        print("  No mission_status.csv found - skipping mission_status plots")
        return

    df = pd.read_csv(status_file)
    if df.empty:
        print("  Mission status CSV is empty - skipping plots")
        return

    df['time_ms'] = df['timestamp_ms'] - df['timestamp_ms'].min()
    master_dir = plots_dir / 'master'
    master_dir.mkdir(parents=True, exist_ok=True)
    print("  Creating mission_status master plots...")
    _create_mission_status_plot_set(df, master_dir, "Mission Status - Master")


def _generate_apriltag_plots(csv_dir, plots_dir):
    """Generate plots from apriltag_detections.csv (distance + bearing per tag)."""
    tag_file = csv_dir / 'apriltag_detections.csv'
    if not tag_file.exists():
        print("  No apriltag_detections.csv found - skipping apriltag plots")
        return

    df = pd.read_csv(tag_file)
    if df.empty:
        print("  AprilTag CSV is empty - skipping plots")
        return

    df['time_ms'] = df['timestamp_ms'] - df['timestamp_ms'].min()
    master_dir = plots_dir / 'master'
    master_dir.mkdir(parents=True, exist_ok=True)
    print("  Creating apriltag master plots...")
    _create_apriltag_plot_set(df, master_dir, "AprilTag Detections - Master")

def _create_telemetry_plot_set(df, output_dir, title_prefix):
    """Create telemetry plots (goal + actual)"""
    
    # Plot 1: Positions
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_ms'], df['pos_rad'], 'b-', alpha=0.8, linewidth=1.5, label='Actual Position')
    plt.plot(df['time_ms'], df['goal'], 'r--', alpha=0.8, linewidth=1.5, label='Goal Position')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Position (rad)', fontsize=11)
    plt.title(f'{title_prefix} - Positions', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'telemetry_positions.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Plot 2: Velocities
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_ms'], df['vel_rad_s'], 'g-', alpha=0.8, linewidth=1.5)
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Velocity (rad/s)', fontsize=11)
    plt.title(f'{title_prefix} - Velocities', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'telemetry_velocities.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Plot 3: Currents
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_ms'], df['curr_A'], 'm-', alpha=0.8, linewidth=1.5)
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Current (A)', fontsize=11)
    plt.title(f'{title_prefix} - Currents', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'telemetry_currents.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Plot 4: Voltages
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_ms'], df['volt_V'], 'c-', alpha=0.8, linewidth=1.5)
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Voltage (V)', fontsize=11)
    plt.title(f'{title_prefix} - Voltages', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'telemetry_voltages.png', dpi=150, bbox_inches='tight')
    plt.close()

def _create_feedback_plot_set(df, output_dir, title_prefix):
    """Create joint_feedback plots (per servo)"""
    
    servo_ids = df['servo_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(servo_ids)))
    
    # Plot 1: Positions (all servos)
    plt.figure(figsize=(12, 6))
    for idx, sid in enumerate(servo_ids):
        servo_df = df[df['servo_id'] == sid]
        plt.plot(servo_df['time_ms'], servo_df['pos_rad'], 
                color=colors[idx], alpha=0.8, linewidth=1.5, 
                label=f'Servo {int(sid)}')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Position (rad)', fontsize=11)
    plt.title(f'{title_prefix} - Positions', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'feedback_positions.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Plot 2: Velocities
    plt.figure(figsize=(12, 6))
    for idx, sid in enumerate(servo_ids):
        servo_df = df[df['servo_id'] == sid]
        plt.plot(servo_df['time_ms'], servo_df['vel_rad_s'], 
                color=colors[idx], alpha=0.8, linewidth=1.5,
                label=f'Servo {int(sid)}')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Velocity (rad/s)', fontsize=11)
    plt.title(f'{title_prefix} - Velocities', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'feedback_velocities.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Plot 3: Currents
    plt.figure(figsize=(12, 6))
    for idx, sid in enumerate(servo_ids):
        servo_df = df[df['servo_id'] == sid]
        plt.plot(servo_df['time_ms'], servo_df['curr_A'], 
                color=colors[idx], alpha=0.8, linewidth=1.5,
                label=f'Servo {int(sid)}')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Current (A)', fontsize=11)
    plt.title(f'{title_prefix} - Currents', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'feedback_currents.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Plot 4: Voltages
    plt.figure(figsize=(12, 6))
    for idx, sid in enumerate(servo_ids):
        servo_df = df[df['servo_id'] == sid]
        plt.plot(servo_df['time_ms'], servo_df['volt_V'], 
                color=colors[idx], alpha=0.8, linewidth=1.5,
                label=f'Servo {int(sid)}')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Voltage (V)', fontsize=11)
    plt.title(f'{title_prefix} - Voltages', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'feedback_voltages.png', dpi=150, bbox_inches='tight')
    plt.close()

def _create_joint_cmd_plot(df, output_dir, title_prefix):
    """Create joint_cmd plot (raw commands)"""
    
    # Plot all data values over time
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_ms'], df['data'].str.len(), 'b-', alpha=0.8, linewidth=1.5)
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Command Data Length', fontsize=11)
    plt.title(f'{title_prefix} - Command Data', fontsize=12, fontweight='bold')
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'joint_cmd_data.png', dpi=150, bbox_inches='tight')
    plt.close()

def _create_mission_status_plot_set(df, output_dir, title_prefix):
    """Create mission_status plots (progress + orientation)."""

    # Plot 1: Mission progress over time
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_ms'], df['progress'], 'b-', alpha=0.8, linewidth=1.5, label='Progress (%)')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Progress (%)', fontsize=11)
    plt.title(f'{title_prefix} - Progress', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'mission_progress.png', dpi=150, bbox_inches='tight')
    plt.close()

    # Plot 2: Body orientation (roll/pitch/yaw) over time
    plt.figure(figsize=(12, 6))
    for col, color in (('roll', 'r'), ('pitch', 'g'), ('yaw', 'b')):
        if col in df.columns:
            plt.plot(df['time_ms'], df[col], color=color, alpha=0.8,
                     linewidth=1.5, label=col)
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Angle (rad)', fontsize=11)
    plt.title(f'{title_prefix} - Orientation', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'mission_orientation.png', dpi=150, bbox_inches='tight')
    plt.close()


def _create_apriltag_plot_set(df, output_dir, title_prefix):
    """Create AprilTag plots (distance + bearing per detected tag)."""

    tag_ids = df['tag_id'].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, max(1, len(tag_ids))))

    # Plot 1: Distance to each tag over time
    plt.figure(figsize=(12, 6))
    for idx, tid in enumerate(tag_ids):
        tag_df = df[df['tag_id'] == tid]
        plt.plot(tag_df['time_ms'], tag_df['distance_m'], color=colors[idx],
                 alpha=0.8, linewidth=1.5, label=f'Tag {int(tid)}')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Distance (m)', fontsize=11)
    plt.title(f'{title_prefix} - Distance', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'apriltag_distance.png', dpi=150, bbox_inches='tight')
    plt.close()

    # Plot 2: Bearing to each tag over time
    plt.figure(figsize=(12, 6))
    for idx, tid in enumerate(tag_ids):
        tag_df = df[df['tag_id'] == tid]
        plt.plot(tag_df['time_ms'], tag_df['bearing_rad'], color=colors[idx],
                 alpha=0.8, linewidth=1.5, label=f'Tag {int(tid)}')
    plt.xlabel('Time (ms)', fontsize=11)
    plt.ylabel('Bearing (rad, +left)', fontsize=11)
    plt.title(f'{title_prefix} - Bearing', fontsize=12, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_dir / 'apriltag_bearing.png', dpi=150, bbox_inches='tight')
    plt.close()


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 record_session.py <session_name>")
        sys.exit(1)
    
    session_name = sys.argv[1]
    session_dir = Path(session_name)
    session_dir.mkdir(parents=True, exist_ok=True)
    
    bag_path = session_dir / "rosbag"
    csv_dir = session_dir / "csv"
    plots_dir = session_dir / "plots"
    
    csv_dir.mkdir(exist_ok=True)
    plots_dir.mkdir(exist_ok=True)
    
    topics = [
        "/mission_input",
        "/mission_cmd",
        "/mission_status",
        "/apriltag_detections",
        "/joint_cmd",
        "/joint_feedback",
        "/telemetry"
    ]
    
    # Record with tegrastats logging
    record_bag(bag_path, topics, session_dir)
    
    # Extract to CSV
    extract_bag_to_csv(bag_path, csv_dir)
    
    # Generate plots
    generate_plots(csv_dir, plots_dir)
    
    print("\nDone!")
    print(f"Session data in: {session_dir}/")
    print(f"  - ROS bag:  {bag_path}/")
    print(f"  - CSV files: {csv_dir}/")
    print(f"  - Plots:     {plots_dir}/")

if __name__ == "__main__":
    main()
