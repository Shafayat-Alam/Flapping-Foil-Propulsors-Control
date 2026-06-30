import subprocess, csv, time, re, os, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

# --- CONFIGURATION ---
TOPIC_MAPS = {
    'joint_feedback':      ['id', 'mode', 'pos_rad', 'vel_rads', 'curr_amps', 'volt_v'],
    'joint_cmd':           ['id', 'mode', 'goal'],
    'telemetry':           ['goal_rad', 'fb_pos_rad', 'fb_vel_rads', 'fb_curr_amps', 'fb_volt_v'],
    'apriltag_detections': ['tag_id', 'dist_m', 'bearing_rad', 'elev_rad', 'valid']
}

def format_time(rel_nanosec):
    seconds = rel_nanosec / 1e9
    return f"{int(seconds // 60):02d}:{seconds % 60:07.4f}"

def process_bag(folder_path):
    print(f"\nStarting post-processing for: {folder_path}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=folder_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Post-processing failed: {e}"); return

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    writers, files, first_ts = {}, {}, None

    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if first_ts is None: first_ts = t
        rel_ns = t - first_ts
        
        msg = deserialize_message(data, get_message(topic_types[topic_name]))
        msg_dict = message_to_ordereddict(msg)
        topic_clean = topic_name.strip('/')

        if topic_name not in writers:
            f = open(os.path.join(folder_path, f"{topic_clean.replace('/', '_')}.csv"), 'w', newline='')
            writer = csv.writer(f)
            headers = ['readable_time', 'time_ms']
            # Header logic (Simplified for conciseness)
            for key, val in msg_dict.items():
                if key == 'layout': continue
                if isinstance(val, (list, tuple)):
                    fields = TOPIC_MAPS.get(topic_clean, [f"{key}"])
                    num = len(val) // len(fields) if topic_clean in TOPIC_MAPS else len(val)
                    for s in range(num): headers.extend([f"{f}_{s}" for f in fields])
                else: headers.append(key)
            writer.writerow(headers)
            writers[topic_name], files[topic_name] = writer, f

        row = [format_time(rel_ns), f"{(rel_ns/1e6):.4f}"]
        for key, val in msg_dict.items():
            if key != 'layout': 
                if isinstance(val, (list, tuple)): row.extend(list(val))
                else: row.append(val)
        writers[topic_name].writerow(row)

    for f in files.values(): f.close()
    print("Post-processing complete.")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 master_recorder.py <folder_name>"); sys.exit(1)

    folder = sys.argv[1]
    os.makedirs(folder, exist_ok=True)
    
    # 1. Start ROS Bag Record (All topics)
    print(f"Recording ROS Bag and Power to: {folder}")
    bag_proc = subprocess.Popen(['ros2', 'bag', 'record', '-a', '-o', folder, '-s', 'mcap'])
    
    # 2. Start Power Logger
    p_log = open(os.path.join(folder, "power_log.csv"), 'w', newline='')
    p_writer = csv.writer(p_log)
    p_writer.writerow(["Timestamp", "Total_Power_mW", "CPU_GPU_mW", "SOC_mW"])
    
    tegrastats = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=subprocess.PIPE, text=True)

    try:
        for line in tegrastats.stdout:
            v_in = re.search(r"VDD_IN (\d+)mW", line)
            v_core = re.search(r"VDD_CPU_GPU_CV (\d+)mW", line)
            v_soc = re.search(r"VDD_SOC (\d+)mW", line)
            if v_in:
                ts = time.strftime("%H:%M:%S")
                p_writer.writerow([ts, v_in.group(1), v_core.group(1) if v_core else "0", v_soc.group(1) if v_soc else "0"])
                p_log.flush()
    except KeyboardInterrupt:
        print("\nStopping recording...")
        tegrastats.terminate(); bag_proc.terminate()
        p_log.close()
        # 3. Process the bag into CSVs
        time.sleep(2) # Give ROS time to close the file
        process_bag(folder)

if __name__ == "__main__":
    main()