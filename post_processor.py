import csv, os, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

TOPIC_MAPS = {
    'joint_feedback':      ['id', 'mode', 'pos_rad', 'vel_rads', 'curr_amps', 'volt_v'],
    'joint_cmd':           ['id', 'mode', 'goal'],
    'telemetry':           ['goal_rad', 'fb_pos_rad', 'fb_vel_rads', 'fb_curr_amps', 'fb_volt_v'],
    'apriltag_detections': ['tag_id', 'dist_m', 'bearing_rad', 'elev_rad', 'valid']
}

def format_time(rel_nanosec):
    seconds = rel_nanosec / 1e9
    return f"{int(seconds // 60):02d}:{seconds % 60:07.4f}"

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 post_processing.py <folder_name>"); sys.exit(1)
    
    folder_path = sys.argv[1]
    print(f"Processing bag in: {folder_path}")
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=folder_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Failed to open bag: {e}"); return

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

if __name__ == "__main__":
    main()