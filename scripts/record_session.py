#!/usr/bin/env python3
"""
record_session.py — Whole-Run Recorder -> One Annotated CSV
===========================================================
Records *every* live ROS2 topic into a rosbag2 for the duration of a run (plus
Jetson power/thermal from `tegrastats`), then on shutdown flattens all of it
into a single wide CSV — one row per message, every field in its own column,
each row tagged with what the robot was doing at that instant (which mission,
and the mission state: SCANNING / LOCKING / HEADING / STUCK / HOVERING / ...).

It is meant to be launched with the stack and torn down with it: start it when
you `ros2 launch`, and when you Ctrl+C the launch it gets the same signal, stops
recording, and writes the CSV.

Usage
-----
  python3 record_session.py <folder>                  # record until terminated
  python3 record_session.py <folder> --convert        # re-export an existing run
  python3 record_session.py <folder> --include-images # also bag raw camera frames

Output
------
  <folder>/
    bag/                rosbag2 (kept, so --convert can rebuild the CSV)
    tegrastats.log      timestamped raw tegrastats lines (if available)
    session_info.txt    run summary (sources, row counts, time span)
    session.csv         one row per message/sample, columns:
        bag_time_s, bag_time_ns, mission_label, mission_state, mission_event,
        source (topic name or 'tegrastats'), msg_type, then every message field.
        Array fields are expanded element-wise (e.g. data.0, data.1, ...); the
        Dynamixel servo_diagnostics layout is documented in that node.

Mission label/state come from the controller's mission_status topic; each row is
stamped with whichever status was most recent at or before its timestamp.  Rows
before the first status read as label 'idle' / state 'UNKNOWN'.

Note: tegrastats rows are stamped with system (wall) time, so they line up with
the bag on real hardware.  Under use_sim_time the two clocks differ — sort/join
on your own knowledge of the run in that case.
"""

import argparse
import bisect
import csv
import json
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime

STATUS_TOPIC = 'mission_status'

# Sequences longer than this are not expanded element-wise into columns (which
# would explode the CSV width — e.g. a 6000-value load-cell grid → 6000 cols).
MAX_INLINE = 256
# Numeric arrays between MAX_INLINE and this are instead kept COMPACTLY as a
# single JSON-blob column (so the load-cell grid is preserved without widening
# the CSV); arrays larger than this (e.g. camera images) are only summarised by
# their length.  split_missions parses the blob back into samples.
BLOB_MAX = 50000

IMAGE_EXCLUDE_REGEX = '.*image_raw|.*/image(_raw)?(/compressed)?|.*/compressed'
TEGRASTATS_INTERVAL_MS = 500

ANNOT = ['bag_time_s', 'bag_time_ns', 'mission_label', 'mission_state',
         'mission_event', 'source', 'msg_type']

_stop = threading.Event()


# ===========================================================================
# Recording
# ===========================================================================

def _resolve_bag_dir(folder, for_writing):
    if for_writing:
        base = os.path.join(folder, 'bag')
        if not os.path.exists(base):
            return base
        return os.path.join(folder, 'bag_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    candidates = []
    direct = os.path.join(folder, 'bag')
    if os.path.isdir(direct):
        candidates.append(direct)
    if os.path.isdir(folder):
        for name in sorted(os.listdir(folder)):
            path = os.path.join(folder, name)
            if name.startswith('bag') and os.path.isdir(path):
                candidates.append(path)
    for path in candidates:
        if os.path.exists(os.path.join(path, 'metadata.yaml')):
            return path
    return candidates[0] if candidates else direct


def _start_tegrastats(folder):
    """Spawn tegrastats and stream timestamped lines to tegrastats.log."""
    try:
        proc = subprocess.Popen(
            ['tegrastats', '--interval', str(TEGRASTATS_INTERVAL_MS)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, start_new_session=True)
    except FileNotFoundError:
        print("[record_session] tegrastats not found — skipping power monitoring.")
        return None
    log = open(os.path.join(folder, 'tegrastats.log'), 'w')

    def _pump():
        try:
            for line in proc.stdout:
                log.write(f"{time.time_ns()} {line}")
        finally:
            log.flush()
            log.close()

    threading.Thread(target=_pump, daemon=True).start()
    print("[record_session] tegrastats power/thermal logging started.")
    return proc


def record(folder, include_images):
    os.makedirs(folder, exist_ok=True)
    bag_dir = _resolve_bag_dir(folder, for_writing=True)

    cmd = ['ros2', 'bag', 'record', '-a', '-o', bag_dir]
    if not include_images:
        cmd += ['--exclude-regex', IMAGE_EXCLUDE_REGEX]

    print(f"[record_session] recording all topics -> {bag_dir}")
    if not include_images:
        print("[record_session] (excluding raw image topics; use --include-images to keep them)")

    bag_proc = subprocess.Popen(cmd, start_new_session=True)
    teg_proc = _start_tegrastats(folder)

    def _on_signal(signum, _frame):
        _stop.set()
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    while not _stop.is_set():
        if bag_proc.poll() is not None:
            print("[record_session] bag recorder exited unexpectedly.")
            break
        _stop.wait(timeout=0.5)

    print("[record_session] stopping recorder...")
    if teg_proc is not None and teg_proc.poll() is None:
        teg_proc.terminate()
    if bag_proc.poll() is None:
        try:
            bag_proc.send_signal(signal.SIGINT)
            bag_proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            print("[record_session] recorder did not stop in time — terminating.")
            bag_proc.terminate()
            try:
                bag_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bag_proc.kill()

    print("[record_session] exporting CSV...")
    convert(folder, bag_dir=bag_dir)


# ===========================================================================
# Bag reading
# ===========================================================================

def _open_reader(bag_dir):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    conv = ConverterOptions(input_serialization_format='cdr',
                            output_serialization_format='cdr')
    last_err = None
    for storage_id in ('', 'sqlite3', 'mcap'):
        try:
            reader = SequentialReader()
            reader.open(StorageOptions(uri=bag_dir, storage_id=storage_id), conv)
            return reader
        except Exception as e:           # noqa: BLE001 — try next backend
            last_err = e
    raise RuntimeError(f"Could not open bag at {bag_dir}: {last_err}")


def _flatten(d, prefix=''):
    """Flatten nested dicts (dotted keys) and expand arrays element-wise."""
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(_flatten(v, key + '.'))
        elif isinstance(v, (bytes, bytearray)):
            out[key + '.len'] = len(v)
        elif isinstance(v, (list, tuple)):
            if len(v) > MAX_INLINE:
                out[key + '.len'] = len(v)
                # Keep a mid-sized numeric array (the load-cell grid) compactly
                # as one JSON-blob column rather than dropping it; huge arrays
                # (images) stay summarised by length only.
                if len(v) <= BLOB_MAX and v and isinstance(v[0], (int, float)):
                    out[key] = json.dumps(list(v), separators=(',', ':'))
            else:
                for i, item in enumerate(v):
                    if isinstance(item, dict):
                        out.update(_flatten(item, f"{key}.{i}."))
                    else:
                        out[f"{key}.{i}"] = item
        else:
            out[key] = v
    return out


# The Dynamixel node packs these Float32MultiArray topics as fixed-width
# per-servo blocks (the first value of each block is the servo id).  Naming the
# columns turns anonymous data.0/data.1/... into s<id>_<field> so the CSV is
# readable without cross-referencing the node.
FB_FIELDS = ['id', 'mode', 'position_rad', 'velocity_rps', 'current_a', 'voltage_v']
DIAG_FIELDS = ['id', 'mode', 'pwm', 'current_a', 'velocity_rps', 'position_rad',
               'velocity_traj_rps', 'position_traj_rad', 'voltage_v', 'temp_C',
               'moving', 'moving_status', 'hw_error']


def _data_indices(flat):
    """Sorted integer indices of the flattened data.N array cells, if any."""
    idx = []
    for k in flat:
        if k.startswith('data.') and k[5:].isdigit():
            idx.append(int(k[5:]))
    return sorted(idx)


def _without_data(flat):
    return {k: v for k, v in flat.items()
            if not (k.startswith('data.') and k[5:].isdigit())}


def _name_servo_blocks(flat, fields):
    """Rename interleaved per-servo blocks: data.N -> s<id>_<field>."""
    idx = _data_indices(flat)
    if not idx:
        return flat
    stride = len(fields)
    out = _without_data(flat)
    for g in range((max(idx) + 1) // stride):
        b = g * stride
        sid_val = flat.get(f'data.{b}')
        if sid_val is None:
            continue
        sid = int(round(sid_val))
        for j, name in enumerate(fields):
            if name == 'id':
                continue                  # id is encoded in the column prefix
            key = f'data.{b + j}'
            if key in flat:
                out[f's{sid}_{name}'] = flat[key]
    return out


def _name_joint_cmd(flat):
    """joint_cmd is [ids..., modes..., values...]; name by target servo id."""
    idx = _data_indices(flat)
    if not idx:
        return flat
    n = (max(idx) + 1) // 3
    if n == 0:
        return flat
    out = _without_data(flat)
    for i in range(n):
        sid_val = flat.get(f'data.{i}')
        if sid_val is None:
            continue
        sid = int(round(sid_val))
        out[f's{sid}_cmd_mode'] = flat.get(f'data.{n + i}')
        out[f's{sid}_cmd_value'] = flat.get(f'data.{2 * n + i}')
    return out


# load_cell_data is a SAMPLES×AXES grid; each row is one time-step's 6-axis
# force/torque reading in this order (the inner dimension).
LOADCELL_AXES = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']


def _name_load_cell(flat):
    """Name the load-cell grid: data.N -> s<sample>_<axis> (e.g. s0_Fx).

    The grid shape comes from the message's MultiArrayLayout dims (inner =
    'axis').  With the expected 6-axis inner dimension we use the F/T axis
    names; otherwise we fall back to s<sample>_a<axis>, or cell_<N> if there's
    no usable layout.  Layout metadata columns are dropped to keep the CSV clean.
    """
    idx = _data_indices(flat)
    if not idx:
        return flat
    try:
        inner = int(flat.get('layout.dim.1.size'))
    except (TypeError, ValueError):
        inner = 0
    out = {k: v for k, v in flat.items()
           if not (k.startswith('data.') and k[5:].isdigit())
           and not k.startswith('layout.')}
    for k in idx:
        key = f'data.{k}'
        if key not in flat:
            continue
        if inner <= 0:
            out[f'cell_{k}'] = flat[key]
            continue
        sample, axis = divmod(k, inner)
        if inner == len(LOADCELL_AXES):
            out[f's{sample}_{LOADCELL_AXES[axis]}'] = flat[key]
        else:
            out[f's{sample}_a{axis}'] = flat[key]
    return out


def _name_fields(topic, flat):
    """Give known array topics readable per-servo / per-cell column names."""
    leaf = (topic or '').rsplit('/', 1)[-1]
    if leaf == 'joint_feedback':
        return _name_servo_blocks(flat, FB_FIELDS)
    if leaf == 'servo_diagnostics':
        return _name_servo_blocks(flat, DIAG_FIELDS)
    if leaf == 'joint_cmd':
        return _name_joint_cmd(flat)
    if leaf == 'load_cell_data':
        return _name_load_cell(flat)
    return flat


def _decode(type_map, topic, data):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py import message_to_ordereddict
    try:
        msg_cls = get_message(type_map[topic])
        msg = deserialize_message(data, msg_cls)
        return _name_fields(topic, _flatten(message_to_ordereddict(msg)))
    except Exception:                    # noqa: BLE001 — skip undecodable frames
        return None


# ===========================================================================
# Mission timeline
# ===========================================================================

class MissionTimeline:
    def __init__(self):
        self._t, self._labels, self._states, self._events = [], [], [], []

    def add_status(self, t_ns, flat):
        # mission_status is a std_msgs/String whose .data is JSON
        raw = flat.get('data') if flat else None
        try:
            s = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return
        label = s.get('label')
        self._t.append(t_ns)
        self._labels.append('idle' if label in (None, '') else str(label))
        self._states.append(str(s.get('state', 'UNKNOWN')))
        self._events.append(s.get('event') or '')

    def at(self, t_ns):
        if not self._t:
            return ('idle', 'UNKNOWN', '')
        i = bisect.bisect_right(self._t, t_ns) - 1
        if i < 0:
            return ('idle', 'UNKNOWN', '')
        return (self._labels[i], self._states[i], self._events[i])


# ===========================================================================
# tegrastats parsing
# ===========================================================================

_RE_RAM = re.compile(r'RAM (\d+)/(\d+)MB')
_RE_SWAP = re.compile(r'SWAP (\d+)/(\d+)MB')
_RE_GR3D = re.compile(r'GR3D_FREQ (\d+)%')
_RE_EMC = re.compile(r'EMC_FREQ (\d+)%')
_RE_TEMP = re.compile(r'(\w+)@([\d.]+)C')
_RE_CPU = re.compile(r'CPU \[([^\]]*)\]')
_RE_CORE = re.compile(r'(\d+)%@(\d+)')
# Power rails like "VDD_IN 3161/3161" (instant/avg mW); avoid RAM/SWAP "x/yMB"
_RE_RAIL = re.compile(r'([A-Z][A-Z0-9_]+) (\d+)/(\d+)(?!MB)')


def _parse_tegrastats_line(text):
    out = {}
    m = _RE_RAM.search(text)
    if m:
        out['ram.used_mb'], out['ram.total_mb'] = int(m.group(1)), int(m.group(2))
    m = _RE_SWAP.search(text)
    if m:
        out['swap.used_mb'], out['swap.total_mb'] = int(m.group(1)), int(m.group(2))
    m = _RE_GR3D.search(text)
    if m:
        out['gpu.gr3d_pct'] = int(m.group(1))
    m = _RE_EMC.search(text)
    if m:
        out['emc.pct'] = int(m.group(1))
    for name, temp in _RE_TEMP.findall(text):
        out[f'temp.{name}_C'] = float(temp)
    m = _RE_CPU.search(text)
    if m:
        for i, core in enumerate(m.group(1).split(',')):
            cm = _RE_CORE.search(core)
            if cm:
                out[f'cpu.{i}.pct'] = int(cm.group(1))
                out[f'cpu.{i}.freq_mhz'] = int(cm.group(2))
    for name, inst, avg in _RE_RAIL.findall(text):
        out[f'pwr.{name}.inst_mw'] = int(inst)
        out[f'pwr.{name}.avg_mw'] = int(avg)
    return out


def _read_tegrastats(folder, timeline):
    """Return (rows, columns); each row is (t_ns, dict) annotated by mission."""
    path = os.path.join(folder, 'tegrastats.log')
    rows, cols = [], set()
    if not os.path.exists(path):
        return rows, cols
    with open(path) as f:
        for line in f:
            parts = line.split(' ', 1)
            if len(parts) < 2:
                continue
            try:
                t_ns = int(parts[0])
            except ValueError:
                continue
            fields = _parse_tegrastats_line(parts[1])
            if not fields:
                continue
            label, state, event = timeline.at(t_ns)
            row = {
                'bag_time_s': t_ns / 1e9, 'bag_time_ns': t_ns,
                'mission_label': label, 'mission_state': state, 'mission_event': event,
                'source': 'tegrastats', 'msg_type': 'tegrastats',
            }
            row.update(fields)
            rows.append((t_ns, row))
            cols.update(fields.keys())
    rows.sort(key=lambda r: r[0])
    return rows, cols


# ===========================================================================
# Conversion
# ===========================================================================

def convert(folder, bag_dir=None):
    if bag_dir is None:
        bag_dir = _resolve_bag_dir(folder, for_writing=False)
    if not os.path.isdir(bag_dir):
        print(f"[record_session] no bag found in {folder} — nothing to convert.")
        return

    # --- Pass 1: mission timeline + the union of all field columns -----------
    timeline = MissionTimeline()
    field_cols = set()
    reader = _open_reader(bag_dir)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        flat = _decode(type_map, topic, data)
        if flat is None:
            continue
        if topic == STATUS_TOPIC:
            timeline.add_status(t_ns, flat)
        field_cols.update(flat.keys())
    del reader

    teg_rows, teg_cols = _read_tegrastats(folder, timeline)
    field_cols.update(teg_cols)
    header = ANNOT + sorted(field_cols)

    # --- Pass 2: stream every message into the single CSV, merging tegrastats -
    counts = {}
    t_first = t_last = None
    teg_i = 0
    out_path = os.path.join(folder, 'session.csv')
    reader = _open_reader(bag_dir)
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header, extrasaction='ignore')
        writer.writeheader()
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            # Emit any tegrastats samples that occurred at/before this message
            while teg_i < len(teg_rows) and teg_rows[teg_i][0] <= t_ns:
                writer.writerow(teg_rows[teg_i][1])
                counts['tegrastats'] = counts.get('tegrastats', 0) + 1
                teg_i += 1

            flat = _decode(type_map, topic, data)
            if flat is None:
                continue
            t_first = t_ns if t_first is None else t_first
            t_last = t_ns
            label, state, event = timeline.at(t_ns)
            row = {
                'bag_time_s': t_ns / 1e9, 'bag_time_ns': t_ns,
                'mission_label': label, 'mission_state': state, 'mission_event': event,
                'source': topic, 'msg_type': type_map.get(topic, ''),
            }
            row.update(flat)
            writer.writerow(row)
            counts[topic] = counts.get(topic, 0) + 1
        # Flush any remaining tegrastats samples after the last message
        while teg_i < len(teg_rows):
            writer.writerow(teg_rows[teg_i][1])
            counts['tegrastats'] = counts.get('tegrastats', 0) + 1
            teg_i += 1
    del reader

    _write_summary(folder, bag_dir, type_map, counts, t_first, t_last)
    total = sum(counts.values())
    print(f"[record_session] wrote {total} rows from {len(counts)} sources -> {out_path}")


def _write_summary(folder, bag_dir, type_map, counts, t_first, t_last):
    lines = [
        "Session recording summary",
        "=========================",
        f"exported_at : {datetime.now().isoformat(timespec='seconds')}",
        f"bag         : {bag_dir}",
    ]
    if t_first is not None:
        span = (t_last - t_first) / 1e9
        lines += [
            f"start       : {datetime.fromtimestamp(t_first / 1e9).isoformat(timespec='seconds')}",
            f"end         : {datetime.fromtimestamp(t_last / 1e9).isoformat(timespec='seconds')}",
            f"duration_s  : {span:.1f}",
        ]
    lines += ["", f"{'source':<28} {'type':<40} {'rows':>10}", "-" * 80]
    sources = set(type_map) | set(counts)
    for src in sorted(sources):
        lines.append(f"{src:<28} {type_map.get(src, 'tegrastats'):<40} {counts.get(src, 0):>10}")
    with open(os.path.join(folder, 'session_info.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', help='output folder for this session')
    ap.add_argument('--convert', action='store_true',
                    help='skip recording; rebuild the CSV from an existing bag')
    ap.add_argument('--include-images', action='store_true',
                    help='also record raw camera image topics (large)')
    args = ap.parse_args()

    try:
        if args.convert:
            convert(args.folder)
        else:
            record(args.folder, include_images=args.include_images)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
