#!/usr/bin/env python3
"""
split_missions.py — Split a session CSV into one CSV per mission command
========================================================================
Post-processes the single wide ``session.csv`` produced by record_session.py
and carves it into ONE folder + ONE CSV per mission command.  It does not
touch record_session or the bag — it only reads the already-exported CSV.

For every mission command found in the session, it makes a folder (in the
current directory by default — the workspace root when run from there) and
writes a CSV holding, on every row:

  * the mission command's parameters — kind/gait, velocity, effort of each
    (roll_effort / pitch_effort), pitch phase-shift, cycles, distance, heading,
    ... — plus a derived gait frequency, and
  * that mission's joint_feedback samples, expanded per servo into position,
    velocity, current, voltage and mode columns.

A mission spans from its ``mission_cmd`` row up to the next mission command
(a HOVER command ends recording without starting a new mission).

Usage
-----
  python3 split_missions.py <session_dir_or_csv>
  python3 split_missions.py my_session/                 # finds session.csv inside
  python3 split_missions.py my_session/session.csv
  python3 split_missions.py my_session/ --base-dir missions   # output parent

Output
------
  <base_dir>/<label>_<HHMMSS>/<label>.csv
"""

import argparse
import csv
import json
import math
import os
import re
import sys
from datetime import datetime

# record_session names joint_feedback columns per servo as s<id>_<field>.
# These are the fields we surface (matches record_session's FB_FIELDS names).
FB_NAMES = ['position_rad', 'velocity_rps', 'current_a', 'voltage_v', 'mode']
_SERVO_COL = re.compile(r'^s(\d+)_(' + '|'.join(FB_NAMES) + r')$')

# record_session names load_cell_data columns per sample as s<n>_<axis>.
# Each packet holds several samples, each a 6-axis force/torque reading.
LOADCELL_AXES = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
_LOADCELL_COL = re.compile(r'^s(\d+)_(' + '|'.join(LOADCELL_AXES) + r')$')

# Fallback for older session.csv files that still use raw data.N columns:
# joint_feedback packs 6 floats per servo [id, mode, pos, vel, current, volt].
FB_STRIDE = 6

KNOWN_CMD_KEYS = [
    'kind', 'label', 'frequency', 'pitch_amp', 'heave_amp', 'phase', 'freq_ratio',
    'pitch_k', 'cycles', 'periods', 'distance', 'heading',
    # legacy velocity/effort format — still read back from old sessions
    'velocity', 'effort', 'roll_effort', 'pitch_effort', 'pitch_phase', 'pshift',
    'target_tag_id', 'target_servo_id', 'max_retries',
]

# Allow the CSV's expanded array columns (data.0 ... data.N) to be wide.
csv.field_size_limit(min(sys.maxsize, 2**31 - 1))


def _leaf(source):
    """Topic leaf name, tolerating a leading namespace ('/joint_feedback')."""
    return (source or '').rsplit('/', 1)[-1]


def _resolve_csv(path):
    if os.path.isdir(path):
        cand = os.path.join(path, 'session.csv')
        if not os.path.isfile(cand):
            sys.exit(f"No session.csv in {path}")
        return cand
    if not os.path.isfile(path):
        sys.exit(f"No such file: {path}")
    return path


def _num(v):
    """Parse a CSV cell to float, or None if blank/non-numeric."""
    if v is None or v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _feedback_servos(row):
    """Expand a joint_feedback row into {sid: {field: val}}.

    Reads record_session's named s<id>_<field> columns; falls back to the raw
    data.N layout for older session.csv files.
    """
    servos = {}
    for k, v in row.items():
        m = _SERVO_COL.match(k)
        if m and v not in (None, ''):
            servos.setdefault(int(m.group(1)), {})[m.group(2)] = _num(v)
    if servos:
        return servos

    # Fallback: raw data.N (6 per servo, [id, mode, pos, vel, current, volt]).
    i = 0
    while f'data.{i * FB_STRIDE}' in row:
        sid = _num(row.get(f'data.{i * FB_STRIDE}'))
        if sid is None:
            break
        b = i * FB_STRIDE
        servos[int(round(sid))] = {
            'mode': _num(row.get(f'data.{b + 1}')),
            'position_rad': _num(row.get(f'data.{b + 2}')),
            'velocity_rps': _num(row.get(f'data.{b + 3}')),
            'current_a': _num(row.get(f'data.{b + 4}')),
            'voltage_v': _num(row.get(f'data.{b + 5}')),
        }
        i += 1
    return servos


def _loadcell_samples(row):
    """Expand a load_cell_data row into {sample_idx: {axis: value}}.

    Reads record_session's named s<n>_<axis> columns; falls back to raw data.N
    (6 axes per sample) for older session.csv files.
    """
    samples = {}
    for k, v in row.items():
        m = _LOADCELL_COL.match(k)
        if m and v not in (None, ''):
            samples.setdefault(int(m.group(1)), {})[m.group(2)] = _num(v)
    if samples:
        return samples

    # Blob format: record_session stores an oversized grid as a compact JSON
    # list in the 'data' column (sample-major, 6 axes per sample).
    blob = row.get('data')
    if blob:
        try:
            vals = json.loads(blob)
        except (ValueError, TypeError):
            vals = None
        if vals:
            n = len(LOADCELL_AXES)
            for i in range(len(vals) // n):
                samples[i] = dict(zip(LOADCELL_AXES,
                                      [_num(vals[i * n + j]) for j in range(n)]))
            if samples:
                return samples

    # Fallback: raw data.N, 6 axes per sample in LOADCELL_AXES order.
    i = 0
    while f'data.{i * 6}' in row:
        vals = [_num(row.get(f'data.{i * 6 + j}')) for j in range(6)]
        if all(x is None for x in vals):
            break
        samples[i] = dict(zip(LOADCELL_AXES, vals))
        i += 1
    return samples


def _derive_freq(params, pitch_max_amp):
    """Gait frequency (Hz).  Commands now carry it directly; the velocity
    derivation (freq = peak_stroke_rate / (2π · pitch_max_amp)) is kept only
    to read back sessions recorded under the old velocity/effort format."""
    f = params.get('frequency')
    if isinstance(f, (int, float)):
        return float(f)
    vel = params.get('velocity')
    if isinstance(vel, (int, float)) and pitch_max_amp > 0:
        return float(vel) / (2.0 * math.pi * pitch_max_amp)
    return None


def split(csv_path, base_dir, pitch_max_amp, loadcell_rate):
    with open(csv_path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("session.csv is empty.")

    # Keep the bag's time order (session.csv is written in bag order).
    def _t(r):
        return _num(r.get('bag_time_ns')) or 0.0
    rows.sort(key=_t)

    os.makedirs(base_dir, exist_ok=True)

    # A mission is delimited by mission_cmd rows.  Collect them (in order),
    # each carrying its start time and parsed params; feedback + load-cell rows
    # in between belong to the mission open at that moment.
    segments = []   # {label, params, start_ns, feedback:[...], loadcell:[...]}
    current = None
    n_written = 0

    for r in rows:
        leaf = _leaf(r.get('source'))
        t_ns = _num(r.get('bag_time_ns'))

        if leaf == 'mission_cmd':
            try:
                params = json.loads(r.get('data', ''))
            except (json.JSONDecodeError, TypeError):
                continue
            label = params.get('label') or params.get('kind') or 'mission'
            if str(label).upper() == 'HOVER':
                current = None          # stop directive: close the open mission
                continue
            if current is not None and label == current['label']:
                continue                # same mission re-dispatched (retry)
            current = {'label': label, 'params': params,
                       'start_ns': t_ns, 'feedback': [], 'loadcell': []}
            segments.append(current)

        elif leaf == 'joint_feedback' and current is not None:
            servos = _feedback_servos(r)
            if servos:
                current['feedback'].append((t_ns, r.get('mission_state', ''), servos))

        elif leaf == 'load_cell_data' and current is not None:
            samples = _loadcell_samples(r)
            if samples:
                current['loadcell'].append((t_ns, samples))

    if not segments:
        sys.exit("No mission commands found in the session CSV.")

    for seg in segments:
        n_written += _write_segment(seg, base_dir, pitch_max_amp, loadcell_rate)

    print(f"Wrote {n_written} mission folder(s) under {os.path.abspath(base_dir)}")


def _write_segment(seg, base_dir, pitch_max_amp, loadcell_rate):
    label = seg['label']
    params = seg['params']
    fb = seg['feedback']

    # Coerce list/dict params to compact JSON so they sit in one cell.
    cmd_fields = {k: (json.dumps(v) if isinstance(v, (list, dict)) else v)
                  for k, v in params.items()}
    freq = _derive_freq(params, pitch_max_amp)

    servo_ids = sorted({sid for _, _, servos in fb for sid in servos})

    # Header: annotations, derived frequency, mission params, per-servo columns.
    cmd_cols = [k for k in KNOWN_CMD_KEYS if k in cmd_fields]
    cmd_cols += [k for k in cmd_fields if k not in cmd_cols]
    header = ['time_s', 'bag_time_s', 'mission_label', 'mission_state', 'frequency_hz']
    header += [f'cmd.{k}' for k in cmd_cols]
    for sid in servo_ids:
        header += [f's{sid}_position_rad', f's{sid}_velocity_rps', f's{sid}_current_a',
                   f's{sid}_voltage_v', f's{sid}_mode']

    const = {f'cmd.{k}': v for k, v in cmd_fields.items()}
    const['frequency_hz'] = '' if freq is None else round(freq, 6)

    ts = datetime.fromtimestamp((seg['start_ns'] or 0) / 1e9).strftime('%H%M%S') \
        if seg['start_ns'] else '000000'
    safe = ''.join(c if (c.isalnum() or c in '-_') else '_' for c in str(label))
    folder = os.path.join(base_dir, f"{safe}_{ts}")
    os.makedirs(folder, exist_ok=True)
    out_path = os.path.join(folder, f"{safe}.csv")

    t0 = fb[0][0] if fb else None
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=header, extrasaction='ignore')
        w.writeheader()
        for t_ns, state, servos in fb:
            row = dict(const)
            row['bag_time_s'] = '' if t_ns is None else round(t_ns / 1e9, 6)
            row['time_s'] = '' if (t_ns is None or t0 is None) else round((t_ns - t0) / 1e9, 4)
            row['mission_label'] = label
            row['mission_state'] = state
            for sid, vals in servos.items():
                row[f's{sid}_position_rad'] = vals.get('position_rad')
                row[f's{sid}_velocity_rps'] = vals.get('velocity_rps')
                row[f's{sid}_current_a'] = vals.get('current_a')
                row[f's{sid}_voltage_v'] = vals.get('voltage_v')
                row[f's{sid}_mode'] = vals.get('mode')
            w.writerow(row)

    print(f"  {label}: {len(fb)} feedback samples, {len(servo_ids)} servo(s) → {out_path}")

    # Sibling load-cell CSV (force/torque time series) for the same mission.
    _write_loadcell(seg, folder, safe, loadcell_rate)
    return 1


def _write_loadcell(seg, folder, safe, loadcell_rate):
    """Write this mission's force/torque samples as <label>_loadcell.csv.

    Long format: one row per F/T sample.  Each packet carries several samples
    1/loadcell_rate apart; the packet's bag timestamp is taken as the time of
    its FIRST sample, so sample i sits at packet_time + i/loadcell_rate.
    """
    lc = seg['loadcell']
    if not lc:
        return
    dt = (1.0 / loadcell_rate) if loadcell_rate and loadcell_rate > 0 else 0.0
    header = (['time_s', 'bag_time_s', 'packet_time_s', 'packet_idx',
               'sample_idx', 'mission_label'] + LOADCELL_AXES)

    t0 = lc[0][0]   # first packet time = mission's first F/T sample
    out_path = os.path.join(folder, f"{safe}_loadcell.csv")
    n_rows = 0
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=header, extrasaction='ignore')
        w.writeheader()
        for p_idx, (t_ns, samples) in enumerate(lc):
            s_idxs = sorted(samples)
            n = len(s_idxs)
            # Space this packet's samples evenly across the ACTUAL interval to
            # the next packet, so times stay monotonic.  Fixed 1/loadcell_rate
            # spacing overlapped (packets arrive faster than n_samples/rate),
            # which put samples back-in-time at every boundary and scrambled the
            # downstream harmonic analysis.  Last packet falls back to the rate.
            if (p_idx + 1 < len(lc) and lc[p_idx + 1][0] is not None
                    and t_ns is not None and n > 0):
                step = (lc[p_idx + 1][0] - t_ns) / n
            else:
                step = dt * 1e9
            for k, s_idx in enumerate(s_idxs):
                axes = samples[s_idx]
                t_sample = (t_ns or 0) + k * step
                row = {
                    'time_s': '' if t0 is None else round((t_sample - t0) / 1e9, 9),
                    'bag_time_s': round(t_sample / 1e9, 9),
                    'packet_time_s': '' if t_ns is None else round(t_ns / 1e9, 6),
                    'packet_idx': p_idx,
                    'sample_idx': s_idx,
                    'mission_label': seg['label'],
                }
                for axis in LOADCELL_AXES:
                    row[axis] = axes.get(axis)
                w.writerow(row)
                n_rows += 1
    print(f"      load cell: {len(lc)} packets, {n_rows} F/T samples → {out_path}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder or session.csv from record_session.py')
    ap.add_argument('--base-dir', default='.',
                    help='parent dir for per-mission folders (default: current dir / root)')
    ap.add_argument('--pitch-max-amp', type=float, default=math.pi / 2,
                    help="pitch max amplitude to derive gait frequency from velocity "
                         "(match the controller's pitch_limit; default π/2)")
    ap.add_argument('--loadcell-rate', type=float, default=10000.0,
                    help="load-cell sample rate (Hz) for per-sample time within a "
                         "packet (match the node's sample_rate; default 10000)")
    args = ap.parse_args()

    split(_resolve_csv(args.session), args.base_dir, args.pitch_max_amp,
          args.loadcell_rate)


if __name__ == '__main__':
    main()
