#!/usr/bin/env python3
"""Execute the harmonic control-authority sweep on the rig.

Reads the plan CSV produced by design_harmonic_sweep.py, runs each mission
in the plan's randomised order, and writes one load-cell CSV per mission
plus a manifest. Analysis is a separate step (analyse_harmonic_sweep.py) so
a bad analysis choice never costs rig time twice.

Per mission: 3 cycles commanded, a 4 s quiet delay before each set, and a
2 s idle tail recorded after motion stops. The idle tail is what allows a
per-mission tare -- the previous campaign saved a calibration block for
only 1 of its 35 cells, which forced the whole analysis to be rewritten
around inferring the baseline. Two seconds of quiet per mission is cheap
insurance against repeating that.

Safety: every point is re-checked against the slew and amplitude limits
immediately before it is commanded, even though the plan was generated
inside them -- a hand-edited plan should not be able to drive the servos
past their limits.

usage: run_harmonic_sweep.py <folder> [--blocks A,B,C] [--dry-run]
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))

import design_harmonic_sweep as dhs   # noqa: E402  (limits + kinematics live here)

SETTLE_S = 4.0
IDLE_TAIL_S = 2.0
N_CYCLES = 3


def mission_line(row, label):
    """Build the forward_paddle mission line for one plan row.

    freq_ratio is INVERTED relative to the plan's column: the plan follows
    the 3D campaign's convention (heave/pitch), while the mission line's
    freq_ratio multiplies the pitch frequency to get heave. Getting this
    backwards silently runs the mirror-image gait, so it is done in one
    place and asserted against the plan's own heave_freq_hz below.
    """
    fr_plan = float(row["freq_ratio"])
    mission_fr = fr_plan          # heave_freq = pitch_freq * mission_fr
    pitch_f = float(row["pitch_freq_hz"])
    expect_heave = pitch_f * mission_fr
    assert abs(expect_heave - float(row["heave_freq_hz"])) < 1e-3, (
        f"freq_ratio convention mismatch on {label}: "
        f"{expect_heave:.4f} vs {row['heave_freq_hz']}")

    parts = [
        "forward_paddle",
        f"frequency:{pitch_f:.6f}",
        f"pitch_amp:{float(row['pitch_amp_rad']):.6f}",
        f"heave_amp:{float(row['heave_amp_rad']):.6f}",
        # Heave phase offset. At a rational freq_ratio the two servos lock
        # into a fixed relative pattern, so peak count is set by the lock's
        # denominator and CANNOT be changed by phase -- which makes phase a
        # free parameter for reshaping the waveform at constant peak count.
        # It was pinned at 0 for every mission of the first three campaigns.
        f"phase:{math.radians(float(row.get('phase_deg', 0.0) or 0.0)):.6f}",
        f"freq_ratio:{mission_fr:.6f}",
        "pitch_k:0.000000",     # 0 = plain sine; a3 carries width instead
    ]
    for key, col in (("p_a2", "p_a2"), ("p_phi2", "p_phi2"), ("p_a3", "p_a3"),
                     ("h_a2", "h_a2"), ("h_phi2", "h_phi2"), ("h_a3", "h_a3")):
        v = float(row.get(col, 0.0) or 0.0)
        if key.endswith("phi2"):
            v = math.radians(v)     # plan stores degrees, mission wants radians
        parts.append(f"{key}:{v:.6f}")
    parts.append(f"pitch_bias:{float(row.get('pitch_bias_rad', 0.0) or 0.0):.6f}")
    parts.append(f"cycles:{N_CYCLES}")
    parts.append(f"label:{label}")
    return " ".join(parts)


def check_limits(row, label):
    """Re-verify slew and amplitude for this row. Returns (ok, message)."""
    pa, ha = float(row["pitch_amp_rad"]), float(row["heave_amp_rad"])
    pf, hf = float(row["pitch_freq_hz"]), float(row["heave_freq_hz"])
    bias = float(row.get("pitch_bias_rad", 0.0) or 0.0)
    pv = dhs.peak_velocity(float(row["p_a2"]), float(row["p_phi2"]),
                           float(row["p_a3"]), pa, pf)
    hv = dhs.peak_velocity(float(row["h_a2"]), float(row["h_phi2"]),
                           float(row["h_a3"]), ha, hf)
    if pv > dhs.SLEW_LIMIT or hv > dhs.SLEW_LIMIT:
        return False, f"slew {pv:.2f}/{hv:.2f} > {dhs.SLEW_LIMIT}"
    if abs(bias) + pa > dhs.PITCH_LIMIT or ha > dhs.HEAVE_LIMIT:
        return False, f"amplitude bias{bias:+.2f}+pitch{pa:.2f} or heave {ha:.2f}"
    return True, f"slew {pv:.2f}/{hv:.2f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--blocks", default="",
                    help="comma-separated block prefixes, e.g. A,B,C")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    folder = args.folder if os.path.isabs(args.folder) else \
        os.path.join(WORKSPACE_ROOT, args.folder)
    plan_path = os.path.join(folder, "sweep_plan.csv")
    rows = list(csv.DictReader(open(plan_path)))
    rows.sort(key=lambda r: int(r["run_order"]))

    if args.blocks:
        want = tuple(b.strip() for b in args.blocks.split(","))
        rows = [r for r in rows if r["block"].split("_")[0] in want]
    if args.limit:
        rows = rows[:args.limit]

    data_dir = os.path.join(folder, "data")
    os.makedirs(data_dir, exist_ok=True)

    print(f"plan: {plan_path}")
    print(f"{len(rows)} missions to run "
          f"({', '.join(sorted({r['block'] for r in rows}))})")
    est = len(rows) * (N_CYCLES / 0.35 + SETTLE_S + IDLE_TAIL_S)
    print(f"estimated {est/60:.0f} min\n")

    bad = [(r["label"], check_limits(r, r["label"])[1]) for r in rows
           if not check_limits(r, r["label"])[0]]
    if bad:
        print("REFUSING TO RUN -- these points breach a limit:")
        for lab, why in bad:
            print(f"   {lab}: {why}")
        return 2

    if args.dry_run:
        for r in rows[:5]:
            print(f"[{r['run_order']:>3}] {r['block']:<14} {r['label']}")
            print(f"      {mission_line(r, r['label'])}\n")
        print(f"... ({len(rows)} total). Dry run -- nothing commanded.")
        return 0

    import soft_propulsors_control.motion_command as mc

    node = mc.start_hil_node()
    manifest = []
    try:
        base = node.capture_rest_baseline()
        print(f"rest baseline: {base}\n")

        for i, r in enumerate(rows, 1):
            label = r["label"]
            line = mission_line(r, label)
            ok, why = check_limits(r, label)
            print(f"[{i}/{len(rows)}] {r['block']:<14} {label:<22} {why}")

            # Quiet delay BEFORE arming, so the wake from the previous
            # point has decayed and does not appear in this capture.
            time.sleep(SETTLE_S)

            pitch_f = float(r["pitch_freq_hz"])
            timeout = (N_CYCLES / pitch_f) * 1.6 + 8.0

            node.start_recording()
            ok = node.send(line, label, timeout=timeout)
            # Keep recording through the idle tail: this is the per-mission
            # tare source, and it must be part of the SAME capture as the
            # motion so no drift can creep in between them.
            time.sleep(IDLE_TAIL_S)
            lc_buf, fb_buf = node.stop_recording()

            if not lc_buf:
                print(f"      !! no load-cell samples -- skipping {label}")
                continue
            if not ok:
                print(f"      (no ACHIEVED within {timeout:.0f}s; keeping capture)")

            lc_buf.sort(key=lambda x: x[0])
            arr = np.array([[x[0], x[1], x[2], x[3]] for x in lc_buf], float)
            t = arr[:, 0] - arr[0, 0]

            # Stored RAW: untared and unfiltered. The tare baseline and the
            # resonance cutoff are analysis choices, and writing them into
            # the recording would make them unrevisable without re-running
            # the rig. The idle tail travels with the data so the tare can
            # be recomputed at any time.
            out = os.path.join(data_dir, f"{label}.csv")
            np.savetxt(out, np.column_stack([t, arr[:, 1], arr[:, 2], arr[:, 3]]),
                       delimiter=",", header="t,Fx_raw,Fy_raw,Fz_raw", comments="")
            manifest.append({**r, "csv": os.path.basename(out),
                             "n_samples": len(t), "duration_s": round(t[-1], 3),
                             "achieved": bool(ok)})
            print(f"      -> {len(t)} samples, {t[-1]:.1f}s -> {os.path.basename(out)}")
    finally:
        mc.stop_hil_node(node)
        if manifest:
            mpath = os.path.join(folder, "manifest.csv")
            with open(mpath, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(manifest[0]))
                w.writeheader()
                w.writerows(manifest)
            print(f"\nwrote {mpath} ({len(manifest)} missions)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
