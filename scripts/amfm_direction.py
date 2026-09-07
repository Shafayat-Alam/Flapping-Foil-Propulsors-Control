#!/usr/bin/env python3
"""Stage 4: thrust along an arbitrary 3D direction, including diagonals.

THE PROBLEM, STATED PROPERLY
----------------------------
Shaping Fx is a special case of shaping the force PROJECTED onto a chosen
unit vector u:

    F_u(t) = Fx(t)*ux + Fy(t)*uy + Fz(t)*uz

For u = (1,0,0) that is the stage-2/3 problem. For a diagonal such as
(1,1,0)/sqrt(2) the SUM of Fx and Fy must carry the target waveform, which
is a different requirement from either component carrying it alone -- two
badly-shaped components can sum to a well-shaped projection, and two
well-shaped components can sum to a poor one.

Three things must hold at once, and they trade against each other:

  1. SHAPE      F_u matches the target waveform.
  2. ALIGNMENT  the force actually points along u -- the off-axis component
                perpendicular to u stays small. Without this, "thrust along
                the diagonal" could be satisfied by a large force pointing
                somewhere else that merely happens to project correctly.
  3. VERTICAL   Fz net stays near zero, at every direction.

STAGE STRUCTURE
---------------
  D1  REACHABILITY.  Before optimising anything, map which directions are
      even reachable. Sweeps a coarse grid of knob settings and records the
      resulting mean force VECTOR, so the achievable cone is measured rather
      than assumed. A direction outside that cone cannot be a closed-loop
      failure -- it was never available -- and the two must not be confused.

  D2  CLOSED LOOP per direction. For each requested u, run the stage-3
      shaper on the projected component, then report shape error, off-axis
      leakage and Fz bias. This is the proof that a direction is achievable,
      or the measurement of how far short it falls.

  D3  DIAGONAL SPECIFICS. The in-plane diagonals are run at finer angular
      spacing than the axes, because that is where Fx and Fy must cooperate
      and where the campaign's existing evidence is weakest -- amp_ratio was
      7.7:1 selective for Fy over Fx, which is partial decoupling, not the
      independent control a clean diagonal needs.

usage:
  amfm_direction.py <folder> --design
  amfm_direction.py <folder> --reach          # D1, open loop
  amfm_direction.py <folder> --closed-loop    # D2/D3, uses amfm_shaper
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))

from amfm_waveform import Knobs, KNOB_NAMES, cycle   # noqa: E402
from amfm_metrics import metrics                     # noqa: E402
import amfm_experiment as EX                         # noqa: E402


def directions_plan():
    """The directions to attempt, with why each is in the list."""
    d = []

    def add(name, vec, group):
        v = np.asarray(vec, float)
        v = v / max(np.linalg.norm(v), 1e-12)
        d.append({"name": name, "group": group,
                  "ux": float(v[0]), "uy": float(v[1]), "uz": float(v[2])})

    # cardinal axes: the baseline, and +X reproduces stage 2/3
    for nm, v in (("+X", (1, 0, 0)), ("-X", (-1, 0, 0)),
                  ("+Y", (0, 1, 0)), ("-Y", (0, -1, 0))):
        add(nm, v, "axis")

    # in-plane sweep at 15 deg: fine enough to see where alignment degrades
    for deg in range(0, 360, 15):
        r = math.radians(deg)
        if deg % 90 == 0:
            continue                        # already covered by the axes
        add(f"XY{deg:03d}", (math.cos(r), math.sin(r), 0.0), "xy_plane")

    # out-of-plane: Fz is the channel we are trying to keep at zero, so
    # asking for thrust WITH a vertical component is the hardest case and
    # the one most likely to expose a conflict with the net-zero constraint
    for deg in (15, 30, 45):
        r = math.radians(deg)
        add(f"XZ{deg:02d}", (math.cos(r), 0.0, math.sin(r)), "out_of_plane")
        add(f"YZ{deg:02d}", (0.0, math.cos(r), math.sin(r)), "out_of_plane")
    return d


def reach_plan():
    """D1: open-loop grid whose measured force vectors map the reachable cone.

    Coarse on purpose. The question is only "which directions does the rig
    produce force in at all", so a handful of well-separated gaits answers it
    far more cheaply than a fine sweep -- and the answer bounds what D2 can
    possibly achieve.
    """
    pts = []
    for a1 in (0.30, 0.45, 0.60):                 # pitch amplitude
        for a2 in (0.20, 0.35, 0.50):             # heave amplitude
            for ph in (0.0, 0.25, 0.5, 0.75):     # relative phase (cycles)
                for hd in (-0.3, 0.0, +0.3):      # differential height
                    pts.append({"A0_1": a1, "A0_2": a2, "phase": ph, "h_diff": hd})
    return pts


def design(folder):
    os.makedirs(folder, exist_ok=True)
    dirs = directions_plan()
    with open(os.path.join(folder, "directions.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(dirs[0]))
        w.writeheader(); w.writerows(dirs)
    reach = reach_plan()
    for i, r in enumerate(reach):
        r["label"] = f"D1_{i:03d}"
    with open(os.path.join(folder, "reach_plan.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(reach[0]))
        w.writeheader(); w.writerows(reach)

    per = (EX.N_CYCLES * EX.PERIOD_S + EX.SETTLE_S
           + EX.PRE_QUIET_S + EX.IDLE_TAIL_S)
    print(f"directions to attempt : {len(dirs)}")
    for g in ("axis", "xy_plane", "out_of_plane"):
        print(f"   {g:14s} {sum(1 for d in dirs if d['group']==g)}")
    print(f"D1 reachability grid  : {len(reach)} missions "
          f"({len(reach)*per/60:.0f} min)")
    print(f"D2/D3 closed loop     : {len(dirs)} directions x ~25 evals "
          f"= ~{len(dirs)*25*per/3600:.1f} h at {per:.0f}s/eval")
    print()
    print("   NOTE: the closed-loop budget is the dominant cost. Run D1 first --")
    print("   directions outside the measured reachable cone should be dropped")
    print("   from D2 rather than spending 25 evaluations discovering they are")
    print("   unreachable.")
    return dirs, reach


def run_reach(folder):
    """D1: open loop over the grid, recording the mean force vector."""
    import amfm_shaper as SH
    rows = list(csv.DictReader(open(os.path.join(folder, "reach_plan.csv"))))
    rig = SH.Rig()
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop(); return 2
    out = []
    try:
        for i, r in enumerate(rows, 1):
            k1 = Knobs(A0=float(r["A0_1"]), n=1, h_diff=float(r["h_diff"]))
            k2 = Knobs(A0=float(r["A0_2"]), n=1)
            paths, err = rig.measure(k1, k2, os.path.join(folder, "data"), r["label"])
            if paths is None:
                print(f"[{i}/{len(rows)}] {r['label']}: {err}")
                continue
            import amfm_analyze as AN
            res = AN.load_force(paths[1])
            if res is None:
                print(f"[{i}/{len(rows)}] {r['label']}: no force")
                continue
            _, fx, fy, fz = res
            mx, my, mz = float(fx.mean()), float(fy.mean()), float(fz.mean())
            mag = math.sqrt(mx*mx + my*my + mz*mz)
            az = math.degrees(math.atan2(my, mx))
            el = math.degrees(math.asin(mz / mag)) if mag > 1e-9 else 0.0
            rec = {**r, "mean_Fx": mx, "mean_Fy": my, "mean_Fz": mz,
                   "mag": mag, "azimuth_deg": az, "elevation_deg": el,
                   "Fx_p2p": float(np.ptp(fx)), "Fy_p2p": float(np.ptp(fy))}
            out.append(rec)
            print(f"[{i}/{len(rows)}] {r['label']}  |F|={mag:6.3f}  "
                  f"az={az:+7.1f} deg  el={el:+6.1f} deg  Fz={mz:+.3f}")
    finally:
        rig.stop()
    if out:
        p = os.path.join(folder, "reachability.csv")
        with open(p, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(out[0]))
            w.writeheader(); w.writerows(out)
        az = np.array([r["azimuth_deg"] for r in out])
        print(f"\nwrote {p}")
        print(f"   azimuth coverage {az.min():+.0f}..{az.max():+.0f} deg "
              f"over {len(out)} gaits")
        gaps = []
        srt = np.sort(az)
        for a, b in zip(srt[:-1], srt[1:]):
            if b - a > 30:
                gaps.append((a, b))
        if gaps:
            print("   UNREACHED azimuth gaps > 30 deg: " +
                  ", ".join(f"{a:+.0f}..{b:+.0f}" for a, b in gaps))
        else:
            print("   no azimuth gap wider than 30 deg")
    return 0


def run_closed_loop(folder, curve="drag", max_evals=25, groups=("axis", "xy_plane")):
    """D2/D3: run the seeded shaper once per direction."""
    dirs = [d for d in csv.DictReader(open(os.path.join(folder, "directions.csv")))
            if d["group"] in groups]
    summary = []
    for i, d in enumerate(dirs, 1):
        sub = os.path.join(folder, f"dir_{d['name']}")
        cmd = [sys.executable, os.path.join(WORKSPACE_ROOT, "scripts", "amfm_shaper.py"),
               sub, "--curve", curve, "--seed", "--max-evals", str(max_evals),
               "--direction", f"{d['ux']},{d['uy']},{d['uz']}"]
        print(f"\n===== [{i}/{len(dirs)}] direction {d['name']} "
              f"({d['ux']:+.2f},{d['uy']:+.2f},{d['uz']:+.2f}) =====")
        subprocess.run(cmd, check=False)
        rp = os.path.join(sub, "result.json")
        if os.path.exists(rp):
            r = json.load(open(rp))
            summary.append({"direction": d["name"], "group": d["group"],
                            **{k: r.get(k) for k in
                               ("err_start", "err_best", "n_evals")}})
    if summary:
        p = os.path.join(folder, "direction_summary.csv")
        with open(p, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summary[0]))
            w.writeheader(); w.writerows(summary)
        print(f"\nwrote {p}")
        print(f"{'direction':12s}{'err start':>11}{'err best':>10}{'evals':>7}")
        for s in summary:
            print(f"{s['direction']:12s}{s['err_start']:11.3f}"
                  f"{s['err_best']:10.3f}{s['n_evals']:7d}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--design", action="store_true")
    ap.add_argument("--reach", action="store_true")
    ap.add_argument("--closed-loop", action="store_true")
    ap.add_argument("--curve", default="drag")
    ap.add_argument("--max-evals", type=int, default=25)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    if a.design:
        design(folder)
    if a.reach:
        return run_reach(folder)
    if a.closed_loop:
        return run_closed_loop(folder, a.curve, a.max_evals)
    return 0


if __name__ == "__main__":
    sys.exit(main())
