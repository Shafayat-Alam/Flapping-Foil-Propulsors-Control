#!/usr/bin/env python3
"""Stage 4, run against the thrust objective instead of shape-matching.

`amfm_direction.py`'s D2/D3 called amfm_shaper.py with --curve/--seed -- the
shape-matching objective that stalled in stages 2/3/3-both. Everything that
actually worked this session (thrust, thrust2) used --objective thrust
instead: reward net force along the requested axis, hinge-penalise whatever
survives off-axis. That generalises directly to 3D -- "off-axis" is just
"perpendicular to u" instead of "the other in-plane channel" -- so this reuses
the SAME objective for all twelve directions rather than the one that failed.

DIRECTIONS: exactly the twelve requested -- six cardinal axes and six
face-diagonals, no in-plane 15-degree sweep and no out-of-plane arcs (that was
the original 30-direction D2/D3 scope; this is a smaller, explicitly chosen
set).

STARTING POINT: nominal, the same for every direction. Warm-starting all
twelve from the +X-optimised thrust2 result was considered and rejected: for
directions like -X that shape is closer to the WRONG answer than to nominal
(it already produces strong force in the opposite sense), and coordinate
descent flipping a whole waveform's dominant sign one knob at a time is a
harder search than climbing from zero. Nominal is a fair, unbiased start for
all twelve, at the cost of a shallower search within a fixed eval budget --
the same trade stage 2 made against stage 3's seeding, made explicit here
because there is no equivalent seed for an arbitrary 3D direction.

usage:  stage4_directions.py <folder> [--max-evals 40]
"""
from __future__ import annotations

import csv
import json
import math
import os
import subprocess
import sys

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX   # noqa: E402

S2 = 1.0 / math.sqrt(2.0)
S3 = 1.0 / math.sqrt(3.0)
CARDIAG = [       # 1D axes + 2D face-diagonals -- the original 12
    ("+Fx",   (1, 0, 0)),   ("-Fx",   (-1, 0, 0)),
    ("+Fy",   (0, 1, 0)),   ("-Fy",   (0, -1, 0)),
    ("+Fz",   (0, 0, 1)),   ("-Fz",   (0, 0, -1)),
    ("+FxFy", (S2, S2, 0)), ("-FxFy", (-S2, -S2, 0)),
    ("+FxFz", (S2, 0, S2)), ("-FxFz", (-S2, 0, -S2)),
    ("+FyFz", (0, S2, S2)), ("-FyFz", (0, -S2, -S2)),
]
CORNERS = [       # 3D diagonals -- all three axes at once, the cube corners
    (f"{'+' if sx>0 else '-'}Fx{'+' if sy>0 else '-'}Fy{'+' if sz>0 else '-'}Fz",
     (sx*S3, sy*S3, sz*S3))
    for sx in (1, -1) for sy in (1, -1) for sz in (1, -1)
]
DIRECTIONS = CARDIAG   # default; overridden by --which in main()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--max-evals", type=int, default=40)
    ap.add_argument("--which", default="cardiag", choices=["cardiag", "corners", "all"])
    ap.add_argument("--only", default=None,
                    help="comma-separated direction names, e.g. '-Fx,-FxFy'. "
                         "NOT nargs='*': a name like '-Fx' is indistinguishable "
                         "from an option flag to argparse once it is its own "
                         "token, which is exactly the bug this flag exists to "
                         "recover from -- the retry pass hit it a second time "
                         "before this was caught.")
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)
    pool = dict(CARDIAG + CORNERS)
    if a.only:
        directions = [(n, pool[n]) for n in a.only.split(",") if n in pool]
    else:
        directions = {"cardiag": CARDIAG, "corners": CORNERS,
                      "all": CARDIAG + CORNERS}[a.which]

    per = EX.N_CYCLES * EX.PERIOD_S + EX.SETTLE_S + EX.PRE_QUIET_S + EX.IDLE_TAIL_S
    total = len(directions) * a.max_evals
    print(f"{len(directions)} directions x {a.max_evals} evals = "
          f"~{total} missions, ~{total*per/60:.0f} min\n")

    summary = []
    for i, (name, u) in enumerate(directions, 1):
        sub = os.path.join(folder, f"dir_{name}")
        cmd = [sys.executable, os.path.join(WORKSPACE_ROOT, "scripts", "amfm_shaper.py"),
               sub, "--objective", "thrust", "--both-servos",
               "--max-evals", str(a.max_evals),
               f"--direction={u[0]:.6f},{u[1]:.6f},{u[2]:.6f}"]
        # The '=' form is required, not cosmetic: argparse treats a bare
        # "-1.000000,0.000000,0.000000" value as an unrecognised OPTION
        # (leading '-') rather than --direction's argument, and every
        # direction with a negative first component (half of the twelve)
        # failed with "expected one argument" until this was caught mid-run.
        print(f"\n===== [{i}/{len(directions)}] direction {name} "
              f"({u[0]:+.3f},{u[1]:+.3f},{u[2]:+.3f}) =====", flush=True)
        subprocess.run(cmd, check=False)
        rp = os.path.join(sub, "result.json")
        if os.path.exists(rp):
            r = json.load(open(rp))
            # err = -net/scale + penalty, so recovering net force from the
            # error alone would need the penalty backed out; read it straight
            # from history instead.
            hp = os.path.join(sub, "history.csv")
            net_best, off_best = None, None
            if os.path.exists(hp):
                import csv as _csv
                import amfm_shaper as SH
                tol = SH.THRUST_SPEC["other_tol"]
                rows = list(_csv.DictReader(open(hp)))
                # 'excess' isn't stored directly -- offaxis_net/other_bias is,
                # and null tolerance is a fixed constant, so it's recovered
                # the same way thrust_error() computed it originally.
                def offaxis(row):
                    v = row.get("m_offaxis_net", row.get("m_other_bias", "0"))
                    return abs(float(v)) if v not in (None, "") else 0.0
                feas = [row for row in rows if offaxis(row) <= tol]
                cand = feas or rows
                if cand:
                    best_row = min(cand, key=lambda row: float(row["err"]))
                    net_best = float(best_row.get("m_bias", "nan"))
                    off_best = offaxis(best_row)
            summary.append({"direction": name, "ux": u[0], "uy": u[1], "uz": u[2],
                            "err_start": r["err_start"], "err_best": r["err_best"],
                            "net_force": net_best, "offaxis": off_best,
                            "n_evals": r["n_evals"]})

    all_names = {name for name, _ in CARDIAG + CORNERS}
    for d in sorted(os.listdir(folder)):
        if not d.startswith("dir_"):
            continue
        name = d[4:]
        if name not in all_names or name in {s["direction"] for s in summary}:
            continue
        rp2 = os.path.join(folder, d, "result.json")
        if not os.path.exists(rp2):
            continue
        r2 = json.load(open(rp2))
        u2 = dict(CARDIAG + CORNERS)[name]
        summary.append({"direction": name, "ux": u2[0], "uy": u2[1], "uz": u2[2],
                        "err_start": r2["err_start"], "err_best": r2["err_best"],
                        "net_force": None, "offaxis": None, "n_evals": r2["n_evals"]})

    if summary:
        p = os.path.join(folder, "direction_summary.csv")
        with open(p, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summary[0]))
            w.writeheader()
            w.writerows(summary)
        print(f"\nwrote {p}")
        print(f"{'direction':10s}{'err start':>11}{'err best':>10}{'net force':>11}"
              f"{'off-axis':>10}{'evals':>7}")
        for s in summary:
            nf = f"{s['net_force']:+.3f}" if s['net_force'] is not None else "  n/a"
            of = f"{s['offaxis']:.3f}" if s['offaxis'] is not None else "  n/a"
            print(f"{s['direction']:10s}{s['err_start']:11.3f}{s['err_best']:10.3f}"
                  f"{nf:>11}{of:>10}{s['n_evals']:7d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
