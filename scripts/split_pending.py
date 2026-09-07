#!/usr/bin/env python3
"""
split_pending.py — batch-split every block under a sweep root that finished
recording (raw/RECORDING_COMPLETE marker present) but hasn't been split into
per-mission folders yet (no PH_* folders) — the deferred-split path from
sweep_amp_freq_phase.py --no-split.

Safe to run anytime, including while a sweep is still going: a block that's
mid-recording has no marker yet, so it's correctly skipped, not split early.

    python3 scripts/split_pending.py <sweep_root>
    python3 scripts/split_pending.py in_house_wet_test_3D
"""
import glob, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SPLITTER = os.path.join(HERE, "split_missions.py")
MARKER = "RECORDING_COMPLETE"


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: split_pending.py <sweep_root>")
    root = sys.argv[1]
    if not os.path.isdir(root):
        sys.exit(f"no such directory: {root}")

    blocks = sorted(d for d in glob.glob(os.path.join(root, "*")) if os.path.isdir(d))
    pending, already_split, not_recorded = [], [], []
    for b in blocks:
        raw = os.path.join(b, "raw")
        has_marker = os.path.exists(os.path.join(raw, MARKER))
        has_ph = bool(glob.glob(os.path.join(b, "PH_*")))
        if has_ph:
            already_split.append(b)
        elif has_marker:
            pending.append(b)
        elif os.path.isdir(raw):
            not_recorded.append(b)   # partial/interrupted recording, no marker

    print(f"{len(blocks)} block folders under {root}")
    print(f"  {len(already_split)} already split (skipped)")
    print(f"  {len(not_recorded)} incomplete recordings, no marker (skipped — "
          f"not safe to split, and not done recording either)")
    print(f"  {len(pending)} pending split")
    if not_recorded:
        print("\n  incomplete (would need re-recording, not splitting):")
        for b in not_recorded:
            print(f"    {os.path.basename(b)}")
    if not pending:
        print("\nnothing to split.")
        return

    print()
    for i, b in enumerate(pending):
        raw = os.path.join(b, "raw")
        print(f"[{i+1}/{len(pending)}] splitting {os.path.basename(b)} ...")
        subprocess.run([sys.executable, SPLITTER, raw, "--base-dir", b],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        n = len(glob.glob(os.path.join(b, "PH_*")))
        print(f"    -> {n} mission folders")

    print(f"\ndone -> {len(pending)} blocks split")


if __name__ == "__main__":
    main()
