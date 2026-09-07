#!/usr/bin/env python3
"""
make_combined_csv_3d.py — single full-native-resolution CSV of the entire
in_house_wet_test_3D sweep: one row per LOADCELL sample (the finer of the
two logged rates, ~10kHz vs encoder feedback's ~72Hz), with the nearest-time
encoder feedback position nearest-matched onto it, plus the commanded
sinusoid position and the sweep parameters (freq_ratio, amp_ratio, phase)
identifying which mission the row belongs to.

Fx/Fy/Fz are written RAW, exactly as recorded -- no axis remap (this
dataset already uses Fx=pitch-associated, Fy=heave-associated, Fz=spanwise
directly, unlike an older/different dataset convention) and no taring (this
is a straight consolidation of the split raw files, not a derived/processed
dataset -- taring is done downstream by analysis scripts as needed).

    python3 scripts/make_combined_csv_3d.py <sweep_root/data> <out.csv>
"""
import csv, glob, os, re, sys, math, bisect

csv.field_size_limit(2**31 - 1)


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def parse_block_tag(name):
    if name == "k1_pf0p500_r1p000":
        return 1.0, 1.0
    m = re.match(r"^k1_fr(\d+)p(\d+)_ar(\d+)p(\d+)$", name)
    if not m:
        return None, None
    return float(f"{m.group(1)}.{m.group(2)}"), float(f"{m.group(3)}.{m.group(4)}")


def _label(md):
    for f in glob.glob(os.path.join(md, "*.csv")):
        b = os.path.basename(f)
        if not b.endswith("_loadcell.csv"):
            return b[:-4]
    return None


def main(root, out):
    blocks = sorted(d for d in glob.glob(os.path.join(root, "k1_*")) if os.path.isdir(d))
    header = ["freq_ratio", "amp_ratio", "phase_deg", "mission_label", "time_s",
              "s1_pitch_cmd_rad", "s1_pitch_fb_rad",
              "s2_heave_cmd_rad", "s2_heave_fb_rad",
              "Fx", "Fy", "Fz"]
    n_out = n_miss = n_blocks = 0
    with open(out, "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(header)
        for bd in blocks:
            n_blocks += 1
            fr, ar = parse_block_tag(os.path.basename(bd))
            if fr is None:
                print(f"  skip unparseable block {os.path.basename(bd)}")
                continue
            mdirs = sorted(glob.glob(os.path.join(bd, "PH_*")))
            for md in mdirs:
                label = _label(md)
                if not label:
                    continue
                fbp = os.path.join(md, f"{label}.csv")
                lcp = os.path.join(md, f"{label}_loadcell.csv")
                if not (os.path.exists(fbp) and os.path.exists(lcp)):
                    continue
                fb = list(csv.DictReader(open(fbp)))
                if not fb:
                    continue
                r0 = fb[0]
                f0 = _num(r0.get("cmd.frequency"))
                pa = _num(r0.get("cmd.pitch_amp")) or 0.0
                ha = _num(r0.get("cmd.heave_amp")) or 0.0
                ph = _num(r0.get("cmd.phase")) or 0.0
                cyc = _num(r0.get("cmd.cycles")) or 4.0
                phase_deg = round(ph * 180.0 / math.pi) % 360
                gait_end = (cyc / f0) if (f0 and f0 > 1e-9) else 0.0

                fb_t = [_num(r.get("time_s")) for r in fb]
                fb_s1 = [_num(r.get("s1_position_rad")) for r in fb]
                fb_s2 = [_num(r.get("s2_position_rad")) for r in fb]

                with open(lcp) as lfh:
                    for row in csv.DictReader(lfh):
                        t = _num(row.get("time_s"))
                        if t is None:
                            continue
                        Fx, Fy, Fz = row.get("Fx"), row.get("Fy"), row.get("Fz")

                        if f0 and t < gait_end:
                            s1c = pa * math.sin(2 * math.pi * f0 * t)
                            s2c = ha * math.sin(2 * math.pi * f0 * t + ph)
                        else:
                            s1c = s2c = 0.0

                        s1fb = s2fb = ""
                        if fb_t:
                            i = bisect.bisect_left(fb_t, t)
                            cand = [j for j in (i - 1, i) if 0 <= j < len(fb_t)]
                            if cand:
                                j = min(cand, key=lambda k: abs((fb_t[k] or 0.0) - t))
                                s1fb = "" if fb_s1[j] is None else round(fb_s1[j], 6)
                                s2fb = "" if fb_s2[j] is None else round(fb_s2[j], 6)

                        w.writerow([fr, ar, phase_deg, label, round(t, 6),
                                   round(s1c, 6), s1fb, round(s2c, 6), s2fb,
                                   Fx, Fy, Fz])
                        n_out += 1
                n_miss += 1
            print(f"  [{n_blocks}/{len(blocks)}] {os.path.basename(bd)} done "
                 f"({n_miss} missions, {n_out} rows so far)")
    print(f"wrote {n_out} rows from {n_miss} missions across {n_blocks} blocks -> {out}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: make_combined_csv_3d.py <sweep_root/data> <out.csv>")
    main(sys.argv[1], sys.argv[2])
