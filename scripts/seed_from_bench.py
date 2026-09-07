#!/usr/bin/env python3
"""
seed_from_bench.py -- find the bench-sweep mission whose measured Fx waveform
most closely MATCHES a target curve, and report its kinematic parameters.

This is the seeding step the controller should start from: the 941-mission
in_house_wet_test_3D sweep already contains real measured waveforms across
the whole parameter space, so the shape closest to a requested target can be
looked up rather than searched for on hardware. Previously the seed was the
sweep's maximum-THRUST point, which is a different question entirely -- it
started the match stage far from the best available shape and made the
load-cell feedback re-discover what the data already knew.

    python3 scripts/seed_from_bench.py <target.json> [out.json]
"""
import csv, glob, json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_for_control import load_mission, cycle2, parse_block
import force_control as fc


def main(target_json, out_path=None,
         root="in_house_wet_test_3D/split_data", phase_step=45):
    spec = fc.load_target_json(target_json)
    t_t, Fx_t = fc.build_target_curve(spec)

    best = None
    n = 0
    for bd in sorted(glob.glob(os.path.join(root, "k1_*"))):
        fr_s, ar = parse_block(os.path.basename(bd))
        if fr_s is None:
            continue
        for md in sorted(glob.glob(os.path.join(bd, "PH_*"))):
            try:
                ph = int(os.path.basename(md).split("_")[1])
            except (IndexError, ValueError):
                continue
            if ph % phase_step:
                continue
            got = load_mission(md)
            if got is None:
                continue
            cmd, t, fx, fy, fz = got
            tc, xc = cycle2(t, fx, cmd["pitch_freq"])
            if len(xc) < 30:
                continue
            # score against BOTH polarities: FX_SIGN is a convention, and the
            # bench data predates it, so the same physical waveform can appear
            # inverted relative to what the controller will measure.
            m = max(fc.waveform_match(tc, xc, t_t, Fx_t),
                    fc.waveform_match(tc, -xc, t_t, Fx_t))
            n += 1
            if best is None or m > best["match"]:
                best = {"match": float(m), "amp_ratio": ar,
                        "freq_ratio_fc": 1.0 / fr_s,
                        "delta_phi": -ph * math.pi / 180.0,
                        "phase_deg": ph, "mission": md,
                        "pitch_amp_deg": math.degrees(cmd["pitch_amp"]),
                        "heave_amp_deg": math.degrees(cmd["heave_amp"]),
                        "pitch_freq": cmd["pitch_freq"],
                        "heave_freq": cmd["heave_freq"]}
        print(f"  scanned {n} missions, best match so far "
              f"{best['match']:.3f}" if best else "  ...", flush=True)

    print(json.dumps(best, indent=2))
    if out_path:
        json.dump(best, open(out_path, "w"), indent=2)
    return best


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
