#!/usr/bin/env python3
"""
make_combined_csv.py — one CSV of commanded position, encoder feedback position,
and load-cell forces for every mission in a phase-sweep stage.

Per mission, one row per encoder-feedback sample, with:
  * s1/s2 commanded position  — the paddle sinusoid amp*sin(2*pi*f*t (+phase)),
    held at 0 (centre) once the gait's cycles are done.  (s1 = pitch, s2 = heave)
  * s1/s2 feedback position   — the encoder reading (joint_feedback)
  * Fx/Fy/Fz                  — load-cell forces at that instant, nearest sample
    by bag time.  Use a tared+axis-corrected stage (e.g. lift_tared) so these are
    thrust / lateral / heave.

    python3 scripts/make_combined_csv.py <stage_dir> <out.csv>
"""
import csv, glob, os, sys, math, bisect

csv.field_size_limit(2**31 - 1)


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _label(md):
    for f in glob.glob(os.path.join(md, '*.csv')):
        b = os.path.basename(f)
        if not b.endswith('_loadcell.csv'):
            return b[:-4]
    return None


def main(stage, out):
    mdirs = [d for d in sorted(glob.glob(os.path.join(stage, '*')))
             if os.path.isdir(d) and os.path.basename(d) != 'raw']
    header = ['mission_label', 'phase_deg', 'time_s',
              's1_pitch_cmd_rad', 's1_pitch_fb_rad',
              's2_heave_cmd_rad', 's2_heave_fb_rad',
              'Fx_thrust_N', 'Fy_lateral_N', 'Fz_heave_N']
    n_out = n_miss = 0
    with open(out, 'w', newline='') as fo:
        w = csv.writer(fo)
        w.writerow(header)
        for md in mdirs:
            label = _label(md)
            if not label:
                continue
            fbp = os.path.join(md, f'{label}.csv')
            lcp = os.path.join(md, f'{label}_loadcell.csv')
            if not (os.path.exists(fbp) and os.path.exists(lcp)):
                continue
            fb = list(csv.DictReader(open(fbp)))
            lc = list(csv.DictReader(open(lcp)))
            if not fb or not lc:
                continue

            r0 = fb[0]
            f = _num(r0.get('cmd.frequency'))
            pa = _num(r0.get('cmd.pitch_amp')) or 0.0
            ha = _num(r0.get('cmd.heave_amp')) or 0.0
            ph = _num(r0.get('cmd.phase')) or 0.0
            cyc = _num(r0.get('cmd.cycles')) or 4.0
            phase_deg = round(ph * 180.0 / math.pi)
            gait_end = (cyc / f) if (f and f > 1e-9) else 0.0

            # skip spurious/latched short segments (no real gait)
            lc_t = [_num(r.get('time_s')) for r in lc]
            dur = max((x for x in lc_t if x is not None), default=0.0)
            if not (gait_end > 0 and dur > gait_end + 2):
                print(f"  {label}: skipped (short/partial segment, {dur:.1f}s)")
                continue

            # load-cell arrays for nearest-time lookup by bag time (monotonic).
            # AXIS CORRECTION (recorded -> true): thrust<-Fy, lateral<-Fz, heave<-Fx.
            # Values are RAW (un-tared) — the static weight/mount bias is still in.
            lc_bt = [_num(r.get('bag_time_s')) for r in lc]
            lc_Fx = [_num(r.get('Fy')) for r in lc]   # thrust  <- recorded Fy
            lc_Fy = [_num(r.get('Fz')) for r in lc]   # lateral <- recorded Fz
            lc_Fz = [_num(r.get('Fx')) for r in lc]   # heave   <- recorded Fx

            for r in fb:
                t = _num(r.get('time_s'))
                bt = _num(r.get('bag_time_s'))
                if t is None:
                    continue
                # commanded paddle sinusoid (logical frame); centre after gait
                if f and t < gait_end:
                    s1c = pa * math.sin(2 * math.pi * f * t)
                    s2c = ha * math.sin(2 * math.pi * f * t + ph)
                else:
                    s1c = s2c = 0.0
                s1fb = _num(r.get('s1_position_rad'))
                s2fb = _num(r.get('s2_position_rad'))
                Fx = Fy = Fz = ''
                if bt is not None and lc_bt and lc_bt[0] is not None:
                    i = bisect.bisect_left(lc_bt, bt)
                    cand = [j for j in (i - 1, i) if 0 <= j < len(lc_bt)]
                    if cand:
                        j = min(cand, key=lambda k: abs((lc_bt[k] or 0.0) - bt))
                        Fx, Fy, Fz = lc_Fx[j], lc_Fy[j], lc_Fz[j]
                w.writerow([label, phase_deg, round(t, 4),
                            round(s1c, 6), '' if s1fb is None else round(s1fb, 6),
                            round(s2c, 6), '' if s2fb is None else round(s2fb, 6),
                            '' if Fx is None else Fx,
                            '' if Fy is None else Fy,
                            '' if Fz is None else Fz])
                n_out += 1
            n_miss += 1
    print(f'wrote {n_out} rows from {n_miss} missions -> {out}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit('usage: make_combined_csv.py <stage_dir> <out.csv>')
    main(sys.argv[1], sys.argv[2])
