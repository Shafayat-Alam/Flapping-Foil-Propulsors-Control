#!/usr/bin/env python3
"""
retare_remap.py — write a TARED + axis-corrected copy of a sweep stage.

The load cell was mounted so its recorded axes are permuted vs the robot frame,
and it reads a large static (weight/mount) bias.  This makes a clean copy:

  axis remap (analysis axis  <-  recorded channel):
    Fx (thrust)   <- recorded Fy
    Fy (lateral)  <- recorded Fz      (should be ~0)
    Fz (heave)    <- recorded Fx      (symmetric up/down)
    Tx <- Ty,  Ty <- Tz,  Tz <- Tx

  taring: each recorded axis' static baseline (median over the at-rest tail of
  the segment, after the gait's cycles) is subtracted, so only the hydrodynamic
  force remains.

Segments much shorter than the gait (spurious/latched leftovers) are skipped.

    python3 scripts/retare_remap.py <src_stage_dir> <dst_stage_dir>
"""
import csv, glob, os, sys, shutil, math

csv.field_size_limit(2**31 - 1)

# analysis (true) axis  <-  recorded channel
REMAP = {'Fx': 'Fy', 'Fy': 'Fz', 'Fz': 'Fx', 'Tx': 'Ty', 'Ty': 'Tz', 'Tz': 'Tx'}
NEW_AXES = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
KEEP = ['time_s', 'bag_time_s', 'packet_time_s', 'packet_idx', 'sample_idx',
        'mission_label']


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _label(mdir):
    for f in glob.glob(os.path.join(mdir, '*.csv')):
        b = os.path.basename(f)
        if not b.endswith('_loadcell.csv'):
            return b[:-4]
    return None


def _median(xs):
    xs = sorted(v for v in xs if v is not None and not math.isnan(v))
    if not xs:
        return 0.0
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def process(src, dst):
    os.makedirs(dst, exist_ok=True)
    for extra in ('sweep_info.txt',):
        p = os.path.join(src, extra)
        if os.path.exists(p):
            shutil.copy(p, dst)

    mdirs = [d for d in sorted(glob.glob(os.path.join(src, '*')))
             if os.path.isdir(d) and os.path.basename(d) != 'raw']
    kept = skipped = 0
    for md in mdirs:
        label = _label(md)
        if not label:
            continue
        fb = os.path.join(md, f'{label}.csv')
        lc = os.path.join(md, f'{label}_loadcell.csv')
        dst_m = os.path.join(dst, os.path.basename(md))

        # gait timing from the feedback command columns
        freq = cycles = None
        if os.path.exists(fb):
            with open(fb) as f:
                r0 = next(csv.DictReader(f), {})
                freq = _num(r0.get('cmd.frequency'))
                cycles = _num(r0.get('cmd.cycles')) or 10.0
        if not os.path.exists(lc):
            print(f'  {label}: no loadcell — skipped'); continue

        # ---- read loadcell with a plain reader (fast) ----
        with open(lc) as f:
            rd = csv.reader(f)
            hdr = next(rd)
            idx = {h: i for i, h in enumerate(hdr)}
            if not all(a in idx for a in NEW_AXES) or 'time_s' not in idx:
                print(f'  {label}: unexpected loadcell columns — skipped'); continue
            rows = list(rd)
        if not rows:
            continue
        ti = idx['time_s']
        t = [_num(r[ti]) for r in rows]
        dur = max((x for x in t if x is not None), default=0.0)
        gait_end = (cycles / freq) if (freq and freq > 1e-9) else 0.0

        # spurious/latched short segment (no real rest tail) -> drop
        if not (gait_end > 0 and dur > gait_end + 2):
            print(f'  {label}: SHORT/partial segment ({dur:.1f}s) — skipped (spurious)')
            skipped += 1
            continue

        # ---- tare per RECORDED axis from the at-rest tail ----
        rest = [j for j, x in enumerate(t) if x is not None and x > gait_end + 1.0]
        tare = {}
        for ax in NEW_AXES:
            ai = idx[ax]
            tare[ax] = _median([_num(rows[j][ai]) for j in rest]) if len(rest) > 20 else 0.0

        # ---- write remapped + tared copy ----
        os.makedirs(dst_m, exist_ok=True)
        if os.path.exists(fb):
            shutil.copy(fb, os.path.join(dst_m, f'{label}.csv'))
        out = os.path.join(dst_m, f'{label}_loadcell.csv')
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(KEEP + NEW_AXES)
            keepidx = [idx.get(c) for c in KEEP]
            for r in rows:
                base = [r[i] if i is not None else '' for i in keepidx]
                vals = []
                for ax in NEW_AXES:
                    rec_ax = REMAP[ax]
                    x = _num(r[idx[rec_ax]])
                    vals.append('' if x is None else round(x - tare[rec_ax], 6))
                w.writerow(base + vals)
        kept += 1
        print(f'  {label}: tared+remapped {len(rows)} rows | '
              f'tare(recFy/Fz/Fx)={tare["Fy"]:.2f}/{tare["Fz"]:.2f}/{tare["Fx"]:.2f}')
    print(f'done: {kept} missions written, {skipped} spurious skipped -> {dst}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit('usage: retare_remap.py <src_stage_dir> <dst_stage_dir>')
    process(sys.argv[1], sys.argv[2])
