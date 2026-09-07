#!/usr/bin/env python3
"""
fft_lift_drag.py — FFT the thrust for each phase and identify which phase is
lift-based (2f-dominant: two thrust peaks per flap) vs drag-based (1f-dominant:
one thrust peak per flap).

Uses the axis-corrected, tared thrust (thrust = recorded Fy).  The load cell is
sampled in non-uniform bursts, so the thrust is resampled onto a uniform grid
before the FFT (a raw FFT of non-uniform samples would smear).

    python3 scripts/fft_lift_drag.py <stage_dir> [out_png]
"""
import csv, glob, os, sys, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

csv.field_size_limit(2**31 - 1)
STEP = 20                       # downsample the 10 kHz load cell (every 20th -> ~500 Hz)
RESAMPLE_HZ = 200.0


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _label(md):
    for f in glob.glob(os.path.join(md, "*.csv")):
        b = os.path.basename(f)
        if not b.endswith("_loadcell.csv"):
            return b[:-4]
    return None


def analyze(md):
    """Return dict with phase_deg and FFT thrust magnitudes at 1f and 2f."""
    label = _label(md)
    if not label:
        return None
    fbp, lcp = os.path.join(md, f"{label}.csv"), os.path.join(md, f"{label}_loadcell.csv")
    if not (os.path.exists(fbp) and os.path.exists(lcp)):
        return None
    r0 = next(csv.DictReader(open(fbp)), {})
    f0 = _num(r0.get("cmd.frequency"))
    ph = _num(r0.get("cmd.phase")) or 0.0
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    if not f0:
        return None
    gait_end = cyc / f0

    # read time + recorded Fy (=thrust), downsampled
    t, thr = [], []
    with open(lcp) as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        ix = {h: i for i, h in enumerate(hdr)}
        if "time_s" not in ix or "Fy" not in ix:
            return None
        for k, row in enumerate(rd):
            if k % STEP:
                continue
            tv = _num(row[ix["time_s"]]); fv = _num(row[ix["Fy"]])
            if tv is not None and fv is not None:
                t.append(tv); thr.append(fv)
    t = np.asarray(t); thr = np.asarray(thr)
    if t.size < 50:
        return None
    if not (t.max() > gait_end + 2):        # spurious short segment
        return None

    # tare with the at-rest tail, then keep the steady gait cycles (skip 1st)
    rest = t > gait_end + 1.0
    if rest.sum() > 10:
        thr = thr - np.median(thr[rest])
    win = (t >= 1.0 / f0) & (t <= gait_end)   # cycles 1..N (drop the first)
    tw, xw = t[win], thr[win]
    if tw.size < 50:
        return None

    # resample to a uniform grid, then FFT
    n = int((tw[-1] - tw[0]) * RESAMPLE_HZ)
    if n < 32:
        return None
    tu = np.linspace(tw[0], tw[-1], n)
    xu = np.interp(tu, tw, xw)
    xu = xu - xu.mean()
    mag = np.abs(np.fft.rfft(xu * np.hanning(n))) * (2.0 / n)
    freq = np.fft.rfftfreq(n, d=(tu[1] - tu[0]))

    def at(fq):
        j = np.argmin(np.abs(freq - fq))
        return float(mag[j])
    a1, a2 = at(f0), at(2 * f0)
    return {"phase_deg": round(ph * 180 / math.pi), "f0": f0,
            "a1": a1, "a2": a2, "ratio": (a2 / a1 if a1 > 1e-9 else float("inf")),
            "freq": freq, "mag": mag, "label": label}


def main(stage, out_png):
    mdirs = [d for d in sorted(glob.glob(os.path.join(stage, "*")))
             if os.path.isdir(d) and os.path.basename(d) != "raw"]
    res = [r for r in (analyze(d) for d in mdirs) if r]
    res.sort(key=lambda r: r["phase_deg"])

    print(f"\n{'phase':>6} {'1f amp':>9} {'2f amp':>9} {'2f/1f':>7}  style")
    print("-" * 44)
    for r in res:
        style = "LIFT (2f)" if r["ratio"] >= 1.0 else "drag (1f)"
        print(f"{r['phase_deg']:>5}° {r['a1']:>9.4f} {r['a2']:>9.4f} {r['ratio']:>7.2f}  {style}")
    print("-" * 44)

    most_lift = max(res, key=lambda r: r["ratio"])
    most_drag = min(res, key=lambda r: r["ratio"])
    print(f"\n  MOST LIFT-BASED phase: {most_lift['phase_deg']}°  "
          f"(2f/1f = {most_lift['ratio']:.2f})")
    print(f"  MOST DRAG-BASED phase: {most_drag['phase_deg']}°  "
          f"(2f/1f = {most_drag['ratio']:.2f})")
    n_lift = sum(1 for r in res if r["ratio"] >= 1.0)
    print(f"  ({n_lift}/{len(res)} phases are 2f-dominant/lift; the rest are 1f-dominant/drag)")

    # ---- plot: spectra of the most-lift and most-drag phase ----
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    for a, r, tag in ((ax[0], most_drag, "MOST DRAG-BASED"),
                      (ax[1], most_lift, "MOST LIFT-BASED")):
        m = r["freq"] <= 3.0
        a.plot(r["freq"][m], r["mag"][m], lw=1.5)
        a.axvline(r["f0"], color="tab:green", ls="--", label=f"1f = {r['f0']:.2f} Hz (drag)")
        a.axvline(2 * r["f0"], color="tab:red", ls="--", label=f"2f = {2*r['f0']:.2f} Hz (lift)")
        a.set_title(f"{tag}: phase {r['phase_deg']}°  (2f/1f={r['ratio']:.2f})")
        a.set_xlabel("frequency (Hz)"); a.set_ylabel("thrust FFT magnitude (N)")
        a.grid(True, alpha=0.3); a.legend()
    fig.suptitle("Thrust FFT — lift-based (2f-dominant) vs drag-based (1f-dominant)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    print(f"\n  plot -> {out_png}")

    # spectrum-vs-phase overview
    fig2, ax2 = plt.subplots(figsize=(10, 5))
    phs = [r["phase_deg"] for r in res]
    ax2.plot(phs, [r["a1"] for r in res], "o-", label="1f amplitude (drag component)")
    ax2.plot(phs, [r["a2"] for r in res], "s-", label="2f amplitude (lift component)")
    ax2.plot(phs, [r["ratio"] for r in res], "^:", color="tab:purple", label="2f/1f ratio")
    ax2.axhline(1.0, color="k", ls=":", alpha=0.5, label="lift/drag boundary (ratio=1)")
    ax2.set_xlabel("phase shift (deg)"); ax2.set_ylabel("thrust FFT amplitude (N) / ratio")
    ax2.set_title("Thrust harmonic content vs phase shift")
    ax2.grid(True, alpha=0.3); ax2.legend()
    fig2.tight_layout()
    out2 = out_png.replace(".png", "_vs_phase.png")
    fig2.savefig(out2, dpi=120)
    print(f"  plot -> {out2}")


if __name__ == "__main__":
    stage = sys.argv[1] if len(sys.argv) > 1 else "in_house_wet_test/lift/01_phase"
    out = sys.argv[2] if len(sys.argv) > 2 else "in_house_wet_test/lift_fft_lift_vs_drag.png"
    main(stage, out)
