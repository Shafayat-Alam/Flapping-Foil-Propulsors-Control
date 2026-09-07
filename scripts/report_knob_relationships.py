#!/usr/bin/env python3
"""Turn the per-mission descriptors into knob -> waveform-feature rules.

Model, per descriptor y:

    y = c0 + a*log2(amp_ratio) + b*log2(freq_ratio)
             + p*sin(phase) + q*cos(phase)

  * log2 on the two ratios: they are multiplicative knobs swept
    geometrically (0.33 .. 3.0), so a doubling should count the same
    whichever end you start from. The coefficient then reads directly as
    "change in y per DOUBLING of the ratio".
  * sin/cos pair on phase: phase is circular, so a plain linear coefficient
    is meaningless (0 and 2*pi are the same rig state). The pair is
    reported as a modulation depth sqrt(p^2+q^2) -- how many newtons (or
    cycle-fractions) the feature swings across a full phase rotation --
    plus the phase angle at which the feature peaks.

Reported alongside every fit: R^2, and each term's own share of explained
variance, so a coefficient that is merely noise is visible as such.
"""
import os
import numpy as np
import pandas as pd

ROOT = "/home/shafa/soft-propulsors-control/in_house_wet_test_3D"
SRC = os.path.join(ROOT, "knob_response_descriptors.csv")
OUT = os.path.join(ROOT, "knob_relationships.csv")

FEATURES = [
    ("pos_peak",   "positive peak height (N)"),
    ("neg_peak",   "negative peak height (N)"),
    ("p2p",        "peak-to-peak (N)"),
    ("net",        "cycle-mean force (N)"),
    ("pos_width",  "positive lobe width (cycle frac)"),
    ("neg_width",  "negative lobe width (cycle frac)"),
    ("pos_duty",   "fraction of cycle with F>0"),
    ("rise_frac",  "rise/fall skew (0.5=symmetric)"),
    ("skew",       "waveform skewness"),
    ("t_pos_frac", "timing of positive peak (cycle frac)"),
    ("n_pos_peaks", "positive peak count"),
]


def fit(df, col):
    y = df[col].to_numpy(float)
    ok = np.isfinite(y)
    if ok.sum() < 20:
        return None
    y = y[ok]
    la = np.log2(df["amp_ratio"].to_numpy(float)[ok])
    lf = np.log2(df["freq_ratio"].to_numpy(float)[ok])
    ph = df["phase_rad"].to_numpy(float)[ok]
    X = np.column_stack([np.ones_like(y), la, lf, np.sin(ph), np.cos(ph)])

    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - np.sum((y - pred) ** 2) / ss_tot if ss_tot > 1e-12 else 0.0

    # Each knob's own contribution: R^2 lost when that knob's column(s)
    # are dropped. This separates a big coefficient that matters from a
    # big coefficient on a term the data barely constrains.
    def drop_r2(cols):
        keep = [c for c in range(X.shape[1]) if c not in cols]
        cf, *_ = np.linalg.lstsq(X[:, keep], y, rcond=None)
        p2 = X[:, keep] @ cf
        return (1 - np.sum((y - p2) ** 2) / ss_tot) if ss_tot > 1e-12 else 0.0

    return {
        "descriptor": col,
        "n": int(ok.sum()),
        "const": coef[0],
        "d_per_amp_doubling": coef[1],
        "d_per_freq_doubling": coef[2],
        "phase_depth": float(np.hypot(coef[3], coef[4])),
        "phase_peak_rad": float(np.arctan2(coef[3], coef[4])),
        "r2": r2,
        "share_amp": max(0.0, r2 - drop_r2([1])),
        "share_freq": max(0.0, r2 - drop_r2([2])),
        "share_phase": max(0.0, r2 - drop_r2([3, 4])),
        "mean": float(np.mean(y)),
        "range": float(np.ptp(y)),
    }


def main():
    df = pd.read_csv(SRC)
    print(f"{len(df)} missions\n")

    rows = []
    for ch, chname in (("fx", "Fx (thrust)"), ("fy", "Fy (lateral)"), ("fz", "Fz (vertical)")):
        print("=" * 100)
        print(f"{chname}")
        print("=" * 100)
        print(f"{'feature':<34} {'/amp x2':>9} {'/freq x2':>9} {'phase dep':>10} "
              f"{'R2':>6}  {'dominant knob':>14}")
        print("-" * 100)
        for feat, label in FEATURES:
            col = f"{ch}_{feat}"
            if col not in df.columns:
                continue
            r = fit(df, col)
            if not r:
                continue
            r["channel"] = ch
            r["feature"] = feat
            rows.append(r)
            shares = {"amp_ratio": r["share_amp"], "freq_ratio": r["share_freq"],
                      "phase": r["share_phase"]}
            dom = max(shares, key=shares.get)
            dom_s = f"{dom} ({shares[dom]:.2f})" if r["r2"] > 0.05 else "-- none --"
            print(f"{label:<34} {r['d_per_amp_doubling']:>+9.3f} "
                  f"{r['d_per_freq_doubling']:>+9.3f} {r['phase_depth']:>10.3f} "
                  f"{r['r2']:>6.3f}  {dom_s:>14}")
        print()

    pd.DataFrame(rows).to_csv(OUT, index=False)
    print(f"wrote {OUT}\n")
    peak_count_structure(df)


def peak_count_structure(df):
    """What actually sets the number of humps per flap.

    Reported as a table over freq_ratio rather than a regression
    coefficient: peak count is an integer that steps, so a slope in
    "peaks per doubling" describes the trend but hides WHERE the
    transitions sit, which is the part that is actionable.
    """
    for ch, name in (("fx", "Fx"), ("fy", "Fy"), ("fz", "Fz")):
        print("=" * 96)
        print(f"{name}: peaks per PITCH cycle (rounded), count of missions in each regime")
        print("=" * 96)
        for sign in ("pos", "neg"):
            col = f"{ch}_{sign}_per_pitch"
            if col not in df.columns:
                continue
            r = df[col].round().clip(0, 4).astype(int)
            tab = pd.crosstab(df["freq_ratio"], r)
            print(f"\n  {sign.upper()} peaks/pitch-cycle  (columns = count)")
            print(tab.to_string())
            # modal regime per freq_ratio, with how dominant it is
            print("  dominant regime per freq_ratio:")
            for fr, sub in df.groupby("freq_ratio"):
                rr = sub[col].round().clip(0, 4).astype(int)
                mode = rr.mode().iloc[0]
                frac = (rr == mode).mean()
                print(f"    fr={fr:<6} -> {mode} peak(s)  ({frac*100:.0f}% of {len(sub)} missions)")
        print()


if __name__ == "__main__":
    main()
