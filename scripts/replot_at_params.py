#!/usr/bin/env python3
"""Re-measure the rig at one fixed parameter set and plot the result.

Why this exists: a control run that is hard-killed (SIGKILL) never reaches
its save path, so its plots and CSVs are lost even though the parameters it
found are still known from the log. This re-drives the rig at those exact
parameters for a single steady measurement and produces the same force /
servo plots the run itself would have written.

The plot is therefore an honest reproduction of that operating point, but it
is a FRESH measurement -- not the original cycle's data. Run-to-run scatter
on this rig is real (waveform_match moved +-0.05 between identical-parameter
cycles), so treat small differences from the logged numbers as noise.

usage: replot_at_params.py <folder> <amp> <freq> <dphi> <scale> <freq_scale>
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import force_control as fc


def main():
    folder = sys.argv[1]
    params = np.array([float(x) for x in sys.argv[2:7]], dtype=float)

    out_dir = os.path.join(fc.WORKSPACE_ROOT, folder)
    os.makedirs(out_dir, exist_ok=True)

    import glob
    cands = [p for p in glob.glob(os.path.join(out_dir, "target_curve_*.json"))
             if "_with_result" not in p]
    spec = fc.load_target_json(cands[0])
    t_target, Fx_target = fc.build_target_curve(spec)

    import soft_propulsors_control.motion_command as mc
    node = mc.start_hil_node()
    try:
        node.capture_rest_baseline()

        def run_plant(p, n_cycles=fc.N_CYCLES_PER_MEASUREMENT):
            return mc.run_plant_HARDWARE(p, n_cycles=n_cycles, node=node)

        t, Fx, Fy, Fz, th1, th2 = fc.collect_steady_measurement(
            params, run_plant, wait_s=fc.SETTLE_WAIT_S)
    finally:
        mc.stop_hil_node(node)

    desc = fc.extract_fx_descriptors(
        t, Fx, expected_freq_hz=fc.decode_params(params)["f1"],
        target=(t_target, Fx_target))
    sec = fc.extract_secondary(Fy, Fz)
    allm = {**desc, **sec}

    print("=" * 70)
    print(f"Re-measured at amp_ratio={params[0]:.3f} freq_ratio={params[1]:.3f} "
          f"delta_phi={params[2]:.3f} scale={params[3]:.3f} freq_scale={params[4]:.3f}")
    for k in sorted(allm):
        print(f"   {k:20s} = {allm[k]:+.4f}")

    prefix = os.path.join(out_dir, "final")
    fc.plot_final_force_curve(Fx, Fy, Fz, t, prefix + "_force_curve.png")
    fc.plot_final_servo_position(th1, th2, t, prefix + "_servo_position.png")

    # Fx against the target curve it was tuned to -- the plot that actually
    # answers "did it match?".
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t - t[0], Fx, lw=1.2, label="measured Fx")
    ax.plot(t_target, Fx_target, "--", lw=2, label="target Fx")
    ax.set_xlabel("time (s)"); ax.set_ylabel("Fx (N)")
    ax.set_title(f"{folder}: measured vs target Fx  "
                 f"(match={allm.get('waveform_match', float('nan')):.3f})")
    ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(prefix + "_fx_vs_target.png", dpi=130)
    print(f"\nwrote {prefix}_force_curve.png, {prefix}_servo_position.png, "
          f"{prefix}_fx_vs_target.png")


if __name__ == "__main__":
    main()
