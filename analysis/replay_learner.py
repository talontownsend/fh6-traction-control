"""Replay a recorded session through the ABS learner and check its claims.

The README states that when this car is driven from tarmac onto dirt, measured
braking grip falls from about 3.65g to about 2.40g, and that the adaptive
setpoint follows the surface down within a couple of hard stops. This script
regenerates both numbers from the bundled sample so the claim can be checked
rather than taken on trust.

It also demonstrates the point of keeping the controller free of I/O: the same
AbsLearner the live engine uses is fed here from a CSV, with no game, no
controller and no drivers involved.

    python analysis/replay_learner.py
    python analysis/replay_learner.py --plot docs/learner-convergence.png

The bundled sample (analysis/sample/surface_transition.csv) is a 340 second
full-rate extract spanning the transition, reduced to the 20 columns this
analysis reads. Recordings the GUI produces carry all 49.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from traction_control import ABS_MIN_SPEED, AbsLearner   # noqa: E402

SAMPLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "sample", "surface_transition.csv")
DT = 0.016                  # telemetry period, 62.5 Hz
DELAY_FRAMES = 8            # measured transport delay, about 130 ms
WHEELS = ("fl", "fr", "rl", "rr")
G = 9.81


def load(path):
    with open(path, newline="") as fh:
        return [r for r in csv.DictReader(fh) if r["is_race_on"] == "1"]


def lockup_slip(row):
    """Braking slip is negative slip_ratio; take the deepest wheel."""
    return max(0.0, -min(float(row["slip_ratio_" + w]) for w in WHEELS))


def replay(rows, start_ceiling):
    """Feed the recording through a fresh learner. Returns per-frame state.

    The evidence gates must see the actuator state from one transport delay
    ago, because the physics in this frame is the response to the torque sent
    then, not to the command being issued now.
    """
    learner = AbsLearner()
    learner.reset(start_ceiling)
    history = []
    out = []
    for i, row in enumerate(rows):
        cut_scale = float(row["abs_scale"])
        sent = float(row["sent_brk"])
        # what the driver asked for, before our modulation
        user_pedal = sent / max(cut_scale, 1e-3)
        history.append((sent, cut_scale))
        if len(history) > DELAY_FRAMES:
            history.pop(0)
        sent_then, scale_then = (history[0] if len(history) == DELAY_FRAMES
                                 else (0.0, 0.0))
        learner.update(
            DT,
            lockup_slip(row),
            -float(row["az"]),                    # positive under braking
            abs(float(row["ax"])),
            min(sent_then, user_pedal),           # a release cannot fake falloff
            float(row["speed_mps"]),
            open_loop=scale_then >= 0.9,          # uncut torque only
            vert=float(row["ay"]),                # airborne frames teach nothing
        )
        out.append(dict(t=i * DT, ceiling=learner.ceiling,
                        cap=learner.effective_cap(),
                        verdict=learner.verdict,
                        live_ceiling=float(row["learn_ceiling"])))
    return out


def matched_decel(rows, lo_t, hi_t):
    """Braking g under matched conditions, so surfaces are comparable.

    Full pedal, near straight, wheels loaded, a fixed speed band, and slip
    below any intervention. Without matching, pedal position and speed swamp
    the surface difference being measured.
    """
    vals = []
    for i, row in enumerate(rows):
        t = i * DT
        if not lo_t <= t < hi_t:
            continue
        scale = float(row["abs_scale"])
        pedal = float(row["sent_brk"]) / max(scale, 1e-3)
        if (pedal >= 0.95
                and abs(float(row["ax"])) < 4.0            # near straight
                and -4.0 < float(row["ay"]) < 12.0         # wheels loaded
                and 39.0 <= float(row["speed_mps"]) <= 61.0
                and lockup_slip(row) < 1.5                 # below any cut
                and float(row["speed_mps"]) > ABS_MIN_SPEED):
            vals.append(-float(row["az"]) / G)
    return vals


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recording", nargs="?", default=SAMPLE,
                    help="recording CSV (default: the bundled sample)")
    ap.add_argument("--split", type=float, default=120.0,
                    help="seconds into the extract where the surface changes")
    ap.add_argument("--start", type=float, default=4.0,
                    help="setpoint the learner starts from, as a slip value")
    ap.add_argument("--plot", metavar="PNG",
                    help="also write a plot (requires matplotlib)")
    args = ap.parse_args()

    rows = load(args.recording)
    span = len(rows) * DT
    print(f"{os.path.basename(args.recording)}: {len(rows)} frames, "
          f"{span:.0f}s of racing\n")

    before = matched_decel(rows, 0.0, args.split)
    after = matched_decel(rows, args.split, span)
    print("Measured braking grip under matched conditions")
    print("  (full pedal, near straight, wheels loaded, 140-220 km/h, "
          "slip below any cut)")
    for label, vals in (("before", before), ("after ", after)):
        if len(vals) >= 8:
            print(f"    {label} the transition: {statistics.median(vals):.2f}g "
                  f"median over {len(vals)} frames")
        else:
            print(f"    {label} the transition: only {len(vals)} matched "
                  f"frames, not enough to report")
    if len(before) >= 8 and len(after) >= 8:
        lost = 1 - statistics.median(after) / statistics.median(before)
        print(f"    grip lost across the transition: {lost:.0%}\n")

    trace = replay(rows, args.start)
    names = {0: "gated", 1: "reference", 2: "brake ramp", 3: "no reference",
             4: "deadband", 5: "grip found", 6: "grip lost",
             7: "post-cut re-grip", 8: "airborne"}
    counts = {}
    for s in trace:
        counts[s["verdict"]] = counts.get(s["verdict"], 0) + 1

    print("Learner replayed from scratch (fresh state, same code as the engine)")
    print(f"    started at {args.start * 100:.0f}% of the nominal slip limit")
    pre = [s["ceiling"] for s in trace if s["t"] < args.split]
    post = [s["ceiling"] for s in trace if s["t"] >= args.split]
    if pre and post:
        print(f"    ceiling before the transition: {max(pre) * 100:.0f}% peak")
        print(f"    ceiling at the end:            {post[-1] * 100:.0f}%")
    print("    evidence frames by verdict: " + ", ".join(
        f"{names.get(k, k)}={v}" for k, v in sorted(counts.items()) if k >= 4))

    live = [s["live_ceiling"] for s in trace]
    if any(live):
        err = [abs(s["ceiling"] - s["live_ceiling"]) for s in trace]
        print(f"\n    mean |replay - live| ceiling difference: "
              f"{statistics.mean(err) * 100:.0f} percentage points")
        print("    (the live run started from a different setpoint and saw "
              "its own probe pulses,\n     so the paths are related but not "
              "identical)")

    if args.plot:
        plot(trace, args.split, args.plot)


def plot(trace, split, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [s["t"] for s in trace]
    fig, ax = plt.subplots(figsize=(9, 4.2), facecolor="#0d1117")
    ax.set_facecolor("#0d1117")
    ax.plot(t, [s["cap"] * 100 for s in trace], color="#58a6ff", lw=0.8,
            alpha=0.45, label="live cap (corner-derated)")
    ax.plot(t, [s["ceiling"] * 100 for s in trace], color="#3fb950", lw=1.9,
            label="learned ceiling")
    lost = [(s["t"], s["ceiling"] * 100) for s in trace if s["verdict"] == 6]
    if lost:
        ax.plot([p[0] for p in lost], [p[1] for p in lost], ".",
                color="#f85149", ms=5, label="grip-loss evidence")
    ax.axvline(split, color="#d29922", lw=1.0, ls="--", alpha=0.8)
    ax.annotate("surface change", xy=(split, ax.get_ylim()[1] * 0.9),
                xytext=(split + 25, ax.get_ylim()[1] * 0.9), color="#d29922",
                fontsize=9)
    ax.set_xlabel("time into extract (s)", color="#c9d1d9")
    ax.set_ylabel("brake-slip setpoint (% of grip limit)", color="#c9d1d9")
    ax.set_title("Adaptive ABS setpoint, replayed from recorded telemetry",
                 color="#c9d1d9")
    ax.grid(True, color="#232a33", lw=0.6)
    ax.tick_params(colors="#c9d1d9")
    for s in ax.spines.values():
        s.set_color("#232a33")
    leg = ax.legend(fontsize=8, facecolor="#161b22", edgecolor="#232a33",
                    labelcolor="#c9d1d9")
    leg.get_frame().set_alpha(0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor="#0d1117")
    print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
