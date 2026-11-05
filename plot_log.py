#!/usr/bin/env python3
"""Plot the path of a run logged by circle_vicon.py or zigzag_vicon.py.  Runs on the laptop.

    python3 plot_log.py logs/192.168.3.7/circle_vicon_osoyoo_5_20261101_171615.csv
    python3 plot_log.py                       # the newest log under logs/

Saves <log>.png next to the log.  Needs matplotlib.
"""
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, GRAY, LIGHT, INK, MUTED, GRID = "#2a6fb0", "#80868b", "#c5cad0", "#202124", "#5f6368", "#eceff1"


def planned_path(script, cfg, first_pose):
    """Points of the path the robot was asked to follow."""
    if script == "circle_vicon":
        radius = cfg["diameter_m"] / 2
        return [(radius * math.cos(i / 100 * 2 * math.pi), radius * math.sin(i / 100 * 2 * math.pi))
                for i in range(101)]
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import zigzag_vicon as Z                        # rebuild the same plan the robot made
    corners = Z.zigzag_corners(first_pose, cfg)
    points = []
    for p in Z.rounded_path(corners, Z.fit_turn_radius(corners, cfg["turn_radius_m"])):
        if isinstance(p, Z.Line):
            points += [p.a, (p.a[0] + p.length * p.ux, p.a[1] + p.length * p.uy)]
        else:
            points += [(p.cx + p.radius * math.cos(p.start + p.sweep * i / 30),
                        p.cy + p.radius * math.sin(p.start + p.sweep * i / 30)) for i in range(31)]
    return points


def main():
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
    else:
        logs = sorted(Path(__file__).resolve().parent.glob("logs/**/*_vicon_*.csv"), key=lambda p: p.stat().st_mtime)
        if not logs:
            sys.exit("no logs found under logs/; run ./fetch_logs.sh first")
        csv_path = logs[-1]
    script = "zigzag_vicon" if csv_path.name.startswith("zigzag_vicon") else "circle_vicon"
    rows = list(csv.DictReader(open(csv_path)))
    cfg = json.loads(csv_path.with_suffix(".json").read_text())
    poses = [(float(r["x_m"]), float(r["y_m"])) if r["x_m"] else None for r in rows]
    seen = [p for p in poses if p]
    if not seen:
        sys.exit(f"{csv_path.name}: Vicon never saw the car, nothing to plot")

    plt.rcParams.update({"font.size": 10, "text.color": INK, "axes.edgecolor": LIGHT, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    planned = planned_path(script, cfg, seen[0])
    xs, ys = [p[0] for p in planned + seen], [p[1] for p in planned + seen]
    shape = (max(ys) - min(ys) + 0.1) / (max(xs) - min(xs) + 0.1)      # make the figure fit the path
    fig, ax = plt.subplots(figsize=(7.5, min(9.0, max(3.0, 7.5 * shape + 1.3))), layout="constrained")
    ax.plot(*zip(*planned), color=GRAY, lw=1.6, ls=(0, (5, 3)), label="planned path")
    segment, labelled = [], False
    for p in poses + [None]:                        # break the line where Vicon lost the car
        if p:
            segment.append(p)
        elif segment:
            ax.plot(*zip(*segment), color=BLUE, lw=2, label=None if labelled else "car (Vicon)")
            segment, labelled = [], True
    ax.plot(*seen[0], "o", ms=9, color=BLUE, mec="white", mew=2, zorder=3, label="start")
    ax.plot(*seen[-1], "s", ms=8, color=BLUE, mec="white", mew=2, zorder=3, label="end")
    if script == "circle_vicon":
        ax.plot(0, 0, "+", color=MUTED, ms=10)
    ax.set(aspect="equal", xlabel="lab x (m)", ylabel="lab y (m)")
    ax.set_title(csv_path.stem, loc="left", color=INK)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    fig.legend(frameon=False, loc="outside lower center", ncol=4, fontsize=9)

    out = csv_path.with_suffix(".png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"figure: {out}")


if __name__ == "__main__":
    main()
