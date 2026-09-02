#!/usr/bin/env python3
"""Plot Livox scan tables and a simulated PLY without Unreal."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def load_csv(path: Path) -> tuple[list[float], list[float]]:
    az, el = [], []
    with path.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 3 or row[0].startswith("Time") or row[0].startswith("#"):
                continue
            try:
                azimuth = float(row[1])
                zenith = float(row[2])
            except ValueError:
                continue
            az.append(azimuth)
            el.append(90.0 - zenith)
    return az, el


def load_ply(path: Path) -> tuple[list[float], list[float], list[float]]:
    xs, ys, zs = [], [], []
    header = True
    with path.open() as f:
        for line in f:
            if header:
                if line.strip() == "end_header":
                    header = False
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            xs.append(float(parts[0]))
            ys.append(float(parts[1]))
            zs.append(float(parts[2]))
    return xs, ys, zs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--ply", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    az, el = load_csv(args.csv)
    fig = plt.figure(figsize=(12, 10))
    fig.suptitle("Livox Mid-360 scan_mode + CPU room simulation (no Unreal)")

    ax0 = fig.add_subplot(2, 2, 1)
    n = min(len(az), 8000)
    ax0.scatter(az[:n], el[:n], s=1, c="tab:blue", alpha=0.35)
    ax0.set_xlabel("Azimuth (deg)")
    ax0.set_ylabel("Elevation (deg)")
    ax0.set_title(f"scan_mode elevations  n={len(az)}")
    ax0.set_xlim(0, 360)
    ax0.axhline(-7, color="k", ls="--", lw=0.8)
    ax0.axhline(52, color="k", ls="--", lw=0.8)

    ax1 = fig.add_subplot(2, 2, 2, projection="polar")
    rad = [math.radians(a) for a in az[:n]]
    ax1.scatter(rad, el[:n], s=1, c="tab:orange", alpha=0.3)
    ax1.set_title("non-repetitive rose (polar)")

    if args.ply and args.ply.exists():
        xs, ys, zs = load_ply(args.ply)
        ax2 = fig.add_subplot(2, 2, 3)
        ax2.scatter(xs[::3], ys[::3], s=0.4, c=zs[::3], cmap="viridis")
        ax2.set_aspect("equal")
        ax2.set_xlabel("X forward (m)")
        ax2.set_ylabel("Y left (m)")
        ax2.set_title(f"top view  points={len(xs)}")
        ax3 = fig.add_subplot(2, 2, 4)
        ax3.scatter(xs[::3], zs[::3], s=0.4, c=ys[::3], cmap="plasma")
        ax3.set_xlabel("X forward (m)")
        ax3.set_ylabel("Z up (m)")
        ax3.set_title("side view")
    else:
        ax2 = fig.add_subplot(2, 2, 3)
        ax2.text(0.5, 0.5, "no PLY", ha="center")
        ax2.axis("off")

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=140)
    print(f"wrote {args.out}  csv_samples={len(az)}")


if __name__ == "__main__":
    main()
