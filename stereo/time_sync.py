#!/usr/bin/env python3
"""
Time-synchronize two independently recorded camera H5 logs.

Camera 1 and camera 2 record on independent clocks, so a timestamp in one
file has no direct meaning in the other's timeline. Each LED beacon's
`decoded` dataset carries a hardware counter (counter_low, counter_high)
that increments once per blink; the *same* counter value seen at beacon X
in both cameras' decoded logs identifies the same physical blink, i.e. the
same real-world instant.

That would be enough to line up the two clocks directly, except the
counter is only 16 bits wide (counter_low + counter_high << 8), so it
wraps roughly every few hundred seconds. Over the two cameras' full
recordings the same 16-bit value recurs many times, so most raw counter
matches pair up blinks that are NOT actually simultaneous - only a
minority (in practice ~20-30%) are genuine matches. RANSAC separates the
genuine matches (which all sit on one consistent line, since both clocks
run at the same rate) from that majority of false ones.

Usage:
    python stereo/time_sync.py track_logs/camera_1/foo.h5 track_logs/camera_2/bar.h5 \
        --output track_logs/stereo/time_sync.json --plot track_logs/stereo/time_sync.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

# Below this many inlying matches, the fitted line isn't trustworthy enough
# to hand off to stereo calibration.
MIN_INLIERS = 20


# =====================================================================
# STAGE 1 - Read each camera's decoded beacon counters.
# =====================================================================


def beacon_id_from_group(name: str) -> int | None:
    tail = name.rsplit("_", 1)[-1]
    try:
        return int(tail)
    except ValueError:
        return None


def load_decoded(path: Path) -> dict[int, np.ndarray]:
    """
    Open an H5 file and build a lookup: beacon_id -> decoded rows
    (ts_start_bit_s, beacon_id, counter_low, counter_high), sorted by time.

    Current H5 files store the decoder column names as dataset metadata. Use
    those names because packet-start time is not the same as the first
    decoder column, and byte1 is the beacon ID rather than a counter byte.
    """
    decoded: dict[int, np.ndarray] = {}
    with h5py.File(path, "r") as f:
        for group_name in f.keys():
            beacon_id = beacon_id_from_group(group_name)
            if beacon_id is None or "decoded" not in f[group_name]:
                continue
            rows = np.asarray(f[group_name]["decoded"][...], dtype=np.float64)
            if len(rows) == 0:
                continue
            dataset = f[group_name]["decoded"]
            raw_columns = dataset.attrs.get("columns")
            columns = [
                value.decode("utf-8") if isinstance(value, bytes) else str(value)
                for value in raw_columns
            ] if raw_columns is not None else []

            if {"ts_start_bit_s", "byte1", "byte2", "byte3"}.issubset(columns):
                time_column = columns.index("ts_start_bit_s")
                beacon_column = columns.index("byte1")
                low_column = columns.index("byte2")
                high_column = columns.index("byte3")
            elif rows.shape[1] == 4:
                # Legacy layout: time, beacon ID, counter low, counter high.
                time_column, beacon_column, low_column, high_column = range(4)
            else:
                raise ValueError(
                    f"Unsupported decoded schema in {path}, group {group_name}: "
                    f"columns={columns or rows.shape[1]}"
                )

            selected = rows[:, [time_column, beacon_column, low_column, high_column]]
            selected = selected[np.all(np.isfinite(selected), axis=1)]
            selected = selected[np.argsort(selected[:, 0], kind="stable")]
            decoded[beacon_id] = selected
    return decoded


def counter16(decoded: np.ndarray) -> np.ndarray:
    """counter_low + (counter_high << 8), per AGENTS.md - the hardware blink counter."""
    counter_low = decoded[:, 2].astype(np.int64)
    counter_high = decoded[:, 3].astype(np.int64)
    return counter_low + (counter_high << 8)


# =====================================================================
# STAGE 2 - Pool every (beacon, counter) match common to both cameras.
# =====================================================================


def match_by_counter(
    dec1: dict[int, np.ndarray], dec2: dict[int, np.ndarray]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    For every beacon seen in both cameras, pair up decoded rows sharing the
    same 16-bit counter value. Returns parallel arrays (t1, t2, beacon_id)
    - one entry per raw match. Most of these matches are spurious (the
    counter recurs many times over a long recording); RANSAC below is what
    actually separates the real ones out.
    """
    t1_all: list[float] = []
    t2_all: list[float] = []
    beacon_all: list[int] = []
    for beacon_id in sorted(set(dec1) & set(dec2)):
        rows1 = dec1[beacon_id]
        rows2 = dec2[beacon_id]
        counters2_to_time = dict(zip(counter16(rows2).tolist(), rows2[:, 0].tolist()))
        for counter, t1 in zip(counter16(rows1).tolist(), rows1[:, 0].tolist()):
            t2 = counters2_to_time.get(counter)
            if t2 is not None:
                t1_all.append(t1)
                t2_all.append(t2)
                beacon_all.append(beacon_id)
    return np.asarray(t1_all), np.asarray(t2_all), np.asarray(beacon_all)


# =====================================================================
# STAGE 3 - RANSAC-fit t_cam2 = a * t_cam1 + b through the genuine matches.
# =====================================================================


def ransac_fit_line(
    x: np.ndarray,
    y: np.ndarray,
    threshold_s: float,
    iterations: int,
    min_inliers: int,
    rng: np.random.Generator,
) -> tuple[float, float, np.ndarray]:
    """
    Robustly fit y = a*x + b by repeatedly fitting a line through two random
    points and keeping whichever line the most points agree with (within
    threshold_s). The winning line is then refit by least squares over just
    its inliers, for a cleaner final estimate.
    """
    n = len(x)
    if n < 2:
        raise ValueError(f"Need at least 2 matches to fit a line, got {n}")

    best_mask: np.ndarray | None = None
    best_count = -1
    for _ in range(iterations):
        i, j = rng.choice(n, 2, replace=False)
        if x[i] == x[j]:
            continue
        a = (y[j] - y[i]) / (x[j] - x[i])
        b = y[i] - a * x[i]
        mask = np.abs(y - (a * x + b)) < threshold_s
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_mask is None or best_count < min_inliers:
        raise ValueError(
            f"RANSAC only found {best_count} consistent matches (need >= {min_inliers}). "
            "Try a larger --threshold-ms or more --iterations."
        )

    a, b = np.polyfit(x[best_mask], y[best_mask], 1)
    return float(a), float(b), best_mask


# =====================================================================
# STAGE 4 - Command-line entry point.
# =====================================================================


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("cam1_h5", type=Path, help="H5 file for camera 1")
    parser.add_argument("cam2_h5", type=Path, help="H5 file for camera 2")
    parser.add_argument(
        "--threshold-ms",
        type=float,
        default=3.0,
        help="RANSAC inlier threshold in milliseconds (default: 3.0)",
    )
    parser.add_argument("--iterations", type=int, default=2000, help="RANSAC iterations (default: 2000)")
    parser.add_argument(
        "--min-inliers",
        type=int,
        default=MIN_INLIERS,
        help=f"Minimum inlier matches required to trust the fit (default: {MIN_INLIERS})",
    )
    parser.add_argument("--seed", type=int, default=0, help="RANSAC random seed, for reproducibility")
    parser.add_argument("--output", type=Path, default=Path("track_logs/stereo/time_sync.json"))
    parser.add_argument("--plot", type=Path, help="Optional path to save a diagnostic PNG scatter plot")
    args = parser.parse_args()

    if args.threshold_ms <= 0 or args.iterations <= 0:
        parser.error("--threshold-ms and --iterations must be positive")

    dec1 = load_decoded(args.cam1_h5)
    dec2 = load_decoded(args.cam2_h5)
    print(f"Camera 1 beacons with decoded data: {sorted(dec1)}")
    print(f"Camera 2 beacons with decoded data: {sorted(dec2)}")

    t1, t2, beacon_ids = match_by_counter(dec1, dec2)
    print(f"\nRaw counter matches (mostly spurious): {len(t1)}")
    if len(t1) < args.min_inliers:
        parser.error(
            f"Only {len(t1)} raw counter matches found - not enough overlap between "
            "the two cameras' decoded beacons to time-sync."
        )

    rng = np.random.default_rng(args.seed)
    threshold_s = args.threshold_ms / 1000.0
    a, b, inlier_mask = ransac_fit_line(t1, t2, threshold_s, args.iterations, args.min_inliers, rng)

    residual = t2[inlier_mask] - (a * t1[inlier_mask] + b)
    num_inliers = int(inlier_mask.sum())
    inlier_beacons = sorted(set(beacon_ids[inlier_mask].tolist()))

    print(f"RANSAC inliers: {num_inliers}/{len(t1)} ({100 * num_inliers / len(t1):.1f}%)")
    print(f"Inlier beacon IDs: {inlier_beacons}")
    print(f"\nFit: t_cam2 = {a:.8f} * t_cam1 + {b:.6f}")
    print(f"Residual std: {residual.std() * 1e3:.4f} ms, max abs: {np.max(np.abs(residual)) * 1e3:.4f} ms")

    output = {
        "cam1_h5": str(args.cam1_h5),
        "cam2_h5": str(args.cam2_h5),
        "formula": "t_cam2 = a * t_cam1 + b",
        "a": a,
        "b": b,
        "threshold_s": threshold_s,
        "iterations": args.iterations,
        "seed": args.seed,
        "num_raw_matches": len(t1),
        "num_inliers": num_inliers,
        "inlier_ratio": num_inliers / len(t1),
        "inlier_beacon_ids": inlier_beacons,
        "residual_std_s": float(residual.std()),
        "residual_max_abs_s": float(np.max(np.abs(residual))),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(f"\nSaved {args.output}")

    if args.plot is not None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        axes[0].scatter(t1[~inlier_mask], t2[~inlier_mask], s=6, c="lightgray", label="outlier")
        axes[0].scatter(t1[inlier_mask], t2[inlier_mask], s=6, c="tab:blue", label="inlier")
        xs = np.array([t1.min(), t1.max()])
        axes[0].plot(xs, a * xs + b, c="tab:red", lw=1, label="fit")
        axes[0].set_xlabel("t_cam1 (s)")
        axes[0].set_ylabel("t_cam2 (s)")
        axes[0].set_title("Matched counter events")
        axes[0].legend()

        axes[1].scatter(t1[inlier_mask], residual * 1e3, s=6, c="tab:blue")
        axes[1].axhline(0, c="tab:red", lw=1)
        axes[1].set_xlabel("t_cam1 (s)")
        axes[1].set_ylabel("residual (ms)")
        axes[1].set_title("Inlier residuals")

        fig.tight_layout()
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=150)
        print(f"Saved {args.plot}")


if __name__ == "__main__":
    main()
