#!/usr/bin/env python3
"""
Build time-synchronized stereo image-point correspondences from two camera
H5 logs.

Combines two things already in this repo:
  1. stereo/time_sync.py's beacon+counter matching -> RANSAC fit of
     t_cam2 = a * t_cam1 + b (the clock relationship between the two cameras).
  2. The tracker-interpolation approach from intrinsic/intrinsic_calibration_2.py
     -> at evenly spaced timestamps (1/--hz seconds apart, on camera 1's
     clock), convert each one to camera 2's clock via the fit above, then
     interpolate every beacon's pixel position in BOTH cameras at that
     synced instant. Only beacons visible (within --max-gap) in both
     cameras at that instant are kept, and a view is dropped if any usable
     beacon fails - this keeps every kept view's point count and beacon
     order identical in both cameras, which cv2.stereoCalibrate needs.

Output is a set of "views": for each sampled instant, one array of 2D pixel
positions per camera (same beacon order, same count, same view order in
both), ready to feed into cv2.stereoCalibrate() once paired with 3D beacon
geometry (see intrinsic/intrinsic_calibration_2.py's load_geometry /
make_led_box_geometry for that side).

Usage:
    python stereo/build_stereo_points.py \
        track_logs/camera_1/foo.h5 track_logs/camera_2/bar.h5 \
        --hz 10 --output track_logs/stereo/img_points.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np

from time_sync import MIN_INLIERS, load_decoded, match_by_counter, ransac_fit_line


# =====================================================================
# STAGE 1 - Load each camera's tracker data (pixel position over time, per
# beacon). Duplicated from intrinsic/intrinsic_calibration_2.py rather than
# imported, following this repo's existing convention (the same functions
# are already copy-pasted across intrinsic_calibration.py,
# intrinsic_calibration_2.py, and utils/calibrate_intrinsics_2.py) - these
# scripts are run standalone, not as an installed package.
# =====================================================================


def beacon_id_from_group(name: str) -> int | None:
    tail = name.rsplit("_", 1)[-1]
    try:
        return int(tail)
    except ValueError:
        return None


def collapse_duplicate_times(rows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Sort [t,x,y], drop bad rows, and median-combine duplicate timestamps."""
    rows = np.asarray(rows, dtype=np.float64)
    if rows.ndim != 2 or rows.shape[1] < 3:
        raise ValueError(f"tracker must have shape (N, >=3), got {rows.shape}")
    rows = rows[:, :3]
    rows = rows[np.all(np.isfinite(rows), axis=1)]
    rows = rows[np.argsort(rows[:, 0], kind="stable")]
    if len(rows) == 0:
        return np.empty(0), np.empty((0, 2))

    times, starts, counts = np.unique(rows[:, 0], return_index=True, return_counts=True)
    xy = rows[starts, 1:3].copy()
    for i in np.flatnonzero(counts > 1):
        start, count = starts[i], counts[i]
        xy[i] = np.median(rows[start : start + count, 1:3], axis=0)
    return times, xy


def load_tracks(path: Path) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    tracks: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    with h5py.File(path, "r") as f:
        for group_name in f.keys():
            beacon_id = beacon_id_from_group(group_name)
            if beacon_id is None or "tracker" not in f[group_name]:
                continue
            times, xy = collapse_duplicate_times(f[group_name]["tracker"][...])
            if len(times):
                tracks[beacon_id] = (times, xy)
    return tracks


def interpolate_if_local(
    times: np.ndarray,
    xy: np.ndarray,
    sample_time: float,
    max_gap_s: float,
    min_speed_dt_s: float = 0.002,
) -> tuple[np.ndarray, float] | None:
    """
    Linear-interpolate a beacon's pixel position at sample_time, refusing to
    guess if sample_time falls outside the beacon's observed range or either
    bracketing measurement is farther than max_gap_s away.

    Returns (xy_at_sample_time, local_speed_px_per_s), or None if no
    trustworthy estimate exists. local_speed_px_per_s is the beacon's
    observed speed between the two bracketing measurements actually used for
    interpolation (0.0 on an exact timestamp match, where no interpolation -
    and therefore no motion-related error - is involved). This is reported
    as a quality signal regardless of filtering, and can also be used as a
    hard gate (see build_stereo_image_points' max_velocity_px_per_s): fast
    motion between two measurements is exactly when linear interpolation is
    least trustworthy, and it correlates with real bursts of stereo
    reprojection error when reprojection RMS is grouped by view_index.

    min_speed_dt_s is a floor on the time gap used ONLY for the speed
    calculation's denominator (never for the interpolation itself, which
    always uses the true dt). Without this floor, two bracketing samples a
    fraction of a millisecond apart - routine tracker timing jitter, not a
    real motion event - can differ by a couple of pixels of ordinary
    measurement noise and still divide out to an absurd, meaningless
    "speed" (thousands to hundreds of thousands of px/s), because the
    denominator is near zero rather than the numerator being large. The
    floor bounds how much any single close-together pair can inflate the
    reported speed, without changing the interpolated position at all.
    """
    right = int(np.searchsorted(times, sample_time, side="left"))
    if right < len(times) and np.isclose(times[right], sample_time, rtol=0, atol=1e-12):
        return xy[right].astype(np.float32), 0.0
    left = right - 1
    if left < 0 or right >= len(times):
        return None
    if sample_time - times[left] > max_gap_s or times[right] - sample_time > max_gap_s:
        return None
    dt = times[right] - times[left]
    if dt <= 0:
        return None
    alpha = (sample_time - times[left]) / dt
    point = ((1.0 - alpha) * xy[left] + alpha * xy[right]).astype(np.float32)
    speed = float(np.linalg.norm(xy[right] - xy[left]) / max(dt, min_speed_dt_s))
    return point, speed


# =====================================================================
# STAGE 2 - Sample camera 1's clock at --hz, map each sample to camera 2's
# clock via t_cam2 = a*t_cam1 + b, and interpolate both cameras' tracker
# position there.
# =====================================================================


def build_stereo_image_points(
    tracks1: dict[int, tuple[np.ndarray, np.ndarray]],
    tracks2: dict[int, tuple[np.ndarray, np.ndarray]],
    a: float,
    b: float,
    hz: float,
    max_gap_s: float,
    max_velocity_px_per_s: float | None = None,
    min_speed_dt_s: float = 0.002,
    min_points: int = 12,
) -> tuple[list[np.ndarray], list[np.ndarray], np.ndarray, list[int], np.ndarray, dict[str, int]]:
    """
    Returns (img_points_cam1, img_points_cam2, sample_times_cam1, beacon_ids,
    view_max_speed_px_per_s, rejection_counts).

    Each img_points_camN[i] has shape (len(beacon_ids), 2): same beacon
    order across every kept view, but NOT every beacon is necessarily
    present in every view - a beacon missing at view i (out of tracker
    range, too far from the nearest sample, or over max_velocity_px_per_s)
    is written as NaN in BOTH cameras for that (view, beacon) slot, rather
    than discarding the whole view. This mirrors intrinsic_calibration_2.py,
    which already accepts a variable number of visible beacons per view (its
    own --min-points) - stereo previously required EVERY beacon ever seen in
    either camera to be simultaneously visible in BOTH cameras at the exact
    same instant, which is an unreasonably strict bar once a board sweep
    covers enough of the frame/tilt range to cause any real occlusion (a
    24-LED board practically never shows all 24 LEDs to both cameras at
    once - "No synchronized views survived" was this, not actually an --hz
    or --max-gap problem). Downstream, evaluation/calibration_quality.py's
    triangulate_and_score() already treats a non-finite pixel coordinate as
    invalid for that point rather than crashing, and stereo_calibrate.py has
    been updated to build each view's object/image points only from that
    view's finite entries - so NaN gaps here propagate safely all the way
    through.

    A view is kept only if at least min_points beacons survive in it (after
    both availability and, if set, the velocity filter); otherwise it's
    rejected entirely (rejection_counts["insufficient_points"]), the same
    failure mode intrinsic_calibration_2.py already reports for a
    too-sparse view.

    view_max_speed_px_per_s[i] is the fastest local interpolation speed (see
    interpolate_if_local) among the SURVIVING beacons in kept view i -
    reported for every kept view regardless of max_velocity_px_per_s, so you
    can check after the fact (e.g. against per-view reprojection RMSE from
    evaluation/calibration_quality.py) whether motion-during-interpolation
    correlates with the views that came out worst, even on a run where
    filtering wasn't enabled.
    """
    usable_ids = sorted(set(tracks1) & set(tracks2))
    if not usable_ids:
        raise ValueError("No beacon IDs overlap between the two cameras' tracker data")

    start = min(tracks1[i][0][0] for i in usable_ids)
    stop = max(tracks1[i][0][-1] for i in usable_ids)
    sample_times_cam1 = np.arange(start, stop + 0.5 / hz, 1.0 / hz)

    img_points_cam1: list[np.ndarray] = []
    img_points_cam2: list[np.ndarray] = []
    kept_times: list[float] = []
    view_max_speed: list[float] = []
    rejected_insufficient_points = 0
    total_missing_or_gap_slots = 0
    total_velocity_rejected_slots = 0

    for t1 in sample_times_cam1:
        t2 = a * t1 + b
        pts1 = np.full((len(usable_ids), 2), np.nan, dtype=np.float32)
        pts2 = np.full((len(usable_ids), 2), np.nan, dtype=np.float32)
        speeds: list[float] = []
        for idx, beacon_id in enumerate(usable_ids):
            r1 = interpolate_if_local(*tracks1[beacon_id], t1, max_gap_s, min_speed_dt_s)
            r2 = interpolate_if_local(*tracks2[beacon_id], t2, max_gap_s, min_speed_dt_s) if r1 is not None else None
            if r1 is None or r2 is None:
                total_missing_or_gap_slots += 1
                continue
            p1, s1 = r1
            p2, s2 = r2
            beacon_speed = max(s1, s2)
            if max_velocity_px_per_s is not None and beacon_speed > max_velocity_px_per_s:
                total_velocity_rejected_slots += 1
                continue
            pts1[idx] = p1
            pts2[idx] = p2
            speeds.append(beacon_speed)

        if len(speeds) < min_points:
            rejected_insufficient_points += 1
            continue
        img_points_cam1.append(pts1)
        img_points_cam2.append(pts2)
        kept_times.append(float(t1))
        view_max_speed.append(max(speeds) if speeds else 0.0)

    rejection_counts = {
        "insufficient_points": rejected_insufficient_points,
        "total_missing_or_gap_slots": total_missing_or_gap_slots,
        "total_velocity_rejected_slots": total_velocity_rejected_slots,
    }
    return (
        img_points_cam1,
        img_points_cam2,
        np.asarray(kept_times),
        usable_ids,
        np.asarray(view_max_speed, dtype=np.float64),
        rejection_counts,
    )


# =====================================================================
# STAGE 3 - Command-line entry point.
# =====================================================================


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("cam1_h5", type=Path, help="H5 file for camera 1")
    parser.add_argument("cam2_h5", type=Path, help="H5 file for camera 2")
    parser.add_argument("--hz", type=float, default=10.0, help="Sampling frequency, on camera 1's clock (default: 10.0)")
    parser.add_argument(
        "--max-gap",
        type=float,
        default=0.05,
        help="Max seconds from sample time to each tracker interpolation endpoint (default: 0.05)",
    )
    parser.add_argument(
        "--threshold-ms",
        type=float,
        default=3.0,
        help="RANSAC inlier threshold in ms, for the time-sync fit (default: 3.0)",
    )
    parser.add_argument("--iterations", type=int, default=2000, help="RANSAC iterations for the time-sync fit (default: 2000)")
    parser.add_argument(
        "--min-inliers",
        type=int,
        default=MIN_INLIERS,
        help=f"Minimum RANSAC inliers required to trust the time-sync fit (default: {MIN_INLIERS})",
    )
    parser.add_argument("--seed", type=int, default=0, help="RANSAC random seed (default: 0)")
    parser.add_argument(
        "--min-points",
        type=int,
        default=12,
        help=(
            "Minimum number of jointly-visible-and-trustworthy beacons required to "
            "keep a candidate view (default: 12, matching "
            "intrinsic_calibration_2.py's own --min-points). A beacon missing from "
            "either camera at that instant, out of --max-gap range, or over "
            "--max-velocity is dropped individually (recorded as NaN for that one "
            "beacon in that one view) rather than discarding the whole view - only "
            "when too FEW beacons survive is the view rejected outright. Without "
            "this, stereo previously required literally every beacon ever seen in "
            "either camera to be simultaneously visible in BOTH cameras at once, "
            "which a full LED board practically never satisfies once there's any "
            "real occlusion (e.g. from tilting the board to cover more of the FOV) - "
            "this was the actual cause of a 'No synchronized views survived' error, "
            "not --hz or --max-gap."
        ),
    )
    parser.add_argument(
        "--max-velocity",
        type=float,
        default=None,
        help=(
            "Reject a candidate view if any beacon's local interpolation speed "
            "(pixel distance between the two bracketing tracker samples, divided "
            "by their time gap) exceeds this many px/s. Fast board motion between "
            "two tracker samples is exactly when linear interpolation is least "
            "trustworthy, and a periodic sweep (rather than discrete held poses) "
            "can otherwise dominate the stereo fit with its worst, fastest-motion "
            "instants. Default: None (disabled - reproduces the original "
            "behavior); view_max_speed_px_per_s is always saved to the output "
            "NPZ regardless, so you can inspect the distribution before deciding "
            "on a threshold."
        ),
    )
    parser.add_argument(
        "--min-speed-dt-ms",
        type=float,
        default=2.0,
        help=(
            "Floor (in ms) on the time gap used in the local-speed calculation's "
            "denominator only (never for the interpolated position itself). "
            "Without this, two bracketing tracker samples a fraction of a "
            "millisecond apart - routine timing jitter, not real motion - can "
            "have ordinary measurement noise divide out to an absurd, "
            "meaningless speed. Default: 2.0ms; raise it if your tracker's own "
            "timing jitter is coarser than that (check residual_std_s in your "
            "time_sync JSON as a reference point)."
        ),
    )
    parser.add_argument("--output", type=Path, default=Path("track_logs/stereo/img_points.npz"))
    args = parser.parse_args()

    if args.hz <= 0 or args.max_gap <= 0:
        parser.error("--hz and --max-gap must be positive")

    # ---- Time sync: reuse stereo/time_sync.py's beacon+counter matching ----
    dec1 = load_decoded(args.cam1_h5)
    dec2 = load_decoded(args.cam2_h5)
    print(f"Camera 1 beacons with decoded data: {sorted(dec1)}")
    print(f"Camera 2 beacons with decoded data: {sorted(dec2)}")

    t1_raw, t2_raw, _ = match_by_counter(dec1, dec2)
    print(f"\nRaw counter matches (mostly spurious): {len(t1_raw)}")
    if len(t1_raw) < args.min_inliers:
        parser.error(
            f"Only {len(t1_raw)} raw counter matches found - not enough overlap between "
            "the two cameras' decoded beacons to time-sync."
        )

    rng = np.random.default_rng(args.seed)
    threshold_s = args.threshold_ms / 1000.0
    a, b, inlier_mask = ransac_fit_line(t1_raw, t2_raw, threshold_s, args.iterations, args.min_inliers, rng)
    num_inliers = int(inlier_mask.sum())
    print(f"RANSAC inliers: {num_inliers}/{len(t1_raw)} ({100 * num_inliers / len(t1_raw):.1f}%)")
    print(f"Fit: t_cam2 = {a:.8f} * t_cam1 + {b:.6f}")

    # ---- Build synced image-point views from tracker data ----
    tracks1 = load_tracks(args.cam1_h5)
    tracks2 = load_tracks(args.cam2_h5)

    img_points_cam1, img_points_cam2, sample_times, beacon_ids, view_max_speed, rejection_counts = (
        build_stereo_image_points(
            tracks1, tracks2, a, b, args.hz, args.max_gap, args.max_velocity,
            args.min_speed_dt_ms / 1000.0, args.min_points,
        )
    )

    num_views = len(img_points_cam1)
    print(f"\nBeacon IDs seen in both cameras' tracker data (not all necessarily in every view): {beacon_ids}")
    if num_views == 0:
        raise SystemExit(
            f"No synchronized views survived at {args.hz} Hz with --min-points {args.min_points} - "
            "try a lower --min-points (fewer required simultaneously-visible beacons per view), a "
            "larger --max-gap, or a lower --hz. This is usually NOT a clock-sync problem - check "
            "rejection_counts above: 'insufficient_points' means views existed but didn't have "
            "enough jointly-visible beacons, which --min-points controls directly."
        )
    points_per_view = np.array([int(np.sum(~np.isnan(v[:, 0]))) for v in img_points_cam1])
    print(f"Sampled at {args.hz} Hz: {num_views} synchronized stereo views kept "
          f"(of {num_views + rejection_counts['insufficient_points']} candidates)")
    print(f"Points/view: min={points_per_view.min()}, median={int(np.median(points_per_view))}, "
          f"max={points_per_view.max()} (out of {len(beacon_ids)} beacons seen overall)")
    print(
        f"Rejected views (fewer than --min-points {args.min_points} beacons survived): "
        f"{rejection_counts['insufficient_points']}"
    )
    print(
        f"Individual beacon-slots dropped (not whole views): "
        f"{rejection_counts['total_missing_or_gap_slots']} missing/out-of-gap, "
        f"{rejection_counts['total_velocity_rejected_slots']} exceeded --max-velocity"
        + (f"={args.max_velocity:g} px/s" if args.max_velocity is not None else " (not set)")
    )
    if num_views:
        print(
            f"Kept views' max interpolation speed (px/s): "
            f"median={np.median(view_max_speed):.1f}  p95={np.percentile(view_max_speed, 95):.1f}  "
            f"max={view_max_speed.max():.1f}"
        )
        if args.max_velocity is None and view_max_speed.max() > 200.0:
            print(
                "NOTE: some kept views have a high interpolation speed even though --max-velocity "
                "is not set. If per-view reprojection RMSE (from evaluation/calibration_quality.py's "
                "observations.csv, grouped by view_index) shows a periodic pattern, try rerunning "
                "with --max-velocity set below this run's p95 value and compare."
            )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        img_points_cam1=np.stack(img_points_cam1),  # (num_views, num_beacons, 2)
        img_points_cam2=np.stack(img_points_cam2),  # (num_views, num_beacons, 2)
        sample_times_cam1=sample_times,  # (num_views,)
        beacon_ids=np.asarray(beacon_ids),
        view_max_speed_px_per_s=view_max_speed,  # (num_views,) - diagnostic even when not filtered on
        a=a,
        b=b,
    )
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
