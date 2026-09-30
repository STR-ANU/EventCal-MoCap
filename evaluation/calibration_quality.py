#!/usr/bin/env python3
"""
Evaluate intrinsic/stereo calibration outputs using the metrics specified in
"Vicon and Event Camera Calibration Evaluation Instructions", version 1.4.

The script computes metrics from synchronized stereo correspondences:

* per-camera and pooled free-triangulation reprojection error;
* square-root Sampson distance (primary) and symmetric epipolar distance;
* acute triangulation angle;
* freely reconstructed LED-pair length errors;
* adjacent-LED spacing consistency, localized per board region/axis (M6;
  diagnostic only - see the warning in the report's adjacent_led_spacing
  section);
* available intrinsic-fit and clock-fit diagnostics.

It deliberately does not turn calibration-fit observations into a final pass.
Use --partition dev or --partition test only with a physically separate
recording that was not used to estimate the supplied calibration. Reliability
metrics are intentionally excluded because the current pipeline does not save
an independently defined opportunity schedule.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from led_spacing import (
    evaluate as evaluate_adjacent_spacing,
    grid_pairs as adjacent_grid_pairs,
    save as save_adjacent_spacing,
)


EPS = 1e-12

# Metric IDs retained for the current pipeline. M9 and the other draft metrics
# are intentionally omitted because the required source evidence is unavailable.
METRIC_DEFINITIONS = {
    "M1": "RMS reprojection consistency",
    "M2": "Median and P95 reprojection consistency",
    "M3": "Acute triangulation angle",
    "M4": "Square-root Sampson epipolar distance (diagnostic by default; --gate-sampson opts back into gating)",
    "M4b": "Symmetric epipolar distance (diagnostic; complements M4, never gated)",
    "M5": "Two-LED reconstructed-length error (long-diagonal pairs; supports independently measured references)",
    "M6": "Adjacent-LED spacing consistency (relative distance vs board geometry) - the PRIMARY distance-based gate",
}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


_H5_TIMESTAMP_RE = re.compile(r"(\d{2}-\d{2}-\d{4}-\d{2}-\d{2}-\d{2})")


def parse_h5_recording_start(h5_path: str) -> datetime | None:
    """Extract the recording-start timestamp encoded in this repo's H5
    filename convention (DD-MM-YYYY-HH-MM-SS_..., e.g.
    '22-09-2026-11-06-20_10000000_20000000.h5'). Returns None if the
    filename doesn't match - callers must treat that as "can't check", not
    as evidence of anything."""
    match = _H5_TIMESTAMP_RE.search(Path(h5_path).name)
    if not match:
        return None
    try:
        return datetime.strptime(match.group(1), "%d-%m-%Y-%H-%M-%S")
    except ValueError:
        return None


def recording_start_gap_seconds(cam1_h5: str, cam2_h5: str) -> float | None:
    """Seconds between the two cameras' recording start times, or None if
    either filename doesn't carry a parseable timestamp. This is a heuristic
    proxy for "were these two recordings actually the same simultaneous
    session" - it cannot prove simultaneity (both could start close together
    and still not be the same session), but a LARGE gap is strong evidence
    they are NOT the same session, which is exactly the failure mode this
    check exists to catch."""
    t1 = parse_h5_recording_start(cam1_h5)
    t2 = parse_h5_recording_start(cam2_h5)
    if t1 is None or t2 is None:
        return None
    return abs((t2 - t1).total_seconds())


def motion_correlation(
    img1: np.ndarray, img2: np.ndarray, min_beacons_per_step: int = 3, min_steps: int = 5
) -> float | None:
    """Correlation between the two cameras' per-view motion magnitude - the
    actual signature of a genuine simultaneous stereo capture, and a
    materially better check than recording_start_gap_seconds() for the
    failure mode it's meant to catch: two intrinsic-only sessions recorded
    close together in time (so the timestamp gap looks fine) can still be
    two INDEPENDENT board motions, one per camera, with no real
    correspondence between them at all. If both cameras watched the SAME
    physical board motion (from different viewpoints), their per-view
    motion magnitude - how far the board moved between consecutive synced
    samples - should rise and fall together (the same pauses and bursts of
    movement, at the same view indices), giving a strong positive
    correlation. Two independently-moving boards have no reason to line up
    this way. This held up cleanly on synthetic data with a simple (roughly
    affine) relationship between the two cameras' 2D motion: genuinely
    simultaneous captures scored 0.99+, independent sessions scored -0.6 to
    +0.3, across 20 random trials each.

    IMPORTANT CAVEAT, found by testing against real data: with a real
    wide-baseline rig and genuine 3D tilting motion (exactly the pose
    coverage this evaluator otherwise recommends), the same physical motion
    does NOT project to similarly-shaped 2D motion from very different
    viewpoints - that's foreshortening, not evidence of unrelated
    recordings. A verified-good, verified-simultaneous real capture scored
    only ~0.52 here, nowhere near the clean synthetic separation above. This
    is why the value is reported as an informational prompt rather than a
    hard pass/fail gate (see calibration_quality's --min-motion-correlation
    help and make_decision(), which does not fail a run for a low value
    here) - useful as a "worth double-checking" signal, not as proof on its
    own.

    img1/img2 are (num_views, num_beacons, 2), already synchronized to the
    same view indices (as build_stereo_points.py produces) - this only
    checks whether that alignment reflects a real shared event, not whether
    the alignment itself was computed correctly.

    Returns None (not 0.0 - a real "can't tell", not "definitely bad") when
    there are too few usable steps or either camera shows essentially zero
    motion throughout (e.g. a static intrinsics-only test recording), since
    correlation is meaningless without genuine variation in both series.
    """
    num_views = img1.shape[0]
    motion1: list[float] = []
    motion2: list[float] = []
    for i in range(num_views - 1):
        d1 = img1[i + 1] - img1[i]
        d2 = img2[i + 1] - img2[i]
        valid1 = np.all(np.isfinite(d1), axis=1)
        valid2 = np.all(np.isfinite(d2), axis=1)
        if valid1.sum() < min_beacons_per_step or valid2.sum() < min_beacons_per_step:
            continue
        motion1.append(float(np.linalg.norm(d1[valid1], axis=1).mean()))
        motion2.append(float(np.linalg.norm(d2[valid2], axis=1).mean()))
    if len(motion1) < min_steps:
        return None
    m1, m2 = np.asarray(motion1), np.asarray(motion2)
    if np.std(m1) < 1e-9 or np.std(m2) < 1e-9:
        return None
    return float(np.corrcoef(m1, m2)[0, 1])


def percentile(values: np.ndarray, q: float) -> float | None:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, q, method="linear")) if len(values) else None


def distribution(values: np.ndarray) -> dict[str, float | int | None]:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"valid_count": 0, "rms": None, "median": None, "p95": None, "maximum": None}
    return {
        "valid_count": int(len(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "median": percentile(values, 50),
        "p95": percentile(values, 95),
        "maximum": float(np.max(values)),
    }


def parse_pairs(spec: str | None, beacon_ids: list[int], columns: int, rows: int) -> list[tuple[int, int]]:
    if spec:
        pairs: list[tuple[int, int]] = []
        for token in spec.split(","):
            fields = token.strip().replace(":", "-").split("-")
            if len(fields) != 2:
                raise ValueError(f"Invalid pair {token!r}; use LED_A-LED_B")
            a, b = int(fields[0]), int(fields[1])
            if a == b:
                raise ValueError(f"Pair {token!r} uses the same LED twice")
            pairs.append(tuple(sorted((a, b))))
        return sorted(set(pairs))

    available = set(beacon_ids)
    first = min(beacon_ids)
    candidates = [
        (first, first + columns - 1),                # long horizontal
        (first, first + columns * (rows - 1)),       # long vertical
        (first, first + columns * rows - 1),         # board diagonal
        (first + columns - 1, first + columns * (rows - 1)),  # opposite diagonal
    ]
    return [pair for pair in candidates if pair[0] in available and pair[1] in available]


def load_pair_references(path: Path | None) -> dict[tuple[int, int], float]:
    """Load independently measured pair lengths.

    Required columns are led_a, led_b and reference_length_mm. Extra columns,
    such as uncertainty_mm and measurement_method, are preserved in the source
    file but are not needed for the numerical calculation.
    """
    if path is None:
        return {}
    references: dict[tuple[int, int], float] = {}
    with path.open(newline="") as fp:
        reader = csv.DictReader(fp)
        required = {"led_a", "led_b", "reference_length_mm"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path} must contain columns {sorted(required)}")
        for row in reader:
            a, b = sorted((int(row["led_a"]), int(row["led_b"])))
            length = float(row["reference_length_mm"])
            if a == b or not np.isfinite(length) or length <= 0:
                raise ValueError(f"Invalid pair-reference row: {row}")
            references[(a, b)] = length
    return references


def camera_parameters(
    intrinsic: dict[str, Any], stereo: dict[str, Any], camera_number: int
) -> tuple[np.ndarray, np.ndarray]:
    # Prefer the frozen values written by stereo_calibrate; fall back to the
    # individual intrinsic file for compatibility with older outputs. The
    # fallback must be lazy (only evaluated when actually needed) - eagerly
    # evaluating intrinsic["camera_matrix"] as a dict.get() default raises
    # KeyError even when the frozen stereo value was going to be used anyway
    # (e.g. intrinsic={} is legitimate once stereo carries frozen values).
    k_key = f"camera_matrix_{camera_number}"
    d_key = f"distortion_{camera_number}"
    K = np.asarray(stereo[k_key] if k_key in stereo else intrinsic["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(
        stereo[d_key] if d_key in stereo else intrinsic["distortion_coefficients"], dtype=np.float64
    ).reshape(-1, 1)
    return K, distortion


def undistort_pixel_points(points: np.ndarray, K: np.ndarray, distortion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(points, K, distortion).reshape(-1, 2)
    pixel = cv2.undistortPoints(points, K, distortion, P=K).reshape(-1, 2)
    return normalized, pixel


def fundamental_from_rt(K1: np.ndarray, K2: np.ndarray, R: np.ndarray, T: np.ndarray) -> np.ndarray:
    tx, ty, tz = T.reshape(3)
    skew_t = np.array([[0.0, -tz, ty], [tz, 0.0, -tx], [-ty, tx, 0.0]])
    return np.linalg.inv(K2).T @ skew_t @ R @ np.linalg.inv(K1)


def epipolar_errors(points1: np.ndarray, points2: np.ndarray, F: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x1 = np.column_stack([points1, np.ones(len(points1))])
    x2 = np.column_stack([points2, np.ones(len(points2))])
    Fx1 = (F @ x1.T).T
    Ftx2 = (F.T @ x2.T).T
    algebraic = np.sum(x2 * Fx1, axis=1)
    n2 = np.hypot(Fx1[:, 0], Fx1[:, 1])
    n1 = np.hypot(Ftx2[:, 0], Ftx2[:, 1])
    sampson_den = np.sqrt(np.square(n1) + np.square(n2))
    valid = (n1 > EPS) & (n2 > EPS) & (sampson_den > EPS)
    sampson = np.full(len(points1), np.nan)
    symmetric = np.full(len(points1), np.nan)
    sampson[valid] = np.abs(algebraic[valid]) / sampson_den[valid]
    symmetric[valid] = 0.5 * np.abs(algebraic[valid]) * (
        1.0 / n1[valid] + 1.0 / n2[valid]
    )
    return sampson, symmetric


def triangulate_and_score(
    img1: np.ndarray,
    img2: np.ndarray,
    K1: np.ndarray,
    d1: np.ndarray,
    K2: np.ndarray,
    d2: np.ndarray,
    R: np.ndarray,
    T: np.ndarray,
) -> dict[str, np.ndarray]:
    baseline_length = float(np.linalg.norm(np.asarray(T, dtype=np.float64).reshape(3)))
    if not np.isfinite(baseline_length) or baseline_length <= EPS:
        raise ValueError(
            f"Degenerate stereo baseline (||T||={baseline_length!r}); a zero-length "
            "or non-finite baseline cannot be triangulated from - check the stereo "
            "calibration's R/T (or camera_parameters()'s fallback), not the images."
        )

    shape = img1.shape
    flat1 = img1.reshape(-1, 2).astype(np.float64)
    flat2 = img2.reshape(-1, 2).astype(np.float64)

    # A NaN/Inf pixel coordinate must never reach cv2.undistortPoints /
    # cv2.triangulatePoints: besides being meaningless, it risks a spurious
    # "valid" result downstream (e.g. a dummy (0,0) pixel can triangulate to
    # a perfectly finite, positive-depth point). Substitute a finite dummy
    # value only for the underlying math, and track the real rows via
    # raw_finite / failure_code so they are excluded regardless of what that
    # placeholder computation produces.
    raw_finite = np.all(np.isfinite(flat1), axis=1) & np.all(np.isfinite(flat2), axis=1)
    safe1 = np.where(raw_finite[:, None], flat1, 0.0)
    safe2 = np.where(raw_finite[:, None], flat2, 0.0)

    norm1, undist_px1 = undistort_pixel_points(safe1, K1, d1)
    norm2, undist_px2 = undistort_pixel_points(safe2, K2, d2)

    P1 = np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = np.hstack([R, T.reshape(3, 1)])
    homogeneous = cv2.triangulatePoints(P1, P2, norm1.T, norm2.T)
    w = homogeneous[3]
    points3d = np.full((len(flat1), 3), np.nan)
    valid_w = np.isfinite(w) & (np.abs(w) > EPS)
    points3d[valid_w] = (homogeneous[:3, valid_w] / w[valid_w]).T

    points_cam2 = (R @ points3d.T + T.reshape(3, 1)).T
    valid = (
        raw_finite
        & valid_w
        & np.all(np.isfinite(points3d), axis=1)
        & (points3d[:, 2] > EPS)
        & (points_cam2[:, 2] > EPS)
    )

    # Per-point diagnosis of WHY a point is invalid, in priority order: a bad
    # input coordinate is reported as such even if the placeholder math
    # happened to also look degenerate; among finite inputs, a failed
    # triangulation (parallel/degenerate rays) is distinguished from a
    # successful-but-behind-either-camera reconstruction.
    failure_code = np.full(len(flat1), "", dtype=object)
    failure_code[~raw_finite] = "nonfinite_image_coordinate"
    unexplained = raw_finite & ~valid
    failure_code[unexplained & ~valid_w] = "triangulation_degenerate"
    failure_code[unexplained & valid_w] = "behind_camera"

    projected1 = np.full_like(flat1, np.nan)
    projected2 = np.full_like(flat2, np.nan)
    if np.any(valid):
        p1, _ = cv2.projectPoints(
            points3d[valid], np.zeros(3), np.zeros(3), K1, d1
        )
        rvec2, _ = cv2.Rodrigues(R)
        p2, _ = cv2.projectPoints(points3d[valid], rvec2, T, K2, d2)
        projected1[valid] = p1.reshape(-1, 2)
        projected2[valid] = p2.reshape(-1, 2)

    residual1 = flat1 - projected1
    residual2 = flat2 - projected2
    reproj1 = np.linalg.norm(residual1, axis=1)
    reproj2 = np.linalg.norm(residual2, axis=1)
    reproj1[~valid] = np.nan
    reproj2[~valid] = np.nan

    camera2_center = -R.T @ T.reshape(3)
    ray1 = points3d.copy()
    ray2 = points3d - camera2_center
    ray1_norm = np.linalg.norm(ray1, axis=1)
    ray2_norm = np.linalg.norm(ray2, axis=1)
    ray_valid = valid & (ray1_norm > EPS) & (ray2_norm > EPS)
    angles = np.full(len(flat1), np.nan)
    dots = np.sum(ray1[ray_valid] * ray2[ray_valid], axis=1) / (
        ray1_norm[ray_valid] * ray2_norm[ray_valid]
    )
    raw_angles = np.degrees(np.arccos(np.clip(dots, -1.0, 1.0)))
    angles[ray_valid] = np.minimum(raw_angles, 180.0 - raw_angles)

    F = fundamental_from_rt(K1, K2, R, T)
    sampson, symmetric = epipolar_errors(undist_px1, undist_px2, F)
    sampson[~valid] = np.nan
    symmetric[~valid] = np.nan

    return {
        "points3d": points3d.reshape(shape[0], shape[1], 3),
        "valid": valid.reshape(shape[0], shape[1]),
        "failure_code": failure_code.reshape(shape[0], shape[1]),
        "projected1": projected1.reshape(shape),
        "projected2": projected2.reshape(shape),
        "residual1": residual1.reshape(shape),
        "residual2": residual2.reshape(shape),
        "reproj1": reproj1.reshape(shape[:2]),
        "reproj2": reproj2.reshape(shape[:2]),
        "sampson": sampson.reshape(shape[:2]),
        "symmetric": symmetric.reshape(shape[:2]),
        "angle": angles.reshape(shape[:2]),
        "F_undistorted_pixels": F,
    }


def geometry_mm(stereo: dict[str, Any], beacon_ids: list[int], unit: str) -> dict[int, np.ndarray]:
    geometry = stereo["geometry"]
    columns = int(geometry["columns"])
    spacing_x = float(geometry["spacing_x"])
    spacing_y = float(geometry["spacing_y"])
    first = int(geometry.get("h5_id_for_led1", 0))
    factor = {"m": 1000.0, "cm": 10.0, "mm": 1.0}[unit]
    result: dict[int, np.ndarray] = {}
    for beacon in beacon_ids:
        index = beacon - first
        row, column = divmod(index, columns)
        result[beacon] = factor * np.array(
            [column * spacing_x, row * spacing_y, 0.0], dtype=np.float64
        )
    return result


def adjacent_led_pairs(
    beacon_ids: list[int], columns: int, rows: int, h5_id_for_led1: int = 0
) -> list[tuple[int, int, str]]:
    """(beacon_a, beacon_b, direction) for every one-grid-step neighbor pair
    on the assumed board layout - i.e. every edge of the LED grid graph, not
    just the long-diagonal M5 pairs. Used for the M6 adjacent-spacing check,
    which complements M4/M5 by localizing errors to a specific board region
    and axis instead of reporting one pooled number.

    Assumes the same row-major beacon_id = h5_id_for_led1 + row * columns +
    column convention as make_led_box_geometry() in stereo_calibrate.py and
    geometry_mm() above. If your board's actual firmware ID mapping doesn't
    follow that convention (see the instructions document, Section 18.2 on
    board-diagram numbering), these pairs - and therefore M6 - will be wrong;
    verify the mapping with a single-LED illumination test first.
    """
    available = set(beacon_ids)
    pairs: list[tuple[int, int, str]] = []
    for r in range(rows):
        for c in range(columns - 1):
            a = h5_id_for_led1 + r * columns + c
            b = a + 1
            if a in available and b in available:
                pairs.append((a, b, "horizontal"))
    for r in range(rows - 1):
        for c in range(columns):
            a = h5_id_for_led1 + r * columns + c
            b = a + columns
            if a in available and b in available:
                pairs.append((a, b, "vertical"))
    return pairs


def length_metrics(
    points3d: np.ndarray,
    valid: np.ndarray,
    beacon_ids: list[int],
    pairs: list[tuple[int, int]],
    references: dict[int, np.ndarray],
    measured_pair_references: dict[tuple[int, int], float],
    scale_to_mm: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    index = {beacon: i for i, beacon in enumerate(beacon_ids)}
    rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for a, b in pairs:
        if a not in index or b not in index:
            continue
        ia, ib = index[a], index[b]
        reference_source = (
            "independently_measured_pair_reference"
            if (a, b) in measured_pair_references
            else "calibration_geometry"
        )
        reference = measured_pair_references.get(
            (a, b), float(np.linalg.norm(references[a] - references[b]))
        )
        errors: list[float] = []
        for view in range(points3d.shape[0]):
            is_valid = bool(valid[view, ia] and valid[view, ib])
            reconstructed = None
            error = None
            if is_valid:
                reconstructed = scale_to_mm * float(
                    np.linalg.norm(points3d[view, ia] - points3d[view, ib])
                )
                error = reconstructed - reference
                errors.append(error)
            rows.append(
                {
                    "view_index": view,
                    "pair_id": f"{a}-{b}",
                    "led_a": a,
                    "led_b": b,
                    "valid": int(is_valid),
                    "reference_length_mm": reference,
                    "reference_source": reference_source,
                    "reconstructed_length_mm": reconstructed,
                    "length_error_mm": error,
                    "absolute_length_error_mm": abs(error) if error is not None else None,
                }
            )

        e = np.asarray(errors, dtype=np.float64)
        if len(e):
            rmse = float(np.sqrt(np.mean(np.square(e))))
            summaries.append(
                {
                    "pair_id": f"{a}-{b}",
                    "valid_count": int(len(e)),
                    "reference_length_mm": reference,
                    "reference_source": reference_source,
                    "bias_mm": float(np.mean(e)),
                    "mae_mm": float(np.mean(np.abs(e))),
                    "rmse_mm": rmse,
                    "relative_rmse_percent": 100.0 * rmse / reference,
                    "sample_sd_mm": float(np.std(e, ddof=1)) if len(e) >= 2 else None,
                    "median_absolute_error_mm": percentile(np.abs(e), 50),
                    "p95_absolute_error_mm": percentile(np.abs(e), 95),
                    "maximum_absolute_error_mm": float(np.max(np.abs(e))),
                    "centered_rms_mm": float(np.sqrt(np.mean(np.square(e - np.mean(e))))),
                }
            )
    return rows, summaries


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def metric_row(metric: str, unit: str, scope: str, stats: dict[str, Any]) -> dict[str, Any]:
    return {"metric": metric, "unit": unit, "scope": scope, **stats}


def candidate_pixel_label(rms: float | None) -> str:
    if rms is None:
        return "NOT_EVALUATED"
    if rms <= 0.5:
        return "GOOD_CANDIDATE"
    if rms <= 1.0:
        return "ACCEPTABLE_CANDIDATE"
    if rms <= 2.0:
        return "UNRELIABLE"
    return "FAILED"


def make_decision(
    partition: str,
    reproj_rms: float | None,
    sampson_rms: float | None,
    pair_relative_rmse: float | None,
    adjacent_relative_rmse: float | None,
    adjacent_evaluable_pairs: int,
    adjacent_planned_pairs: int,
    angle_p5: float | None,
    heldout_data_confirmed: bool,
    coverage_complete: bool,
    repeatability_runs: int,
    gate_sampson: bool,
    thresholds: argparse.Namespace,
    recording_start_gap_s: float | None = None,
    motion_corr: float | None = None,
) -> dict[str, Any]:
    # The relative-distance check (M6, adjacent-LED spacing) is the primary
    # distance-based gate. M4 (Sampson) is diagnostic-only by default - pass
    # --gate-sampson to opt back into the older behavior of also gating on it.
    gates: dict[str, dict[str, Any]] = {
        "pooled_reprojection_rms": {
            "value": reproj_rms,
            "threshold": thresholds.max_reprojection_rms,
            "pass": reproj_rms is not None and reproj_rms <= thresholds.max_reprojection_rms,
        },
        "pair_balanced_relative_rmse": {
            "value": pair_relative_rmse,
            "threshold": thresholds.max_relative_length_rmse,
            "pass": pair_relative_rmse is not None
            and pair_relative_rmse <= thresholds.max_relative_length_rmse,
        },
        "adjacent_spacing_relative_rmse": {
            "value": adjacent_relative_rmse,
            "threshold": thresholds.max_adjacent_spacing_relative_rmse,
            "pass": adjacent_relative_rmse is not None
            and adjacent_relative_rmse <= thresholds.max_adjacent_spacing_relative_rmse,
        },
        "triangulation_angle_p5": {
            "value": angle_p5,
            "threshold": thresholds.min_angle_p5,
            "pass": angle_p5 is not None and angle_p5 >= thresholds.min_angle_p5,
        },
    }
    if gate_sampson:
        gates["sqrt_sampson_rms"] = {
            "value": sampson_rms,
            "threshold": thresholds.max_sampson_rms,
            "pass": sampson_rms is not None and sampson_rms <= thresholds.max_sampson_rms,
        }

    failed = [name for name, gate in gates.items() if gate["value"] is not None and not gate["pass"]]
    missing = [name for name, gate in gates.items() if gate["value"] is None]

    # A large gap between the two cameras' recording start TIMESTAMPS (parsed
    # from each .h5's filename) used to be treated as a hard data-integrity
    # failure. It is now informational only, not gated: this pipeline's
    # actual cross-camera synchronization is established independently in
    # stereo/time_sync.py, by matching each camera's decoded LED blink
    # counter values via RANSAC - a signal completely decoupled from when
    # either camera's recording FILE happened to start. For a rig where the
    # two cameras are driven by two separate machines (so a sub-few-second
    # gap between recording starts isn't achievable), this filename-based
    # heuristic produces exactly the kind of false positive it warned about
    # in its own docstring ("it cannot prove simultaneity ... a LARGE gap is
    # strong evidence they are NOT the same session" - which is not true
    # here: the gap reflects unavoidable multi-machine start lag, not a
    # provenance problem). time_sync.py's own inlier ratio and residual
    # statistics are the correct, direct evidence of whether the two
    # cameras' correspondences are real; this check is kept only as a cheap,
    # non-authoritative sanity note alongside them.
    recording_gap_exceeded = (
        recording_start_gap_s is not None
        and recording_start_gap_s > thresholds.max_recording_start_gap_s
    )

    # This is a genuinely useful signal (it's what caught the "two
    # independent intrinsics-only sessions fed into stereo" bug in the first
    # place), but real-data testing showed it does NOT cleanly separate
    # good-vs-bad the way idealized synthetic data suggested: a real
    # wide-baseline rig viewing genuine 3D tilting motion (exactly the
    # coverage this evaluator otherwise recommends) can show only moderate
    # raw motion-magnitude correlation between the two cameras even on a
    # known-good, verified-simultaneous capture, because the same physical
    # motion doesn't project to affinely-related 2D motion from very
    # different viewpoints - that's foreshortening, not evidence of two
    # unrelated recordings. So this is reported for you to look at, not
    # gated: treat a low value as a prompt to double-check provenance
    # (alongside recording_start_gap_seconds and just knowing what you
    # recorded), not as proof by itself.
    motion_uncorrelated = (
        motion_corr is not None and motion_corr < thresholds.min_motion_correlation
    )

    if partition == "calibration_fit":
        missing.append("physically_disjoint_heldout_dataset")
    if not heldout_data_confirmed:
        missing.append("heldout_data_provenance_declaration")
    if not coverage_complete:
        missing.append("required_depth_region_orientation_coverage")
    if repeatability_runs < 3:
        missing.append("three_independent_calibration_repetitions")
    if adjacent_evaluable_pairs < adjacent_planned_pairs:
        # Distinct from an id being automatically excluded (handled upstream
        # in id_availability/led_spacing's own accounting): this means a
        # pair between two IDs that WERE both available still couldn't be
        # scored in any view - a genuine coverage gap, not a filtering
        # side-effect.
        missing.append("one_or_more_physical_adjacent_pairs_unobserved")

    if failed:
        status = "FAILED"
    elif missing:
        status = "NOT_EVALUATED"
    else:
        status = "PASS"

    recommendations: list[str] = []

    if recording_gap_exceeded:
        recommendations.append(
            f"FYI, not a failure: cam1_h5 and cam2_h5 started recording "
            f"{recording_start_gap_s:.0f}s apart (above the informational "
            f"{thresholds.max_recording_start_gap_s:.0f}s reference). This isn't gated because "
            "this pipeline's actual cross-camera synchronization comes from matching decoded LED "
            "blink counter values (stereo/time_sync.py's RANSAC fit), not from the two recording "
            "files' start timestamps - a real multi-machine rig (each camera on its own laptop, "
            "with unavoidable startup lag between them) will always show a gap here even on a "
            "genuinely well-synchronized capture. Check time_sync.py's own inlier_ratio and "
            "residual_std_s instead for the real evidence of synchronization quality; a low "
            "inlier ratio or large residual there - not this timestamp gap - is what would "
            "actually indicate a provenance problem."
        )
    if motion_uncorrelated:
        recommendations.append(
            f"FYI, not a failure: the two cameras' raw motion correlation is {motion_corr:.2f} "
            f"(below the informational {thresholds.min_motion_correlation:.2f} reference). This "
            "isn't gated because a real wide-baseline rig with genuine 3D tilting motion can "
            "legitimately score low here even on a good simultaneous capture (the same physical "
            "motion doesn't project to similarly-shaped 2D motion from very different "
            "viewpoints). Worth a manual sanity check if you have any doubt about provenance - "
            "did both cameras record the SAME board motion, not two separate sessions - but a "
            "low number alone isn't proof of a problem."
        )

    if partition == "calibration_fit":
        recommendations.append(
            "Status can only reach PASS on a partition of 'dev' or 'test'. To produce one: "
            "(1) with the SAME physical rig/mount, record a NEW clip where the board is NOT static "
            "in the same spot used for calibration - this is the disjoint recording, not a rerun of "
            "the same one; (2) run tracking/decoding/build_stereo_points.py on that new clip to get "
            "a new stereo_points_*.npz; (3) re-run this evaluator passing that NEW npz as "
            "stereo_points_npz, but keep stereo_calibration_json/cam1_intrinsics_json/"
            "cam2_intrinsics_json pointed at THIS run's already-fitted, frozen calibration (don't "
            "re-fit them); (4) add --partition dev --heldout-data-confirmed. A 'test' partition, if "
            "you use one, should be a second, separate recording again - dev and test should not be "
            "the same clip either."
        )
    elif not heldout_data_confirmed:
        recommendations.append(
            "--partition is dev/test but --heldout-data-confirmed wasn't passed. Add it once you've "
            "verified: this NPZ came from a recording session physically separate from whatever "
            "produced the frozen calibration (not just a different time slice of the same "
            "recording), and cam1_intrinsics_json/cam2_intrinsics_json/stereo_calibration_json here "
            "are the SAME frozen files used to fit the calibration, unmodified."
        )
    if not coverage_complete:
        recommendations.append(
            "--coverage-complete wasn't passed. Before adding it, confirm the recording actually "
            "covers: the board visible in all four corners of the frame (not just centered), at "
            "least 3 distinct distances spanning your intended working range (near/mid/far), the "
            "board tilted roughly +/-30-45 degrees in both axes at multiple positions (not just "
            "fronto-parallel), and the board held STATIONARY for a brief dwell at each pose rather "
            "than swept continuously (continuous sweeping degrades stereo correspondence quality - "
            "see the per-view reprojection RMSE / view_max_speed_px_per_s diagnostics if M1 looks "
            "worse than expected despite good coverage)."
        )
    if repeatability_runs < 3:
        recommendations.append(
            f"--repeatability-runs is {repeatability_runs}; need >= 3. Record 3 independent physical "
            "calibration sessions (unmount/remount or at least fully redo the capture, not just "
            "reprocess the same recording 3 times), run the full pipeline on each, then compare "
            "fx/fy/cx/cy, R, T, and baseline across the 3 resulting stereo_calibration_*.json files. "
            "Large swings (e.g. baseline varying by more than a few percent) indicate the rig itself "
            "isn't rigid/stable enough for a trustworthy single calibration, independent of what any "
            "single run's M1-M6 numbers say."
        )
    if adjacent_evaluable_pairs < adjacent_planned_pairs:
        recommendations.append(
            f"{adjacent_planned_pairs - adjacent_evaluable_pairs} of {adjacent_planned_pairs} planned "
            "adjacent-LED pairs (see adjacent_spacing_report.json's excluded_pairs) were never "
            "jointly observed in any single view, even though both LEDs in each pair were available "
            "at some point. Check your board-pose coverage includes views where BOTH of those "
            "specific LEDs are visible together, not just each individually at different times."
        )
    if reproj_rms is not None and reproj_rms > thresholds.max_reprojection_rms:
        recommendations.append(
            f"M1 pooled reprojection RMS ({reproj_rms:.3f} px) exceeds {thresholds.max_reprojection_rms:.3f} px. "
            "Check in this order: (1) cam1_h5/cam2_h5 recording start times aren't far apart (see "
            "above); (2) observations.csv grouped by view_index for a periodic or one-off spike "
            "pattern (fast/continuous board motion - see build_stereo_points.py's --max-velocity); "
            "(3) observations.csv grouped by beacon_id/board row-column for a spatial bowl pattern "
            "(board pose coverage or LED geometry); (4) individual camera intrinsic_training_fit RMS "
            "in this same report - if those are already good (sub-pixel) but stereo is bad, the "
            "problem is specific to cross-camera correspondence/sync, not the cameras individually."
        )
    if gate_sampson and sampson_rms is not None and sampson_rms > thresholds.max_sampson_rms:
        recommendations.append(
            f"M4 Sampson RMS ({sampson_rms:.3f} px) exceeds {thresholds.max_sampson_rms:.3f} px with "
            "--gate-sampson set. This specifically implicates the epipolar geometry (R/T) rather "
            "than triangulated scale - check camera time synchronization (time_sync json's "
            "inlier_ratio and inlier_beacon_ids) and correspondence identity before re-fitting R/T."
        )
    if pair_relative_rmse is not None and pair_relative_rmse > thresholds.max_relative_length_rmse:
        recommendations.append(
            f"M5 pair-balanced relative RMSE ({pair_relative_rmse:.3f}%) exceeds "
            f"{thresholds.max_relative_length_rmse:.3f}%. Verify the LED geometry passed to "
            "stereo_calibrate.py (--spacing-x/--spacing-y, --columns/--rows) matches the physical "
            "board exactly, that its units match --geometry-unit here, and check "
            "heldout_distance_audit.csv per-pair for whether the error is uniform (points at a "
            "global scale/unit error) or concentrated in specific pairs (points at board geometry "
            "or a wiring/LED-numbering issue for those specific LEDs)."
        )
    if (
        adjacent_relative_rmse is not None
        and adjacent_relative_rmse > thresholds.max_adjacent_spacing_relative_rmse
    ):
        recommendations.append(
            f"M6 adjacent-LED spacing ({adjacent_relative_rmse:.3f}%) exceeds "
            f"{thresholds.max_adjacent_spacing_relative_rmse:.3f}%. Re-measure led_spacing_x/"
            "led_spacing_y on the physical board with calipers (don't trust a nominal/assumed "
            "value), check the board for non-planarity/warp, and compare "
            "adjacent_spacing_report.json's aggregates.horizontal vs .vertical: if one direction is "
            "consistently worse than the other, that points at a single mismeasured spacing "
            "constant for that axis rather than random noise."
        )
    if angle_p5 is not None and angle_p5 < thresholds.min_angle_p5:
        recommendations.append(
            f"M3 triangulation angle P5 ({angle_p5:.2f} deg) is below {thresholds.min_angle_p5:.2f} "
            "deg. Increase the stereo baseline, or record with the board closer to the cameras "
            "relative to the baseline - shallow ray angles make depth/Z weakly constrained "
            "regardless of how good the 2D correspondences are."
        )
    if not recommendations and status == "PASS":
        recommendations.append(
            "Calibration passes the configured DEV/TEST-frozen gates. Preserve the camera mount "
            "exactly as-is (don't loosen/retighten anything), and if you haven't already, confirm "
            "repeatability with >= 3 independent calibration sessions before relying on this for "
            "production 3D measurements."
        )
    if not recommendations:
        recommendations.append("Review all failed and missing gates before using the calibration for 3D measurements.")

    return {
        "status": status,
        "pixel_band_candidate": candidate_pixel_label(reproj_rms),
        "gates": gates,
        "failed_gates": sorted(set(failed)),
        "missing_required_evidence": sorted(set(missing)),
        "recommendations": recommendations,
        "threshold_note": (
            "These are configurable operational gates, not universal camera specifications. "
            "Freeze them on DEV before scoring TEST."
        ),
    }


def main() -> None:
    # Force line-buffered stdout. Without this, Python fully buffers stdout
    # whenever it isn't attached to an interactive terminal (e.g. when piped
    # into `tee`, redirected to a log file, or launched from another
    # process). In that situation nothing appears until the process exits,
    # which is what "the code shows nothing on the terminal" usually means -
    # the scorecard was only ever waiting in the buffer. `python -u` and
    # `stdbuf -oL` fix the same problem from outside the script; this fixes
    # it regardless of how the script is invoked.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass  # Older Python or a stream that doesn't support reconfigure.

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stereo_points_npz", type=Path)
    parser.add_argument("stereo_calibration_json", type=Path)
    parser.add_argument("cam1_intrinsics_json", type=Path)
    parser.add_argument("cam2_intrinsics_json", type=Path)
    parser.add_argument("--time-sync-json", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("track_logs/evaluation"))
    parser.add_argument(
        "--partition",
        choices=["calibration_fit", "dev", "test"],
        default="calibration_fit",
        help="Use dev/test only for a physically disjoint recording not used for calibration.",
    )
    parser.add_argument("--run-id", default="calibration_run")
    parser.add_argument("--geometry-unit", choices=["m", "cm", "mm"], default="m")
    parser.add_argument("--pairs", help="Comma-separated beacon pairs, e.g. 0-3,0-20,0-23,3-20")
    parser.add_argument(
        "--pair-reference-csv",
        type=Path,
        help="Optional measured references with led_a,led_b,reference_length_mm.",
    )
    parser.add_argument(
        "--heldout-data-confirmed",
        action="store_true",
        help="Declare that DEV/TEST inputs are physically disjoint and parameters were frozen.",
    )
    parser.add_argument(
        "--coverage-complete",
        action="store_true",
        help="Declare required depth/region/orientation/condition coverage complete.",
    )
    parser.add_argument("--repeatability-runs", type=int, default=1)
    parser.add_argument("--max-reprojection-rms", type=float, default=1.0)
    parser.add_argument("--max-sampson-rms", type=float, default=1.0)
    parser.add_argument("--max-relative-length-rmse", type=float, default=2.0)
    parser.add_argument("--max-adjacent-spacing-relative-rmse", type=float, default=2.0)
    parser.add_argument("--min-angle-p5", type=float, default=1.0)
    parser.add_argument(
        "--max-recording-start-gap-s",
        type=float,
        default=300.0,
        help=(
            "If --time-sync-json is provided and its cam1_h5/cam2_h5 filenames encode a "
            "start timestamp (DD-MM-YYYY-HH-MM-SS), fail the run when the two cameras' "
            "recording start times differ by more than this many seconds (default: 300s = "
            "5 minutes, generous enough for normal startup sequencing between the two "
            "cameras). This catches an obviously-different-session recording, but NOT two "
            "sessions started close together that captured unrelated motion - see "
            "--min-motion-correlation for that."
        ),
    )
    parser.add_argument(
        "--min-motion-correlation",
        type=float,
        default=0.5,
        help=(
            "Informational reference only (NOT gated - see motion_correlation()'s docstring for "
            "why): a value below this is printed as a prompt to double-check recording "
            "provenance, since it's how the 'two independent intrinsics-only sessions fed into "
            "stereo' bug was originally found. But real-data testing showed a genuine "
            "wide-baseline capture with real 3D tilting motion can legitimately score well below "
            "this even when correct (the same physical motion doesn't project to similarly-shaped "
            "2D motion from very different viewpoints), so this cannot safely be a hard gate."
        ),
    )
    parser.add_argument(
        "--gate-sampson",
        action="store_true",
        help=(
            "Also gate on M4 (square-root Sampson epipolar distance). By default M4 (and M4b, "
            "symmetric epipolar distance) are diagnostic-only, and the relative-distance check "
            "(M6, adjacent-LED spacing) is the primary distance-based gate."
        ),
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "Exit with status 1 if the final decision is not PASS (FAILED or NOT_EVALUATED). "
            "By default the process exits 0 whenever it completes normally, regardless of the "
            "decision, so CI pipelines opt into treating a non-PASS result as a build failure."
        ),
    )
    args = parser.parse_args()

    npz = np.load(args.stereo_points_npz)
    img1 = np.asarray(npz["img_points_cam1"], dtype=np.float64)
    img2 = np.asarray(npz["img_points_cam2"], dtype=np.float64)
    beacon_ids_all = [int(x) for x in npz["beacon_ids"].tolist()]
    sample_times = np.asarray(npz["sample_times_cam1"], dtype=np.float64)
    if img1.shape != img2.shape or img1.ndim != 3 or img1.shape[2] != 2:
        raise ValueError(f"Expected matching (views, beacons, 2) arrays, got {img1.shape} and {img2.shape}")
    if img1.shape[0] == 0:
        raise ValueError(
            f"{args.stereo_points_npz} has 0 views (no synchronized stereo correspondences) - "
            "nothing to evaluate."
        )

    # Upstream availability filtering: if the NPZ records which beacon IDs
    # were actually decoded per camera (decoded_beacon_ids_cam1/cam2 -
    # optional, distinct from which IDs later triangulated successfully),
    # any ID missing from EITHER camera's decoded set is automatically
    # excluded from every metric below, not just silently scored as invalid.
    # This must be established from upstream decode metadata (or an explicit
    # --availability-json-equivalent), never inferred from which distances
    # or triangulations happen to succeed - that would be circular.
    decoded1 = set(int(x) for x in npz["decoded_beacon_ids_cam1"]) if "decoded_beacon_ids_cam1" in npz.files else None
    decoded2 = set(int(x) for x in npz["decoded_beacon_ids_cam2"]) if "decoded_beacon_ids_cam2" in npz.files else None
    if decoded1 is not None or decoded2 is not None:
        avail1 = decoded1 if decoded1 is not None else set(beacon_ids_all)
        avail2 = decoded2 if decoded2 is not None else set(beacon_ids_all)
        evaluated_ids = sorted(set(beacon_ids_all) & avail1 & avail2)
        availability_source = "npz_decoded_beacon_ids"
    else:
        evaluated_ids = list(beacon_ids_all)
        availability_source = "no_upstream_availability_supplied"
    automatically_excluded_ids = sorted(set(beacon_ids_all) - set(evaluated_ids))
    id_availability = {
        "evaluated_ids": evaluated_ids,
        "automatically_excluded_ids": automatically_excluded_ids,
        "source": availability_source,
    }
    conditional_reconstruction = {
        "attempted": len(evaluated_ids),
        "automatically_excluded_count": len(automatically_excluded_ids),
        "total_beacon_ids_in_npz": len(beacon_ids_all),
    }
    keep_mask = np.array([b in set(evaluated_ids) for b in beacon_ids_all])
    beacon_ids = [b for b, keep in zip(beacon_ids_all, keep_mask) if keep]
    img1 = img1[:, keep_mask, :]
    img2 = img2[:, keep_mask, :]

    stereo = load_json(args.stereo_calibration_json)
    intrinsic1 = load_json(args.cam1_intrinsics_json)
    intrinsic2 = load_json(args.cam2_intrinsics_json)
    K1, d1 = camera_parameters(intrinsic1, stereo, 1)
    K2, d2 = camera_parameters(intrinsic2, stereo, 2)
    R = np.asarray(stereo["R"], dtype=np.float64)
    T = np.asarray(stereo["T"], dtype=np.float64).reshape(3)

    scored = triangulate_and_score(img1, img2, K1, d1, K2, d2, R, T)
    num_valid = int(np.sum(scored["valid"]))
    attempted = int(scored["valid"].size)

    reproj1_stats = distribution(scored["reproj1"])
    reproj2_stats = distribution(scored["reproj2"])
    pooled_reproj = distribution(
        np.concatenate([scored["reproj1"].ravel(), scored["reproj2"].ravel()])
    )
    sampson_stats = distribution(scored["sampson"])
    symmetric_stats = distribution(scored["symmetric"])
    angle_values = scored["angle"][np.isfinite(scored["angle"])]
    angle_stats = {
        "valid_count": int(len(angle_values)),
        "median": percentile(angle_values, 50),
        "p5": percentile(angle_values, 5),
        "minimum": float(np.min(angle_values)) if len(angle_values) else None,
    }

    geometry = stereo["geometry"]
    pairs = parse_pairs(
        args.pairs, beacon_ids, int(geometry["columns"]), int(geometry["rows"])
    )
    references = geometry_mm(stereo, beacon_ids, args.geometry_unit)
    measured_pair_references = load_pair_references(args.pair_reference_csv)
    if measured_pair_references and args.pairs is None:
        pairs = sorted(measured_pair_references)
    scale_to_mm = {"m": 1000.0, "cm": 10.0, "mm": 1.0}[args.geometry_unit]
    length_rows, pair_summaries = length_metrics(
        scored["points3d"],
        scored["valid"],
        beacon_ids,
        pairs,
        references,
        measured_pair_references,
        scale_to_mm,
    )
    pair_relative_values = np.asarray(
        [row["relative_rmse_percent"] for row in pair_summaries], dtype=np.float64
    )
    pair_balanced_relative_rmse = (
        float(np.sqrt(np.mean(np.square(pair_relative_values))))
        if len(pair_relative_values)
        else None
    )

    observations: list[dict[str, Any]] = []
    for view in range(img1.shape[0]):
        for led_index, beacon in enumerate(beacon_ids):
            point = scored["points3d"][view, led_index]
            r1 = scored["residual1"][view, led_index]
            r2 = scored["residual2"][view, led_index]
            code = scored["failure_code"][view, led_index]
            observations.append(
                {
                    "run_id": args.run_id,
                    "partition": args.partition,
                    "view_index": view,
                    "timestamp_cam1_s": float(sample_times[view]),
                    "beacon_id": beacon,
                    "u1": float(img1[view, led_index, 0]),
                    "v1": float(img1[view, led_index, 1]),
                    "u2": float(img2[view, led_index, 0]),
                    "v2": float(img2[view, led_index, 1]),
                    "valid_3d": int(scored["valid"][view, led_index]),
                    "failure_code": code if code else "",
                    "X_cam1_mm": float(point[0] * scale_to_mm) if np.all(np.isfinite(point)) else None,
                    "Y_cam1_mm": float(point[1] * scale_to_mm) if np.all(np.isfinite(point)) else None,
                    "Z_cam1_mm": float(point[2] * scale_to_mm) if np.all(np.isfinite(point)) else None,
                    "du1_px": float(r1[0]) if np.all(np.isfinite(r1)) else None,
                    "dv1_px": float(r1[1]) if np.all(np.isfinite(r1)) else None,
                    "reprojection1_px": float(scored["reproj1"][view, led_index])
                    if np.isfinite(scored["reproj1"][view, led_index]) else None,
                    "du2_px": float(r2[0]) if np.all(np.isfinite(r2)) else None,
                    "dv2_px": float(r2[1]) if np.all(np.isfinite(r2)) else None,
                    "reprojection2_px": float(scored["reproj2"][view, led_index])
                    if np.isfinite(scored["reproj2"][view, led_index]) else None,
                    "sqrt_sampson_px": float(scored["sampson"][view, led_index])
                    if np.isfinite(scored["sampson"][view, led_index]) else None,
                    "symmetric_epipolar_px": float(scored["symmetric"][view, led_index])
                    if np.isfinite(scored["symmetric"][view, led_index]) else None,
                    "acute_triangulation_angle_deg": float(scored["angle"][view, led_index])
                    if np.isfinite(scored["angle"][view, led_index]) else None,
                }
            )

    # M6 - adjacent-LED spacing consistency, the PRIMARY relative-distance
    # gate (see METRIC_DEFINITIONS). Delegated entirely to led_spacing.py,
    # which checks every physically-adjacent grid pair (not just the 4
    # long-diagonal M5 pairs) and handles duplicate/availability/exclusion
    # bookkeeping far more rigorously than a hand-rolled version here would.
    # It reuses the SAME `observations` records built above (including their
    # u1/v1/u2/v2 fields, which is how it independently infers per-camera
    # per-session availability rather than trusting valid_3d alone).
    spacing_columns = int(geometry["columns"])
    spacing_rows = int(geometry["rows"])
    spacing_first_id = int(geometry.get("h5_id_for_led1", 0))
    spacing_x_mm = float(geometry["spacing_x"]) * scale_to_mm
    spacing_y_mm = float(geometry["spacing_y"]) * scale_to_mm
    adjacent_pairs_grid = adjacent_grid_pairs(spacing_columns, spacing_rows, spacing_first_id)
    adjacent_result = evaluate_adjacent_spacing(
        observations, adjacent_pairs_grid, spacing_x_mm, spacing_y_mm
    )
    adjacent_relative_rmse = adjacent_result["aggregates"]["all"]["relative_rmse_percent"]

    # Heuristic check: were the two cameras' recordings actually simultaneous?
    # Requires --time-sync-json (its cam1_h5/cam2_h5 fields) and filenames
    # that carry a parseable start timestamp; silently skipped (None) if
    # either is unavailable, rather than guessing.
    recording_gap_s = None
    if args.time_sync_json:
        time_sync_data = load_json(args.time_sync_json)
        cam1_h5 = time_sync_data.get("cam1_h5")
        cam2_h5 = time_sync_data.get("cam2_h5")
        if cam1_h5 and cam2_h5:
            recording_gap_s = recording_start_gap_seconds(cam1_h5, cam2_h5)

    # The direct test of whether the two cameras observed one shared
    # physical event (see motion_correlation()'s docstring) - computed on
    # the same (already availability-filtered) img1/img2 arrays used for
    # everything else, so it reflects exactly the data being scored.
    motion_corr = motion_correlation(img1, img2)

    decision = make_decision(
        args.partition,
        pooled_reproj["rms"],
        sampson_stats["rms"],
        pair_balanced_relative_rmse,
        adjacent_relative_rmse,
        adjacent_result["evaluable_pairs"],
        adjacent_result["planned_pairs"],
        angle_stats["p5"],
        args.heldout_data_confirmed,
        args.coverage_complete,
        args.repeatability_runs,
        args.gate_sampson,
        args,
        recording_gap_s,
        motion_corr,
    )

    def file_sha256(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    input_sha256 = {
        "stereo_points_npz": file_sha256(args.stereo_points_npz),
        "stereo_calibration_json": file_sha256(args.stereo_calibration_json),
        "cam1_intrinsics_json": file_sha256(args.cam1_intrinsics_json),
        "cam2_intrinsics_json": file_sha256(args.cam2_intrinsics_json),
    }
    if args.time_sync_json:
        input_sha256["time_sync_json"] = file_sha256(args.time_sync_json)

    summary_rows = [
        metric_row("M1_rms_reprojection_consistency", "px", "camera_1", reproj1_stats),
        metric_row("M1_rms_reprojection_consistency", "px", "camera_2", reproj2_stats),
        metric_row("M1_rms_reprojection_consistency", "px", "pooled", pooled_reproj),
        metric_row("M2_reprojection_distribution", "px", "camera_1", reproj1_stats),
        metric_row("M2_reprojection_distribution", "px", "camera_2", reproj2_stats),
        metric_row("M3_acute_triangulation_angle", "deg", "stereo", angle_stats),
        metric_row("M4_sqrt_sampson_distance", "px", "stereo", sampson_stats),
        metric_row("M4b_symmetric_epipolar_distance", "px", "stereo", symmetric_stats),
    ]
    for item in pair_summaries:
        summary_rows.append(metric_row("M5_led_pair_length", "mm", item["pair_id"], item))
    for item in adjacent_result["per_pair"]:
        summary_rows.append(
            metric_row(f"M6_adjacent_led_spacing_{item['direction']}", "mm", item["pair"], item)
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "observations.csv", observations)
    write_csv(args.output_dir / "heldout_distance_audit.csv", length_rows)
    write_csv(args.output_dir / "summary.csv", summary_rows)
    # led_spacing's own save() writes adjacent_spacing_audit.csv (per-pair),
    # adjacent_spacing_samples.csv (per-observation), adjacent_spacing_by_view.csv,
    # adjacent_spacing_by_direction.csv, and its own adjacent_spacing_report.json.
    save_adjacent_spacing(adjacent_result, args.output_dir)

    report = {
        "run_id": args.run_id,
        "partition": args.partition,
        "reported_metric_ids": METRIC_DEFINITIONS,
        "input_sha256": input_sha256,
        "id_availability": id_availability,
        "conditional_reconstruction": conditional_reconstruction,
        "evidence_warning": (
            "Current correspondences are labelled calibration_fit and must not be represented as held-out "
            "accuracy." if args.partition == "calibration_fit" else None
        ),
        "inputs": {
            "stereo_points_npz": str(args.stereo_points_npz),
            "stereo_calibration_json": str(args.stereo_calibration_json),
            "cam1_intrinsics_json": str(args.cam1_intrinsics_json),
            "cam2_intrinsics_json": str(args.cam2_intrinsics_json),
        },
        "intrinsic_training_fit": {
            "camera_1_opencv_rms_px": intrinsic1.get("opencv_rms_px"),
            "camera_2_opencv_rms_px": intrinsic2.get("opencv_rms_px"),
            "camera_1_num_views": intrinsic1.get("num_views"),
            "camera_2_num_views": intrinsic2.get("num_views"),
            "note": "Training-fit metrics only; median/P95 per-point intrinsic residuals require saved observations.",
        },
        "stereo_training_fit": {
            "opencv_stereo_rms_px": stereo.get("stereo_rms_px"),
            "num_views": stereo.get("num_views"),
            "baseline_mm": float(np.linalg.norm(T)) * scale_to_mm,
        },
        "stereo_reprojection_consistency": {
            "camera_1_px": reproj1_stats,
            "camera_2_px": reproj2_stats,
            "pooled_px": pooled_reproj,
            "warning": (
                "Points were triangulated from and reprojected to the same two measurements. "
                "This is a stereo-consistency residual, not independent held-out board-model accuracy."
            ),
        },
        "epipolar": {
            "primary_definition": "square-root Sampson distance on undistorted pixel coordinates",
            "sqrt_sampson_px": sampson_stats,
            "symmetric_px": symmetric_stats,
        },
        "triangulation_angle_deg": angle_stats,
        "length_evaluation": {
            "classification": (
                "same-board calibration-fit consistency"
                if args.partition == "calibration_fit"
                else "same-board held-out geometric consistency"
            ),
            "geometry_unit_interpreted_as": args.geometry_unit,
            "planned_pairs": [f"{a}-{b}" for a, b in pairs],
            "evaluable_pairs": len(pair_summaries),
            "pair_reference_csv": str(args.pair_reference_csv) if args.pair_reference_csv else None,
            "pair_balanced_relative_rmse_percent": pair_balanced_relative_rmse,
            "per_pair": pair_summaries,
            "warning": (
                "Measured pair references were used where available; any remaining pair uses calibration geometry."
                if measured_pair_references
                else "Reference lengths come from the calibration geometry; this is not independent scale metrology."
            ),
        },
        "clock_fit": load_json(args.time_sync_json) if args.time_sync_json else None,
        "recording_simultaneity_check": {
            "start_time_gap_s": recording_gap_s,
            "max_allowed_gap_s": args.max_recording_start_gap_s,
            "checked": recording_gap_s is not None,
            "note": (
                "None means this couldn't be checked (--time-sync-json not given, or its "
                "cam1_h5/cam2_h5 filenames don't carry a parseable DD-MM-YYYY-HH-MM-SS "
                "timestamp) - not evidence the recordings were fine."
                if recording_gap_s is None
                else "A large gap here is a red flag, but a small gap does NOT prove the two "
                "cameras watched the same event - see motion_correlation_check for that."
            ),
        },
        "motion_correlation_check": {
            "correlation": motion_corr,
            "informational_reference": args.min_motion_correlation,
            "gated": False,
            "checked": motion_corr is not None,
            "note": (
                "None means this couldn't be checked (too few usable consecutive-view steps, or "
                "one camera showed essentially no motion throughout) - not evidence the "
                "recordings were fine."
                if motion_corr is None
                else "A low value is worth double-checking (it's how the 'two independent "
                "intrinsics-only sessions fed into stereo' bug was originally found), but this is "
                "NOT gated: real-data testing showed a genuine wide-baseline capture with real 3D "
                "tilting motion can legitimately score well below the informational reference even "
                "when correct, since the same physical motion doesn't project to similarly-shaped "
                "2D motion from very different viewpoints. See motion_correlation()'s docstring."
            ),
        },
        "adjacent_spacing": {
            k: v for k, v in adjacent_result.items() if k not in ("per_observation", "by_view")
        }
        | {
            "classification": "M6 - the PRIMARY relative-distance gate (see decision.gates.adjacent_spacing_relative_rmse)",
            "geometry_unit_interpreted_as": args.geometry_unit,
            "note": (
                "per_observation and by_view are omitted here to keep this report readable; see "
                "adjacent_spacing_samples.csv and adjacent_spacing_by_view.csv for the full detail, "
                "and adjacent_spacing_report.json for the complete led_spacing.evaluate() output."
            ),
            "circularity_warning": (
                "Every physically-adjacent LED pair is compared against the SAME led_spacing_x/"
                "led_spacing_y assumption that set this calibration's scale (via the object points "
                "passed to cv2.stereoCalibrate). This is not independent scale metrology. Its value "
                "is localization: a persistent, direction-specific bias (e.g. horizontal pairs all "
                "reading short while vertical pairs all read long - see aggregates.horizontal vs "
                "aggregates.vertical below) points at led_spacing_x/led_spacing_y not matching the "
                "true physical pitch, board non-planarity/warp, or anisotropic fx/fy, rather than at "
                "random noise."
            ),
        },
        "decision": decision,
    }
    (args.output_dir / "evaluation_report.json").write_text(json.dumps(report, indent=2) + "\n")
    (args.output_dir / "recommendations.txt").write_text(
        f"Calibration quality status: {decision['status']}\n"
        f"Pixel-band candidate: {decision['pixel_band_candidate']}\n\n"
        + "\n".join(f"- {text}" for text in decision["recommendations"])
        + "\n"
    )

    def display_number(value: float | None, unit: str) -> str:
        if value is None:
            return "Not evaluated"
        precision = 4 if unit == "px" else 3
        return f"{value:.{precision}f} {unit}"

    def gate_result(gate_name: str) -> str:
        gate = decision["gates"][gate_name]
        if gate["value"] is None:
            return "NOT EVALUATED"
        return "PASS" if gate["pass"] else "FAIL"

    def print_gate_line(
        metric_id: str,
        label: str,
        gate_name: str,
        unit: str,
        comparator: str,
    ) -> None:
        gate = decision["gates"][gate_name]
        value_text = display_number(gate["value"], unit)
        threshold_text = f"{comparator} {display_number(gate['threshold'], unit)}"
        emit(
            f"{metric_id:<4} {gate_result(gate_name):<15} {label:<33} "
            f"{value_text:>14}  {threshold_text}"
        )

    # Capture everything emitted below into terminal_scorecard.txt too, so the
    # scorecard is self-contained even when this script is invoked directly
    # (not piped through `tee` by a wrapper shell script) - this is exactly
    # what run_evaluation.sh does today, and relying on an external `tee` to
    # produce this file is itself the "nothing shows on the terminal" trap
    # from before, just for the file instead of the terminal. A distinct name
    # (not shadowing the builtin `print`) is used deliberately: reassigning
    # `print` anywhere in this function makes Python treat it as a local for
    # the WHOLE function body, breaking every print() call above this point
    # too (an UnboundLocalError) - shadowing builtins mid-function is a real
    # foot-gun, not just a style choice.
    _scorecard_lines: list[str] = []

    def emit(text: object = "") -> None:
        line = str(text)
        _scorecard_lines.append(line + "\n")
        print(line)

    border = "=" * 78
    emit(f"\n{border}")
    emit("CALIBRATION QUALITY SCORECARD")
    emit(border)
    emit(f"Run:       {args.run_id}")
    emit(f"Evidence:  {args.partition}")
    calibration_fit_gates_passed = (
        decision["status"] == "NOT_EVALUATED" and not decision["failed_gates"]
    )
    status_line = f"Status:    {decision['status']}"
    if calibration_fit_gates_passed:
        status_line += "  (calibration-fit gates: PASSED)"
    emit(status_line)
    emit(f"Candidate: {decision['pixel_band_candidate']}")
    if calibration_fit_gates_passed:
        emit(
            "\nCALIBRATION-FIT GATES PASSED: every accuracy gate on this partition (M1/M3/M5/M6)\n"
            "PASSED. Status stays NOT_EVALUATED rather than PASS because that label is reserved\n"
            "for a calibration validated on genuinely held-out data with demonstrated\n"
            "repeatability - not because anything here failed or looks wrong.\n"
            "\n"
            "Three independent calibration runs (repeatability) and a held-out dev/test\n"
            "evaluation are the next steps to build confidence in this calibration's\n"
            "reliability and robustness - see RECOMMENDATIONS below for exactly how to\n"
            "produce them. Until then, today's gates passing means the fit is internally\n"
            "consistent on the data used to produce it; it does not yet demonstrate that\n"
            "the same accuracy holds on a different recording, or that the physical rig\n"
            "stays stable enough to reproduce these numbers a second time."
        )
    if args.partition == "calibration_fit":
        emit(
            "\nNOTE: partition=calibration_fit evaluates the SAME correspondences used to fit\n"
            "this calibration. A FAILED status here means the fit already exceeds the\n"
            "configured gates on its own training data - it is not yet a held-out DEV/TEST\n"
            "verdict, which additionally requires --partition dev/test on a physically\n"
            "separate recording. Fix the issues below, then re-run on held-out data."
        )

    emit("\nM1-M6 RESULTS")
    emit("-" * 78)
    emit(f"{'ID':<4} {'Result':<15} {'Metric':<33} {'Value':>14}  Threshold")
    print_gate_line(
        "M1", "Pooled reprojection RMS", "pooled_reprojection_rms", "px", "<="
    )
    emit(
        f"M2   {'INFO':<15} {'Cam 1 median / P95':<33} "
        f"{display_number(reproj1_stats['median'], 'px'):>14}  "
        f"/ {display_number(reproj1_stats['p95'], 'px')}"
    )
    emit(
        f"M2   {'INFO':<15} {'Cam 2 median / P95':<33} "
        f"{display_number(reproj2_stats['median'], 'px'):>14}  "
        f"/ {display_number(reproj2_stats['p95'], 'px')}"
    )
    print_gate_line(
        "M3", "Triangulation angle P5", "triangulation_angle_p5", "deg", ">="
    )
    if args.gate_sampson:
        print_gate_line(
            "M4", "Square-root Sampson RMS", "sqrt_sampson_rms", "px", "<="
        )
    else:
        emit(
            f"M4   {'INFO':<15} {'Square-root Sampson RMS':<33} "
            f"{display_number(sampson_stats['rms'], 'px'):>14}  "
            f"(diagnostic; pass --gate-sampson to gate on this)"
        )
    emit(
        f"M4b  {'INFO':<15} {'Symmetric epipolar distance':<33} "
        f"{display_number(symmetric_stats['rms'], 'px'):>14}  (diagnostic; never gated)"
    )
    print_gate_line(
        "M5", "Pair-balanced relative RMSE", "pair_balanced_relative_rmse", "%", "<="
    )
    print_gate_line(
        "M6", "Adjacent spacing, all directions", "adjacent_spacing_relative_rmse", "%", "<="
    )
    emit(
        f"M6   {'INFO':<15} {'Adjacent spacing, horizontal':<33} "
        f"{display_number(adjacent_result['aggregates']['horizontal']['relative_rmse_percent'], '%'):>14}  "
        f"(breakdown; the M6 gate above pools both directions)"
    )
    emit(
        f"M6   {'INFO':<15} {'Adjacent spacing, vertical':<33} "
        f"{display_number(adjacent_result['aggregates']['vertical']['relative_rmse_percent'], '%'):>14}  "
        f"(breakdown; the M6 gate above pools both directions)"
    )

    if decision["failed_gates"]:
        emit("\nFAILED GATES")
        emit("-" * 78)
        gate_to_metric = {
            "pooled_reprojection_rms": "M1",
            "triangulation_angle_p5": "M3",
            "sqrt_sampson_rms": "M4",
            "pair_balanced_relative_rmse": "M5",
            "adjacent_spacing_relative_rmse": "M6",
        }
        for gate_name in decision["failed_gates"]:
            emit(f"- {gate_to_metric.get(gate_name, '?')}: {gate_name.replace('_', ' ')}")

    if recording_gap_s is not None or motion_corr is not None:
        emit("\nRECORDING PROVENANCE (informational)")
        emit("-" * 78)
        if recording_gap_s is not None:
            above_reference = recording_gap_s > args.max_recording_start_gap_s
            flag = " <- above the informational reference; not gated, see notes" if above_reference else ""
            emit(
                f"cam1/cam2 recording start gap: {recording_gap_s:.0f}s "
                f"(informational reference: {args.max_recording_start_gap_s:.0f}s){flag}"
            )
        if motion_corr is not None:
            below_reference = motion_corr < args.min_motion_correlation
            flag = " <- below the informational reference; not gated, see notes" if below_reference else ""
            emit(
                f"Cross-camera motion correlation: {motion_corr:.3f} "
                f"(informational reference: {args.min_motion_correlation:.2f}){flag}"
            )

    if decision["missing_required_evidence"]:
        emit("\nMISSING REQUIRED EVIDENCE")
        emit("-" * 78)
        for item in decision["missing_required_evidence"]:
            emit(f"- {item.replace('_', ' ').capitalize()}")

    emit("\nRECOMMENDATIONS")
    emit("-" * 78)
    for number, recommendation in enumerate(decision["recommendations"], start=1):
        emit(f"{number}. {recommendation}")

    if id_availability["automatically_excluded_ids"]:
        emit("\nID AVAILABILITY")
        emit("-" * 78)
        emit(
            f"Automatically excluded (unavailable in one or both cameras): "
            f"{id_availability['automatically_excluded_ids']}"
        )
        emit(f"Evaluated IDs: {id_availability['evaluated_ids']}")

    emit("\nOUTPUT FILES")
    emit("-" * 78)
    for filename in (
        "evaluation_report.json",
        "summary.csv",
        "observations.csv",
        "heldout_distance_audit.csv",
        "adjacent_spacing_audit.csv",
        "adjacent_spacing_samples.csv",
        "adjacent_spacing_by_view.csv",
        "adjacent_spacing_by_direction.csv",
        "adjacent_spacing_report.json",
        "recommendations.txt",
        "terminal_scorecard.txt",
    ):
        emit(f"- {args.output_dir / filename}")
    emit(border)

    (args.output_dir / "terminal_scorecard.txt").write_text("".join(_scorecard_lines))

    if args.strict and decision["status"] != "PASS":
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        sys.exit(2)