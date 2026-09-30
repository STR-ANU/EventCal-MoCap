#!/usr/bin/env python3
"""
Compares stereo calibration results across >= 2 independent calibration runs
(>= 3 physically separate capture sessions recommended - see
calibration_quality.py's --repeatability-runs gate) to check whether the
physical rig is rigid/stable enough to trust a single calibration.

IMPORTANT: this does NOT replace calibration_quality.py and does not affect
its --repeatability-runs flag (that flag is still just a self-declared count
- calibration_quality.py trusts you to have actually done this check). This
script performs the comparison the "record 3 independent physical
calibration sessions ... compare fx/fy/cx/cy, R, T, and baseline"
recommendation describes, since nothing in the existing pipeline computes it
automatically.

Reads a single stereo_calibration_*.json per run - the direct output of
stereo/stereo_calibrate.py. No separate intrinsics files are needed:
stereo_calibrate.py runs cv2.stereoCalibrate() with CALIB_FIX_INTRINSIC, so
its "camera_matrix_1"/"camera_matrix_2" are exactly the per-camera
intrinsics fed in, just echoed through - not a joint re-fit. Its "baseline"
field is used directly (it's ||T||, computed the same way this script would
otherwise recompute it) along with "R" and "T".

Usage:
    python evaluation/check_repeatability.py \\
        --run track_logs/repeatability/run_1/stereo_calibration.json \\
        --run track_logs/repeatability/run_2/stereo_calibration.json \\
        --run track_logs/repeatability/run_3/stereo_calibration.json \\
        --output-dir track_logs/repeatability

Normally you won't type this by hand - run_repeatability.sh drives 3+ full
pipeline runs and calls this automatically with the right paths.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def load_stereo_calibration(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text())

    for required_key in ("camera_matrix_1", "camera_matrix_2", "R", "T", "baseline"):
        if required_key not in data:
            raise ValueError(
                f"{path} is missing '{required_key}' - is this really a "
                "stereo_calibrate.py output file?"
            )

    camera_matrix_1 = np.asarray(data["camera_matrix_1"], dtype=np.float64)
    camera_matrix_2 = np.asarray(data["camera_matrix_2"], dtype=np.float64)
    R = np.asarray(data["R"], dtype=np.float64)
    T = np.asarray(data["T"], dtype=np.float64).reshape(3)

    return {
        "fx1": float(camera_matrix_1[0, 0]),
        "fy1": float(camera_matrix_1[1, 1]),
        "cx1": float(camera_matrix_1[0, 2]),
        "cy1": float(camera_matrix_1[1, 2]),
        "fx2": float(camera_matrix_2[0, 0]),
        "fy2": float(camera_matrix_2[1, 1]),
        "cx2": float(camera_matrix_2[0, 2]),
        "cy2": float(camera_matrix_2[1, 2]),
        "R": R,
        "T": T,
        "baseline": float(data["baseline"]),
        "is_real_scale": data.get("geometry", {}).get("is_real_scale"),
    }


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle (degrees) of the relative rotation between two
    rotation matrices - 0 deg means identical orientation."""
    R_rel = R_a.T @ R_b
    trace = float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(trace))


def vector_angle_deg(v_a: np.ndarray, v_b: np.ndarray) -> float:
    """Angle (degrees) between two direction vectors, ignoring magnitude -
    used to check T's DIRECTION separately from its length (baseline), since
    a rig could keep the same baseline length while the stereo pair's
    relative bearing drifts, or vice versa."""
    cos_angle = float(
        np.dot(v_a, v_b) / (np.linalg.norm(v_a) * np.linalg.norm(v_b))
    )
    cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
    return math.degrees(math.acos(cos_angle))


def swing_stats(values: list[float]) -> dict[str, Any]:
    arr = np.asarray(values, dtype=np.float64)
    mean = float(arr.mean())
    spread = float(arr.max() - arr.min())
    return {
        "values": [float(v) for v in arr],
        "mean": mean,
        "min": float(arr.min()),
        "max": float(arr.max()),
        "std": float(arr.std(ddof=0)),
        "swing_abs": spread,
        # Guard divide-by-zero for a quantity that happens to average to 0.
        # Shouldn't happen for fx/fy/baseline in practice, but cx/cy could
        # in principle be 0 for an unusual crop/origin convention.
        "swing_percent_of_mean": (
            (spread / abs(mean) * 100.0) if mean != 0 else float("nan")
        ),
    }


def fmt_row(label: str, s: dict[str, Any], unit: str) -> str:
    return (
        f"{label:<22} mean={s['mean']:.4f}{unit}  "
        f"min={s['min']:.4f}{unit}  max={s['max']:.4f}{unit}  "
        f"std={s['std']:.4f}{unit}  swing={s['swing_abs']:.4f}{unit} "
        f"({s['swing_percent_of_mean']:.2f}% of mean)"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--run",
        action="append",
        metavar="STEREO_CALIBRATION_JSON",
        required=True,
        dest="runs",
        help="One independent calibration run's stereo_calibrate.py output. Repeat --run once per session (>=3 recommended).",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--length-unit",
        default="m",
        help="Label only (doesn't affect computation) - the physical unit your led_spacing_x/y values are in. Default 'm' matches this pipeline's convention.",
    )
    parser.add_argument(
        "--max-focal-swing-percent",
        type=float,
        default=2.0,
        help="Flag fx/fy if they swing by more than this percent of their mean across runs.",
    )
    parser.add_argument(
        "--max-principal-point-swing-px",
        type=float,
        default=5.0,
        help="Flag cx/cy if they swing by more than this many pixels across runs.",
    )
    parser.add_argument(
        "--max-baseline-swing-percent",
        type=float,
        default=3.0,
        help="Flag the stereo baseline (||T||) if it swings by more than this percent of its mean across runs. Matches the 'more than a few percent' language calibration_quality.py's own recommendation uses.",
    )
    parser.add_argument(
        "--max-rotation-swing-deg",
        type=float,
        default=0.5,
        help="Flag stereo R if any pair of runs' rotations differ by more than this many degrees.",
    )
    args = parser.parse_args()

    if len(args.runs) < 2:
        parser.error(
            "Need at least 2 --run entries to compare anything; "
            ">=3 is required for calibration_quality.py's own repeatability gate."
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    cam1_fx: list[float] = []
    cam1_fy: list[float] = []
    cam1_cx: list[float] = []
    cam1_cy: list[float] = []
    cam2_fx: list[float] = []
    cam2_fy: list[float] = []
    cam2_cx: list[float] = []
    cam2_cy: list[float] = []
    Rs: list[np.ndarray] = []
    Ts: list[np.ndarray] = []
    baselines: list[float] = []
    is_real_scale_flags: list[Any] = []

    for run_path_str in args.runs:
        run_path = Path(run_path_str)
        if not run_path.is_file():
            parser.error(f"File not found: {run_path}")

        run = load_stereo_calibration(run_path)

        cam1_fx.append(run["fx1"])
        cam1_fy.append(run["fy1"])
        cam1_cx.append(run["cx1"])
        cam1_cy.append(run["cy1"])
        cam2_fx.append(run["fx2"])
        cam2_fy.append(run["fy2"])
        cam2_cx.append(run["cx2"])
        cam2_cy.append(run["cy2"])
        Rs.append(run["R"])
        Ts.append(run["T"])
        baselines.append(run["baseline"])
        is_real_scale_flags.append(run["is_real_scale"])

    n_runs = len(args.runs)

    # If different runs were fit with different led_spacing_x/y settings
    # (real-scale vs. grid-step units), their baselines aren't in the same
    # unit and comparing swing between them would be meaningless - catch
    # that up front rather than silently reporting a bogus percentage.
    distinct_scale_flags = {flag for flag in is_real_scale_flags if flag is not None}
    if len(distinct_scale_flags) > 1:
        parser.error(
            "Runs disagree on is_real_scale (some used real led_spacing_x/y, "
            "some used the 1.0 grid-step default) - their baselines are in "
            "different units and cannot be compared. Re-run with consistent "
            "led_spacing_x/y across all sessions."
        )

    stats: dict[str, Any] = {
        "n_runs": n_runs,
        "camera_1": {
            "fx": swing_stats(cam1_fx),
            "fy": swing_stats(cam1_fy),
            "cx": swing_stats(cam1_cx),
            "cy": swing_stats(cam1_cy),
        },
        "camera_2": {
            "fx": swing_stats(cam2_fx),
            "fy": swing_stats(cam2_fy),
            "cx": swing_stats(cam2_cx),
            "cy": swing_stats(cam2_cy),
        },
        "stereo": {
            "baseline": swing_stats(baselines),
        },
    }

    # Every pair of runs, not just consecutive ones: a rig that drifted
    # monotonically across sessions 1 -> 2 -> 3 could show small 1-vs-2 and
    # 2-vs-3 differences individually while 1-vs-3 is actually the largest
    # gap - only checking neighbors would miss that.
    pairwise_rotation_deg: dict[str, float] = {}
    pairwise_t_direction_deg: dict[str, float] = {}
    for i in range(n_runs):
        for j in range(i + 1, n_runs):
            key = f"run{i + 1}_vs_run{j + 1}"
            pairwise_rotation_deg[key] = rotation_angle_deg(Rs[i], Rs[j])
            pairwise_t_direction_deg[key] = vector_angle_deg(Ts[i], Ts[j])

    stats["stereo"]["pairwise_rotation_deg"] = pairwise_rotation_deg
    stats["stereo"]["pairwise_t_direction_deg"] = pairwise_t_direction_deg
    stats["stereo"]["max_pairwise_rotation_deg"] = max(pairwise_rotation_deg.values())
    stats["stereo"]["max_pairwise_t_direction_deg"] = max(pairwise_t_direction_deg.values())

    # Gate each quantity against the thresholds. Mirrors calibration_quality.py's
    # style (explicit pass/fail with a plain-text reason), but this is a
    # separate, standalone check - it does not modify or feed back into
    # calibration_quality.py itself, and calibration_quality.py's own
    # --repeatability-runs flag still has to be set by hand after you've
    # reviewed this report.
    failures: list[str] = []

    def check_percent_swing(label: str, swing_percent: float, limit: float) -> None:
        if not math.isnan(swing_percent) and swing_percent > limit:
            failures.append(f"{label} swung {swing_percent:.2f}% across runs (limit {limit:.2f}%).")

    def check_abs_swing(label: str, swing_abs: float, limit: float, unit: str) -> None:
        if swing_abs > limit:
            failures.append(f"{label} swung {swing_abs:.4f}{unit} across runs (limit {limit:.4f}{unit}).")

    for cam_label, cam_stats in (("camera_1", stats["camera_1"]), ("camera_2", stats["camera_2"])):
        check_percent_swing(f"{cam_label} fx", cam_stats["fx"]["swing_percent_of_mean"], args.max_focal_swing_percent)
        check_percent_swing(f"{cam_label} fy", cam_stats["fy"]["swing_percent_of_mean"], args.max_focal_swing_percent)
        check_abs_swing(f"{cam_label} cx", cam_stats["cx"]["swing_abs"], args.max_principal_point_swing_px, "px")
        check_abs_swing(f"{cam_label} cy", cam_stats["cy"]["swing_abs"], args.max_principal_point_swing_px, "px")

    check_percent_swing(
        "stereo baseline", stats["stereo"]["baseline"]["swing_percent_of_mean"], args.max_baseline_swing_percent
    )
    check_abs_swing(
        "stereo rotation (max pairwise)",
        stats["stereo"]["max_pairwise_rotation_deg"],
        args.max_rotation_swing_deg,
        " deg",
    )

    status = "FAILED" if failures else "PASS"

    report = {
        "status": status,
        "n_runs": n_runs,
        "failures": failures,
        "thresholds": {
            "max_focal_swing_percent": args.max_focal_swing_percent,
            "max_principal_point_swing_px": args.max_principal_point_swing_px,
            "max_baseline_swing_percent": args.max_baseline_swing_percent,
            "max_rotation_swing_deg": args.max_rotation_swing_deg,
        },
        "stats": stats,
        "note": (
            "This checks whether repeated physical calibration sessions agree with "
            "each other - it does NOT check whether any single run's fit is accurate "
            "(that's calibration_quality.py's M1-M6 job). A PASS here only means the "
            "rig produced consistent fitted parameters across the runs actually compared. "
            + (
                f"It does not set calibration_quality.py's --repeatability-runs for you - "
                f"pass that separately (--repeatability-runs {n_runs}) once you've reviewed this report."
                if n_runs >= 3
                else
                f"Only {n_runs} run(s) were compared here, below the >= 3 independent "
                "physical sessions calibration_quality.py's own repeatability gate requires "
                "- this report is NOT sufficient evidence for that gate, and its run count "
                "should NOT be passed to --repeatability-runs. Treat this as a pipeline "
                "mechanics check only, not a repeatability conclusion."
            )
        ),
    }

    (args.output_dir / "repeatability_report.json").write_text(json.dumps(report, indent=2))

    unit = args.length_unit
    lines: list[str] = []
    lines.append(f"REPEATABILITY CHECK ({n_runs} runs)")
    lines.append("=" * 70)
    lines.append(f"Status: {status}")
    lines.append("")

    for cam_label, cam_stats in (("Camera 1", stats["camera_1"]), ("Camera 2", stats["camera_2"])):
        lines.append(cam_label)
        lines.append(fmt_row("  fx", cam_stats["fx"], " px"))
        lines.append(fmt_row("  fy", cam_stats["fy"], " px"))
        lines.append(fmt_row("  cx", cam_stats["cx"], " px"))
        lines.append(fmt_row("  cy", cam_stats["cy"], " px"))
        lines.append("")

    lines.append("Stereo")
    lines.append(fmt_row("  baseline", stats["stereo"]["baseline"], f" {unit}"))
    lines.append(f"  Pairwise rotation differences (deg): {pairwise_rotation_deg}")
    lines.append(f"  Max pairwise rotation difference: {stats['stereo']['max_pairwise_rotation_deg']:.4f} deg")
    lines.append(f"  Pairwise T-direction differences (deg): {pairwise_t_direction_deg}")
    lines.append(f"  Max pairwise T-direction difference: {stats['stereo']['max_pairwise_t_direction_deg']:.4f} deg")
    lines.append("")

    if failures:
        lines.append("FAILURES")
        for failure in failures:
            lines.append(f"  - {failure}")
    else:
        lines.append("All quantities were within the configured swing thresholds.")

    lines.append("")
    lines.append(report["note"])

    report_text = "\n".join(lines)
    (args.output_dir / "repeatability_report.txt").write_text(report_text)

    print(report_text)
    print(f"\nSaved: {args.output_dir / 'repeatability_report.json'}")
    print(f"Saved: {args.output_dir / 'repeatability_report.txt'}")


if __name__ == "__main__":
    main()
