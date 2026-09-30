#!/usr/bin/env python3
"""
Stereo-calibrate two cameras from their individual intrinsics plus
synchronized stereo image-point correspondences.

Combines:
  1. Each camera's individual intrinsics (camera_matrix, distortion) from
     intrinsic/intrinsic_calibration_2.py.
  2. The synchronized stereo image points from stereo/build_stereo_points.py
     (img_points_cam1, img_points_cam2, beacon_ids - same view/point order
     in both cameras, at the same real-world instants).
  3. The same LED-box 3D geometry used for intrinsic calibration - pass the
     SAME --spacing-x/--spacing-y/--columns/--rows/--h5-id-for-led1 used
     when running intrinsic_calibration_2.py, or the object_points built
     here won't match the LED layout the intrinsics were solved against.

Runs cv2.stereoCalibrate() with CALIB_FIX_INTRINSIC (the per-camera
intrinsics are already solved and trusted), solving only for the
rotation/translation (R, T) between the two cameras, plus the essential (E)
and fundamental (F) matrices.

NOTE ON SCALE: if --spacing-x/--spacing-y are left at their default of 1.0
(no real measurement), object_points are in "LED grid step" units, not real
distance - the translation vector T comes out in the same units. Multiply T
by the real LED pitch (e.g. in mm) once measured to get the real physical
baseline between the two cameras.

Usage:
    python stereo/stereo_calibrate.py \
        track_logs/stereo/img_points.npz \
        intrinsic/output/camera_1_intrinsics.json \
        intrinsic/output/camera_2_intrinsics.json \
        --spacing-x 1 --spacing-y 1 \
        --output track_logs/stereo/stereo_calibration.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


# =====================================================================
# STAGE 1 - 3D LED-box geometry. Duplicated from
# intrinsic/intrinsic_calibration_2.py (see that file for details) -
# following this repo's existing convention of standalone scripts rather
# than a shared package.
# =====================================================================


def make_led_box_geometry(
    spacing_x: float,
    spacing_y: float,
    columns: int,
    rows: int,
    h5_id_for_led1: int = 0,
) -> dict[int, np.ndarray]:
    points: dict[int, np.ndarray] = {}
    for zero_based_led in range(columns * rows):
        row, column = divmod(zero_based_led, columns)
        beacon_id = h5_id_for_led1 + zero_based_led
        points[beacon_id] = np.array([column * spacing_x, row * spacing_y, 0.0], dtype=np.float32)
    return points


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    """Reads the camera_matrix/distortion_coefficients/image_size fields written
    by intrinsic/intrinsic_calibration_2.py's --output JSON."""
    data = json.loads(path.read_text())
    camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(data["distortion_coefficients"], dtype=np.float64)
    width, height = data["image_size"]
    return camera_matrix, distortion, (width, height)


# =====================================================================
# STAGE 2 - Command-line entry point.
# =====================================================================


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("img_points_npz", type=Path, help="Output of stereo/build_stereo_points.py")
    parser.add_argument("cam1_intrinsics_json", type=Path, help="Output of intrinsic_calibration_2.py for camera 1")
    parser.add_argument("cam2_intrinsics_json", type=Path, help="Output of intrinsic_calibration_2.py for camera 2")
    parser.add_argument(
        "--spacing-x",
        type=float,
        default=1.0,
        help="Same LED pitch used for intrinsic calibration (default: 1.0, unitless grid step)",
    )
    parser.add_argument("--spacing-y", type=float, default=1.0)
    parser.add_argument("--columns", type=int, default=4, help="Must match intrinsic_calibration_2.py's --columns")
    parser.add_argument("--rows", type=int, default=6, help="Must match intrinsic_calibration_2.py's --rows")
    parser.add_argument("--h5-id-for-led1", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("track_logs/stereo/stereo_calibration.json"))
    args = parser.parse_args()

    npz = np.load(args.img_points_npz)
    img_points_cam1 = npz["img_points_cam1"]  # (num_views, num_beacons, 2)
    img_points_cam2 = npz["img_points_cam2"]
    beacon_ids = npz["beacon_ids"].tolist()

    cam1_matrix, cam1_dist, image_size1 = load_intrinsics(args.cam1_intrinsics_json)
    cam2_matrix, cam2_dist, image_size2 = load_intrinsics(args.cam2_intrinsics_json)
    if image_size1 != image_size2:
        raise ValueError(f"Camera image sizes differ: {image_size1} vs {image_size2}")
    image_size = tuple(image_size1)

    geometry = make_led_box_geometry(args.spacing_x, args.spacing_y, args.columns, args.rows, args.h5_id_for_led1)
    missing = [b for b in beacon_ids if b not in geometry]
    if missing:
        raise ValueError(
            f"Beacon IDs {missing} from {args.img_points_npz} have no entry in the "
            f"{args.columns}x{args.rows} LED-box geometry - check --columns/--rows/--h5-id-for-led1."
        )

    object_points_single = np.stack([geometry[b] for b in beacon_ids])  # (num_beacons, 3)
    num_views = img_points_cam1.shape[0]

    # A view's beacon slots can be NaN (a beacon not visible in both cameras
    # at that instant - see build_stereo_points.py's build_stereo_image_points,
    # which now allows partial per-view visibility rather than requiring
    # every beacon ever seen to be present in every single view). Each
    # view's object/image points here are built from only that view's
    # finite entries, matching cv2.stereoCalibrate's native support for a
    # variable number of points per view (the same pattern
    # intrinsic_calibration_2.py already uses for cv2.calibrateCamera).
    min_points_per_view = 6  # a planar PnP-like pose needs at least this many to be well-posed
    object_points: list[np.ndarray] = []
    image_points1: list[np.ndarray] = []
    image_points2: list[np.ndarray] = []
    skipped_sparse_views = 0
    per_view_point_counts: list[int] = []
    for i in range(num_views):
        valid = np.all(np.isfinite(img_points_cam1[i]), axis=1) & np.all(np.isfinite(img_points_cam2[i]), axis=1)
        count = int(valid.sum())
        if count < min_points_per_view:
            skipped_sparse_views += 1
            continue
        per_view_point_counts.append(count)
        object_points.append(object_points_single[valid].copy())
        image_points1.append(img_points_cam1[i][valid].astype(np.float32))
        image_points2.append(img_points_cam2[i][valid].astype(np.float32))

    if skipped_sparse_views:
        print(
            f"Skipped {skipped_sparse_views}/{num_views} views with fewer than "
            f"{min_points_per_view} finite points in both cameras (a planar pose needs at "
            "least that many to be well-determined)."
        )
    if len(object_points) == 0:
        raise ValueError(
            f"No usable views remain after requiring >= {min_points_per_view} points/view - "
            f"{args.img_points_npz} may need a lower --min-points when it was built, or more "
            "board-pose coverage."
        )
    per_view_point_counts_arr = np.asarray(per_view_point_counts)
    print(
        f"Views: {len(object_points)}, points/view: min={per_view_point_counts_arr.min()} "
        f"median={int(np.median(per_view_point_counts_arr))} max={per_view_point_counts_arr.max()} "
        f"(of {len(beacon_ids)} beacons seen overall)"
    )
    print(f"Cam1 intrinsics: {args.cam1_intrinsics_json}")
    print(f"Cam2 intrinsics: {args.cam2_intrinsics_json}")

    (
        rms,
        cam1_matrix_out,
        cam1_dist_out,
        cam2_matrix_out,
        cam2_dist_out,
        R,
        T,
        E,
        F,
    ) = cv2.stereoCalibrate(
        object_points,
        image_points1,
        image_points2,
        cam1_matrix,
        cam1_dist,
        cam2_matrix,
        cam2_dist,
        image_size,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )

    baseline = float(np.linalg.norm(T))
    is_real_scale = args.spacing_x != 1.0 or args.spacing_y != 1.0
    scale_note = "real units (spacing was measured)" if is_real_scale else "LED-grid-step units - multiply by real LED pitch for a real-world distance"

    print(f"\nStereo RMS reprojection error: {rms:.4f} px")
    print(f"Rotation matrix R (cam1 -> cam2):\n{R}")
    print(f"Translation vector T (cam1 -> cam2):\n{T.ravel()}")
    print(f"Baseline |T|: {baseline:.4f} ({scale_note})")

    output = {
        "img_points_npz": str(args.img_points_npz),
        "cam1_intrinsics_json": str(args.cam1_intrinsics_json),
        "cam2_intrinsics_json": str(args.cam2_intrinsics_json),
        "geometry": {
            "spacing_x": args.spacing_x,
            "spacing_y": args.spacing_y,
            "columns": args.columns,
            "rows": args.rows,
            "h5_id_for_led1": args.h5_id_for_led1,
            "is_real_scale": is_real_scale,
        },
        "num_views": len(object_points),
        "num_candidate_views": num_views,
        "num_views_skipped_sparse": skipped_sparse_views,
        "beacon_ids": beacon_ids,
        "image_size": list(image_size),
        "stereo_rms_px": float(rms),
        "camera_matrix_1": cam1_matrix_out.tolist(),
        "distortion_1": cam1_dist_out.ravel().tolist(),
        "camera_matrix_2": cam2_matrix_out.tolist(),
        "distortion_2": cam2_dist_out.ravel().tolist(),
        "R": R.tolist(),
        "T": T.ravel().tolist(),
        "E": E.tolist(),
        "F": F.tolist(),
        "baseline": baseline,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(f"\nSaved {args.output}")


if __name__ == "__main__":
    main()
