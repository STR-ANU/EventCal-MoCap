#!/usr/bin/env python3
"""
End-to-end intrinsic calibration: HDF5 tracker data -> image points -> cv2.calibrateCamera -> RMS.

Each beacon's `tracker` dataset holds (time_s, x_px, y_px) rows recorded
independently, so beacons are rarely sampled at exactly the same instant.
Tracker time is used to interpolate pixel positions because it describes when
each position was estimated. Decoder `ts_start_bit_s` is used differently: it
determines the phase of the 10 Hz sampling grid. In this project's pipeline,
the per-track .es files and tracker log both use seconds relative to the same
first event, so these two timestamps can be compared directly.

With --time-source auto (the default), the script estimates a common 10 Hz
phase from all decoded packet-start timestamps. It then interpolates tracker
positions at phase-aligned times separated by exactly 1/--hz seconds. If an
older H5 file has no usable packet-start column, it falls back to a tracker-time
grid.

Beacons that never appear in the H5 file are ignored. A beacon may also be
missing from an individual view if its nearby tracker samples are unavailable.
OpenCV permits different point counts in different views, provided each view's
object-point and image-point arrays contain the same beacon IDs in the same
order.

Usage:
    python utils/calibrate_intrinsics.py track_logs/foo.h5 \
        --spacing-x 1 --spacing-y 1 --hz 10 --output intrinsics.json

Or with measured geometry:
    python utils/calibrate_intrinsics.py track_logs/foo.h5 geometry.csv --hz 10
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import h5py
import numpy as np

# A "view" is one synchronized snapshot of the visible board LEDs—the
# equivalent of one checkerboard photograph. Multiple views at different board
# positions and angles are needed to constrain the camera model.
MIN_VIEWS = 10


# =====================================================================
# STAGE 1 - Read the H5 file: figure out where each beacon was, and when.
# =====================================================================


def beacon_id_from_group(name: str) -> int | None:
    """
    Each beacon's data lives in its own top-level H5 group, named either
    "0", "1", ... or "BeaconID_0", "BeaconID_1", .... This pulls the
    trailing integer out of either naming style. Returns None for a group
    name that isn't beacon-shaped at all (so callers can skip it).
    """
    tail = name.rsplit("_", 1)[-1]
    try:
        return int(tail)
    except ValueError:
        return None


def collapse_duplicate_times(rows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Clean up one beacon's raw tracker rows: (time_s, x_px, y_px).

    Raw data can have two problems: a few NaN/inf rows from a bad detection,
    and, occasionally, two rows logged at the exact same timestamp. This
    function drops the bad rows, sorts everything by time, and - where a
    timestamp repeats - collapses those rows to their median x/y. The result
    is a clean (times, xy) pair with strictly unique, increasing timestamps,
    which is what the interpolation step below assumes it can rely on.
    """
    rows = np.asarray(rows, dtype=np.float64)
    if rows.ndim != 2 or rows.shape[1] < 3:
        raise ValueError(f"tracker must have shape (N, >=3), got {rows.shape}")

    # Keep only (t, x, y) - some datasets carry extra trailing columns we don't need.
    rows = rows[:, :3]
    # Drop any row containing NaN/inf in time, x, or y.
    rows = rows[np.all(np.isfinite(rows), axis=1)]
    # Sort by timestamp so later steps can binary-search this array.
    rows = rows[np.argsort(rows[:, 0], kind="stable")]
    if len(rows) == 0:
        return np.empty(0), np.empty((0, 2))

    # np.unique groups equal timestamps together and tells us where each
    # group starts (starts) and how many rows are in it (counts).
    times, starts, counts = np.unique(rows[:, 0], return_index=True, return_counts=True)
    # Start by assuming every timestamp is unique (count == 1): just take its x/y directly.
    xy = rows[starts, 1:3].copy()
    # For any timestamp that actually repeated, overwrite with the median x/y
    # of all rows sharing that timestamp (more robust than taking the first).
    for i in np.flatnonzero(counts > 1):
        start, count = starts[i], counts[i]
        xy[i] = np.median(rows[start : start + count, 1:3], axis=0)
    return times, xy


def load_tracks(path: Path) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    """
    Open the H5 file and build a lookup: beacon_id -> (times, xy).
    This is "everywhere this beacon was seen, and when," for every beacon
    that has tracker data - the raw material the rest of the script works from.
    """
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


def load_decoder_start_times(path: Path) -> dict[int, np.ndarray]:
    """
    Read `ts_start_bit_s` for every decoded beacon.

    New H5 files store decoder column names as a dataset attribute. For H5 files
    made immediately before that metadata was added, a five-column decoded
    array is recognized as the current layout:

        t_us_as_seconds, ts_start_bit_s, byte1, byte2, byte3

    Four-column legacy arrays are not guessed because their second column is
    byte1 rather than a timestamp.
    """
    result: dict[int, np.ndarray] = {}

    with h5py.File(path, "r") as f:
        for group_name in f.keys():
            beacon_id = beacon_id_from_group(group_name)
            group = f[group_name]
            if beacon_id is None or not isinstance(group, h5py.Group) or "decoded" not in group:
                continue

            dataset = group["decoded"]
            rows = np.asarray(dataset[...], dtype=np.float64)
            if rows.ndim != 2 or len(rows) == 0:
                continue

            raw_columns = dataset.attrs.get("columns")
            columns: list[str] = []
            if raw_columns is not None:
                columns = [
                    value.decode("utf-8") if isinstance(value, bytes) else str(value)
                    for value in raw_columns
                ]

            if "ts_start_bit_s" in columns:
                column = columns.index("ts_start_bit_s")
            elif rows.shape[1] == 5:
                # Current decoder schema, but from an H5 file created before
                # filter_decoded_files.py started saving column metadata.
                column = 1
            else:
                continue

            times = rows[:, column]
            times = np.unique(times[np.isfinite(times) & (times >= 0.0)])
            if len(times):
                result[beacon_id] = times

    return result


def make_sample_times(
    tracks: dict[int, tuple[np.ndarray, np.ndarray]],
    usable_ids: list[int],
    decoder_start_times: dict[int, np.ndarray],
    hz: float,
    requested_source: str,
) -> tuple[np.ndarray, str, float | None]:
    """
    Build a regular sampling grid for calibration views.

    `ts_start_bit_s` values need not be numerically identical because onset
    detection and PLL correction introduce small timing differences. Treating
    time modulo the expected period as an angle gives a circular mean phase
    without being confused by the wrap from 0.099... s back to 0.000... s.

    The returned concentration is between 0 and 1:
      * near 1: packet starts share a clear phase;
      * near 0: packet starts are spread around the cycle, so no common phase
        can be trusted.
    """
    period = 1.0 / hz
    start = min(tracks[i][0][0] for i in usable_ids)
    stop = max(tracks[i][0][-1] for i in usable_ids)

    packet_arrays = [
        decoder_start_times[i]
        for i in usable_ids
        if i in decoder_start_times and len(decoder_start_times[i])
    ]

    use_decoder = requested_source in {"auto", "decoder"} and packet_arrays
    concentration: float | None = None

    if use_decoder:
        packet_times = np.concatenate(packet_arrays)
        angles = 2.0 * np.pi * packet_times / period
        mean_cos = float(np.mean(np.cos(angles)))
        mean_sin = float(np.mean(np.sin(angles)))
        concentration = float(np.hypot(mean_cos, mean_sin))

        # A weak concentration means the LEDs do not share a reliable common
        # packet phase. Auto mode safely falls back; explicit decoder mode
        # reports the problem instead of silently using misleading timing.
        if concentration < 0.5:
            if requested_source == "decoder":
                raise ValueError(
                    "Decoder packet starts do not have a clear common phase "
                    f"(concentration={concentration:.3f}). Use --time-source tracker "
                    "or verify that all LEDs transmit synchronously."
                )
            use_decoder = False

    if requested_source == "decoder" and not packet_arrays:
        raise ValueError(
            "No ts_start_bit_s data was found. Regenerate the H5 file with the "
            "current decoder/filter pipeline, or use --time-source tracker."
        )

    if use_decoder:
        phase_angle = np.arctan2(mean_sin, mean_cos) % (2.0 * np.pi)
        phase = float(phase_angle * period / (2.0 * np.pi))
        source = "decoder ts_start_bit_s phase"
    else:
        phase = 0.0
        source = "tracker-time zero phase"

    first_tick = phase + np.ceil((start - phase - 1e-12) / period) * period
    sample_times = np.arange(first_tick, stop + 0.5 * period, period)
    return sample_times, source, concentration


# =====================================================================
# STAGE 2 - Where the beacons physically are (the 3D "object points").
# =====================================================================
# cv2.calibrateCamera needs to know both where each point is in the image
# (2D, from the H5 above) AND where it is in real space (3D). These two
# functions supply the 3D side, either from a measurement file or from the
# known LED-box layout.


def load_geometry(path: Path) -> dict[int, np.ndarray]:
    """Read measured 3D beacon positions from a CSV with columns beacon_id,X,Y[,Z]."""
    points: dict[int, np.ndarray] = {}
    with path.open(newline="") as fp:
        reader = csv.DictReader(fp)
        required = {"beacon_id", "X", "Y"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"{path} must contain columns beacon_id,X,Y and optionally Z")
        for row in reader:
            beacon_id = int(row["beacon_id"])
            if beacon_id in points:
                raise ValueError(f"Duplicate beacon_id {beacon_id} in {path}")
            points[beacon_id] = np.array(
                [float(row["X"]), float(row["Y"]), float(row.get("Z") or 0.0)],
                dtype=np.float32,
            )
    return points


def make_led_box_geometry(
    spacing_x: float,
    spacing_y: float,
    columns: int,
    rows: int,
    h5_id_for_led1: int = 0,
) -> dict[int, np.ndarray]:
    """
    Generate 3D positions for a `columns`-wide x `rows`-tall LED-box/checkerboard
    layout, instead of requiring a measurement CSV. Beacon 0 sits at the top-left
    corner (0, 0, 0); beacon ids increase left-to-right, then top-to-bottom,
    each offset by one grid step (spacing_x, spacing_y). The board is flat,
    so every Z is 0.

    Note: since intrinsic calibration only depends on the *ratio* of
    spacing_x to spacing_y (not their absolute size), spacing_x=1, spacing_y=1
    is a perfectly valid choice for a board with equal horizontal/vertical
    LED pitch - the resulting camera_matrix/distortion come out correct in
    real pixels either way. Only the translation vectors (tvecs) end up in
    "grid step" units instead of real-world ones.
    """
    if not np.isfinite(spacing_x) or not np.isfinite(spacing_y):
        raise ValueError("LED spacing must be finite")
    if spacing_x <= 0 or spacing_y <= 0:
        raise ValueError("LED spacing must be positive")
    if columns <= 0 or rows <= 0:
        raise ValueError("columns and rows must be positive")

    points: dict[int, np.ndarray] = {}
    for zero_based_led in range(columns * rows):
        row, column = divmod(zero_based_led, columns)
        beacon_id = h5_id_for_led1 + zero_based_led
        points[beacon_id] = np.array([column * spacing_x, row * spacing_y, 0.0], dtype=np.float32)
    return points


# =====================================================================
# STAGE 3 - Synchronize independent per-beacon tracks onto shared timestamps.
# =====================================================================
# Every beacon logs pixel positions on its own independent schedule, so there
# is no timestamp where all 22+ beacons were naturally measured together. To
# build one "view" (all beacons at one instant), we pick a target time and
# interpolate each beacon's position there from its two nearest real samples.


def interpolate_if_local(
    times: np.ndarray, xy: np.ndarray, sample_time: float, max_gap_s: float
) -> np.ndarray | None:
    """
    Estimate one beacon's pixel position at `sample_time` by linear
    interpolation between its two closest real measurements - but only if
    those measurements are actually close enough in time to trust.

    Returns None (refuses to guess) when:
      - sample_time falls before the first or after the last measurement
        (that would be extrapolation, not interpolation), or
      - the nearest real measurement on either side is more than max_gap_s
        away (e.g. the beacon was briefly out of view) - better to skip this
        view for this beacon than to fabricate a position across the gap.
    """
    # Binary search: find where sample_time would slot into the sorted times array.
    right = int(np.searchsorted(times, sample_time, side="left"))

    # Exact hit (within floating-point tolerance): no interpolation needed.
    if right < len(times) and np.isclose(times[right], sample_time, rtol=0, atol=1e-12):
        return xy[right].astype(np.float32)

    left = right - 1
    # sample_time is outside the beacon's observed time range - can't interpolate.
    if left < 0 or right >= len(times):
        return None
    # Either neighboring measurement is too far away in time - don't trust it.
    if sample_time - times[left] > max_gap_s or times[right] - sample_time > max_gap_s:
        return None

    dt = times[right] - times[left]
    if dt <= 0:
        return None
    # alpha = 0 at the left sample, 1 at the right sample; blend x/y accordingly.
    alpha = (sample_time - times[left]) / dt
    return ((1.0 - alpha) * xy[left] + alpha * xy[right]).astype(np.float32)


def extract_image_points(
    tracks: dict[int, tuple[np.ndarray, np.ndarray]],
    geometry: dict[int, np.ndarray],
    hz: float,
    max_gap_s: float,
    min_points: int,
    image_size: tuple[int, int],
    sample_times: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray], np.ndarray, list[int], list[list[int]]]:
    """
    Build the (object_points, image_points) lists that cv2.calibrateCamera
    expects - one entry per "view" (synchronized snapshot of all beacons).

    Steps:
      1. Only use beacons that appear BOTH in the H5 tracker data AND in the
         geometry - a beacon missing from either side can't form a correspondence,
         so it's dropped from both the image points and the 3D geometry.
      2. Walk across the whole recording at evenly spaced timestamps, `1/hz`
         seconds apart.
       3. At each timestamp, interpolate every locally available beacon.
          A missing beacon is omitted from that view instead of invalidating
          the complete view.
       4. Keep the view only if enough valid, non-collinear board points remain.
       5. Require at least MIN_VIEWS surviving timestamps, or bail out with a
         clear error (not enough synchronized coverage to calibrate on).
    """
    usable_ids = sorted(set(tracks) & set(geometry))
    dropped = sorted(set(geometry) - set(tracks))
    if dropped:
        print(f"Beacons missing from H5 entirely, dropped from geometry: {dropped}")
    if not usable_ids:
        raise ValueError("No beacon IDs overlap between H5 tracker data and geometry")

    width, height = image_size

    object_points: list[np.ndarray] = []  # 3D positions, one array per kept view
    image_points: list[np.ndarray] = []  # matching 2D pixel positions, one array per kept view
    kept_times: list[float] = []  # the timestamp each kept view corresponds to
    view_beacon_ids: list[list[int]] = []  # IDs used in each individual view

    for t in sample_times:
        img: list[np.ndarray] = []
        obj: list[np.ndarray] = []
        ids: list[int] = []

        for beacon_id in usable_ids:
            p = interpolate_if_local(*tracks[beacon_id], t, max_gap_s)
            if p is None:
                continue

            # A corrupted tracker estimate outside the sensor cannot be a real
            # image observation, so omit it.
            if not (0.0 <= p[0] < width and 0.0 <= p[1] < height):
                continue

            img.append(p)
            obj.append(geometry[beacon_id])
            ids.append(beacon_id)

        if len(img) < min_points:
            continue

        obj_array = np.asarray(obj, dtype=np.float32)
        img_array = np.asarray(img, dtype=np.float32)

        # Points from only one row or one column cannot define a planar
        # homography. Rank 2 confirms that the retained board points spread in
        # both the board's X and Y directions.
        centred_board_xy = obj_array[:, :2] - np.mean(obj_array[:, :2], axis=0)
        if np.linalg.matrix_rank(centred_board_xy) < 2:
            continue

        object_points.append(obj_array)
        image_points.append(img_array)
        kept_times.append(float(t))
        view_beacon_ids.append(ids)

    if len(object_points) < MIN_VIEWS:
        raise ValueError(
            f"Only {len(object_points)} valid image-point sets at {hz} Hz "
            f"(need at least {MIN_VIEWS}, with at least {min_points} points each). "
            "Try a lower --min-points, a larger --max-gap, or check tracker coverage."
        )

    return object_points, image_points, np.asarray(kept_times), usable_ids, view_beacon_ids


# =====================================================================
# STAGE 4 - Score the calibration: how far off are the re-projected points?
# =====================================================================
# After cv2.calibrateCamera solves for a camera model, we can check its
# quality by projecting the known 3D points back through that model and
# measuring how far the result lands from where the points were actually
# observed in pixels. Smaller = better.


def per_view_rmse(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: tuple[np.ndarray, ...],
    tvecs: tuple[np.ndarray, ...],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> list[float]:
    """
    For each view, re-project its 3D object points through the solved camera
    model (using that view's own rotation/translation) and compute the true
    root-mean-square pixel distance to the actually observed image points.
    This is the standard, trustworthy per-view error metric.
    """
    values: list[float] = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        residual = img - projected.reshape(-1, 2)
        # sqrt(mean over points of (dx^2 + dy^2)) = RMS pixel distance for this view.
        values.append(float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))))
    return values


def mean_reprojection_error(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: tuple[np.ndarray, ...],
    tvecs: tuple[np.ndarray, ...],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> float:
    """
    Reproduces the exact error formula from the OpenCV calibration tutorial,
    for direct comparison against that reference. Note this number is NOT
    directly comparable to per_view_rmse() above: cv2.norm(..., NORM_L2) here
    takes the L2 norm of the whole flattened per-view point array and divides
    by the point count, which understates the true average distance by a
    factor of sqrt(N) (N = points per view). Kept only because it's the
    tutorial's own method - per_view_rmse()/the overall cv2.calibrateCamera
    RMS are the numbers to actually trust.
    """
    total_error = 0.0
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        total_error += cv2.norm(img, projected.reshape(-1, 2), cv2.NORM_L2) / len(projected)
    return total_error / len(object_points)


# =====================================================================
# STAGE 5 - Command-line entry point: wire every stage above together.
# =====================================================================


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("h5", type=Path)
    parser.add_argument(
        "geometry_csv",
        type=Path,
        nargs="?",
        help="Optional CSV with beacon_id,X,Y,Z. If omitted, generate a grid layout "
        "from --columns/--rows/--spacing-x/--spacing-y.",
    )
    parser.add_argument("--spacing-x", type=float, help="Measured horizontal centre-to-centre LED pitch")
    parser.add_argument("--spacing-y", type=float, help="Measured vertical centre-to-centre LED pitch")
    parser.add_argument(
        "--columns", type=int, default=4, help="Number of columns in the LED-box/checkerboard grid (default: 4)"
    )
    parser.add_argument(
        "--rows", type=int, default=6, help="Number of rows in the LED-box/checkerboard grid (default: 6)"
    )
    parser.add_argument(
        "--h5-id-for-led1",
        type=int,
        default=0,
        help="HDF5 beacon ID corresponding to the document's physical LED 1 (default: 0)",
    )
    parser.add_argument("--hz", type=float, default=10.0, help="Sample rate for evenly spaced timestamps")
    parser.add_argument(
        "--time-source",
        choices=("auto", "decoder", "tracker"),
        default="auto",
        help=(
            "How to choose the phase of the regular sampling grid: auto uses "
            "ts_start_bit_s when coherent, decoder requires it, and tracker "
            "anchors the grid at tracker time zero (default: auto)"
        ),
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=0.05,
        help="Maximum seconds from sample time to each interpolation endpoint",
    )
    parser.add_argument(
        "--min-points",
        type=int,
        default=12,
        help="Minimum locally valid LED correspondences in each view (default: 12)",
    )
    parser.add_argument("--width", type=int, default=1280, help="Sensor width in pixels")
    parser.add_argument("--height", type=int, default=720, help="Sensor height in pixels")
    parser.add_argument(
        "--radial-coefficients",
        type=int,
        choices=(2, 3),
        default=3,
        help=(
            "Number of radial distortion terms to solve for: 3 (k1,k2,k3 - "
            "OpenCV's default) or 2 (k1,k2, with k3 fixed at 0 via "
            "cv2.CALIB_FIX_K3 - matches MATLAB's "
            "NumRadialDistortionCoefficients=2). Default: 3 (unchanged "
            "behavior)."
        ),
    )
    parser.add_argument(
        "--estimate-tangential-distortion",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Solve for tangential distortion (p1,p2). Pass "
            "--no-estimate-tangential-distortion to fix p1=p2=0 via "
            "cv2.CALIB_ZERO_TANGENT_DIST, matching MATLAB's "
            "EstimateTangentialDistortion=false. Default: true (unchanged "
            "behavior)."
        ),
    )
    parser.add_argument(
        "--outlier-rejection-rounds",
        type=int,
        default=0,
        help=(
            "After the initial fit, repeat this many times: drop the views "
            "with the worst per-view reprojection RMSE (see "
            "--outlier-rejection-fraction) and refit on what remains. 0 "
            "(default) reproduces the original single-shot behavior; your "
            "friend's MATLAB workflow did this at least once."
        ),
    )
    parser.add_argument(
        "--outlier-rejection-fraction",
        type=float,
        default=0.1,
        help="Fraction of remaining views dropped per outlier-rejection round (default: 0.1 = worst 10%%).",
    )
    parser.add_argument("--output", type=Path, default=Path("intrinsics.json"))
    args = parser.parse_args()

    if args.hz <= 0 or args.max_gap <= 0:
        parser.error("--hz and --max-gap must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    if args.columns <= 0 or args.rows <= 0:
        parser.error("--columns and --rows must be positive")
    if args.outlier_rejection_rounds < 0:
        parser.error("--outlier-rejection-rounds must be >= 0")
    if not (0.0 < args.outlier_rejection_fraction < 1.0):
        parser.error("--outlier-rejection-fraction must be between 0 and 1 (exclusive)")
    if args.min_points < 4:
        parser.error("--min-points must be at least 4")

    # ---- Stage 1: read the H5 tracker data for every beacon ----
    tracks = load_tracks(args.h5)
    decoder_start_times = load_decoder_start_times(args.h5)

    # ---- Stage 2: get 3D beacon geometry, either measured (CSV) or generated (LED-box grid) ----
    if args.geometry_csv is not None:
        if args.spacing_x is not None or args.spacing_y is not None:
            parser.error("Use either geometry_csv or --spacing-x/--spacing-y, not both")
        geometry = load_geometry(args.geometry_csv)
        geometry_source = str(args.geometry_csv)
    else:
        if args.spacing_x is None or args.spacing_y is None:
            parser.error(
                "The grid layout needs a physical pitch. Supply "
                "--spacing-x and --spacing-y, or provide geometry_csv."
            )
        geometry = make_led_box_geometry(
            args.spacing_x, args.spacing_y, args.columns, args.rows, args.h5_id_for_led1
        )
        geometry_source = f"generated {args.columns}x{args.rows} LED-box layout"

    print(f"H5 beacon IDs:       {sorted(tracks)}")
    print(f"Geometry beacon IDs: {sorted(geometry)} ({geometry_source})")

    # ---- Stage 3: synchronize into (object_points, image_points) per view ----
    usable_for_timing = sorted(set(tracks) & set(geometry))
    if not usable_for_timing:
        raise ValueError(
            "No beacon IDs overlap between the H5 tracker data and board geometry. "
            "Check --h5-id-for-led1 or the IDs in geometry_csv."
        )
    sample_grid, timestamp_source, phase_concentration = make_sample_times(
        tracks,
        usable_for_timing,
        decoder_start_times,
        args.hz,
        args.time_source,
    )
    print(f"Sampling time source: {timestamp_source}")
    if phase_concentration is not None:
        print(f"Decoder phase concentration: {phase_concentration:.3f} (1.0 is ideal)")

    object_points, image_points, sample_times, beacon_ids, view_beacon_ids = extract_image_points(
        tracks,
        geometry,
        args.hz,
        args.max_gap,
        args.min_points,
        (args.width, args.height),
        sample_grid,
    )
    print(f"\nUsable beacon IDs ({len(beacon_ids)}): {beacon_ids}")
    print(f"Image-point sets: {len(sample_times)}")
    point_counts = np.asarray([len(ids) for ids in view_beacon_ids])
    print(
        "Points/view: "
        f"min={point_counts.min()}, median={np.median(point_counts):.0f}, "
        f"max={point_counts.max()}"
    )
    print(f"Time span: {sample_times[0]:.3f}s - {sample_times[-1]:.3f}s")

    # ---- The actual calibration: this one call solves for the camera model ----
    # rms         : OpenCV's own overall RMS reprojection error (pooled across every
    #               point in every view) - the headline "how good is this?" number.
    # camera_matrix: the 3x3 intrinsic matrix [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
    # distortion  : lens distortion coefficients (k1, k2, p1, p2, k3).
    # rvecs/tvecs : per-view rotation/translation - where the "board" was relative
    #               to the camera in each view (one pair per view, in the same order
    #               as object_points/image_points).
    #
    # calibration_flags default to reproducing the exact original behavior
    # (no flags -> OpenCV solves k1,k2,k3,p1,p2, skew always fixed at 0).
    # --radial-coefficients 2 and/or --no-estimate-tangential-distortion make
    # this match a MATLAB estimateCameraParameters() run with
    # NumRadialDistortionCoefficients=2 / EstimateTangentialDistortion=false.
    # There is no flag for MATLAB's EstimateSkew=true: OpenCV's calibrateCamera
    # camera model has no skew term at all (K[0][1] is always exactly 0), so a
    # MATLAB run with EstimateSkew=true cannot be reproduced by this function
    # regardless of flags - expect fx/fy/cx/cy to be close but not identical,
    # and don't expect OpenCV's K[0][1] to match MATLAB's nonzero skew entry.
    calibration_flags = 0
    if args.radial_coefficients == 2:
        calibration_flags |= cv2.CALIB_FIX_K3
    if not args.estimate_tangential_distortion:
        calibration_flags |= cv2.CALIB_ZERO_TANGENT_DIST

    active_object_points = list(object_points)
    active_image_points = list(image_points)
    active_view_indices = list(range(len(object_points)))  # into the ORIGINAL lists, for reporting
    dropped_view_indices: list[int] = []

    for round_index in range(args.outlier_rejection_rounds + 1):
        rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            active_object_points, active_image_points, (args.width, args.height), None, None,
            flags=calibration_flags,
        )
        if round_index == args.outlier_rejection_rounds:
            break  # just solved the final, reported fit - don't drop anything after it

        view_errors = per_view_rmse(
            active_object_points, active_image_points, rvecs, tvecs, camera_matrix, distortion
        )
        num_to_drop = max(1, round(len(active_object_points) * args.outlier_rejection_fraction))
        # Never drop so many views that too few remain to solve for the
        # camera model (roughly: fewer than ~5 views makes the fit unstable
        # regardless of points/view).
        num_to_drop = min(num_to_drop, max(0, len(active_object_points) - 5))
        if num_to_drop == 0:
            print(
                f"Outlier rejection round {round_index + 1}: stopping early - "
                f"only {len(active_object_points)} views remain."
            )
            break

        worst_first = sorted(range(len(view_errors)), key=lambda i: view_errors[i], reverse=True)
        drop_positions = set(worst_first[:num_to_drop])
        print(
            f"Outlier rejection round {round_index + 1}: overall RMS {rms:.4f} px, "
            f"dropping {num_to_drop}/{len(active_object_points)} worst views "
            f"(view RMSE range being dropped: "
            f"{min(view_errors[i] for i in drop_positions):.3f}-"
            f"{max(view_errors[i] for i in drop_positions):.3f} px)"
        )

        dropped_view_indices.extend(active_view_indices[i] for i in drop_positions)
        active_object_points = [v for i, v in enumerate(active_object_points) if i not in drop_positions]
        active_image_points = [v for i, v in enumerate(active_image_points) if i not in drop_positions]
        active_view_indices = [v for i, v in enumerate(active_view_indices) if i not in drop_positions]

    object_points = active_object_points
    image_points = active_image_points
    # active_view_indices tracks which ORIGINAL views survived rejection, so
    # every other per-view array reported below must be re-indexed the same
    # way - otherwise beacon_ids/points_per_view/sample_times would still
    # describe the original (pre-rejection) view count while
    # object_points/image_points/rvecs/tvecs/per-view RMSE describe the
    # reduced set, silently misaligning every array position after the first
    # dropped view.
    view_beacon_ids = [view_beacon_ids[i] for i in active_view_indices]
    point_counts = point_counts[active_view_indices]
    sample_times = sample_times[active_view_indices]

    # ---- Stage 4: score the result ----
    view_rmse = per_view_rmse(object_points, image_points, rvecs, tvecs, camera_matrix, distortion)
    mean_error = mean_reprojection_error(object_points, image_points, rvecs, tvecs, camera_matrix, distortion)

    # ---- Save everything needed to reuse or audit this calibration later ----
    output = {
        "h5_file": str(args.h5),
        "geometry_source": geometry_source,
        "image_size": [args.width, args.height],
        "hz": args.hz,
        "max_gap_s": args.max_gap,
        "min_points_per_view": args.min_points,
        "position_timestamp_source": (
            "tracker time in seconds, relative to the first event processed "
            "by the tracker"
        ),
        "sampling_grid_phase_source": timestamp_source,
        "decoder_phase_concentration": phase_concentration,
        "distortion_model": {
            "radial_coefficients": args.radial_coefficients,
            "tangential_distortion_estimated": args.estimate_tangential_distortion,
            "skew_estimated": False,
            "skew_note": (
                "cv2.calibrateCamera has no skew term in its camera model "
                "(K[0][1] is always exactly 0), unlike MATLAB's "
                "EstimateSkew option - not reproducible here regardless of "
                "flags."
            ),
        },
        "outlier_rejection": {
            "rounds_requested": args.outlier_rejection_rounds,
            "fraction_per_round": args.outlier_rejection_fraction,
            "views_dropped": len(dropped_view_indices),
            "dropped_view_indices": dropped_view_indices,
        },
        "num_views": len(object_points),
        "beacon_ids": beacon_ids,
        "view_beacon_ids": view_beacon_ids,
        "points_per_view": point_counts.tolist(),
        "sample_times_s": sample_times.tolist(),
        "opencv_rms_px": float(rms),
        "mean_reprojection_error_px": mean_error,
        "per_view_rmse_px": view_rmse,
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.ravel().tolist(),
        "rvecs": [x.ravel().tolist() for x in rvecs],
        "tvecs": [x.ravel().tolist() for x in tvecs],
    }
    args.output.write_text(json.dumps(output, indent=2) + "\n")

    # ---- Human-readable summary to the terminal ----
    print(
        f"\nViews: {len(object_points)}  "
        f"Beacons/view: {point_counts.min()}-{point_counts.max()}"
    )
    print(f"OpenCV RMS reprojection error: {rms:.4f} px")
    print(f"Mean reprojection error (tutorial method): {mean_error:.4f} px")
    print("Per-view RMSE (px):")
    for t, e in zip(sample_times, view_rmse):
        print(f"  t={t:6.3f}s  rmse={e:.4f}")
    print("Camera matrix K:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(distortion.ravel())
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
