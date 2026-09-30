#!/usr/bin/env python3
"""
Run intrinsic_calibration.py once per camera, from already-existing .h5
tracker files (one per camera). This is purely the intrinsic-calibration
stage - no C++ tracker/decoder step here - so you can re-run just this part
for one camera or many, without going through the full run_calibration.sh
pipeline. It's a thin wrapper: each camera's actual calibration is done by
your existing intrinsic_calibration.py, unmodified, so results are
identical to running that script by hand once per camera - this just saves
you from retyping the shared flags and gives you a combined summary.

Usage (repeat --camera once per camera):
    python run_intrinsics_for_all_cameras.py \
        --camera camera_1 track_logs/camera_1/some_file.h5 \
        --camera camera_2 track_logs/camera_2/other_file.h5 \
        --columns 4 --rows 6 --spacing-x 0.12 --spacing-y 0.12

Each camera gets its own intrinsics.json, written next to its H5 file as
<h5_stem>_intrinsics.json (matching run_calibration.sh's own naming
convention) unless --output-dir is given. A manifest.json listing every
camera name -> intrinsics path is also written, for convenience when
feeding these into stereo_calibrate.py later (which needs one
intrinsics.json path per camera).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

THIS_DIR = Path(__file__).resolve().parent
INTRINSIC_SCRIPT = THIS_DIR / "intrinsic_calibration.py"


def run_one_camera(
    camera_name: str,
    h5_path: Path,
    output_path: Path,
    shared_args: list[str],
) -> dict[str, Any]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(INTRINSIC_SCRIPT), str(h5_path), *shared_args, "--output", str(output_path)]
    print(f"\n=== {camera_name}: {h5_path} ===")
    print(" ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode != 0:
        raise RuntimeError(f"intrinsic_calibration.py failed for {camera_name} (exit {result.returncode})")
    return json.loads(output_path.read_text())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--camera",
        nargs=2,
        metavar=("NAME", "H5_PATH"),
        action="append",
        required=True,
        help="Repeatable: one camera's name and its .h5 tracker file. Give this once per camera.",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None,
        help="Where to write each camera's intrinsics.json (default: next to its own .h5 file, "
        "as <h5_stem>_intrinsics.json)",
    )
    parser.add_argument("--manifest", type=Path, default=Path("intrinsics_manifest.json"))

    # Shared calibration parameters, passed straight through to
    # intrinsic_calibration.py for every camera - same names/defaults as
    # that script, so leaving these unset reproduces its own default
    # behavior exactly. If your cameras genuinely need different values
    # (e.g. different sensor sizes), run intrinsic_calibration.py directly
    # for that one camera instead of through this wrapper.
    parser.add_argument("--columns", type=int, default=4)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--spacing-x", type=float, default=None)
    parser.add_argument("--spacing-y", type=float, default=None)
    parser.add_argument("--h5-id-for-led1", type=int, default=0)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--time-source", choices=("auto", "decoder", "tracker"), default="auto")
    parser.add_argument("--max-gap", type=float, default=0.05)
    parser.add_argument("--min-points", type=int, default=12)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--radial-coefficients", type=int, choices=(2, 3), default=3)
    parser.add_argument("--estimate-tangential-distortion", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--outlier-rejection-rounds", type=int, default=0)
    parser.add_argument("--outlier-rejection-fraction", type=float, default=0.1)
    args = parser.parse_args()

    if not INTRINSIC_SCRIPT.exists():
        raise SystemExit(
            f"Error: {INTRINSIC_SCRIPT} not found - run this script from the same directory as "
            "intrinsic_calibration.py, or move it there."
        )

    shared_args = [
        "--columns", str(args.columns),
        "--rows", str(args.rows),
        "--h5-id-for-led1", str(args.h5_id_for_led1),
        "--hz", str(args.hz),
        "--time-source", args.time_source,
        "--max-gap", str(args.max_gap),
        "--min-points", str(args.min_points),
        "--width", str(args.width),
        "--height", str(args.height),
        "--radial-coefficients", str(args.radial_coefficients),
        "--outlier-rejection-rounds", str(args.outlier_rejection_rounds),
        "--outlier-rejection-fraction", str(args.outlier_rejection_fraction),
    ]
    if args.spacing_x is not None:
        shared_args += ["--spacing-x", str(args.spacing_x)]
    if args.spacing_y is not None:
        shared_args += ["--spacing-y", str(args.spacing_y)]
    if not args.estimate_tangential_distortion:
        shared_args += ["--no-estimate-tangential-distortion"]

    manifest: dict[str, dict[str, str]] = {}
    summary_rows: list[tuple[str, float | None, int | None, str]] = []
    for camera_name, h5_path_str in args.camera:
        h5_path = Path(h5_path_str)
        if not h5_path.exists():
            raise SystemExit(f"Error: {h5_path} does not exist (camera '{camera_name}')")

        if args.output_dir is not None:
            output_path = args.output_dir / f"{camera_name}_intrinsics.json"
        else:
            output_path = h5_path.parent / f"{h5_path.stem}_intrinsics.json"

        result = run_one_camera(camera_name, h5_path, output_path, shared_args)
        manifest[camera_name] = {
            "h5_file": str(h5_path),
            "intrinsics_json": str(output_path),
        }
        summary_rows.append(
            (camera_name, result.get("opencv_rms_px"), result.get("num_views"), str(output_path))
        )

    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n")

    print("\n" + "=" * 78)
    print("SUMMARY")
    print("=" * 78)
    print(f"{'Camera':<15} {'RMS (px)':>10} {'Views':>8}  Output")
    for camera_name, rms, num_views, output_path in summary_rows:
        rms_str = f"{rms:.4f}" if rms is not None else "N/A"
        print(f"{camera_name:<15} {rms_str:>10} {num_views!s:>8}  {output_path}")
    print(f"\nManifest written to {args.manifest}")


if __name__ == "__main__":
    main()
