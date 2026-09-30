#!/bin/bash
# Runs run_calibration.sh once per PHYSICALLY SEPARATE calibration
# session (re-record the board each time - ideally unmount/remount the rig
# between sessions too) and compares the resulting stereo calibration across
# all runs, per calibration_quality.py's repeatability recommendation
# ("record 3 independent physical calibration sessions ... compare fx/fy/
# cx/cy, R, T, and baseline").
#
# calibration_quality.py itself does NOT compute this comparison - it only
# trusts a self-declared --repeatability-runs count. This wrapper (plus
# evaluation/check_repeatability.py) is what actually produces the numbers
# behind that count.
#
# Only the stereo_calibration_*.json is archived per run - stereo_calibrate.py
# runs cv2.stereoCalibrate() with CALIB_FIX_INTRINSIC, so its
# "camera_matrix_1"/"camera_matrix_2" fields are the per-camera intrinsics
# echoed straight through (not a joint re-fit), and its "baseline" field is
# already ||T|| - so this one file has everything check_repeatability.py
# needs; the separate per-camera *_intrinsics.json files don't need
# archiving too.
#
# Each argument is a FULL calibration_config.sh for one session. These
# should be identical except for event_filepaths (and global_frame_* paths,
# if used) - same led_columns/led_rows/led_spacing_x/led_spacing_y, same
# stereo_pair_selection, etc. - since you're testing rig repeatability, not
# comparing different settings.
#
# Usage:
#   ./run_repeatability.sh [--min-runs N] session1_config.sh session2_config.sh session3_config.sh [...]
#
# --min-runs N (default 3, matching calibration_quality.py's own gate):
# lowers the minimum number of session configs required to proceed. This
# exists ONLY to let you smoke-test the pipeline mechanics (does
# run_calibration.sh + archiving actually work?) before you've recorded
# every session for real. A run with --min-runs < 3 is NOT a valid
# repeatability conclusion - the script says so loudly, in the terminal and
# in the final summary, and refuses to suggest a --repeatability-runs value
# for calibration_quality.py. Note evaluation/check_repeatability.py
# separately requires >= 2 runs to compute any comparison at all - that
# floor is a mathematical requirement (you can't measure a spread from one
# point) and is not affected by this flag.
#
# Optional: set REPEATABILITY_PAIR="i,j" (0-based camera indices, matching
# your config's camera_names ordering) to pick which stereo pair to compare
# when a config calibrates more than one pair. Defaults to "0,1".
set -e
set -o pipefail

usage() {
    echo "Usage: $0 [--min-runs N] session1_config.sh session2_config.sh session3_config.sh [...]" >&2
    echo "(calibration_quality.py's repeatability gate requires >= 3 runs; --min-runs lets you go lower ONLY to test pipeline mechanics.)" >&2
}

min_runs=3
positional=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --min-runs)
            min_runs="$2"
            shift 2
            ;;
        --min-runs=*)
            min_runs="${1#--min-runs=}"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            while [[ $# -gt 0 ]]; do
                positional+=("$1")
                shift
            done
            ;;
        -*)
            echo "Error: unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            positional+=("$1")
            shift
            ;;
    esac
done

if [[ ! "$min_runs" =~ ^[0-9]+$ ]] || (( min_runs < 1 )); then
    echo "Error: --min-runs must be a positive integer, got '$min_runs'." >&2
    exit 1
fi

if (( ${#positional[@]} < min_runs )); then
    echo "Error: need at least $min_runs session config file(s) (--min-runs $min_runs), got ${#positional[@]}." >&2
    usage
    exit 1
fi

if (( min_runs < 3 )); then
    echo "
##########################################################################
# WARNING: --min-runs $min_runs (< 3). This run does NOT satisfy the      #
# calibration_quality.py repeatability requirement (>= 3 independent      #
# physical sessions). Use this ONLY to check the pipeline mechanics work  #
# - do NOT treat any report from this run as a valid repeatability        #
# conclusion, and do NOT pass its run count to --repeatability-runs.      #
##########################################################################
" >&2
fi

set -- "${positional[@]}"

archive_root="./track_logs/repeatability"
mkdir -p "$archive_root"
run_args_file="$archive_root/run_args.txt"
: > "$run_args_file"

run_number=0
for config_file in "$@"; do
    run_number=$((run_number + 1))
    echo "
=======================================================
REPEATABILITY RUN $run_number/$#: $config_file
======================================================="

    if [[ ! -f "$config_file" ]]; then
        echo "Error: config file not found: $config_file" >&2
        exit 1
    fi

    # Run the full pipeline for this session exactly as normal.
    bash run_calibration.sh "$config_file"

    # Re-source this session's config in a subshell (so its variables don't
    # leak into the next iteration) purely to find out which file it just
    # produced - same camera_names/stereo_output_dir logic
    # run_calibration.sh itself uses.
    (
        # shellcheck source=/dev/null
        source "$config_file"

        if (( ${#camera_names[@]} == 0 )); then
            camera_names=()
            for ((i = 0; i < number_camera; i++)); do
                camera_names+=("camera_$((i + 1))")
            done
        fi

        pair="${REPEATABILITY_PAIR:-0,1}"
        IFS=',' read -r pair_i pair_j <<< "$pair"
        cam1_name="${camera_names[$pair_i]}"
        cam2_name="${camera_names[$pair_j]}"

        stereo_json="$stereo_output_dir/stereo_calibration_${cam1_name}_${cam2_name}.json"

        if [[ ! -f "$stereo_json" ]]; then
            echo "Error: expected stereo calibration not found: $stereo_json" >&2
            echo "Check REPEATABILITY_PAIR='$pair' matches a pair this config actually calibrates." >&2
            exit 1
        fi

        # Archive this run's output now, before the NEXT run overwrites
        # "$stereo_output_dir/stereo_calibration_${cam1_name}_${cam2_name}.json"
        # (stereo_output_dir is typically the same path across session
        # configs, since only event_filepaths/global_frame_* differ).
        run_dir="$archive_root/run_$run_number"
        mkdir -p "$run_dir"
        cp "$stereo_json" "$run_dir/stereo_calibration.json"

        echo "$run_dir/stereo_calibration.json" >> "$run_args_file"

        echo "Archived run $run_number ($cam1_name + $cam2_name) to $run_dir"
    )
done

echo "
=======================================================
Comparing $# run(s)
======================================================="

compare_args=()
while IFS= read -r stereo_json; do
    compare_args+=(--run "$stereo_json")
done < "$run_args_file"

python evaluation/check_repeatability.py \
    "${compare_args[@]}" \
    --output-dir "$archive_root"

echo "
======================================================="
if (( $# >= 3 )); then
    echo "Repeatability report: $archive_root/repeatability_report.txt"
    echo "Once reviewed, pass --repeatability-runs $# to calibration_quality.py."
else
    echo "Repeatability report: $archive_root/repeatability_report.txt (from only $# run(s))"
    echo "NOT a valid repeatability conclusion - --min-runs was used below 3."
    echo "Do not pass this run count to calibration_quality.py's --repeatability-runs."
fi
echo "======================================================="
