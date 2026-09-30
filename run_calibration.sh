#!/bin/bash

# Usage:
#   cp calibration_config.example.sh calibration_config.sh   # once, edit paths
#   ./run_calibration.sh                                     # uses calibration_config.sh
#   ./run_calibration.sh path/to/other_config.sh             # or pass one explicitly
trap 'exit 130' INT TERM
set -e
set -o pipefail

# Cap internal math-library threading (Eigen/OpenMP) to 1 thread per
# process. Several track_and_decode() calls run AT THE SAME TIME (see
# PARALLEL EXECUTION below) - each is its own OS process, so real
# parallelism across them comes from the OS scheduling those processes
# across your CPU cores, not from further threading inside any single one
# of them. Without this, each process's own internal math work could try
# to use ALL your cores by itself, so N concurrent processes would fight
# each other for N times more threads than you actually have cores -
# capping each process to 1 thread avoids that oversubscription.
# `: "${VAR:=1}"` only sets a default - if you've already exported a
# different value yourself before running this script, that value wins.
: "${OMP_NUM_THREADS:=1}"
: "${EIGEN_NUM_THREADS:=1}"
export OMP_NUM_THREADS
export EIGEN_NUM_THREADS

config_file="${1:-calibration_config.sh}"
if [[ ! -f "$config_file" ]]; then
    echo "Error: config file not found: $config_file" >&2
    echo "Copy calibration_config.example.sh to calibration_config.sh, edit the paths, and re-run." >&2
    exit 1
fi
# shellcheck source=calibration_config.example.sh
source "$config_file"

# Global frame calibration (optional): triangulates an LED board that was
# placed stationary on the floor (observed by one stereo pair) to attach a
# global coordinate system to that pair's Camera 1 frame - see
# stereo/global_frame_calibrate.py. Defaults keep configs that predate this
# step working unchanged; set run_global_frame_calibration=true and the
# variables below in calibration_config.sh to enable it.
: "${run_global_frame_calibration:=false}"
: "${global_frame_reference_pair:=}"
: "${global_frame_cam1_event_filepath:=}"
: "${global_frame_cam2_event_filepath:=}"
: "${global_frame_output_dir:=${stereo_output_dir:-./track_logs/stereo}/global_frame}"
: "${global_frame_max_duration_s:=}"
: "${evaluation_coverage_complete:=false}"
: "${evaluation_repeatability_runs:=1}"

# Held-out dev/test evaluation (optional): re-uses a stereo pair's already-
# fitted, frozen calibration (from the stereo loop below) and evaluates it
# against a NEW, physically disjoint recording - the only way
# calibration_quality.py's status can reach PASS (it never can on
# partition=calibration_fit, since that recording was also used to FIT the
# calibration). Requires run_stereo_calibration=true and
# run_quality_evaluation=true.
: "${run_heldout_evaluation:=false}"
: "${heldout_partition:=dev}"
: "${heldout_reference_pair:=}"
: "${heldout_cam1_event_filepath:=}"
: "${heldout_cam2_event_filepath:=}"
: "${heldout_coverage_complete:=false}"
: "${heldout_output_dir:=${stereo_output_dir:-./track_logs/stereo}/heldout_${heldout_partition}}"

if [[ "$run_global_frame_calibration" == true && "$run_stereo_calibration" != true ]]; then
    echo "Error: run_global_frame_calibration=true requires run_stereo_calibration=true (the global frame step reuses a stereo pair's R/T)." >&2
    exit 1
fi

if [[ "$run_heldout_evaluation" == true && "$run_stereo_calibration" != true ]]; then
    echo "Error: run_heldout_evaluation=true requires run_stereo_calibration=true (it evaluates against a stereo pair's frozen calibration)." >&2
    exit 1
fi
if [[ "$run_heldout_evaluation" == true && "$run_quality_evaluation" != true ]]; then
    echo "Error: run_heldout_evaluation=true requires run_quality_evaluation=true (calibration_quality.py is what runs the held-out evaluation)." >&2
    exit 1
fi
if [[ "$run_heldout_evaluation" == true && "$heldout_partition" != "dev" && "$heldout_partition" != "test" ]]; then
    echo "Error: heldout_partition must be 'dev' or 'test', got '$heldout_partition'." >&2
    exit 1
fi

# Fail early when number_camera and the two arrays disagree. This avoids
# processing one camera with another camera's input path.
if (( number_camera <= 0 )); then
    echo "Error: number_camera must be greater than zero." >&2
    exit 1
fi

# camera_names is optional: leave it as camera_names=() in the config to get
# camera_1, camera_2, ... camera_N automatically from number_camera. Only set
# it explicitly if you want different names (e.g. "left"/"right", or a rig's
# physical labels) - if you do, it must have exactly number_camera entries.
if (( ${#camera_names[@]} == 0 )); then
    camera_names=()
    for ((camera_index = 0; camera_index < number_camera; camera_index++)); do
        camera_names+=("camera_$((camera_index + 1))")
    done
elif (( ${#camera_names[@]} != number_camera )); then
    echo "Error: number_camera=$number_camera, but camera_names has ${#camera_names[@]} entries (leave camera_names=() to auto-generate camera_1..camera_$number_camera instead)." >&2
    exit 1
fi

if (( ${#event_filepaths[@]} != number_camera )); then
    echo "Error: number_camera=$number_camera, but event_filepaths has ${#event_filepaths[@]} entries." >&2
    exit 1
fi
if (( ${#intrinsic_event_filepaths[@]} != number_camera )); then
    echo "Error: number_camera=$number_camera, but intrinsic_event_filepaths has ${#intrinsic_event_filepaths[@]} entries." >&2
    exit 1
fi
if [[ "$run_stereo_calibration" == true ]]; then
    declare -a resolved_stereo_pairs=()
    case "$stereo_pair_selection" in
        all_pairs)
            for ((i = 0; i < number_camera; i++)); do
                for ((j = i + 1; j < number_camera; j++)); do
                    resolved_stereo_pairs+=("$i,$j")
                done
            done
            ;;
        explicit)
            resolved_stereo_pairs=("${stereo_pairs[@]}")
            ;;
        *)
            echo "Error: stereo_pair_selection must be 'all_pairs' or 'explicit', got '$stereo_pair_selection'." >&2
            exit 1
            ;;
    esac

    if (( ${#resolved_stereo_pairs[@]} == 0 )); then
        echo "Error: run_stereo_calibration=true but no camera pairs were resolved." >&2
        echo "With stereo_pair_selection=all_pairs this means number_camera < 2." >&2
        echo "With stereo_pair_selection=explicit this means stereo_pairs is empty." >&2
        exit 1
    fi

    # Validate every resolved pair up front, before running anything, so a
    # typo in an explicit pair list fails fast instead of partway through a
    # long run.
    for pair in "${resolved_stereo_pairs[@]}"; do
        IFS=',' read -r pair_i pair_j <<< "$pair"
        if ! [[ "$pair_i" =~ ^[0-9]+$ && "$pair_j" =~ ^[0-9]+$ ]]; then
            echo "Error: stereo pair '$pair' must be two non-negative integers separated by a comma (e.g. '0,1')." >&2
            exit 1
        fi
        if (( pair_i < 0 || pair_i >= number_camera || pair_j < 0 || pair_j >= number_camera )); then
            echo "Error: stereo pair '$pair' references an out-of-range camera index (have $number_camera cameras: indices 0-$((number_camera - 1)))." >&2
            exit 1
        fi
        if (( pair_i == pair_j )); then
            echo "Error: stereo pair '$pair' has identical camera indices; a stereo pair needs two different cameras." >&2
            exit 1
        fi
    done

    declare -a required_scripts=(
        intrinsic/intrinsic_calibration.py
        stereo/time_sync.py
        stereo/build_stereo_points.py
        stereo/stereo_calibrate.py
        evaluation/calibration_quality.py
    )
    if [[ "$run_global_frame_calibration" == true ]]; then
        required_scripts+=(stereo/global_frame_calibrate.py)
    fi
    for required_script in "${required_scripts[@]}"; do
        if [[ ! -f "$required_script" ]]; then
            echo "Error: required calibration script does not exist: $required_script" >&2
            exit 1
        fi
    done

    # Validate the global frame step up front too, so a bad reference pair or
    # a missing "board on floor" recording fails before any tracking runs,
    # not after the whole rig has already been processed.
    if [[ "$run_global_frame_calibration" == true ]]; then
        if [[ -z "$global_frame_reference_pair" ]]; then
            echo "Error: run_global_frame_calibration=true requires global_frame_reference_pair (e.g. \"0,1\"), naming one of the pairs this run calibrates." >&2
            exit 1
        fi
        global_frame_pair_is_valid=false
        for pair in "${resolved_stereo_pairs[@]}"; do
            if [[ "$pair" == "$global_frame_reference_pair" ]]; then
                global_frame_pair_is_valid=true
                break
            fi
        done
        if [[ "$global_frame_pair_is_valid" != true ]]; then
            echo "Error: global_frame_reference_pair='$global_frame_reference_pair' is not among the stereo pairs this run will calibrate (${resolved_stereo_pairs[*]})." >&2
            exit 1
        fi
        if [[ -z "$global_frame_cam1_event_filepath" || ! -f "$global_frame_cam1_event_filepath" ]]; then
            echo "Error: global_frame_cam1_event_filepath does not exist: $global_frame_cam1_event_filepath" >&2
            echo "This should be the recording of the LED board placed stationary on the floor, from the first camera of global_frame_reference_pair." >&2
            exit 1
        fi
        if [[ -z "$global_frame_cam2_event_filepath" || ! -f "$global_frame_cam2_event_filepath" ]]; then
            echo "Error: global_frame_cam2_event_filepath does not exist: $global_frame_cam2_event_filepath" >&2
            echo "This should be the recording of the LED board placed stationary on the floor, from the second camera of global_frame_reference_pair." >&2
            exit 1
        fi
    fi

    # Validate the held-out evaluation step up front too, for the same
    # reason as the global frame step above.
    if [[ "$run_heldout_evaluation" == true ]]; then
        if [[ -z "$heldout_reference_pair" ]]; then
            echo "Error: run_heldout_evaluation=true requires heldout_reference_pair (e.g. \"0,1\"), naming one of the pairs this run calibrates." >&2
            exit 1
        fi
        heldout_pair_is_valid=false
        for pair in "${resolved_stereo_pairs[@]}"; do
            if [[ "$pair" == "$heldout_reference_pair" ]]; then
                heldout_pair_is_valid=true
                break
            fi
        done
        if [[ "$heldout_pair_is_valid" != true ]]; then
            echo "Error: heldout_reference_pair='$heldout_reference_pair' is not among the stereo pairs this run will calibrate (${resolved_stereo_pairs[*]})." >&2
            exit 1
        fi
        if [[ -z "$heldout_cam1_event_filepath" || ! -f "$heldout_cam1_event_filepath" ]]; then
            echo "Error: heldout_cam1_event_filepath does not exist: $heldout_cam1_event_filepath" >&2
            echo "This should be a NEW recording, physically disjoint from the one used to fit the calibration - board moved differently, not a rerun of the fitting clip." >&2
            exit 1
        fi
        if [[ -z "$heldout_cam2_event_filepath" || ! -f "$heldout_cam2_event_filepath" ]]; then
            echo "Error: heldout_cam2_event_filepath does not exist: $heldout_cam2_event_filepath" >&2
            echo "This should be a NEW recording, physically disjoint from the one used to fit the calibration - board moved differently, not a rerun of the fitting clip." >&2
            exit 1
        fi
    fi
fi

#=============================================================================#
# PARALLEL EXECUTION
#=============================================================================#
# Tracking/decoding is the most compute-heavy stage (it processes the whole
# raw recording), and every recording's tracking is independent of every
# other recording's - a camera's intrinsics recording doesn't need to wait
# for another camera's stereo recording, etc. So instead of processing
# recordings one at a time, this script launches ALL of them in the
# background at once, then only blocks (via `wait`) at the specific points
# where a later step genuinely depends on an earlier one - e.g. stereo
# calibration for a pair needs BOTH cameras' stereo tracking AND BOTH
# cameras' intrinsic calibration to be done, but does not care whether
# global-frame or held-out tracking (for a possibly different recording)
# has finished yet.
#
# Two real correctness hazards had to be fixed to make this safe, not just
# "add an & and hope":
#   1. track_and_decode() used to rebuild the AEMOT tracker and event
#      decoder on every call. Running several calls at once would mean
#      several concurrent `cmake --build` invocations racing on the SAME
#      build directory - ensure_toolchain_built() below now does this
#      ONCE, up front, before anything is backgrounded.
#   2. track_and_decode() (via aemotevt_to_es_faery.py) used to write every
#      call's per-track intermediate files into one SHARED
#      track_logs/temp/ directory, which it also `rm -rf`s at the start of
#      each call. Two calls running at once would let one call's cleanup
#      delete another's in-progress files - each call now uses its own
#      track_logs/temp_<label>/ directory instead (see
#      aemotevt_to_es_faery.py's --temp-dir).

# Builds the AEMOT tracker and event decoder ONCE, before any (possibly
# parallel) track_and_decode calls. See PARALLEL EXECUTION above for why
# this can no longer live inside track_and_decode() itself.
ensure_toolchain_built() {
    echo "
---------------------------------------
-----   BUILDING AEMOT + DECODER  -----
---------------------------------------"
    local aemot_build_dir="AEMOT-realtime/build"
    if [ ! -d "$aemot_build_dir" ]; then
        echo "AEMOT build dir doesn't exist"
        cmake -S AEMOT-realtime/ -B AEMOT-realtime/build -DCMAKE_BUILD_TYPE=Release
    fi
    cmake --build ./AEMOT-realtime/build/ --parallel

    local decoder_build_dir="event_decoder/build"
    if [ ! -d "$decoder_build_dir" ]; then
        echo "Decoder build dir doesn't exist"
        cmake -S event_decoder/ -B event_decoder/build -DCMAKE_BUILD_TYPE=Release
    fi
    cmake --build ./event_decoder/build/ --parallel
    echo "Done."
}

# Runs the AEMOT tracker and event decoder on one recording, producing an
# HDF5 file of decoded beacon detections. Safe to run several of these AT
# THE SAME TIME - see PARALLEL EXECUTION above. Does NOT build the
# AEMOT/decoder binaries itself any more; call ensure_toolchain_built once,
# first.
#
# track_log_dir is deliberately NOT a separate parameter: aemot_replay names
# its own output directory after the label (its 3rd CLI argument), so the
# directory this function globs for .bees files afterward must be derived
# from that same label rather than passed in independently - otherwise the
# two can silently point at different directories and the glob below finds
# nothing.
#
# Bash functions can't return strings, and a call running in a BACKGROUNDED
# subshell (`track_and_decode ... &`) loses its variables entirely once
# that subshell exits - so besides setting TRACK_AND_DECODE_H5,
# TRACK_AND_DECODE_TRACKER_OUTPUT and TRACK_AND_DECODE_DIR directly (for a
# synchronous caller), this ALSO persists the same three values to
# "<track_log_dir>/.track_and_decode_result" on disk. After `wait`-ing for
# a backgrounded call's PID, call load_track_and_decode_result with the
# same label to repopulate those variables in whichever shell needs them.
track_and_decode() {
    local event_filepath="$1"
    local label="$2"
    local track_log_dir="./track_logs/$label"
    local temp_dir_name="temp_$label"
    local temp_dir="./track_logs/$temp_dir_name"

    if [[ ! -f "$event_filepath" ]]; then
        echo "Error: input event file does not exist: $event_filepath" >&2
        exit 1
    fi

    mkdir -p "$track_log_dir"

    #--------------------------------------------#
    #-----             TRACKING             -----#
    #--------------------------------------------#
    echo "
---------------------------------------
-----       RUNNING TRACKER ($label)       -----
---------------------------------------"
    echo "Running aemot_replay on:    $event_filepath"
    echo "Using camera log directory: $track_log_dir"

    ./AEMOT-realtime/build/aemot_replay calibration "$event_filepath" "$label"

    echo "Done."

    echo "
---------------------------------------
-----  CONVERTING TRACKER OUTPUT ($label)  -----
---------------------------------------"
    # Select the newest .bees tracker log specifically. The old code selected
    # the newest file of any type, which could accidentally choose .beesum or
    # .aemotevt.
    shopt -s nullglob
    tracker_logs=("$track_log_dir"/*.bees)
    shopt -u nullglob
    if (( ${#tracker_logs[@]} == 0 )); then
        echo "Error: tracker produced no .bees file in $track_log_dir" >&2
        exit 1
    fi

    local tracker_log_file="${tracker_logs[0]}"
    for candidate in "${tracker_logs[@]:1}"; do
        if [[ "$candidate" -nt "$tracker_log_file" ]]; then
            tracker_log_file="$candidate"
        fi
    done
    local tracker_output_file
    tracker_output_file="$(basename "${tracker_log_file%.bees}")"

    echo "Convert aemotevt file:    $tracker_output_file"
    # Convert logged associated events - into THIS label's own temp
    # directory, not a directory shared with any other concurrently
    # running track_and_decode call.

    rm -rf "$temp_dir"
    mkdir -p "$temp_dir"

    python ./track_logs/aemotevt_to_es_faery.py \
        "$label/$tracker_output_file" \
        --output "$tracker_output_file" \
        --temp-dir "$temp_dir_name"
    echo "Done."

    #--------------------------------------------#
    #-----             DECODING             -----#
    #--------------------------------------------#
    echo "
---------------------------------------
-----       RUNNING DECODER ($label)       -----
---------------------------------------"
    shopt -s nullglob
    track_es_files=("$temp_dir"/*.es)
    shopt -u nullglob
    if (( ${#track_es_files[@]} == 0 )); then
        echo "Error: no per-track .es files were created for $label" >&2
        exit 1
    fi

    # Absolute paths, computed BEFORE any `cd`, so they still resolve
    # correctly once the subshell below changes directory.
    local decoder_bin_abs="$(pwd)/event_decoder/build/decoder_replay"
    local decoder_config_abs="$(pwd)/event_decoder/configs/default.yaml"

    for track_es_file in "${track_es_files[@]}"; do
        echo "Running decoder:    ${track_es_file}"

        # decoder_replay writes its output to a FIXED, hardcoded
        # ./decoder_output.csv in the current working directory, regardless
        # of input filename - harmless when only one call is ever active at
        # once, but a real race if two labels' decoder_replay calls run at
        # the same time (see PARALLEL EXECUTION above) and both try to
        # write that same relative path. Run it from inside THIS label's
        # own (already-unique) temp_dir instead, so its output can only
        # ever collide with another .es file from the SAME label - and
        # those are already processed one at a time, in this same loop.
        ( cd "$temp_dir" && "$decoder_bin_abs" "$decoder_config_abs" "$(basename "$track_es_file")" )

        decoded_file="${track_es_file%.es}_decoded.csv"
        mv "$temp_dir/decoder_output.csv" "$decoded_file"
        # rm "$track_es_file"
    done

    echo "
---------------------------------------
-----     PROCESSING DECODER ($label)      -----
---------------------------------------"
    python utils/filter_decoded_files.py \
        --decoder_directory "$temp_dir" \
        --log_directory "$track_log_dir/$tracker_output_file.bees" \
        --output "$track_log_dir/$tracker_output_file.h5" \
        -vv

    TRACK_AND_DECODE_H5="$track_log_dir/$tracker_output_file.h5"
    TRACK_AND_DECODE_TRACKER_OUTPUT="$tracker_output_file"
    TRACK_AND_DECODE_DIR="$track_log_dir"

    {
        echo "TRACK_AND_DECODE_H5=$TRACK_AND_DECODE_H5"
        echo "TRACK_AND_DECODE_TRACKER_OUTPUT=$TRACK_AND_DECODE_TRACKER_OUTPUT"
        echo "TRACK_AND_DECODE_DIR=$TRACK_AND_DECODE_DIR"
    } > "$track_log_dir/.track_and_decode_result"
}

# Reads back a track_and_decode() result written by a (possibly already
# finished, possibly backgrounded) call for the given label, repopulating
# TRACK_AND_DECODE_H5 / TRACK_AND_DECODE_TRACKER_OUTPUT / TRACK_AND_DECODE_DIR
# in whichever shell calls this. Use this after `wait`-ing for a
# backgrounded track_and_decode call's PID.
load_track_and_decode_result() {
    local label="$1"
    local result_file="./track_logs/$label/.track_and_decode_result"
    if [[ ! -f "$result_file" ]]; then
        echo "Error: no track_and_decode result found for '$label' at $result_file" >&2
        echo "(the background tracking job for this label may have failed before writing its result - check its log)" >&2
        exit 1
    fi
    # shellcheck source=/dev/null
    source "$result_file"
}

# Waits for a background job's PID and fails loudly (printing the tail of
# its log file) if it exited non-zero. Plain `wait` alone does not trigger
# `set -e`, so a failed background job would otherwise be silently ignored.
wait_or_fail() {
    local pid="$1"
    local job_label="$2"
    local log_file="${3:-}"
    if ! wait "$pid"; then
        echo "Error: background job '$job_label' (PID $pid) failed." >&2
        if [[ -n "$log_file" && -f "$log_file" ]]; then
            echo "--- last 40 lines of $log_file ---" >&2
            tail -n 40 "$log_file" >&2
        fi
        exit 1
    fi
}

mkdir -p ./track_logs
ensure_toolchain_built

declare -a generated_h5_files
declare -a generated_intrinsics_files
declare -a intrinsic_track_pid
declare -a stereo_track_pid
declare -a intrinsic_track_log
declare -a stereo_track_log

#-----------------------------------------------------------------------#
# PHASE 1: launch every recording's tracking/decoding job AT THE SAME TIME
#-----------------------------------------------------------------------#
echo "
=======================================================
LAUNCHING TRACKING JOBS IN PARALLEL
======================================================="
for ((camera_index = 0; camera_index < number_camera; camera_index++)); do
    camera_name="${camera_names[$camera_index]}"
    intrinsic_event_filepath="${intrinsic_event_filepaths[$camera_index]}"
    stereo_event_filepath="${event_filepaths[$camera_index]}"

    intrinsic_track_log[$camera_index]="./track_logs/${camera_name}_intrinsics.track.log"
    ( track_and_decode "$intrinsic_event_filepath" "${camera_name}_intrinsics" ) \
        > "${intrinsic_track_log[$camera_index]}" 2>&1 &
    intrinsic_track_pid[$camera_index]=$!
    echo "  [intrinsics tracking] $camera_name -> PID ${intrinsic_track_pid[$camera_index]} (log: ${intrinsic_track_log[$camera_index]})"

    stereo_track_log[$camera_index]="./track_logs/${camera_name}_stereo.track.log"
    ( track_and_decode "$stereo_event_filepath" "${camera_name}_stereo" ) \
        > "${stereo_track_log[$camera_index]}" 2>&1 &
    stereo_track_pid[$camera_index]=$!
    echo "  [stereo tracking]     $camera_name -> PID ${stereo_track_pid[$camera_index]} (log: ${stereo_track_log[$camera_index]})"
done

if [[ "$run_global_frame_calibration" == true ]]; then
    IFS=',' read -r global_cam1_index global_cam2_index <<< "$global_frame_reference_pair"
    global_cam1_name="${camera_names[$global_cam1_index]}"
    global_cam2_name="${camera_names[$global_cam2_index]}"

    global_frame_track_log_1="./track_logs/${global_cam1_name}_global_frame.track.log"
    ( track_and_decode "$global_frame_cam1_event_filepath" "${global_cam1_name}_global_frame" ) \
        > "$global_frame_track_log_1" 2>&1 &
    global_frame_track_pid_1=$!
    echo "  [global-frame tracking] $global_cam1_name -> PID $global_frame_track_pid_1 (log: $global_frame_track_log_1)"

    global_frame_track_log_2="./track_logs/${global_cam2_name}_global_frame.track.log"
    ( track_and_decode "$global_frame_cam2_event_filepath" "${global_cam2_name}_global_frame" ) \
        > "$global_frame_track_log_2" 2>&1 &
    global_frame_track_pid_2=$!
    echo "  [global-frame tracking] $global_cam2_name -> PID $global_frame_track_pid_2 (log: $global_frame_track_log_2)"
fi

if [[ "$run_heldout_evaluation" == true ]]; then
    IFS=',' read -r heldout_cam1_index heldout_cam2_index <<< "$heldout_reference_pair"
    heldout_cam1_name="${camera_names[$heldout_cam1_index]}"
    heldout_cam2_name="${camera_names[$heldout_cam2_index]}"

    heldout_track_log_1="./track_logs/${heldout_cam1_name}_heldout_${heldout_partition}.track.log"
    ( track_and_decode "$heldout_cam1_event_filepath" "${heldout_cam1_name}_heldout_${heldout_partition}" ) \
        > "$heldout_track_log_1" 2>&1 &
    heldout_track_pid_1=$!
    echo "  [held-out tracking]     $heldout_cam1_name -> PID $heldout_track_pid_1 (log: $heldout_track_log_1)"

    heldout_track_log_2="./track_logs/${heldout_cam2_name}_heldout_${heldout_partition}.track.log"
    ( track_and_decode "$heldout_cam2_event_filepath" "${heldout_cam2_name}_heldout_${heldout_partition}" ) \
        > "$heldout_track_log_2" 2>&1 &
    heldout_track_pid_2=$!
    echo "  [held-out tracking]     $heldout_cam2_name -> PID $heldout_track_pid_2 (log: $heldout_track_log_2)"
fi

#-----------------------------------------------------------------------#
# PHASE 2: per camera - wait for INTRINSICS tracking, then calibrate
# intrinsics (also parallel across cameras: each camera's intrinsic
# calibration only depends on that SAME camera's own tracking job)
#-----------------------------------------------------------------------#
echo "
=======================================================
WAITING FOR INTRINSICS TRACKING + CALIBRATING INTRINSICS (per camera, in parallel)
======================================================="
declare -a intrinsic_calib_pid
declare -a intrinsic_calib_log
for ((camera_index = 0; camera_index < number_camera; camera_index++)); do
    camera_name="${camera_names[$camera_index]}"

    wait_or_fail "${intrinsic_track_pid[$camera_index]}" "$camera_name intrinsics tracking" "${intrinsic_track_log[$camera_index]}"
    load_track_and_decode_result "${camera_name}_intrinsics"
    intrinsics_h5_file="$TRACK_AND_DECODE_H5"
    intrinsics_track_log_dir="$TRACK_AND_DECODE_DIR"
    intrinsics_tracker_output_file="$TRACK_AND_DECODE_TRACKER_OUTPUT"

    output_json="$intrinsics_track_log_dir/${intrinsics_tracker_output_file}_intrinsics.json"
    generated_intrinsics_files[$camera_index]="$output_json"

    intrinsic_calib_log[$camera_index]="./track_logs/${camera_name}_intrinsics.calib.log"
    (
        # spacing-x and spacing-y are the physical spacing between LED lights from the LED BOX in meters
        python intrinsic/intrinsic_calibration.py \
            "$intrinsics_h5_file" \
            --columns "$led_columns" \
            --rows "$led_rows" \
            --spacing-x "$led_spacing_x" \
            --spacing-y "$led_spacing_y" \
            --hz 10 \
            --time-source auto \
            --min-points 12 \
            --output "$output_json"
    ) > "${intrinsic_calib_log[$camera_index]}" 2>&1 &
    intrinsic_calib_pid[$camera_index]=$!
    echo "  [intrinsic calibration] $camera_name -> PID ${intrinsic_calib_pid[$camera_index]}"
done

for ((camera_index = 0; camera_index < number_camera; camera_index++)); do
    camera_name="${camera_names[$camera_index]}"
    wait_or_fail "${intrinsic_calib_pid[$camera_index]}" "$camera_name intrinsic calibration" "${intrinsic_calib_log[$camera_index]}"
    echo "Completed intrinsic calibration for $camera_name -> ${generated_intrinsics_files[$camera_index]}"
done

#-----------------------------------------------------------------------#
# PHASE 3: per camera - wait for STEREO tracking to collect each camera's H5
#-----------------------------------------------------------------------#
echo "
=======================================================
WAITING FOR STEREO TRACKING (per camera)
======================================================="
for ((camera_index = 0; camera_index < number_camera; camera_index++)); do
    camera_name="${camera_names[$camera_index]}"
    wait_or_fail "${stereo_track_pid[$camera_index]}" "$camera_name stereo tracking" "${stereo_track_log[$camera_index]}"
    load_track_and_decode_result "${camera_name}_stereo"
    generated_h5_files[$camera_index]="$TRACK_AND_DECODE_H5"
    echo "Completed stereo tracking for $camera_name -> ${generated_h5_files[$camera_index]}"
done

echo "
=======================================================
Completed calibration for all $number_camera camera(s).
======================================================="

#=============================================================================#
# STEREO CALIBRATION (parallel across pairs)
#=============================================================================#
if [[ "$run_stereo_calibration" == true ]]; then
    echo "
=======================================================
Running stereo calibration for ${#resolved_stereo_pairs[@]} camera pair(s) IN PARALLEL:
$(for pair in "${resolved_stereo_pairs[@]}"; do
    IFS=',' read -r i j <<< "$pair"
    echo "  - ${camera_names[$i]} + ${camera_names[$j]}"
done)
======================================================="

    declare -a stereo_pair_pid
    declare -a stereo_pair_log
    declare -a stereo_pair_label
    pair_counter=0
    for pair in "${resolved_stereo_pairs[@]}"; do
        IFS=',' read -r stereo_cam1_index stereo_cam2_index <<< "$pair"
        cam1_name="${camera_names[$stereo_cam1_index]}"
        cam2_name="${camera_names[$stereo_cam2_index]}"
        cam1_h5="${generated_h5_files[$stereo_cam1_index]}"
        cam2_h5="${generated_h5_files[$stereo_cam2_index]}"
        cam1_intrinsics="${generated_intrinsics_files[$stereo_cam1_index]}"
        cam2_intrinsics="${generated_intrinsics_files[$stereo_cam2_index]}"

        time_sync_json="$stereo_output_dir/time_sync_${cam1_name}_${cam2_name}.json"
        stereo_points_npz="$stereo_output_dir/stereo_points_${cam1_name}_${cam2_name}.npz"
        stereo_calibration_json="$stereo_output_dir/stereo_calibration_${cam1_name}_${cam2_name}.json"
        # Each pair gets its own evaluation subdirectory - with more than one
        # pair, a shared fixed directory would let the second pair's
        # evaluation silently overwrite the first pair's report.
        pair_evaluation_output_dir="$evaluation_output_dir/${cam1_name}_${cam2_name}"

        mkdir -p "$stereo_output_dir"
        mkdir -p "$pair_evaluation_output_dir"

        stereo_pair_label[$pair_counter]="${cam1_name}_${cam2_name}"
        stereo_pair_log[$pair_counter]="$stereo_output_dir/${cam1_name}_${cam2_name}.stereo_calib.log"

        (
            set -e
            echo "--- $cam1_name + $cam2_name: estimating camera clocks ---"
            python stereo/time_sync.py \
                "$cam1_h5" \
                "$cam2_h5" \
                --threshold-ms "$stereo_threshold_ms" \
                --iterations "$stereo_ransac_iterations" \
                --min-inliers "$stereo_min_inliers" \
                --seed "$stereo_random_seed" \
                --output "$time_sync_json"

            echo "--- $cam1_name + $cam2_name: building stereo points ---"
            stereo_build_points_args=(
                "$cam1_h5"
                "$cam2_h5"
                --hz "$stereo_hz"
                --max-gap "$stereo_max_gap"
                --threshold-ms "$stereo_threshold_ms"
                --iterations "$stereo_ransac_iterations"
                --min-inliers "$stereo_min_inliers"
                --seed "$stereo_random_seed"
                --min-points "$stereo_min_points"
                --output "$stereo_points_npz"
            )
            # Optional: reject candidate stereo views where a beacon moved faster
            # than this between the two tracker samples bracketing it (see
            # build_stereo_points.py's --max-velocity help). Left unset
            # (stereo_max_velocity_px_per_s="") by default - this reproduces the
            # exact original behavior.
            if [[ -n "$stereo_max_velocity_px_per_s" ]]; then
                stereo_build_points_args+=(--max-velocity "$stereo_max_velocity_px_per_s")
            fi
            python stereo/build_stereo_points.py "${stereo_build_points_args[@]}"

            echo "--- $cam1_name + $cam2_name: calibrating stereo pair ---"
            python stereo/stereo_calibrate.py \
                "$stereo_points_npz" \
                "$cam1_intrinsics" \
                "$cam2_intrinsics" \
                --columns "$led_columns" \
                --rows "$led_rows" \
                --spacing-x "$led_spacing_x" \
                --spacing-y "$led_spacing_y" \
                --h5-id-for-led1 "$led_h5_id_for_led1" \
                --output "$stereo_calibration_json"

            if [[ "$run_quality_evaluation" == true ]]; then
                echo "--- $cam1_name + $cam2_name: evaluating calibration quality ---"
                evaluation_args=(
                    "$stereo_points_npz"
                    "$stereo_calibration_json"
                    "$cam1_intrinsics"
                    "$cam2_intrinsics"
                    --time-sync-json "$time_sync_json"
                    --output-dir "$pair_evaluation_output_dir"
                    --partition "$evaluation_partition"
                    --run-id "${cam1_name}_${cam2_name}"
                    --geometry-unit "$evaluation_geometry_unit"
                    --max-reprojection-rms "$evaluation_max_reprojection_rms_px"
                    --max-sampson-rms "$evaluation_max_sampson_rms_px"
                    --max-relative-length-rmse "$evaluation_max_relative_length_rmse_percent"
                    --min-angle-p5 "$evaluation_min_triangulation_angle_p5_deg"
                    --repeatability-runs "$evaluation_repeatability_runs"
                )
                if [[ "$evaluation_coverage_complete" == true ]]; then
                    evaluation_args+=(--coverage-complete)
                fi
                # -u disables Python output buffering so the scorecard is
                # captured promptly in this pair's own log/tee file.
                python -u evaluation/calibration_quality.py "${evaluation_args[@]}" 2>&1 \
                    | tee "$pair_evaluation_output_dir/terminal_scorecard.txt"
            fi

            echo "Stereo calibration:   $stereo_calibration_json"
            echo "Evaluation report:    $pair_evaluation_output_dir/evaluation_report.json"
        ) > "${stereo_pair_log[$pair_counter]}" 2>&1 &
        stereo_pair_pid[$pair_counter]=$!
        echo "  [stereo calibration] $cam1_name + $cam2_name -> PID ${stereo_pair_pid[$pair_counter]} (log: ${stereo_pair_log[$pair_counter]})"
        pair_counter=$((pair_counter + 1))
    done

    for ((i = 0; i < pair_counter; i++)); do
        wait_or_fail "${stereo_pair_pid[$i]}" "stereo calibration (${stereo_pair_label[$i]})" "${stereo_pair_log[$i]}"
        echo "Completed stereo calibration: ${stereo_pair_label[$i]} (log: ${stereo_pair_log[$i]})"
    done

    echo "
=======================================================
Completed stereo calibration for all ${#resolved_stereo_pairs[@]} pair(s).
======================================================="

    #=========================================================================#
    # GLOBAL FRAME CALIBRATION + HELD-OUT EVALUATION
    # Run in parallel with EACH OTHER: both only need their own tracking
    # (already launched in PHASE 1 above) plus the reference pair's stereo
    # calibration (already completed just above) - neither depends on the
    # other.
    #=========================================================================#
    global_frame_pid=""
    global_frame_log=""
    if [[ "$run_global_frame_calibration" == true ]]; then
        global_cam1_intrinsics="${generated_intrinsics_files[$global_cam1_index]}"
        global_cam2_intrinsics="${generated_intrinsics_files[$global_cam2_index]}"
        # Must match the naming used when this pair's stereo_calibrate.py call
        # was built, above.
        reference_stereo_calibration_json="$stereo_output_dir/stereo_calibration_${global_cam1_name}_${global_cam2_name}.json"

        if [[ ! -f "$reference_stereo_calibration_json" ]]; then
            echo "Error: expected stereo calibration output not found: $reference_stereo_calibration_json" >&2
            echo "global_frame_reference_pair must name a pair this run actually calibrated above." >&2
            exit 1
        fi

        mkdir -p "$global_frame_output_dir"
        global_frame_log="$global_frame_output_dir/global_frame.log"

        # These waits must happen HERE, in the main shell - not inside the
        # backgrounded subshell below - because bash's `wait` only works on
        # a shell's own direct children, and global_frame_track_pid_1/2 were
        # forked by this main shell back in PHASE 1, not by that subshell.
        # By this point in the script (after intrinsic + stereo calibration
        # have already taken real wall-clock time), these tracking jobs
        # have very likely already finished, so these waits are cheap in
        # practice - only the actual computation below needs to be
        # backgrounded to run in parallel with held-out evaluation.
        wait_or_fail "$global_frame_track_pid_1" "$global_cam1_name global-frame tracking" "$global_frame_track_log_1"
        wait_or_fail "$global_frame_track_pid_2" "$global_cam2_name global-frame tracking" "$global_frame_track_log_2"
        load_track_and_decode_result "${global_cam1_name}_global_frame"
        global_frame_cam1_h5="$TRACK_AND_DECODE_H5"
        load_track_and_decode_result "${global_cam2_name}_global_frame"
        global_frame_cam2_h5="$TRACK_AND_DECODE_H5"

        (
            set -e
            global_frame_time_sync_json="$global_frame_output_dir/time_sync_${global_cam1_name}_${global_cam2_name}.json"
            global_frame_points_npz="$global_frame_output_dir/stereo_points_${global_cam1_name}_${global_cam2_name}.npz"
            global_frame_calibration_json="$global_frame_output_dir/global_frame_calibration_${global_cam1_name}_${global_cam2_name}.json"

            echo "--- estimating camera clocks (board on floor) ---"
            python stereo/time_sync.py \
                "$global_frame_cam1_h5" \
                "$global_frame_cam2_h5" \
                --threshold-ms "$stereo_threshold_ms" \
                --iterations "$stereo_ransac_iterations" \
                --min-inliers "$stereo_min_inliers" \
                --seed "$stereo_random_seed" \
                --output "$global_frame_time_sync_json"

            echo "--- building stereo points (board on floor) ---"
            python stereo/build_stereo_points.py \
                "$global_frame_cam1_h5" \
                "$global_frame_cam2_h5" \
                --hz "$stereo_hz" \
                --max-gap "$stereo_max_gap" \
                --threshold-ms "$stereo_threshold_ms" \
                --iterations "$stereo_ransac_iterations" \
                --min-inliers "$stereo_min_inliers" \
                --seed "$stereo_random_seed" \
                --min-points "$stereo_min_points" \
                --output "$global_frame_points_npz"

            # Note: the board here is the same physical LED board used for the
            # intrinsic/stereo waving calibration above, just held stationary on
            # the floor instead - so it reuses the same led_* geometry variables.
            echo "--- calibrating global frame ---"
            global_frame_calibrate_args=(
                "$global_frame_points_npz"
                "$global_cam1_intrinsics"
                "$global_cam2_intrinsics"
                "$reference_stereo_calibration_json"
                --spacing-x "$led_spacing_x"
                --spacing-y "$led_spacing_y"
                --columns "$led_columns"
                --rows "$led_rows"
                --h5-id-for-led1 "$led_h5_id_for_led1"
                --output "$global_frame_calibration_json"
            )
            if [[ -n "$global_frame_max_duration_s" ]]; then
                global_frame_calibrate_args+=(--max-duration-s "$global_frame_max_duration_s")
            fi
            python stereo/global_frame_calibrate.py "${global_frame_calibrate_args[@]}"

            echo "Global frame calibration: $global_frame_calibration_json"
        ) > "$global_frame_log" 2>&1 &
        global_frame_pid=$!
        echo "  [global frame calibration] -> PID $global_frame_pid (log: $global_frame_log)"
    fi

    heldout_pid=""
    heldout_log=""
    if [[ "$run_heldout_evaluation" == true ]]; then
        heldout_cam1_intrinsics="${generated_intrinsics_files[$heldout_cam1_index]}"
        heldout_cam2_intrinsics="${generated_intrinsics_files[$heldout_cam2_index]}"
        # Must match the naming used when this pair's stereo_calibrate.py call
        # was built, above - this is the FROZEN calibration; nothing in this
        # block re-fits it.
        heldout_frozen_stereo_calibration_json="$stereo_output_dir/stereo_calibration_${heldout_cam1_name}_${heldout_cam2_name}.json"

        if [[ ! -f "$heldout_frozen_stereo_calibration_json" ]]; then
            echo "Error: expected stereo calibration output not found: $heldout_frozen_stereo_calibration_json" >&2
            echo "heldout_reference_pair must name a pair this run actually calibrated above." >&2
            exit 1
        fi

        mkdir -p "$heldout_output_dir"
        heldout_log="$heldout_output_dir/heldout.log"

        # Same reasoning as the global-frame block above: these waits must
        # happen in the main shell, not inside the backgrounded subshell
        # below, since heldout_track_pid_1/2 are this main shell's own
        # direct children (forked back in PHASE 1), not the subshell's.
        wait_or_fail "$heldout_track_pid_1" "$heldout_cam1_name held-out tracking" "$heldout_track_log_1"
        wait_or_fail "$heldout_track_pid_2" "$heldout_cam2_name held-out tracking" "$heldout_track_log_2"
        load_track_and_decode_result "${heldout_cam1_name}_heldout_${heldout_partition}"
        heldout_cam1_h5="$TRACK_AND_DECODE_H5"
        load_track_and_decode_result "${heldout_cam2_name}_heldout_${heldout_partition}"
        heldout_cam2_h5="$TRACK_AND_DECODE_H5"

        (
            set -e
            heldout_time_sync_json="$heldout_output_dir/time_sync_${heldout_cam1_name}_${heldout_cam2_name}.json"
            heldout_points_npz="$heldout_output_dir/stereo_points_${heldout_cam1_name}_${heldout_cam2_name}.npz"
            heldout_evaluation_output_dir="$heldout_output_dir/evaluation"

            echo "--- estimating camera clocks (held-out) ---"
            python stereo/time_sync.py \
                "$heldout_cam1_h5" \
                "$heldout_cam2_h5" \
                --threshold-ms "$stereo_threshold_ms" \
                --iterations "$stereo_ransac_iterations" \
                --min-inliers "$stereo_min_inliers" \
                --seed "$stereo_random_seed" \
                --output "$heldout_time_sync_json"

            echo "--- building stereo points (held-out) ---"
            heldout_build_points_args=(
                "$heldout_cam1_h5"
                "$heldout_cam2_h5"
                --hz "$stereo_hz"
                --max-gap "$stereo_max_gap"
                --threshold-ms "$stereo_threshold_ms"
                --iterations "$stereo_ransac_iterations"
                --min-inliers "$stereo_min_inliers"
                --seed "$stereo_random_seed"
                --min-points "$stereo_min_points"
                --output "$heldout_points_npz"
            )
            if [[ -n "$stereo_max_velocity_px_per_s" ]]; then
                heldout_build_points_args+=(--max-velocity "$stereo_max_velocity_px_per_s")
            fi
            python stereo/build_stereo_points.py "${heldout_build_points_args[@]}"

            # Note: cam1/cam2_intrinsics and the stereo_calibration_json above are
            # the pair's ALREADY-FITTED files - nothing here re-fits them. Only
            # heldout_points_npz (from the new, disjoint recording) is new.
            echo "--- evaluating on held-out ${heldout_partition} data ---"
            mkdir -p "$heldout_evaluation_output_dir"
            heldout_evaluation_args=(
                "$heldout_points_npz"
                "$heldout_frozen_stereo_calibration_json"
                "$heldout_cam1_intrinsics"
                "$heldout_cam2_intrinsics"
                --time-sync-json "$heldout_time_sync_json"
                --output-dir "$heldout_evaluation_output_dir"
                --partition "$heldout_partition"
                --heldout-data-confirmed
                --run-id "${heldout_cam1_name}_${heldout_cam2_name}_${heldout_partition}"
                --geometry-unit "$evaluation_geometry_unit"
                --max-reprojection-rms "$evaluation_max_reprojection_rms_px"
                --max-sampson-rms "$evaluation_max_sampson_rms_px"
                --max-relative-length-rmse "$evaluation_max_relative_length_rmse_percent"
                --min-angle-p5 "$evaluation_min_triangulation_angle_p5_deg"
                --repeatability-runs "$evaluation_repeatability_runs"
            )
            if [[ "$heldout_coverage_complete" == true ]]; then
                heldout_evaluation_args+=(--coverage-complete)
            fi
            python -u evaluation/calibration_quality.py "${heldout_evaluation_args[@]}" 2>&1 \
                | tee "$heldout_evaluation_output_dir/terminal_scorecard.txt"

            echo "Held-out evaluation report: $heldout_evaluation_output_dir/evaluation_report.json"
        ) > "$heldout_log" 2>&1 &
        heldout_pid=$!
        echo "  [held-out evaluation] -> PID $heldout_pid (log: $heldout_log)"
    fi

    if [[ -n "$global_frame_pid" ]]; then
        wait_or_fail "$global_frame_pid" "global frame calibration" "$global_frame_log"
        echo "
=======================================================
Completed global frame calibration.
Reference frame: Camera 1 of $global_cam1_name + $global_cam2_name
Log: $global_frame_log
======================================================="
    fi
    if [[ -n "$heldout_pid" ]]; then
        wait_or_fail "$heldout_pid" "held-out evaluation" "$heldout_log"
        echo "
=======================================================
Completed held-out $heldout_partition evaluation: $heldout_cam1_name + $heldout_cam2_name
Log: $heldout_log
======================================================="
    fi
else
    echo "Stereo calibration disabled (run_stereo_calibration=false)."
fi
