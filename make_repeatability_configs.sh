#!/bin/bash
# Generates one full calibration_config_session_N.sh per line of a sessions
# manifest, for use with run_repeatability.sh.
#
# WHY THIS EXISTS: calibration_config.sh only ever encodes ONE physical
# session's data paths (event_filepaths, intrinsic_event_filepaths,
# global_frame_cam1/2_event_filepath are single fixed values) - there's no
# built-in convention for organizing multiple sessions' raw recordings, and
# run_repeatability.sh needs a separate, complete config per session. This
# script doesn't invent a directory convention for you either: you tell it,
# per session, exactly which files to use, via a manifest.
#
# HOW IT WORKS: copies your base config verbatim, then appends
# intrinsic_event_filepaths=(...), event_filepaths=(...) (and, if needed,
# global_frame_cam1/2_event_filepath) overrides at the END of the copy.
# Bash executes a sourced file top to bottom, so a later assignment simply
# replaces the base config's original value - everything else in the file
# (led geometry, thresholds, stereo pairs, etc.) stays identical across all
# session configs, which is what you want: repeatability is about the RIG's
# stability, not different settings.
#
# MANIFEST FORMAT (one line per session, comma-separated, no spaces after
# commas, '#' for comments, blank lines ignored). Fields are grouped by
# PURPOSE, not by camera - all intrinsics paths first, then all stereo
# paths, matching calibration_config.sh's own intrinsic_event_filepaths /
# event_filepaths split:
#   intrinsic_event_filepath_cam1,intrinsic_event_filepath_cam2[,...one per camera],event_filepath_cam1,event_filepath_cam2[,...one per camera]
# If the base config has run_global_frame_calibration=true, append exactly
# two more fields per line (every session needs its own global-frame
# recording too):
#   ...,global_frame_cam1_path,global_frame_cam2_path
#
# BREAKING CHANGE from the pre-intrinsic/stereo-split format: manifests
# written for the old (stereo-only) event_filepaths convention are NOT
# compatible with this version and will be silently misinterpreted rather
# than rejected outright for a 2-camera rig, since both the old format's 4
# fields (2 stereo + 2 global) and this version's first 4 fields (2
# intrinsics + 2 stereo) are valid-looking 4-field lines with different
# meanings. Regenerate any existing manifest in the new format - don't
# reuse an old one as-is.
#
# Usage:
#   ./make_repeatability_configs.sh calibration_config.sh sessions_manifest.txt ./repeatability_configs
#   ./run_repeatability.sh ./repeatability_configs/calibration_config_session_1.sh \
#                          ./repeatability_configs/calibration_config_session_2.sh \
#                          ./repeatability_configs/calibration_config_session_3.sh
set -e
set -o pipefail

base_config="$1"
manifest="$2"
output_dir="${3:-./repeatability_configs}"

if [[ -z "$base_config" || -z "$manifest" ]]; then
    echo "Usage: $0 <base_calibration_config.sh> <sessions_manifest.txt> [output_dir]" >&2
    exit 1
fi
if [[ ! -f "$base_config" ]]; then
    echo "Error: base config not found: $base_config" >&2
    exit 1
fi
if [[ ! -f "$manifest" ]]; then
    echo "Error: manifest not found: $manifest" >&2
    exit 1
fi

# Read number_camera and run_global_frame_calibration from the base config
# in an isolated subshell, so nothing it sets leaks into this script.
config_summary="$(
    bash -c '
        source "'"$base_config"'"
        echo "number_camera=$number_camera"
        echo "needs_global_frame=$run_global_frame_calibration"
    '
)"
eval "$config_summary"

if [[ -z "$number_camera" ]]; then
    echo "Error: could not read number_camera from $base_config" >&2
    exit 1
fi

mkdir -p "$output_dir"

session_number=0
manifest_line_number=0
while IFS= read -r line || [[ -n "$line" ]]; do
    manifest_line_number=$((manifest_line_number + 1))

    # Skip blank lines and comments.
    [[ -z "$line" || "$line" == \#* ]] && continue

    session_number=$((session_number + 1))
    IFS=',' read -ra fields <<< "$line"

    required_fields=$((2 * number_camera))
    if (( ${#fields[@]} < required_fields )); then
        echo "Error: $manifest line $manifest_line_number (session $session_number) has ${#fields[@]} field(s), need at least $required_fields ($number_camera intrinsic_event_filepaths + $number_camera event_filepaths, for this base config's number_camera)." >&2
        exit 1
    fi

    if [[ "$needs_global_frame" == true && ${#fields[@]} -lt $((required_fields + 2)) ]]; then
        echo "Error: $manifest line $manifest_line_number (session $session_number) is missing the global_frame_cam1/cam2 event filepaths." >&2
        echo "Base config has run_global_frame_calibration=true, so every session needs its own floor-board recording too - append 2 more comma-separated paths to this line." >&2
        exit 1
    fi

    session_config="$output_dir/calibration_config_session_${session_number}.sh"
    cp "$base_config" "$session_config"

    # Read heldout_partition from the base config too (needed to reconstruct
    # heldout_output_dir's exact formula below) - same isolated-subshell
    # pattern as number_camera/needs_global_frame above, so nothing leaks.
    session_heldout_partition="$(
        bash -c '
            source "'"$base_config"'"
            echo "$heldout_partition"
        '
    )"

    {
        echo ""
        echo "#=============================================================================#"
        echo "# REPEATABILITY OVERRIDE (session $session_number)"
        echo "# Appended by make_repeatability_configs.sh - everything above this point is"
        echo "# an unmodified copy of $base_config; only these paths differ per session."
        echo "# Source: $manifest, line $manifest_line_number - check that line if two"
        echo "# sessions ever look suspiciously identical below."
        echo "#=============================================================================#"
        echo "intrinsic_event_filepaths=("
        for ((i = 0; i < number_camera; i++)); do
            echo "    \"${fields[$i]}\""
        done
        echo ")"
        echo "event_filepaths=("
        for ((i = 0; i < number_camera; i++)); do
            echo "    \"${fields[$((number_camera + i))]}\""
        done
        echo ")"

        if [[ "$needs_global_frame" == true ]]; then
            echo "global_frame_cam1_event_filepath=\"${fields[$((required_fields))]}\""
            echo "global_frame_cam2_event_filepath=\"${fields[$((required_fields + 1))]}\""
        fi

        # BUG FIX: without this block, every session config below still
        # points at the BASE config's shared stereo_output_dir (and the
        # three paths derived from it) - so session 2's run silently
        # overwrites session 1's stereo_calibration.json/evaluation_report.json/
        # terminal_scorecard.txt etc. before you ever get to compare them,
        # and by the time session 3 finishes only its own output survives
        # on disk. Bash string-interpolates "$stereo_output_dir/evaluation"
        # etc. at the moment those lines run inside the COPIED base config
        # above - overriding stereo_output_dir alone, later, does NOT
        # retroactively change evaluation_output_dir/global_frame_output_dir/
        # heldout_output_dir, since they already captured the OLD value.
        # All four must be re-derived here, in this same form, with a
        # session-unique root.
        echo "stereo_output_dir=\"./track_logs/repeatability/session_${session_number}/stereo\""
        echo "evaluation_output_dir=\"\$stereo_output_dir/evaluation\""
        echo "global_frame_output_dir=\"\$stereo_output_dir/global_frame\""
        echo "heldout_output_dir=\"\$stereo_output_dir/heldout_${session_heldout_partition}\""
    } >> "$session_config"

    echo "Wrote $session_config"
done < "$manifest"

if (( session_number == 0 )); then
    echo "Error: manifest had no session lines (all blank/comments?): $manifest" >&2
    exit 1
fi
if (( session_number < 3 )); then
    echo "Warning: only $session_number session config(s) generated; calibration_quality.py's repeatability gate needs >= 3." >&2
fi

echo "
Generated $session_number session config(s) in $output_dir.
Next:
  ./run_repeatability.sh $(for ((n = 1; n <= session_number; n++)); do printf '%s ' "$output_dir/calibration_config_session_${n}.sh"; done)"
