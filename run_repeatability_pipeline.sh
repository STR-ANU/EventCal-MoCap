#!/bin/bash
# Single entry point for the whole repeatability pipeline: generates one
# calibration_config_session_N.sh per manifest line (make_repeatability_configs.sh),
# then immediately runs run_calibration.sh on each of them and compares
# the results (run_repeatability.sh) - one command instead of two.
#
# You still have to author sessions_manifest.txt yourself first (one line
# per physically separate recording session, see
# make_repeatability_configs.sh's header for the exact format) - nothing can
# automate telling this pipeline which real recording belongs to which
# session; that mapping only exists in your head/notes until you write it
# down.
#
# Assumes make_repeatability_configs.sh and run_repeatability.sh sit next to
# this script (repo root), same as run_calibration.sh's own convention.
#
# Usage:
#   ./run_repeatability_pipeline.sh [calibration_config.sh] [sessions_manifest.txt] [output_dir] [--repeats N] [--min-runs N]
#
# All three positional arguments are optional and default to
# calibration_config.sh, sessions_manifest.txt, and ./repeatability_configs
# respectively - matching run_calibration.sh's own "defaults to
# calibration_config.sh if you don't pass one" convention. So if your files
# are named exactly that, just running:
#   ./run_repeatability_pipeline.sh
# is enough. Pass explicit paths only when yours are named differently.
#
# --repeats N is optional. Without it, every active (non-blank, non-'#')
# line in the manifest is used. With it, only the FIRST N active lines are
# used (in the order they appear in the manifest) - useful for keeping extra
# sessions in the manifest for later without running them yet.
#
# --min-runs N is optional and is forwarded straight through to
# run_repeatability.sh (default 3, matching calibration_quality.py's own
# gate) - see run_repeatability.sh's header for what it does and why a
# value below 3 is NOT a valid repeatability conclusion.
set -e
set -o pipefail

usage() {
    echo "Usage: $0 [calibration_config.sh] [sessions_manifest.txt] [output_dir] [--repeats N] [--min-runs N]" >&2
    echo "All three positional arguments are optional - see this script's header for their defaults." >&2
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

repeats=""
min_runs=""
positional=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --repeats)
            repeats="$2"
            shift 2
            ;;
        --repeats=*)
            repeats="${1#--repeats=}"
            shift
            ;;
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

base_config="${positional[0]:-calibration_config.sh}"
manifest="${positional[1]:-sessions_manifest.txt}"
output_dir="${positional[2]:-./repeatability_configs}"

if [[ -z "$base_config" || -z "$manifest" ]]; then
    usage
    exit 1
fi
if [[ -n "$repeats" && ! "$repeats" =~ ^[0-9]+$ ]]; then
    echo "Error: --repeats must be a positive integer, got '$repeats'." >&2
    exit 1
fi
if [[ "$repeats" == "0" ]]; then
    echo "Error: --repeats must be at least 1." >&2
    exit 1
fi
if [[ ! -f "$script_dir/make_repeatability_configs.sh" ]]; then
    echo "Error: make_repeatability_configs.sh not found next to this script in $script_dir" >&2
    exit 1
fi
if [[ ! -f "$script_dir/run_repeatability.sh" ]]; then
    echo "Error: run_repeatability.sh not found next to this script in $script_dir" >&2
    exit 1
fi
if [[ ! -f "$manifest" ]]; then
    echo "Error: manifest not found: $manifest" >&2
    echo "Create it first - see make_repeatability_configs.sh's header comment for the exact format (one line per physically separate recording session)." >&2
    exit 1
fi

# If --repeats was given, build a temporary manifest containing only the
# first N active (non-blank, non-'#') lines of the real one, and feed THAT
# to make_repeatability_configs.sh instead. Comments/blank lines in the
# original are not counted as sessions, matching make_repeatability_configs.sh's
# own parsing.
manifest_to_use="$manifest"
temp_manifest=""
if [[ -n "$repeats" ]]; then
    active_line_count="$(grep -vc -e '^[[:space:]]*$' -e '^[[:space:]]*#' "$manifest" || true)"
    if (( active_line_count < repeats )); then
        echo "Error: --repeats $repeats requested, but $manifest only has $active_line_count active session line(s)." >&2
        echo "Add more session lines to the manifest, or lower --repeats." >&2
        exit 1
    fi

    temp_manifest="$(mktemp)"
    trap 'rm -f "$temp_manifest"' EXIT
    grep -v -e '^[[:space:]]*$' -e '^[[:space:]]*#' "$manifest" | head -n "$repeats" > "$temp_manifest"
    manifest_to_use="$temp_manifest"

    echo "--repeats $repeats: using the first $repeats of $active_line_count active session line(s) in $manifest."
fi

echo "
=======================================================
STEP 1/2: GENERATING SESSION CONFIGS
======================================================="
"$script_dir/make_repeatability_configs.sh" "$base_config" "$manifest_to_use" "$output_dir"

# Collect the configs that step just wrote, sorted NUMERICALLY by session
# number (not lexicographically - otherwise session_10 would sort before
# session_2 once you have more than 9 sessions).
shopt -s nullglob
session_configs=("$output_dir"/calibration_config_session_*.sh)
shopt -u nullglob

if (( ${#session_configs[@]} == 0 )); then
    echo "Error: no session configs were generated in $output_dir" >&2
    exit 1
fi

mapfile -t session_configs < <(
    for f in "${session_configs[@]}"; do
        n="${f##*_session_}"
        n="${n%.sh}"
        echo "$n $f"
    done | sort -n | awk '{print $2}'
)

run_repeatability_args=()
if [[ -n "$min_runs" ]]; then
    run_repeatability_args+=(--min-runs "$min_runs")
fi

echo "
=======================================================
STEP 2/2: RUNNING PIPELINE + COMPARING ${#session_configs[@]} SESSION(S)
======================================================="
"$script_dir/run_repeatability.sh" "${run_repeatability_args[@]}" "${session_configs[@]}"
