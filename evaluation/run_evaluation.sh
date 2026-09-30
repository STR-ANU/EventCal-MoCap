#!/usr/bin/env bash
# Works from any working directory. Arguments are paths relative to caller.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${PYTHON:-python3}" -u "$script_dir/calibration_quality.py" "$@"
