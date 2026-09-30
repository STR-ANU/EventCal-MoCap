# EventCameraCalibration

Intrinsic + stereo calibration for event cameras, using a blinking LED board
as the calibration target. Tracks the board with AEMOT, decodes each LED's
beacon ID, fits per-camera intrinsics and stereo extrinsics, and evaluates
the result against a set of accuracy and process-evidence gates before
you'd trust it for 3D measurements.

The whole pipeline is driven by one config file and one script:

```bash
./run_calibration.sh calibration_config.sh
```

Everything below explains what that one command actually does, and how to
configure it.

---

## Contents

- [Setup](#setup)
- [Data collection](#data-collection)
- [Repository layout](#repository-layout)
- [Quick start](#quick-start)
- [Pipeline stages](#pipeline-stages)
- [Configuration reference](#configuration-reference)
- [Reading the calibration quality report](#reading-the-calibration-quality-report)
- [Repeatability checking](#repeatability-checking)
- [Troubleshooting](#troubleshooting)
- [Known limitations](#known-limitations)

---

## Setup

### C++ dependencies

```bash
sudo apt install cmake build-essential pkg-config \
    libboost-dev libopencv-dev libyaml-cpp-dev libeigen3-dev \
    libusb-1.0-0-dev event_stream
```

`libusb` is only needed to connect to a live camera - not required if you're
only ever calibrating from recorded `.es` files.

Build the tracker and decoder once:

```bash
cmake -S AEMOT-realtime -B AEMOT-realtime/build -DCMAKE_BUILD_TYPE=Release
cmake --build AEMOT-realtime/build --parallel

cmake -S event_decoder -B event_decoder/build -DCMAKE_BUILD_TYPE=Release
cmake --build event_decoder/build --parallel
```

(`run_calibration.sh` also does this automatically on first run if the
build directories don't exist yet - see [Tracking + decoding](#1-tracking-decoding).)

### Python dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy h5py faery
```

The evaluation suite (`evaluation/calibration_quality.py` and friends) has
its own, separate setup - see [`evaluation/README.md`](evaluation/README.md)
(Python 3.10+, headless OpenCV). Its dependencies are listed in
[`evaluation/requirements.txt`](evaluation/requirements.txt); use a separate
environment for it, as described there. Don't mix the two virtual
environments, and don't distribute either `.venv` - both are gitignored.

### Known build gotcha

Building `gen4` (the recording app, see below) can fail with a missing
`<memory>` include. If so, add it to `gen4/common/usb.hpp`:

```cpp
#include <memory>
```

---

## Data collection

Recording is done with a separate app, `gen4`: https://github.com/neuromorphicsystems/gen4

Build it, then run `./gen4` - it has a record button that writes a `.es`
file to `./recordings/`. You need one such recording **per camera, per
pipeline stage**:

- One waved-board recording per camera for stereo fitting
  (`event_filepaths` in the config) - the board visible to ALL cameras in a
  stereo pair simultaneously throughout.
- One *separate* waved-board recording per camera for intrinsic
  calibration (`intrinsic_event_filepaths` in the config) - only needs the
  board filling that one camera's own frame; does not need to be visible to
  any other camera at the same time.
- If using global frame calibration: one stationary board-on-floor recording
  per camera (`global_frame_cam1/2_event_filepath`).
- If doing a held-out dev/test evaluation: one *new*, differently-waved
  recording per camera (`heldout_cam1/2_event_filepath`) - physically
  disjoint from the fitting recording, not a rerun of it.
- If checking repeatability: the whole waved-board recording, repeated for
  each independent physical session (ideally unmounting/remounting the rig
  between sessions).

**If you're recording two cameras on two separate laptops**, get both
recordings actively running before you start the real calibration motion,
and trim any long lead-in on whichever camera started first - see
[Troubleshooting](#the-two-cameras-start-recording-minutes-apart) for why
this matters and what the tool can and can't detect about it.

---

## Repository layout

```
run_calibration.sh          Main pipeline entry point
calibration_config.sh          Your machine-specific config (gitignored)

intrinsic/
  intrinsic_calibration.py     Per-camera intrinsics from tracked+decoded LED positions

stereo/
  time_sync.py                 Cross-camera clock offset, from matching decoder counters
  build_stereo_points.py       Synchronized (cam1, cam2) LED position pairs -> .npz
  stereo_calibrate.py          cv2.stereoCalibrate() -> R, T, baseline
  global_frame_calibrate.py    Anchors a world coordinate frame to a stationary board

evaluation/
  calibration_quality.py       M1-M6 accuracy gates + process-evidence gates -> PASS/FAIL
  README.md                    Evaluation suite's own setup + metric reference

utils/
  filter_decoded_files.py      Merges tracker state + decoded beacon ID into one .h5
  perform_calibration.py       Example of reading the .h5 output directly

make_repeatability_configs.sh  Generates one config per repeatability session from a manifest
run_repeatability.sh           Runs the pipeline once per session, compares the results
run_repeatability_pipeline.sh  Single-command wrapper for the two scripts above

AEMOT-realtime/                Event-camera object tracker (submodule/vendored)
event_decoder/                 Beacon-ID decoder (submodule/vendored)
gen4/                          Recording app (submodule/vendored)

track_logs/                    All generated output lands here (gitignored)
```

---

## Quick start

1. Record your calibration data (see [Data collection](#data-collection)).
2. Create your local config from the tracked template, then edit its paths:
  ```bash
  cp calibration_config.example.sh calibration_config.sh
  ```
  `calibration_config.sh` is gitignored so machine-specific paths stay local.
3. At minimum, set:
   - `number_camera`, `camera_names`, `event_filepaths`, `intrinsic_event_filepaths`
   - `led_columns`, `led_rows`, `led_spacing_x`, `led_spacing_y` (measure the
     physical board with calipers - don't guess)
4. Run it:
   ```bash
   ./run_calibration.sh calibration_config.sh
   ```
5. Read the printed scorecard, or open
   `track_logs/stereo/evaluation/<cam1>_<cam2>/terminal_scorecard.txt`
   afterward. See
   [Reading the calibration quality report](#reading-the-calibration-quality-report).

Everything else in the config (global frame, held-out evaluation,
repeatability) is optional and off by default - turn each on only once
you're ready to use it (see [Configuration reference](#configuration-reference)).

---

## Pipeline stages

`run_calibration.sh` runs these in order. Every stage after the first is
gated behind a `run_*=true/false` flag in the config, so you can enable them
incrementally rather than needing everything on day one.

### 1. Tracking + decoding

For each camera's recording - and now run **twice per camera**, once for
`intrinsic_event_filepaths` and once for `event_filepaths`, since these are
two separate recordings (see [Configuration reference](#configuration-reference)
above):

1. **AEMOT tracker** (`AEMOT-realtime/build/aemot_replay`) tracks the LED
   board's blinking points across the recording, producing a `.bees` file
  (validated track states) and a combined `.aemotevt` event stream. The
  `.bees` file contains validated tracks; `.aemotevt` can also contain
  candidate tracks that have no corresponding state estimates.
2. **`track_logs/aemotevt_to_es_faery.py`** splits the combined event stream
  into one `.es` file per track ID in `track_logs/temp/`.
3. **event_decoder** (`event_decoder/build/decoder_replay`) reads each
  track's blink pattern and writes decoded rows containing the event
  timestamp, beacon ID, and lower and upper counter bytes. A correctly
  decoded track should keep a stable beacon ID and have an incrementing
  combined counter; gaps can indicate dropped messages.
4. **`utils/filter_decoded_files.py`** matches decoded beacon IDs back to
  tracker states. A beacon can get a new track ID if it leaves and
  re-enters the frame, so the output is grouped by decoded beacon ID rather
  than track ID. Each camera's `.h5` file contains `State` and `Decoded`
  data for each beacon.

This is the most compute-heavy stage - it processes the entire raw
recording, regardless of what happens downstream, and now does so twice per
camera given the two separate recordings.

### 2. Intrinsic calibration

`intrinsic/intrinsic_calibration.py` samples each camera's tracked LED
positions (from the `intrinsic_event_filepaths` recording specifically, not
the stereo one) at a fixed rate, interpolates positions between tracker
samples, and fits per-camera intrinsics (`camera_matrix`,
`distortion_coefficients`) via `cv2.calibrateCamera()`. Runs once per
camera.

### 3. Stereo calibration

For each configured camera pair:

1. **`stereo/time_sync.py`** finds the clock offset between the two
   cameras, by matching decoder counter values for beacons visible in both
   (a RANSAC fit of `t_cam2 = a * t_cam1 + b`).
2. **`stereo/build_stereo_points.py`** samples both cameras at synchronized
   instants, keeping only views where enough beacons are jointly visible in
   both, and saves the result as `stereo_points_*.npz`.
3. **`stereo/stereo_calibrate.py`** runs `cv2.stereoCalibrate()` with
   `CALIB_FIX_INTRINSIC` (the per-camera intrinsics from step 2 are held
   fixed, not re-fit) to solve for `R`, `T`, and baseline (`||T||`).
4. **`evaluation/calibration_quality.py`** scores the fit - see
   [Reading the calibration quality report](#reading-the-calibration-quality-report).

### 4. Global frame calibration (optional)

Stereo calibration gives you the relative pose between two cameras, with
one camera's frame as the arbitrary origin - not a real-world coordinate
system. To attach a meaningful world frame:

1. Place the calibration board flat and **stationary** on the floor, record
   it with both cameras (`global_frame_cam1/2_event_filepath`).
2. `stereo/global_frame_calibrate.py` triangulates the board's known LED
   geometry through the already-fitted stereo pair, and solves the rigid
   transform from that pair's Camera 1 into the board's own frame - origin
   at LED 0, +X/+Y along the grid, +Z the board normal (so "up" is only
   correct if the board is genuinely flat, LEDs facing up).

Since the board is stationary, only a couple of seconds of data is really
needed for this fit - `global_frame_max_duration_s` (default `2.0`) trims
the *already-decoded* views down to that window before fitting. This does
**not** reduce tracking/decoding time on the raw recording - that already
processed the whole file. If wall-clock time matters, trim or re-record the
raw `.es` file itself to a couple of seconds instead.

### 5. Held-out dev/test evaluation (optional)

`evaluation_partition=calibration_fit` evaluates the calibration against
the *same* recording used to fit it - which can only ever tell you the fit
is self-consistent, never that it's accurate on new data. `PASS` status is
structurally impossible on this partition; the evaluator's own gating logic
won't allow it.

To actually earn `PASS`:

1. Record a **new**, physically disjoint clip - the board moved through
   different poses than the fitting recording, same rig/mount.
2. Set `run_heldout_evaluation=true` and the `heldout_*` config variables.
3. Re-run the pipeline. It tracks/decodes the new recording, builds fresh
   stereo points from it, then evaluates against the stereo pair's
   **already-fitted, frozen** calibration - nothing is re-fit.

Use `heldout_partition="dev"` while iterating on your recording process,
and `"test"` (a third, separate recording) for the one final, unbiased
check once you're done tuning.

### 6. Repeatability checking (optional, separate from the main run)

See [Repeatability checking](#repeatability-checking) below - this is a
separate set of scripts, not a flag inside the main pipeline, since it
requires running the *entire* pipeline multiple times against different
physical recording sessions.

---

## Configuration reference

All of this lives in `calibration_config.sh`; the file itself has the full
comments. This is a summary of what each section controls.

| Section | Key variables | Purpose |
|---|---|---|
| **Camera inputs** | `number_camera`, `camera_names`, `event_filepaths`, `intrinsic_event_filepaths` | How many cameras, and the two separate recordings for each - `event_filepaths` for stereo (board visible to all cameras at once), `intrinsic_event_filepaths` for that camera's own intrinsics alone |
| **Stereo calibration** | `run_stereo_calibration`, `stereo_pair_selection`, `stereo_pairs`, `led_*` | Which camera pairs to calibrate, and the board's physical geometry |
| | `stereo_hz`, `stereo_max_gap`, `stereo_threshold_ms`, `stereo_min_points`, `stereo_max_velocity_px_per_s` | Time-sync and view-sampling tuning - see the file's own comments if you hit `"No synchronized views survived"` (usually `stereo_min_points`, not `--hz`/`--max-gap`, despite that error's wording) |
| **Quality evaluation** | `run_quality_evaluation`, `evaluation_partition`, `evaluation_max_*` | Whether to run `calibration_quality.py`, and its accuracy thresholds |
| | `evaluation_coverage_complete` | Self-declared: confirms the waved-board recording actually covered all four corners, 3+ distances, ±30-45° tilts, held stationary per pose. **Not verified from your data** - see [below](#self-declared-flags-are-not-verified) |
| | `evaluation_repeatability_runs` | Self-declared count of independently-verified repeatability sessions - see [Repeatability checking](#repeatability-checking). Leave at `1` until you actually have one |
| **Global frame** | `run_global_frame_calibration`, `global_frame_reference_pair`, `global_frame_cam1/2_event_filepath`, `global_frame_max_duration_s` | See [stage 4](#4-global-frame-calibration-optional) above |
| **Held-out evaluation** | `run_heldout_evaluation`, `heldout_partition`, `heldout_reference_pair`, `heldout_cam1/2_event_filepath`, `heldout_coverage_complete` | See [stage 5](#5-held-out-devtest-evaluation-optional) above |

### Self-declared flags are not verified

`--coverage-complete`, `--heldout-data-confirmed`, and `--repeatability-runs`
are all pure declarations `calibration_quality.py` trusts you to make
honestly - none of them are computed from your actual recording. The tool
has no way to check whether your board really reached all four corners, or
whether two recordings are really physically independent. Setting these
without having actually verified the underlying claim will produce a report
that looks more trustworthy than it is.

---

## Reading the calibration quality report

Each run prints (and saves) a scorecard like:

```
Status:    FAILED
Candidate: ACCEPTABLE_CANDIDATE

M1-M6 RESULTS
M1   PASS   Pooled reprojection RMS          0.585 px  <= 1.000 px
M3   PASS   Triangulation angle P5          46.734 deg >= 1.000 deg
M4   INFO   Square-root Sampson RMS          0.829 px  (diagnostic; pass --gate-sampson to gate on this)
M4b  INFO   Symmetric epipolar distance      1.181 px  (diagnostic; never gated)
M5   PASS   Pair-balanced relative RMSE       0.402 %  <= 2.000 %
M6   PASS   Adjacent spacing, all directions  1.116 %  <= 2.000 %
```

| ID | What it checks | Gated by default? |
|---|---|---|
| **M1** | Pooled reprojection RMS across the stereo pair | Yes |
| **M2** | Per-camera median/P95 reprojection error | Informational only |
| **M3** | Triangulation angle (P5) - shallow ray angles mean weakly-constrained depth | Yes |
| **M4** | Square-root Sampson RMS (epipolar geometry) | Only with `--gate-sampson` |
| **M4b** | Symmetric epipolar distance | Never - diagnostic only, complements M4 |
| **M5** | Pair-balanced relative length RMSE (long-diagonal LED pairs) | Yes |
| **M6** | Adjacent-LED spacing consistency - the primary distance/scale gate | Yes |

If a metric fails, its recommendation (with the specific numbers and a
concrete next step) appears in `recommendations.txt`. Failing metrics are
independent - two failing metrics produce two separate recommendations in
the same report, not one combined message.

Status can also be blocked by process-evidence gates that aren't accuracy
metrics at all:

- **Cameras not recorded simultaneously** - a filename-timestamp heuristic
  (not an inspection of the actual event data); see
  [Troubleshooting](#the-two-cameras-start-recording-minutes-apart) if this
  fires on a setup where you're confident the capture was actually
  synchronized.
- **`calibration_fit` partition** - can never reach `PASS`; requires
  [held-out evaluation](#5-held-out-devtest-evaluation-optional).
- **Coverage / repeatability not declared** - see
  [above](#self-declared-flags-are-not-verified).

---

## Repeatability checking

A single calibration run can produce excellent M1-M6 numbers while still
sitting on a rig that isn't physically stable - a loose mount, a flexing
bracket, thermal drift. None of the accuracy metrics can catch that, because
they only ever see one session's data. Repeatability checking runs the
*entire* pipeline multiple times, against independently-recorded sessions,
and compares the resulting `fx/fy/cx/cy`, `R`, `T`, and baseline.

### One-command workflow

```bash
./run_repeatability_pipeline.sh
```

With no arguments, this defaults to `calibration_config.sh`,
`sessions_manifest.txt`, and `./repeatability_configs` - matching
`run_calibration.sh`'s own default-filename convention. Pass explicit
paths if yours are named differently.

### The manifest

`sessions_manifest.txt` is one line per physically separate session,
comma-separated:

```
event_filepath_cam1,event_filepath_cam2,global_frame_cam1_path,global_frame_cam2_path
```

(the global-frame pair is only needed if `run_global_frame_calibration=true`
in your base config). `#` comments and blank lines are ignored. Every other
setting - LED geometry, thresholds, stereo pair selection - comes from your
one base `calibration_config.sh` and stays identical across all sessions,
since repeatability is about the rig's stability, not different settings.

### What happens

1. **`make_repeatability_configs.sh`** generates one full
   `calibration_config_session_N.sh` per manifest line (a copy of your base
   config with just that line's paths substituted in).
2. **`run_repeatability.sh`** runs `run_calibration.sh` on each session
   config in turn, archiving each one's `stereo_calibration_*.json` before
   the next run overwrites the shared output directory.
3. **`evaluation/check_repeatability.py`** compares `fx/fy/cx/cy` (both
   cameras), the stereo baseline, and rotation/translation across every
   pair of sessions, and writes `repeatability_report.json`/`.txt` with a
   `PASS`/`FAILED` verdict against configurable swing thresholds.

`calibration_quality.py`'s own `--repeatability-runs` gate needs `>= 3`
sessions and is **not** set automatically - once you have a genuine `PASS`
here from `>= 3` sessions, set `evaluation_repeatability_runs` in your main
config to match, by hand.

### Smoke-testing without all sessions recorded yet

```bash
./run_repeatability_pipeline.sh --repeats 1 --min-runs 1
```

`--min-runs` lets `run_repeatability.sh` proceed with fewer than 3 sessions,
purely to confirm the pipeline mechanics work before you've recorded
everything. It loudly warns that any report from a below-3 run is not a
valid repeatability conclusion, and won't suggest a `--repeatability-runs`
value. (`evaluation/check_repeatability.py` separately always requires
`>= 2` runs to compare at all - that floor is mathematical, not a setting.)

---

## Troubleshooting

### "No synchronized views survived"

Usually caused by `stereo_min_points` (default `12`), not `--hz` or
`--max-gap` despite the error's wording. It means candidate stereo views
existed but didn't have enough beacons jointly visible in *both* cameras at
once - common once there's any real occlusion (e.g. tilting the board to
cover more of the frame). Lower `stereo_min_points`, widen coverage, or
check `view_max_speed_px_per_s` in the saved `.npz` for fast-motion
rejection if you're also using `stereo_max_velocity_px_per_s`.

### A bowl-shaped reprojection-error pattern, worst at the board's edges

Usually a wrong `led_spacing_x`/`led_spacing_y` - re-measure with calipers,
don't leave it at a guessed value.

### The two cameras start recording minutes apart

`calibration_quality.py` flags this from the **timestamp embedded in each
recording's filename** - not from inspecting the actual event data for
overlap. If you're recording on two separate laptops with two operators,
some gap between when each *file* starts is unavoidable, and doesn't by
itself mean the actual board motion wasn't captured simultaneously once
both were rolling.

Two things in the same report are more informative if you're unsure:

- **Cross-camera motion correlation** (near the gap warning) - this *does*
  look at correlated motion, not just filenames.
- **M1 and M3** - a genuinely desynced pair usually drags reprojection RMS
  and triangulation angle down too, not just the standalone timestamp gap.

If those look good despite the gap, it's likely the filename heuristic
being too blunt for a multi-laptop setup rather than a real problem. Best
fix either way: minimize the actual gap between both cameras actively
recording (a count-in for both operators), and trim any long lead-in before
saving the file.

### `--coverage-complete` / `--heldout-data-confirmed` didn't change my score

They don't affect M1-M6 at all - they only control whether the
corresponding *process-evidence* recommendation appears (see
[above](#self-declared-flags-are-not-verified)). If your actual accuracy
numbers are the problem, look at the M1-M6 table instead.

---

## Known limitations

- The event-camera tracker is not currently fast enough to run real-time
  against the LED board, so calibration is done from recorded files, not
  live.
- `utils/filter_decoded_files.py`'s ID-matching logic (mode of decoded ID
  per track) is not robust to significant decoder errors.
- `--max-recording-start-gap-s`'s default (300s) is not currently exposed
  in `calibration_config.sh` - if your setup has an unavoidable, verified
  benign gap larger than that, it needs a CLI override today.
