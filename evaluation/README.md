> Automatic ID availability: no LED IDs are hard-coded. Evaluation uses IDs
> available in **both** cameras after decoding/tracking. Pairs containing IDs
> absent from either camera are excluded from planned distances and missing-pair
> checks. IDs are never renumbered. Temporary per-view dropouts and invalid 3D
> reconstructions for otherwise eligible IDs remain visible in the audit.

# Event-camera stereo calibration evaluation, revision 2

Evaluate **free stereo reconstruction**, with adjacent LED distance as the
primary physical metric. Sampson distance remains available as a diagnostic;
`--gate-sampson` explicitly restores its legacy pass/fail gate. No calibration
parameters, board poses, or scales are fitted by this evaluator.

## Ubuntu quick start

From the project root, create a fresh environment (do not distribute `.venv`):

```bash
python3 -m venv .venv-evaluation
source .venv-evaluation/bin/activate
python3 -m pip install -r evaluation/requirements.txt
python3 -m unittest discover -s evaluation -v
```

Python 3.10+ is required. `python3-venv` must be installed on Ubuntu. The headless
OpenCV dependency does not require a display or Qt. If this environment already
uses `opencv-python`, use that package instead of also installing the headless
variant. Do not mix competing OpenCV distributions in the same environment.
This requirements file covers evaluation, not the C++ tracking/decoding pipeline.

### Audit the existing observations immediately

```bash
python3 -u evaluation/adjacent_led_spacing_check.py observations.csv \
  --columns 4 --rows 6 \
  --spacing-x-mm 120 --spacing-y-mm 120 \
  --output-dir spacing_evaluation
```

This command needs only NumPy and the standard library. It prints a scorecard
and saves per-pair, per-observation, per-view and per-direction reports. Legacy
`--output some.csv` is supported as an extra per-pair CSV destination.
Distances in `observations.csv` must already be in millimetres. The input columns
are `view_index,beacon_id,valid_3d,X_cam1_mm,Y_cam1_mm,Z_cam1_mm` plus optional
`run_id,partition`. Both endpoints must belong to the same run, partition and
synchronised view. Duplicate observations are errors, not silently averaged.

### Evaluate from frozen camera calibration

```bash
bash evaluation/run_evaluation.sh \
  track_logs/stereo/stereo_points_camera_1_camera_2.npz \
  track_logs/stereo/stereo_calibration_camera_1_camera_2.json \
  track_logs/camera_1/camera_1_intrinsics.json \
  track_logs/camera_2/camera_2_intrinsics.json \
  --geometry-unit m \
  --spacing-x-mm 120 --spacing-y-mm 120 \
  --partition calibration_fit \
  --output-dir track_logs/stereo/evaluation
```

Replace paths with your actual output locations. `--geometry-unit` describes
**the units of the saved stereo T and calibration geometry**; it is not a request
to relabel arbitrary grid units as metres. The physical spacing arguments are
always in millimetres. Wrong T units invalidate all distance results. Translation
must obey `X_cam2 = R @ X_cam1 + T`; the 3D output is in camera 1 coordinates.

For an actual held-out recording use `--partition dev` or `--partition test`
and `--heldout-data-confirmed`, while keeping the intrinsic and stereo JSONs
frozen. Do not set these flags on calibration-fit observations. Existing
`--coverage-complete` and `--repeatability-runs` declarations are retained for
compatibility; the program does **not** independently verify coverage or compute
cross-run repeatability from a count. A PASS is a configured single-recording
check, not proof that the entire experimental protocol is complete.

Default thresholds are provisional: reprojection RMS 1 px, length relative RMSE
2%, triangulation-angle P5 1 degree, conditional valid fraction 95%. At 120 mm,
2% is 2.4 mm RMS. Choose application requirements on DEV before locking TEST.
No threshold was relaxed to make the supplied recording pass.

## Physical board mapping and general users

Default mapping: `beacon_id = first_id + row * columns + column`.
The full evaluator reads the first ID and board dimensions from stereo JSON;
the standalone CSV auditor accepts `--first-id`, `--columns`, and `--rows`.
A 4-column x 6-row board has 18 horizontal and 20 vertical adjacent pairs.
Verify physical LED centres and the firmware ID map; image orientation is not
board row/column orientation. Never infer missing IDs by renumbering the LEDs.

For arbitrary firmware numbering, pass `--id-map-csv board_map.csv`:

```csv
beacon_id,row,column
7,0,0
3,0,1
```

The file must include **every** grid cell, including undetected LEDs (the two-row
example describes only a 2-column x 1-row board). The mapping is an evaluation
reference; if the calibration used a wrong physical mapping, that calibration
must also be rerun with the correct object points. The evaluator cannot repair
incorrect frozen calibration parameters. Arbitrary surveyed non-grid targets
and fisheye models are not implemented; do not pass fisheye coefficients into
the pinhole model. The upstream intrinsic calibration currently uses a regular
planar board and must be extended separately for surveyed non-planar targets.

Optional `--pair-reference-csv measured_pairs.csv` accepts:

```csv
led_a,led_b,reference_length_mm,uncertainty_mm,measurement_method
0,1,120.08,0.10,caliper
```

Record real measurements, not fabricated precision. Uncertainty/method columns
are source metadata, not automatically propagated into confidence intervals.
The same references are used for adjacent pairs where supplied. Other adjacent
pairs use nominal spacing. The report preserves that distinction. Supplying a
measurement CSV does not by itself prove traceability or independence.
`--pairs 0-3,0-20,0-23,3-20` chooses supplemental M5 pairs; M6 still checks every
physical neighbour. With a custom map and no explicit pairs, supplemental M5
uses all observed pairs rather than assuming numeric ID order is spatial order.

## Metrics and interpretation

For each synchronised pair, independently triangulate the two LEDs using
undistorted normalised image coordinates and frozen `[I|0]`, `[R|T]` projection
matrices. Use `L_hat = norm(X_a-X_b)` and `error = L_hat - L_reference`.
For a 120 mm neighbour: `relative_error_percent = 100 * error / 120`.
Report signed bias, MAE, RMSE, sample SD, P95 absolute error and relative RMSE.
For mixed references the pooled relative RMSE is
`100 * sqrt(mean((error / reference)**2))`. Direction aggregates weight every
valid pair-observation equally, rather than averaging pair RMSEs. M5 retains
its separately labelled pair-balanced aggregation.

M1/M2: same-point stereo reprojection consistency (not independent board-model
accuracy). M3: acute triangulation angle. M4: Sampson epipolar diagnostic.
M5: selected long/reference-pair lengths. M6: all physical adjacent lengths.
Q: valid free reconstructions divided by supplied correspondences. Q is **not**
end-to-end detection yield; missed events before input creation are unavailable.
Finite epipolar observations are retained even if their reconstructed depths
are negative. Such reconstructions are excluded from 3D distances with an
explicit reason; their count is still reported. No residual-based trimming or
outlier rejection is used to improve the score.

A board's nominal pitch also sets the scale during calibration. Testing that
same nominal pitch cannot establish independent absolute scale. New held-out
recordings test generalisation; separately surveyed geometry/a separate measured
target strengthens scale validation. Good distances do not establish absolute
world-coordinate accuracy because distances are invariant to rigid transforms.

Reference for the projection/distortion/triangulation conventions:
[OpenCV calibration and reconstruction documentation](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
See also Sections 3, 4 and 6 of the supplied evaluation instructions. Revision 2
intentionally changes Sampson from a required gate to a diagnostic, as requested;
it does not redefine or conceal the original M4 quantity.

## Terminal and saved outputs

Both entry points print progress and a scorecard without a graphical desktop.
The full evaluator saves `terminal_scorecard.txt` itself, including when called
directly without `tee`. The revised v3 pipeline uses a separate
`evaluation_console.log` so two writers do not corrupt the scorecard.
`run_calibration.sh` and `run_calibration_v2.sh` do not invoke evaluation in the
supplied archive; use the independent command above or revised v3 runner.
If an earlier pipeline stage fails, evaluation is not reached. Revised v3
reports the stopping line. Paths in that runner remain installation-specific;
configure both camera inputs before running. It is not a generic installer.

Outputs include:

- `evaluation_report.json`: decisions, input SHA256 hashes, command and versions.
- `observations.csv`: coordinates, residuals, valid flags and failure reasons.
- `summary.csv`, `heldout_distance_audit.csv`: legacy M1–M5 tables (the latter's
  filename does not establish held-out provenance; inspect `partition`).
- `adjacent_spacing_audit.csv`: every eligible planned pair, including pairs with invalid reconstructions.
- `adjacent_spacing_samples.csv`: valid and missing/invalid pair opportunities.
- `adjacent_spacing_by_direction.csv`, `adjacent_spacing_by_view.csv`.
- `adjacent_spacing_report.json`, `terminal_scorecard.txt`, `recommendations.txt`.

Exit 2 means bad input/dependency/runtime error. Full evaluator default exit 0
means reports were successfully produced, even if calibration quality FAILED.
Use `--strict` for exit 1 on FAILED or NOT_EVALUATED in automated validation.
The standalone spacing-only command reports a spacing status, never an overall
calibration PASS, and exits 0 when report generation succeeds.

## Before public release

Publish source, dependency instructions, synthetic tests and a permitted sample
dataset; exclude local environments, compiled binaries, raw private recordings
and machine-specific paths. Retain third-party notices. The supplied project
contains multiple upstream components; select a licence only after confirming
authorship and dependency compatibility. No project-wide licence is invented
by this patch. Camera hardware capture, decoding and calibration are separate
stages with their own dependencies and are not claimed portable or validated by
the evaluator's tests. Supported scope here: two calibrated pinhole cameras,
matched NPZ inputs or an observations CSV, and a verified rectangular LED board.

## Automatic upstream availability (current behaviour)

The updated `stereo/build_stereo_points.py` filters tracker IDs using the decoded
IDs from each camera, then saves `decoded_beacon_ids_cam1` and
`decoded_beacon_ids_cam2` arrays with the existing shared `beacon_ids` in NPZ.
The evaluator intersects these lists with IDs having finite image observations
in each camera. IDs unavailable in either camera are excluded from **all**
full-evaluator metrics, including distance pair planning. Reports record the
source, per-camera availability, evaluated IDs and excluded IDs.

Older NPZ files already contain shared `beacon_ids`; these plus finite image
observations provide the fallback without rerunning decoding. CSV-only audits
infer availability separately for each run/partition from `u1,v1,u2,v2` when
present; older CSVs without image columns use supplied row membership as the
best available upstream evidence. A missing per-view endpoint remains a missing
sample if that ID is otherwise available in the run. No ID is excluded merely
because its 3D error is large, its triangulation fails, or `valid_3d=0`.

Other decoding systems can provide `--availability-json available_ids.json` to
either entry point, using actual successful IDs, for example:

```json
{"camera1_ids": [0, 1, 2, 3], "camera2_ids": [0, 2, 3]}
```

Here ID 1 is automatically unavailable for stereo evaluation. The metadata is
intersected with input observations, never used to invent points. The sidecar
must describe the recording being evaluated. In a CSV containing several runs,
provide a sidecar only if it applies to all runs; otherwise use per-run inputs
or the run-aware CSV inference. Never infer camera availability from successful
3D reconstruction alone.

`--exclude-led-ids` is an optional **additional manual** exclusion; its default
is `none`. It is unnecessary for normal use. `--exclude-led-ids none` does not
turn off automatic availability filtering. The 120 mm physical grid mapping
stays unchanged; neighbours are not connected across a removed LED.

The supplied data automatically yield 31/31 eligible adjacent pairs because IDs
14 and 21 are absent. A full 24-ID dataset automatically evaluates all 38 pairs.
A recording with other missing IDs automatically uses its own eligible pairs.
