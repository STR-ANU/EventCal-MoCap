#!/bin/bash
# Edit calibration_config.sh
#  ./run_calibration.sh calibration_config.sh
#
# calibration_config.sh is listed in .gitignore so your machine-specific
# paths never get committed or accidentally shipped to other users.

#=============================================================================#
# CAMERA INPUTS
#=============================================================================#
# Set the number of cameras, then add one name and one input event-file path
# for each camera. Array entry 0 describes camera 1, entry 1 describes camera 2,
# and so on. This is NOT limited to 2 cameras - add as many entries as you
# have cameras and set number_camera to match; run_calibration.sh validates
# these arrays before starting any processing. For example, a 3-camera rig:
#   number_camera=3
#   camera_names=("camera_1" "camera_2" "camera_3")
#   event_filepaths=("/abs/path/cam1.es" "/abs/path/cam2.es" "/abs/path/cam3.es")
number_camera=1

camera_names=(
    "camera_1"
    # "camera_2"
)

# INTRINSICS recording: one per camera, collected SEPARATELY and
# INDEPENDENTLY of the stereo recording below. Each camera only needs the
# board filling its own frame at varied distances/angles here - it does NOT
# need to be visible to the other camera at the same time, so this is
# usually easier to collect with good coverage (closer to the lens, more
# freedom of motion) than the stereo recording.
intrinsic_event_filepaths=(
    # "/home/vutran/event_camera/camera_calibration/Data/intrinsics/camera_1/2026-09-09T22-47-50Z_10000000_30000000.es"
    # "/home/vutran/event_camera/camera_calibration/Data/intrinsics/camera_2/2026-09-09T22-51-49Z_10000000_30000000.es"

    # "/home/vutran/event_camera/camera_calibration/Data/280926-CalibrationData/left_camera/280926-led/intrinsics-left-camera/2026-09-28T05-43-06Z.es"
    # "/home/vutran/event_camera/camera_calibration/Data/280926-CalibrationData/right_camera/280926-led/intrinsics-right-camera/2026-09-28T05-40-13Z.es"
    "/home/vutran/event_camera/camera_calibration/Data/300926-CalibData/2026-09-30T06-42-23Z.es"
)


# STEREO recording: the board waved so it's visible to BOTH cameras
# simultaneously throughout the whole clip - this is what
# stereo/time_sync.py and stereo/build_stereo_points.py consume below. It's
# the more constrained of the two recordings (has to satisfy every camera's
# framing at once), so keep it as its own separate pass rather than reusing
# an intrinsics-only recording here.
event_filepaths=(
    "/home/vutran/event_camera/camera_calibration/Data/280926-CalibrationData/left_camera/280926-led/extrinsics-left-camera/2026-09-28T05-44-34Z.es"
    # "/home/vutran/event_camera/camera_calibration/Data/280926-CalibrationData/right_camera/280926-led/extrinsics-right-camera/2026-09-28T05-44-30Z.es"
)


#=============================================================================#
# STEREO CALIBRATION SETTINGS
#=============================================================================#
# Stereo calibration runs after ALL cameras above have completed tracking,
# decoding, and intrinsic calibration - this works the same whether you have
# 2 cameras or 20.
run_stereo_calibration=false #true

# Which camera pairs to stereo-calibrate. Two modes:
#   "all_pairs" - every unique combination of the cameras listed above
#                 (C(N,2) pairs: 1 for 2 cameras, 3 for 3, 6 for 4, ...).
#                 Simplest choice for small rigs; grows quickly with more
#                 cameras, so switch to "explicit" once that's too many.
#   "explicit"  - only the pairs listed in stereo_pairs below.
stereo_pair_selection="all_pairs"

# Only used when stereo_pair_selection="explicit". One "i,j" string per pair,
# using 0-based indices into camera_names/event_filepaths above (so "0,1"
# pairs the first two cameras, "0,2" pairs the first and third, etc). Useful
# for a multi-camera rig where you only care about specific pairs (e.g. every
# camera paired with a single reference camera) rather than every
# combination.
stereo_pairs=(
    "0,1"
)

stereo_output_dir="./track_logs/stereo"

# These geometry values must be identical to the values passed to
# intrinsic_calibration.py, and must match your physical LED board.
# Measure led_spacing_x/led_spacing_y with calipers - do not leave them at a
# guessed value. A wrong spacing here shows up later as a bowl-shaped
# reprojection-error pattern that grows from the centre row/column of the
# board out to its edges (see the row/column diagnostic in
# evaluation/README.md).
led_columns=4
led_rows=6
led_spacing_x=0.12
led_spacing_y=0.12
led_h5_id_for_led1=0

# Time synchronization and tracker interpolation settings.
stereo_hz=10
stereo_max_gap=0.05
stereo_threshold_ms=3.0
stereo_ransac_iterations=2000
stereo_min_inliers=20
stereo_random_seed=0

# Minimum number of beacons that must be jointly visible (and trustworthy)
# in BOTH cameras at the same instant to keep a candidate stereo view.
# Matches intrinsic_calibration.py's own --min-points default (12). This
# used to be implicitly "every beacon ever seen in either camera, always" -
# which a full LED board essentially never satisfies once there's any real
# occlusion (e.g. from tilting the board to cover more of the FOV), and was
# the actual cause of "No synchronized views survived" - not --hz or
# --max-gap, despite that error message's old wording.
stereo_min_points=1

# Optional: reject a candidate stereo view if any beacon's local
# interpolation speed (pixels between the two bracketing tracker samples,
# divided by their time gap) exceeds this many px/s. Leave empty ("") to
# disable (the default - reproduces the original behavior exactly).
# view_max_speed_px_per_s is always saved to the stereo points NPZ
# regardless of this setting, so you can inspect the actual speed
# distribution (and check for a periodic pattern in per-view reprojection
# RMSE, via evaluation/calibration_quality.py's observations.csv) before
# picking a threshold. A continuous back-and-forth sweep of the board
# (rather than pausing at each pose) is exactly when this matters: fast
# motion between two tracker samples makes linear interpolation unreliable,
# and can dominate the whole stereo fit with its worst instants even though
# the same tracker data gives good per-camera intrinsics.
stereo_max_velocity_px_per_s=""

#=============================================================================#
# CALIBRATION QUALITY EVALUATION SETTINGS
#=============================================================================#
# The stereo points generated by this script are also used to FIT the stereo
# calibration, so they are calibration-fit evidence, not held-out validation.
# Keep evaluation_partition=calibration_fit here. Only switch to dev/test when
# you point stereo_points_npz (via --stereo-points-override, see README) at a
# physically separate recording and keep the calibration frozen.
run_quality_evaluation=true
evaluation_partition="calibration_fit"
evaluation_output_dir="$stereo_output_dir/evaluation"

# Geometry in this runner is specified in metres; the evaluator converts all
# reconstructed distances and the stereo baseline to millimetres.
evaluation_geometry_unit="m"

# Provisional operational gates. These are NOT universal camera specs -
# freeze application-specific values on DEV data before using them for a
# final TEST decision. Loosening them does not fix a bad calibration; it only
# changes when the script tells you about it.
evaluation_max_reprojection_rms_px=1.0
evaluation_max_sampson_rms_px=1.0
evaluation_max_relative_length_rmse_percent=2.0
evaluation_min_triangulation_angle_p5_deg=1.0

# Self-declared coverage checklist. calibration_quality.py does NOT inspect
# your recording to verify any of this - it only checks whether
# --coverage-complete was passed (set true here -> run_calibration.sh
# adds that flag; false/unset -> it's omitted and the evaluator keeps
# recommending it). Setting this to true is a promise you're making, not a
# measurement the tool takes for you.
#
# Before setting this to true, physically confirm your WAVED-BOARD
# recording (the event_filepaths above, NOT the stationary floor
# recording used for global frame calibration) actually included:
#   - The board visible in all four corners of the frame, not just centered
#   - At least 3 distinct distances spanning your real working range
#     (near/mid/far) - not just one distance from the cameras
#   - The board tilted roughly +/-30-45 degrees in both axes at multiple
#     positions - not just held flat/fronto-parallel to the cameras
#   - The board held STATIONARY for a brief dwell at each pose, rather than
#     swept continuously (continuous sweeping degrades stereo correspondence
#     quality - see the per-view reprojection RMSE / view_max_speed_px_per_s
#     diagnostics in the evaluation report if M1 looks worse than expected
#     despite otherwise-good coverage)
#
# If your existing recording already covers all of the above: set this to
# true now, no re-recording needed. If it doesn't: leave this false,
# re-record the waved-board clip covering the missing cases, re-run the
# full pipeline (tracking -> decoding -> intrinsics -> stereo ->
# evaluation) on the NEW recording, and only THEN set this to true.
evaluation_coverage_complete=true

# How many independent physical calibration sessions you've actually
# compared with run_repeatability.sh / run_repeatability_pipeline.sh (see
# those scripts). This is NOT the same thing as those scripts' own
# --min-runs flag - --min-runs only controls how many sessions THEY require
# before running; it has no connection to this variable and does not set it
# for you. Passed straight through as calibration_quality.py's own
# --repeatability-runs - like --coverage-complete, this is self-declared:
# calibration_quality.py does not verify it, it just trusts the number.
# Leave at 1 (the honest default - "I haven't done a repeatability check
# yet") until you have a real repeatability_report.json with status PASS
# from >= 3 genuinely separate sessions - then set this to that same run
# count (3, or however many you actually compared).
evaluation_repeatability_runs=1

#=============================================================================#
# GLOBAL FRAME CALIBRATION SETTINGS
#=============================================================================#
# Runs last, after stereo calibration above. Triangulates a recording of the
# SAME LED board, held stationary and flat on the floor (not waved), through
# one already-calibrated stereo pair, and uses the board's own known geometry
# (led_columns/led_rows/led_spacing_x/led_spacing_y above) to solve the rigid
# transform from that pair's Camera 1 into the board's frame - which becomes
# your world/global coordinate system (origin at LED 0, +X/+Y along the grid,
# +Z the board normal - see stereo/global_frame_calibrate.py). Requires
# run_stereo_calibration=true.
run_global_frame_calibration=false #true

# Which already-calibrated stereo pair to anchor the global frame to. Must be
# one of the pairs resolved above (from stereo_pairs, or from all_pairs).
global_frame_reference_pair="0,1"

# Event recordings of the board sitting motionless on the floor, one per
# camera in global_frame_reference_pair - camera_1's recording first, then
# camera_2's. These are separate files from event_filepaths above (which are
# the waved-board intrinsic/stereo recordings), captured in their own pass
# with the board laid flat and still.
global_frame_cam1_event_filepath="/home/vutran/event_camera/camera_calibration/Data/experiment-1/extrinsics/left-camera-global/2026-09-25T00-51-01Z_1000000_4000000.es"
global_frame_cam2_event_filepath="/home/vutran/event_camera/camera_calibration/Data/experiment-1/extrinsics/right-camera-global/2026-09-25T00-50-55Z_5000000_9000000.es"

# Optional: defaults to "$stereo_output_dir/global_frame" if left unset.
global_frame_output_dir="$stereo_output_dir/global_frame"

# Optional: use only the first N seconds of the board-on-floor recording's
# (already time-synced) stereo views for global_frame_calibrate.py's own
# fit, instead of every view build_stereo_points.py produced. The board is
# STATIONARY for this recording, so a couple of seconds is normally plenty
# of data - this bounds THIS script's compute and gives a fixed-size,
# reproducible fit regardless of how long you left the recording running.
#
# IMPORTANT: this does NOT reduce tracking/decoding time for the raw
# recording above - track_and_decode still processes the whole file
# (tracking/decoding is the expensive step for a long recording; this
# setting only trims data AFTER that's already finished, going into a
# small, already-fast numpy fit). If your actual goal is a shorter
# wall-clock run, trim/re-record the raw global_frame_cam1/2_event_filepath
# files themselves to a couple of seconds instead - this setting alone
# won't achieve that.
#
# Leave empty ("") to use every synced view (the default, previous
# behavior).
global_frame_max_duration_s="2.0"

#=============================================================================#
# HELD-OUT DEV/TEST EVALUATION SETTINGS
#=============================================================================#
# Runs last, after stereo calibration (and global frame, if enabled) above.
# Status can NEVER reach PASS on evaluation_partition=calibration_fit above -
# that partition evaluates the SAME recording used to fit the calibration,
# so a good score there proves the fit is self-consistent, not that it's
# accurate on data it hasn't seen. This step is the only way to actually
# earn PASS: it takes a NEW, physically disjoint recording (the board waved
# differently, NOT a rerun of the fitting clip), and evaluates the stereo
# pair's ALREADY-FITTED calibration against it - nothing here re-fits
# anything. Requires run_stereo_calibration=true and
# run_quality_evaluation=true.
run_heldout_evaluation=false

# "dev" while you're still iterating on your recording process; switch to
# "test" for the one final, unbiased check once you're done tuning - dev and
# test should each be their own separate recording, never the same clip.
heldout_partition="dev"

# Which already-calibrated stereo pair's frozen calibration to evaluate
# against. Must be one of the pairs resolved above (from stereo_pairs, or
# from all_pairs).
heldout_reference_pair="0,1"

# The NEW held-out recording, one file per camera in heldout_reference_pair.
# Record this with the SAME physical rig/mount as the fitting recording, but
# move the board through DIFFERENT poses - not a rerun of event_filepaths
# above, and not the same clip as global_frame_cam1/2_event_filepath either.
heldout_cam1_event_filepath=""
heldout_cam2_event_filepath=""

# Self-declared, like evaluation_coverage_complete above, but about THIS
# held-out recording specifically (a different clip, so its own coverage
# claim) - confirm the same checklist (all four corners, >=3 distances,
# +/-30-45 deg tilts, held stationary per pose) before setting true.
heldout_coverage_complete=false

# Optional: defaults to "$stereo_output_dir/heldout_$heldout_partition" if
# left unset (so "dev" and "test" runs land in separate directories
# automatically).
heldout_output_dir="$stereo_output_dir/heldout_${heldout_partition}"
