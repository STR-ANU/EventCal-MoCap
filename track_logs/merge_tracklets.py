"""
Offline tracklet stitching for the AEMOT output.

Given the per-track summary log (.beesum file) and the per-event log (.bees file),
this builds a start state and end state for every track, gates all time-ordered
pairs by Mahalanobis distance (under the same constant-velocity dynamics pulled straight
from the tracker's YAML file), and finds the globally-optimal stitching via the Hungarian
algorithm. Match chains are then chained into full stitched groups if more than two are joined
in a chain.
"""

import argparse
from dataclasses import dataclass
import os
import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2
from extract_tracks import read_log, read_summary_log

STATE_DIM = 4 ## we stitch on [x, y, vx, vy] only

@dataclass
class DynamicsConfig:
    """
    The subset of the tracker's YAML config needed to reproduce its constant-
    velocity F/Q and initial P0.
    """
    dt: float
    var_x: float
    var_y: float
    var_vx: float
    var_vy: float
    q_x: float
    q_y: float
    q_vx: float
    q_vy: float

    @classmethod
    def from_yaml(cls, path):
        with open(path, "r") as f:
            config = yaml.safe_load(f)
        try:
            return cls(
                dt=float(config["dt"]),
                var_x=float(config["var_x"]),
                var_y=float(config["var_y"]),
                var_vx=float(config["var_vx"]),
                var_vy=float(config["var_vy"]),
                q_x=float(config["q_x"]), q_y=float(config["q_y"]),
                q_vx=float(config["q_vx"]), q_vy=float(config["q_vy"]),
            )
        except KeyError as e:
            raise KeyError(
                f"Config at {path} is missing expected key {e}."
                f"This loader assumes flat top-level keys matching parameters"
                f"(dt, var_x, var_y, var_vx, var_vy, q_x, q_y, q_vx, q_vy)."
            ) from e

    def P0(self):
        return np.diag([self.var_x, self.var_y, self.var_vx, self.var_vy])

def build_Q(gap_seconds, sigma_ax, sigma_ay):
    """
    Discrete White Noise Acceleration (DWNA) process noise for an
    arbitrary elapsed time `gap_seconds`, one 2x2 [pos, vel] block per axis,
    assembled into the 4x4 [x, y, vx, vy] layout:
 
        Q_axis(dt) = sigma_a^2 * [[dt^3/3, dt^2/2],
                                   [dt^2/2, dt    ]]
 
    sigma_ax/sigma_ay are the per-axis standard deviation of unmodelled
    acceleration, in pixels/s^2 - a genuine per-second physical rate, unlike
    the online tracker's q_x/q_y/q_vx/q_vy (which are tuned per associated-
    event update, not per real second).
    """
    def axis_block(sigma_a):
        s2 = sigma_a ** 2
        dt = gap_seconds
        return np.array([
            [s2 * dt**3 / 3.0, s2 * dt**2 / 2.0],
            [s2 * dt**2 / 2.0, s2 * dt],
        ])

    Qx = axis_block(sigma_ax)
    Qy = axis_block(sigma_ay)

    Q = np.zeros((4, 4))
    Q[np.ix_([0, 2], [0, 2])] = Qx
    Q[np.ix_([1, 3], [1, 3])]= Qy

    return Q

def build_F(dt):
    """
    Constant-velcity state transition for arbitrary elapsed time dt,
    taking the general structure of F in processing.cpp::setup() -
    (F(j*n_state,2)=dt for x+=vx*dt, F(j*n_state+1,3)=dt for y+=vy*dt)
    """
    return np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=float)

def extract_track_endpoints(summary_data, event_data, config: DynamicsConfig):
    """
    Builds for every track in summary_data:
        start: (t_created, x_start[4], P_start = config.P0())
        end: (t_deleted, x_end[4], P_end = summary's P_at_deletion, sliced to 4x4)
    x_start comes from that track's earliest record in the dense per-event log (event_data).
    matched by track_id.
    """
    endpoints = {}
    for rec in summary_data:
        tid = int(rec["track_id"])
        mask = event_data["track_id"] == tid
        if not np.any(mask):
            #raise ValueError("Warning! Mask was empty!! This shouldnt happen for validated tracks...")
            continue
        track_rows = event_data[mask]
        first_row = track_rows[np.argmin(track_rows["ts"])]

        x_start = first_row["x_hat"][:STATE_DIM].astype(float)
        P_start = config.P0()
        t_start = float(first_row["ts"])

        x_end = rec["x_hat_at_deletion"][:STATE_DIM].astype(float)
        P_full_end = rec["P_at_deletion"].reshape(8, 8)
        P_end = P_full_end[:STATE_DIM, :STATE_DIM]
        t_end = float(rec["t_deleted"])

        endpoints[tid] = {
            "t_start": t_start, "x_start": x_start, "P_start": P_start,
            "t_end": t_end, "x_end": x_end, "P_end": P_end,
        }
    return endpoints

def return_config(config_path):
    return DynamicsConfig.from_yaml(config_path)


def compute_track_kinematics(event_data, resample_dt=0.01):
    """Returns (speeds, accels, dts) pooled across every track in
    event_data: speeds = instantaneous px/s at every raw record (from the
    Kalman filter's own vx,vy state - x_hat indices 2,3, no resampling
    needed here), accels = px/s^2 computed between RESAMPLED points spaced
    >= resample_dt apart (see below for why), dts = the resampled dt values
    actually used (returned for diagnostics).
 
    Why resampling, not just filtering out the smallest gaps: on this data
    the online filter updates every associated event - median inter-record
    dt came back at ~35 MICROSECONDS. Differentiating velocity between two
    updates that close together means dividing by a near-zero denominator,
    so even ordinary state-estimation jitter (a fraction of a px/s of
    noise, completely normal for a Kalman filter) turns into an apparent
    acceleration of hundreds of thousands of px/s^2 - that's differentiation
    noise, not real bee kinematics. A real bee's flight maneuvers happen on
    the order of tens of milliseconds, not tens of microseconds, so
    resample_dt=0.01 (10ms) targets a timescale where genuine direction/
    speed changes actually happen, rather than one dominated by per-update
    filter noise. This is a real trade-off - a coarser resample_dt smooths
    out noise but can also blur together fast genuine maneuvers, so treat
    this as an estimate, and sanity-check the result against diagnose_pair()
    on tracks you can see turning sharply in the plot.
    """
    speeds, accels, used_dts = [], [], []
    for tid in np.unique(event_data["track_id"]):
        rows = event_data[event_data["track_id"] == tid]
        rows = rows[np.argsort(rows["ts"])]
        vx_all, vy_all = rows["x_hat"][:, 2], rows["x_hat"][:, 3]
        speeds.append(np.hypot(vx_all, vy_all))
 
        # Greedy stride-sample: walk forward, keeping a point only once at
        # least resample_dt has elapsed since the last kept point.
        ts = rows["ts"]
        keep_idx = [0]
        last_ts = ts[0]
        for i in range(1, len(ts)):
            if ts[i] - last_ts >= resample_dt:
                keep_idx.append(i)
                last_ts = ts[i]
        if len(keep_idx) < 2:
            continue
 
        keep_idx = np.array(keep_idx)
        vx, vy, kts = vx_all[keep_idx], vy_all[keep_idx], ts[keep_idx]
        dt = np.diff(kts)
        dvx, dvy = np.diff(vx), np.diff(vy)
        accels.append(np.hypot(dvx, dvy) / dt)
        used_dts.append(dt)
 
    speeds = np.concatenate(speeds) if speeds else np.array([])
    accels = np.concatenate(accels) if accels else np.array([])
    used_dts = np.concatenate(used_dts) if used_dts else np.array([])
    return speeds, accels, used_dts
 
 
def suggest_stitch_params(event_data, max_gap_seconds=1.0, resample_dt=0.01,
                           speed_percentile=99, accel_percentile=95, safety_factor=1.5,
                           frame_width=1280, frame_height=720):
    """Prints observed speed/acceleration statistics from event_data and a
    suggested starting (sigma_a, max_pixel_jump) pair. Treat the printed
    suggestion as a starting point for diagnose_pair()-guided tuning, not a
    final answer - percentiles/safety_factor/resample_dt are themselves
    judgement calls.
 
    resample_dt: see compute_track_kinematics()'s docstring - acceleration
    is computed between points resampled to at least this far apart in
    time, not between raw consecutive per-event records, because
    differentiating at the raw per-event rate is typically dominated by
    Kalman-filter estimation noise rather than real motion.
 
    frame_width/frame_height: max_pixel_jump is clipped to the frame
    diagonal - a jump larger than that is impossible within a single frame
    and would make the hard-clamp backstop inert (never actually reject
    anything). Pass None to disable clipping.
    """
    speeds, accels, dts = compute_track_kinematics(event_data, resample_dt=resample_dt)
    if len(speeds) == 0:
        print("No records found - can't compute kinematics.")
        return None
 
    speed_p = np.percentile(speeds, speed_percentile)
    accel_p = np.percentile(accels, accel_percentile) if len(accels) else 0.0
 
    suggested_jump = speed_p * max_gap_seconds * safety_factor
    suggested_sigma_a = accel_p
 
    print(f"Resampled dt used for accel (target >= {resample_dt}s): "
          f"median={np.median(dts):.4f}, min={dts.min():.4f}" if len(dts) else
          "No resampled dt pairs found - tracks may all be shorter than resample_dt.")
    print(f"Speed (px/s, raw per-event): median={np.median(speeds):.1f}, "
          f"p{speed_percentile}={speed_p:.1f}, max={speeds.max():.1f}")
    if len(accels):
        print(f"Accel (px/s^2, resampled): median={np.median(accels):.1f}, "
              f"p{accel_percentile}={accel_p:.1f}, max={accels.max():.1f}")
    else:
        print("Accel: no resampled pairs found (tracks too short relative to resample_dt?)")
 
    if frame_width is not None and frame_height is not None:
        diagonal = float(np.hypot(frame_width, frame_height))
        if suggested_jump > diagonal:
            print(f"\nNOTE: raw suggestion ({suggested_jump:.1f}px) exceeds the "
                  f"{frame_width}x{frame_height} frame diagonal ({diagonal:.1f}px) - "
                  f"clipping to the diagonal, since a jump that large is impossible "
                  f"within a single frame and would make max_pixel_jump a no-op.")
            suggested_jump = diagonal
 
    print(f"\nSuggested starting point for max_gap_seconds={max_gap_seconds}:")
    print(f"  --sigma_a {suggested_sigma_a:.1f}")
    print(f"  --max_pixel_jump {suggested_jump:.1f}")
    print(f"(both include a {safety_factor}x safety margin over the "
          f"{speed_percentile}th/{accel_percentile}th percentile observed - "
          f"tighten or loosen from here using diagnose_pair() on real pairs)")
 
    return suggested_sigma_a, suggested_jump






def mahalanobis_sq(residual, S):
    try:
        solved = np.linalg.solve(S, residual)
    except np.linalg.LinAlgError:
        solved = np.linalg.pinv(S) @ residual
    return float(residual @ solved)

def build_cost_matrix(endpoints, sigma_ax, sigma_ay, max_gap_seconds, max_pixel_jump, confidence=0.99):
    track_ids = sorted(endpoints.keys())
    n = len(track_ids)
    cost = np.full((n, n), np.inf)
    threshold = chi2.ppf(confidence, df=STATE_DIM)

    for i, tid_a in enumerate(track_ids):
        a = endpoints[tid_a]
        for j, tid_b in enumerate(track_ids):
            if tid_a == tid_b:
                continue # don't compare a tracklet to itself
            b = endpoints[tid_b]
            gap = b["t_start"] - a["t_end"]
            if gap <= 0 or gap > max_gap_seconds:
                continue # B doesnt start after A ends, or the gap is too large to bridge

            F = build_F(gap)
            x_pred = F @ a["x_end"]
            P_pred = F @ a["P_end"] @ F.T + build_Q(gap, sigma_ax, sigma_ay)

            residual = b["x_start"] - x_pred
            euclidean_jump = float(np.hypot(residual[0], residual[1]))
            if euclidean_jump > max_pixel_jump:
                continue # hard clamp here

            S = P_pred + b["P_start"]
            d2 = mahalanobis_sq(residual, S)

            if d2 <= threshold:
                cost[i, j] = d2

    return track_ids, cost

def solve_assignment(track_ids, cost):
    """
    Runs the Hungarian assignment over the (possibly mostly inf) cost
    matrix and returns {predecessor_track_id: successor_track_id} for only
    the pairs that were actually within the gate (finite cost).

    The solver is forced to produce a full assignment even where every option is inf,
    so these need to be filtered out afterwards rather than trustinf the output as real matches.
    """

    n = len(track_ids)
    finite_mask = np.isfinite(cost)
    sentinel = (cost[finite_mask].max() if finite_mask.any() else 1.0) + 1e6
    cost_finite = np.where(finite_mask, cost, sentinel)

    row_idx, col_idx = linear_sum_assignment(cost_finite)

    matches = {}
    for r, c in zip(row_idx, col_idx):
        if finite_mask[r, c]:
            matches[track_ids[r]] = track_ids[c]
    return matches

def chain_matches(track_ids, matches):
    """
    Follows predecessor->successor links to build full stitched groups.
    Returns (groups, track_to_group) where groups is a list of ordered
    track_id lists (each a full stitched track), and track_to_group maps
    every track_id (stitched or standalone) to a group id (the first track_id
    in its chain).
    """
    successors = dict(matches) # a -> b
    predecessors = {b: a for a, b in matches.items()} # b -> a

    groups = []
    track_to_group = {}
    visited = set()

    # start each chain from a track with no predecessor, so every chain is
    # walked exactly once, start to finish
    chain_starts = [tid for tid in track_ids if tid not in predecessors]
    for start in chain_starts:
        if start in visited:
            continue
        chain = [start]
        visited.add(start)
        cur = start
        while cur in successors:
            cur = successors[cur]
            if cur in visited:
                break # shouldn't happen, but guard anyways
            chain.append(cur)
            visited.add(cur)
        groups.append(chain)
        for tid in chain:
            track_to_group[tid] = start

    return groups, track_to_group

def diagnose_pair(endpoints, tid_a, tid_b, sigma_ax, sigma_ay, confidence=0.99):
    """
    Prints a breakdown of the gate for one specific (a_end -> b_start) pair.
    Use this to sanity-check the sigma_ax/sigma_ay/max_pixel_jump against a track
    pair that you know should or shouldn't stitch, rather than tuning blind against
    the aggregate group count.
    """
    a, b = endpoints[tid_a], endpoints[tid_b]
    gap = b["t_start"] - a["t_end"]
    print(f"track {tid_a} end (t={a['t_end']:.4f}) -> track {tid_b} start (t={b['t_start']:.4f})")
    print(f"  gap = {gap:.4f}s")
    if gap <= 0:
        print("  REJECTED: b does not start after a ends")
        return

    F = build_F(gap)
    x_pred = F @ a["x_end"]
    Q = build_Q(gap, sigma_ax, sigma_ay)
    P_pred = F @ a["P_end"] @ F.T + Q

    residual = b["x_start"] - x_pred
    euclidean_jump = float(np.hypot(residual[0], residual[1]))
    S = P_pred + b["P_start"]
    d2 = mahalanobis_sq(residual, S)
    threshold = chi2.ppf(confidence, df=STATE_DIM)
 
    print(f"  predicted b start (from a):     {x_pred[:2]}")
    print(f"  actual b start:                 {b['x_start'][:2]}")
    print(f"  euclidean position jump:        {euclidean_jump:.1f}px")
    print(f"  P_pred diag (x,y,vx,vy):        {np.diag(P_pred)}")
    print(f"  S diag (x,y,vx,vy):             {np.diag(S)}")
    print(f"  squared Mahalanobis distance:   {d2:.2f}  (chi2 threshold @ {confidence}: {threshold:.2f})")
    print(f"  would pass chi2 gate:           {d2 <= threshold}")


def stitch_tracks(summary_data, event_data, config_path, sigma_ax, sigma_ay,
                  max_gap_seconds=1.0, max_pixel_jump=100.0, confidence=0.99):
    """
    Full stitching pipeline. 
    """

    config = DynamicsConfig.from_yaml(config_path)
    endpoints = extract_track_endpoints(summary_data, event_data, config)
    track_ids, cost = build_cost_matrix(endpoints, sigma_ax, sigma_ay, max_gap_seconds, max_pixel_jump, confidence)
    matches = solve_assignment(track_ids, cost)
    groups, track_to_group = chain_matches(track_ids, matches)

    return groups, track_to_group

## entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="path to the .bees per-event log")
    #parser.add_argument("summary_path", help="path to the .beesum track summary log")
    parser.add_argument("config_name", help="name of the tracker's YAML config in the config/ folder (without .yaml !)")

    parser.add_argument("--sigma_a", type=float, default=0.0,
                        help="per-axis acceleration noise std dev, px/s^2 - "
                            "see stitch_tracks()'s docstring. Defaults to 0 "
                            "(tightest gate, pure constant-velocity) - raise "
                            "this deliberately, don't trust the default.")

    parser.add_argument("--max_gap_seconds", type=float, default=1.0)
    parser.add_argument("--max_pixel_jump", type=float, default=100.0)
    parser.add_argument("--confidence", type=float, default=0.99)

    parser.add_argument("--suggest", action="store_true",
                         help="print empirically-derived starting sigma_a/max_pixel_jump "
                              "from the event log's own velocity data, then exit "
                              "without stitching")
    parser.add_argument("--frame_width", type=int, default=1280)
    parser.add_argument("--frame_height", type=int, default=720)
    parser.add_argument("--resample_dt", type=float, default=0.01,
                         help="timescale (s) acceleration is estimated over for --suggest "
                              "- see compute_track_kinematics()'s docstring")


    args = parser.parse_args()

    this_file_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(this_file_dir, args.log + ".bees")

    summary_path = os.path.join(this_file_dir, args.log + ".beesum")

    event_data = read_log(log_path)
    if args.suggest:
        suggest_stitch_params(event_data, max_gap_seconds=args.max_gap_seconds,
                               resample_dt=args.resample_dt,
                               frame_width=args.frame_width, frame_height=args.frame_height)

        raise SystemExit(0)

    summary_data, _ = read_summary_log(summary_path)

    this_file_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(this_file_dir, '..', 'configs', args.config_name + ".yaml")
    config_path = os.path.normpath(config_path)

    groups, track_to_group = stitch_tracks(
        summary_data, event_data, config_path,
        sigma_ax=args.sigma_a, sigma_ay=args.sigma_a,
        max_gap_seconds=args.max_gap_seconds, 
        max_pixel_jump=args.max_pixel_jump,
        confidence=args.confidence,
    )

    multi = [g for g in groups if len(g) > 1]
    print(f"{len(summary_data)} tracks -> {len(groups)} groups"
          f"({len(multi)} actually stitched, {len(groups) - len(multi)} standalone)")
    for g in multi:
        print("  " + " -> ".join(str(t) for t in g))