"""
Plots every track from an AEMOT .bees track log on a single image, one
colour per track. Also reads the companion .beesum per-track summary log
For offline tracklet stitching, see stitch_tracks.py, which uses 
read_log()/read_summary_log() from this file.
 
.bees Binary layout (per-event log - must match track_logger.hpp):
    KalmanLogFileHeader (24 bytes):
        char     magic[8]        "AEMOTLOG"
        uint32   version          (2 - x_hat trimmed from 10 to 8 wide)
        uint32   n_state          (8)
        uint32   record_size      (80)
        uint32   reserved
    KalmanLogRecord (80 bytes), repeated:
        uint64   track_id
        double   ts
        double   x_hat[8]         state vector: x, y, vx, vy, lambda1, lambda2, theta, q
 
.beesum Binary layout  (per-track summary log - must match track_summary_logger.hpp):
    TrackSummaryFileHeader (24 bytes):
        char     magic[8]        "AEMOTSUM"
        uint32   version          (2 - validation-time state/covariance dropped)
        uint32   state_dim        (8)
        uint32   record_size      (624)
        uint32   reserved
    TrackSummaryRecord (624 bytes), repeated:
        uint64   track_id
        double   t_created
        double   t_validated       timestamp only - no state/covariance snapshot
        double   t_deleted
        uint32   num_records
        uint32   delete_reason     bitmask, see TrackDeleteReason in the .hpp
        double   event_rate_at_deletion
        double   x_hat_at_deletion[8]
        double   P_at_deletion[64]       row-major 8x8

"""

import numpy as np
import argparse
import os
import struct
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib as mpl
from datetime import datetime, timedelta

## -------------
EXPECTED_BEES_VERSION = 2
EXPECTED_BEESUM_VERSION = 2
## -------------

HEADER_FMT = "<8sIIII" # magic, version, n_state, record_size, reserved
HEADER_SIZE = 24  # magic(8) + version(4) + n_state(4) + record_size(4) + reserved(4)

SUMMARY_HEADER_FMT = "<8sIIII"  # same shape as the per-event header
SUMMARY_HEADER_SIZE = 24

def load_kalman_log(path):
    """Returns {track_id: {"ts": np.ndarray, "x_hat": np.ndarray[N,10]}}."""
    with open(path, "rb") as f:
        f.seek(HEADER_SIZE)  # skip KalmanLogFileHeader
        records = np.fromfile(f, dtype=RECORD_DTYPE)

    tracks = {}
    for tid in np.unique(records["track_id"]):
        mask = records["track_id"] == tid
        tracks[int(tid)] = {
            "ts": records["ts"][mask],
            "x_hat": records["x_hat"][mask],
        }
    return tracks

def read_log(path):
    """Reads the header, then returns a structured numpy array of records."""
    with open(path, "rb") as f:
        header_bytes = f.read(HEADER_SIZE)
        magic, version, n_state, record_size, _ = struct.unpack(HEADER_FMT, header_bytes)
        print(f"Magic: {magic}, Version: {version}, n_state: {n_state}, record_size: {record_size}")

        if magic != b"AEMOTLOG":
            raise ValueError(f"Invalid magic bytes in log file: expected AEMOTLOG, got {magic!r} - wrong file or format has changed...")

        if version != EXPECTED_BEES_VERSION:
            raise ValueError(f"This .bees file has the wrong version! Expected {EXPECTED_BEES_VERSION}, but read {version} from file.")

        ## for track log files (.bees):
        RECORD_DTYPE = np.dtype([
            ("track_id", "<u8"),
            ("ts",       "<f8"),
            ("x_hat",    "<f8", (n_state,)),
        ])
        if RECORD_DTYPE.itemsize != record_size:
            raise ValueError(f"Record size mismatch: expected {RECORD_DTYPE.itemsize}, got {record_size}")

        data = np.fromfile(f, dtype=RECORD_DTYPE)
    return data

def read_summary_log(path):
    """Reads the per-track summary header, then returns a structured numpy array
    of TrackSummaryRecord entries - one row per validated track."""
    with open(path, "rb") as f:
        header_bytes = f.read(SUMMARY_HEADER_SIZE)
        magic, version, state_dim, record_size, _ = struct.unpack(SUMMARY_HEADER_FMT, header_bytes)

        if magic != b"AEMOTSUM":
            raise ValueError(f"Invalid magic bytes in summary file: expected AEMOTSUM, got {magic!r} - wrong file or format has changed...")

        if version != EXPECTED_BEESUM_VERSION:
            raise ValueError(f"This .beesum file has the wrong version! Expected {EXPECTED_BEESUM_VERSION}, but read {version} from file."
                             f"Version 2 dropped the covarience at validation storage, which was unneeded.")

        SUMMARY_RECORD_DTYPE = np.dtype([
            ("track_id", "<u8"),
            ("t_created", "<f8"),
            ("t_validated", "<f8"),
            ("t_deleted", "<f8"),
            ("num_records", "<u4"),
            ("delete_reason", "<u4"),
            ("event_rate_at_deletion", "<f8"),
            ("x_hat_at_deletion", "<f8", (state_dim,)),
            ("P_at_deletion", "<f8", (state_dim * state_dim,)),
        ])
        if SUMMARY_RECORD_DTYPE.itemsize != record_size:
            raise ValueError(f"Record size mismatch: expected {SUMMARY_RECORD_DTYPE.itemsize}, got {record_size}")

        data = np.fromfile(f, dtype=SUMMARY_RECORD_DTYPE)
    return data, state_dim

def get_P_matrix(rec, which, state_dim):
    """Reshapes a flat row-major P array back into a (state_dim, state_dim) matrix.
    'which' is 'validation' or 'deletion'."""
    flat = rec[f"P_at_{which}"]
    return flat.reshape(state_dim, state_dim)


def plot_tracks(data, background_image=None, output_path="temp.png", x_idx=0, y_idx=1, invert_y=True, track_groups=None):
    """Draws every track's (x, y) path over background_image (or a blank
    canvas if none given).
 
    track_groups: optional dict mapping track_id -> group_id. When given,
    tracks sharing a group_id (e.g. output from stitch_tracks.py) are drawn
    in the same colour and labelled with the group id instead of the raw
    track id - handy for visually checking whether stitched groups look
    like single continuous objects. When omitted (default), every track_id
    is its own group.
    """

    track_ids = np.unique(data["track_id"])

    if track_groups is None:
        track_groups = {tid: tid for tid in track_ids}
    group_ids = sorted(set(track_groups.get(tid, tid) for tid in track_ids), key=str)
    group_color_idx = {gid: i for i, gid in enumerate(group_ids)}
    colors = mpl.colormaps["tab20"].resampled(len(group_ids))

    fig, ax = plt.subplots(figsize=(12, 8))

    if background_image is not None:
        img = plt.imread(background_image)
        ax.imshow(img, cmap="gray" if img.ndim == 2 else None)
    elif invert_y:
        ax.invert_yaxis()

    labelled_groups = set()
    for tid in track_ids:
        mask = data["track_id"] == tid
        xs = data["x_hat"][mask, x_idx]
        ys = data["x_hat"][mask, y_idx]
        gid = track_groups.get(tid, tid)
        i = group_color_idx[gid]
        color = colors(i % 20) if len(group_ids) <= 20 else cm.hsv(i / len(group_ids))

        ax.plot(xs, ys, "-", linewidth=1, alpha=0.8, color=color)
        ax.plot(xs[0], ys[0], "o", markersize=4, color=color) # start marker
        ax.plot(xs[-1], ys[-1], "x", markersize=6, color=color)  # end marker

        if gid not in labelled_groups:
            ax.annotate(str(gid), (xs[-1], ys[-1]), fontsize=6, color=color)
            labelled_groups.add(gid)

    ax.set_title(f"{len(track_ids)} validated tracks, {len(group_ids)} group(s)")
    ax.set_xlabel("x (px)")
    ax.set_ylabel("y (px)")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    print(f"Saved plot with {len(track_ids)} tracks ({len(group_ids)} groups) to {output_path}")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="name of the .bees log file inside the track_logs/ folder")
    parser.add_argument("--background", help="path to background image for plotting")   
    parser.add_argument("--output", default=None, help="custom name to save the plot image. Defaults to the same name as the input .bees file")
    parser.add_argument("--summary", default=None, help="custom name of the companion .beesum track summary file inside the track_logs/ folder. If not set, will default to the same name as the .bees file.")
    parser.add_argument("--config", default=None,
                         help="name of the tracker's YAML config inside the configs/ folder (for --stitch, to match "
                              "dynamics/covariance with the online tracker - see stitch_tracks.py)")
    parser.add_argument("--stitch", action="store_true",
                        help="if set (requires --summary_path and --config_path), run "
                            "stitch_tracks.stitch_tracks() and recolour the plot by "
                            "stitched group instead of raw track_id")

    parser.add_argument("--sigma_a", type=float, default=0.0,
                         help="passed through to stitch_tracks() - see its docstring")
    parser.add_argument("--max_gap_seconds", type=float, default=1.0)
    parser.add_argument("--max_pixel_jump", type=float, default=100.0)
    parser.add_argument("--confidence", type=float, default=0.99)

    args = parser.parse_args()

    this_file_dir = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(this_file_dir, args.log + ".bees")

    if args.summary is None:
        summary_path = os.path.join(this_file_dir, args.log + ".beesum")
    else:
        summary_path = os.path.join(this_file_dir, args.summary + ".beesum")

    if args.output is None:
        if args.stitch:
            output_path = os.path.join(this_file_dir, args.log + "_stitched.png")
        else:
            output_path = os.path.join(this_file_dir, args.log + ".png")
    else:
        output_path = os.path.join(this_file_dir, args.output + ".png")


    print(f"[Info] Loading .bees log from {log_path}...")
    tracks = read_log(log_path)

    # print .bees file info
    print("[Info] .bees log loaded successfully:")
    print("")
    print("------------------------------------------")
    print("")
    track_ids = np.unique(tracks["track_id"])
    for tid in track_ids:
        mask = tracks["track_id"] == tid
        n_records = mask.sum()
        print(f"Track {tid}: {n_records} records")
    print(f"[end of file] Total tracks: {len(track_ids)}")
    print("")
    print("------------------------------------------")
    print("")

    print(f"[Info] Loading .beesum track summaries from {summary_path}...")
    summary_data, state_dim = read_summary_log(summary_path)

    print("[Info] .beesum track summaries loaded successfully:")
    print(f"Loaded {len(summary_data)} track summaries "
    f"(state_dim={state_dim}) from {summary_path}")

    print("")
    print("------------------------------------------")
    print("")

    track_groups = None
    if args.stitch:
        if not(args.config):
            parser.error("--stitch requires --config")
        from merge_tracklets import stitch_tracks

        print("[Info] Stitching requested, merging tracklets into groups...")

        config_path = os.path.join(this_file_dir, '..', 'configs', args.config + ".yaml")
        
        groups, track_to_group = stitch_tracks(
            summary_data, tracks, config_path,
            sigma_ax=args.sigma_a, sigma_ay=args.sigma_a,
            max_gap_seconds=args.max_gap_seconds,
            max_pixel_jump=args.max_pixel_jump,
            confidence=args.confidence,
        )
        print(f"[Info] Successfully stitched {len(summary_data)} tracks into {len(groups)} group(s)")
        track_groups = track_to_group

    print("")
    print("------------------------------------------")
    print("")

    print("[Info] Plotting tracks and saving to png...")

    plot_tracks(tracks, background_image=args.background, output_path=output_path, track_groups=track_groups)

    # ##debug diagnose tracks
    # print("[Info] Diagnosing track pair 1768 and 1668...")
    # from merge_tracklets import diagnose_pair, extract_track_endpoints, return_config
    # config = return_config(config_path)
    # endpoints = extract_track_endpoints(summary_data, tracks, config)
    # diagnose_pair(endpoints, 1668, 1768, args.sigma_a, args.sigma_a, args.confidence)


    
if __name__ == "__main__":
    main()