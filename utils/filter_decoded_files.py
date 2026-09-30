import os
import argparse
from pathlib import Path
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
import struct
import h5py

## -------------
EXPECTED_BEES_VERSION = 2
EXPECTED_BEESUM_VERSION = 2
## -------------

HEADER_FMT = "<8sIIII" # magic, version, n_state, record_size, reserved
HEADER_SIZE = 24  # magic(8) + version(4) + n_state(4) + record_size(4) + reserved(4)

SUMMARY_HEADER_FMT = "<8sIIII"  # same shape as the per-event header
SUMMARY_HEADER_SIZE = 24

def read_log(path):
    """Reads the header, then returns a structured numpy array of records."""
    with open(path, "rb") as f:
        header_bytes = f.read(HEADER_SIZE)
        magic, version, n_state, record_size, _ = struct.unpack(HEADER_FMT, header_bytes)
        print(f"Log File: {path} \n\tMagic: {magic}, Version: {version}, n_state: {n_state}, record_size: {record_size}")

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


MAX_PLAUSIBLE_COUNTER_GAP = 1000  # see remove_counter_spikes() docstring for justification


def remove_counter_spikes(decoder, header):
    """
    Detects and discards isolated rows where the reconstructed 16-bit
    counter (byte2 + byte3*256) is wildly inconsistent with its immediate
    neighbors in time - the counter jumps away by an implausibly large
    amount, then immediately jumps back to a value consistent with
    resuming the original sequence, as if the bad row were never there.

    This is the counter-only counterpart of the byte1-mode filter below:
    some misdecoded packets happen to have byte1 land on the beacon's
    correct value purely by chance (~1/256 of the time), so byte2/byte3 -
    despite being equally garbage - survive that filter untouched. A
    genuine forward gap (packets that were dropped/missed, not decoded at
    all) is always small - empirically, the 99th percentile of real
    consecutive-row gaps in this codebase's own recordings is ~120, and
    there is a large, clean separation before the next real value appears
    (tens of thousands). A jump of that scale within a single row-to-row
    step, immediately followed by the sequence resuming exactly where it
    left off, is not a physically plausible blink rate - it's a corrupted
    packet, not a real counter value.

    A row is only discarded if BOTH the jump into it and the jump out of
    it are implausible, AND skipping the row entirely restores a plausible
    transition between its neighbors - this specifically targets isolated
    single-row spikes and will not remove genuine (if unusual) data.
    """
    try:
        time_col = header.index("ts_start_bit_s")
        low_col = header.index("byte2")
        high_col = header.index("byte3")
    except ValueError:
        return decoder  # older/unrecognized schema - nothing to check against

    order = np.argsort(decoder[:, time_col], kind="stable")
    counter = decoder[order, low_col].astype(np.int64) + decoder[order, high_col].astype(np.int64) * 256

    def is_valid_step(d):
        # Forward progress of a plausible size, or a genuine 16-bit wrap.
        return (0 < d <= MAX_PLAUSIBLE_COUNTER_GAP) or (d < -60000)

    n = len(counter)
    spike = np.zeros(n, dtype=bool)
    for i in range(1, n - 1):
        d_in = counter[i] - counter[i - 1]
        d_out = counter[i + 1] - counter[i]
        d_skip = counter[i + 1] - counter[i - 1]
        if not is_valid_step(d_in) and not is_valid_step(d_out) and is_valid_step(d_skip):
            spike[i] = True

    if not spike.any():
        return decoder

    keep_original_order = np.zeros(len(decoder), dtype=bool)
    keep_original_order[order[~spike]] = True
    return decoder[keep_original_order]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--decoder_directory", help="Directory of decoded csv files.")
    parser.add_argument("--log_directory", help="Filepath of log file.")
    parser.add_argument("--output", help="Output HDF5 filepath.")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="Enable verbose output (increasing)")

    args = parser.parse_args()

    #----------------------------------#
    #-----       Parameters       -----#
    #----------------------------------#
    RATIO_THRESHOLD = 0.75
    EXPECTED_KEYS = [int(i) for i in range(24)]

    #----------------------------------#
    #-----       Load Data        -----#
    #----------------------------------#
    if (args.verbose >= 2):
        print(f"Decoder Directory:    {args.decoder_directory}")
        print(f"Log Directory:    {args.log_directory}")

    # Make sure the directory exists and that there are files in it
    if not os.path.exists(args.decoder_directory) :
        raise RuntimeError(f"Input decoder_directory doesn't exist: \t {args.decoder_directory}")
    # if not os.path.exists(args.log_directory+".bees"):
    if not os.path.exists(args.log_directory):
        raise RuntimeError(f"Input log_directory doesn't exist: \t {args.log_directory}")

    # Get valid filenames (.csv)
    input_path = Path(args.decoder_directory)
    files = [f for f in input_path.iterdir() if f.is_file() and f.suffix == ".csv"]


    if len(files) == 0:
        raise RuntimeError("Input directory is empty.")

    try:
        tracker_log = read_log(args.log_directory)
    except:
        raise RuntimeError("Couldn't open tracker log file.")
    
    ## TODO -> Check that this is correct
    # Make sure there are the same number of files as track_id's in the log
    # if len(np.unique(tracker_log["track_id"])) != len(files):
    #     print(f"Number of tracker_log ID's:\t{len(np.unique(tracker_log["track_id"]))}")
    #     print(f"Number of decoder files:\t{len(files)}")
    #     raise RuntimeError("Number of tracking ID's does not match number of decoded files.")


    #----------------------------------#
    #-----    Process Log Data    -----#
    #----------------------------------#
    # Separate the combined data log into each file (ID in AEMOT)
    track_id = tracker_log["track_id"]
    track_x_hat = np.hstack([tracker_log["ts"].reshape(-1,1), tracker_log["x_hat"]])

    tracker_log_id = defaultdict(np.ndarray)
    for id in np.unique(track_id):
        # Get the tracker timestamp and position associated with that file ID 
        tracker_log_id[id] = track_x_hat[(track_id == id),0:3]

    validated_AEMOT_IDs = np.unique(track_id)


    #----------------------------------#
    #-----      Filter Data       -----#
    #----------------------------------#
    file_id     = defaultdict(list)
    decoded_id  = defaultdict(list)

    discarded_file_id = defaultdict(list)
    discarded_decoded_id = defaultdict(list)
    decoder_header = None

    for i, f in enumerate(files):
        file_id_number = int(f.stem.split("_")[-2])

        if file_id_number not in validated_AEMOT_IDs:
            if (args.verbose >= 2): print(f"Skipping file {i} - not associated with a validated track.")
            continue

        # Check if there is actually data
        with open(f) as fp:
            next(fp, None)
            has_data = any(line.strip() for line in fp)
        if not has_data:
            if (args.verbose >= 2): print(f"File {f} discarded - no data")
            continue

        with open(f, newline="") as fp:
            header = next(fp).strip().split(",")
        if decoder_header is None:
            decoder_header = header
        elif header != decoder_header:
            raise RuntimeError(
                f"Decoded CSV schemas do not match. Expected {decoder_header}, "
                f"but {f} contains {header}"
            )

        data_id = np.loadtxt(f, skiprows=1, delimiter=",")
        if (data_id.ndim == 1) or (data_id.shape[0] <= 5):
            if (args.verbose >= 2): print(f"File {f} discarded - no data")
            continue

        # Find the beacon-ID column by name. This remains correct after
        # ts_start_bit_s was inserted between t_us and byte1.
        try:
            beacon_id_column = header.index("byte1")
        except ValueError:
            raise RuntimeError(f"Decoded file {f} has no byte1 beacon-ID column: {header}")

        values, counts = np.unique(data_id[:, beacon_id_column], return_counts=True)
        ratios = np.sort(counts / len(data_id))[::-1]
        mode = values[np.argmax(counts)].astype(int)

        # If there is only 1 ID, add it to that file.
        if (len(values) == 1) or ((ratios[0] - ratios[1]) > RATIO_THRESHOLD):
            data_id[:,0] *= 1e-6 # convert time to seconds
            # Only keep rows whose OWN decoded byte1 actually matches the
            # mode - a track being unanimous/dominant enough to accept does
            # NOT mean every row in it decoded correctly. A misdecoded
            # packet corrupts its whole payload (byte1, byte2, byte3, and
            # the timestamp columns together), not just byte1 in isolation
            # - these rows are garbage, not just "slightly off", and were
            # previously being kept and passed downstream unfiltered
            # (matched against another camera's genuine counter values in
            # stereo/time_sync.py, contributing spurious noise to that fit).
            row_mask = data_id[:, beacon_id_column] == mode
            n_discarded = len(data_id) - int(row_mask.sum())
            if n_discarded > 0 and (args.verbose >= 2):
                print(
                    f"File {f}: discarded {n_discarded}/{len(data_id)} row(s) "
                    f"with byte1 != mode ({mode}) - misdecoded packets, not "
                    "genuine minority beacon IDs."
                )
            data_id = data_id[row_mask]
            decoded_id[mode].append(data_id)
            file_id[mode].append(tracker_log_id[file_id_number])
            continue

        if (args.verbose >= 2): 
            idx = np.argsort(counts)[::-1]
            print(f"File {f} discarded - no unanimous ID: \t {len(values)} decoded ID's \n\t {values[idx][0 : min([10,len(idx)])]}")
            data_id[:,0] *= 1e-6 # convert time to seconds
            discarded_decoded_id[mode].append(data_id)
            discarded_file_id[mode].append(tracker_log_id[file_id_number])


    if not decoded_id:
        raise RuntimeError(
            "No decoded beacon tracks survived filtering. Check the decoder CSV schema "
            "and tracker/decoder IDs before calibrating."
        )

    missing_ID = [x for x in EXPECTED_KEYS if x not in decoded_id.keys()]
    if (args.verbose >= 1 ): print("Detected ID's:\t", list(decoded_id.keys()),"\nMising ID's:\t", missing_ID)

    for id in discarded_decoded_id.keys():
        print(f"Mode of discarded files: {id}")
        if id in missing_ID:
            print(f"Mode of discarded [{mode}] is in missing ID list.")


    #----------------------------------#
    #-----      Write to HDF      -----#
    #----------------------------------#
    output_filepath = args.output or f"./track_logs/{Path(args.log_directory).stem}.h5"
    if (args.verbose >= 1 ): print(f"Writing output to:   {output_filepath}")
    with h5py.File(output_filepath, "w") as f:  
        for key in decoded_id.keys():
            decoder = np.vstack(decoded_id[key])
            decoder = decoder[decoder[:,0].argsort(),:]
            decoder = remove_counter_spikes(decoder, decoder_header)
            tracker = np.vstack(file_id[key])
            tracker = tracker[tracker[:,0].argsort(),:]

            group = f.create_group(str(key))
            decoded_dataset = group.create_dataset("decoded", data=decoder)
            tracker_dataset = group.create_dataset("tracker", data=tracker)

            # Store column meanings with the H5 file so downstream code does
            # not have to guess after the decoder CSV schema changes.
            decoded_dataset.attrs["columns"] = np.asarray(decoder_header, dtype="S")
            tracker_dataset.attrs["columns"] = np.asarray(
                ["time_s", "x_px", "y_px"], dtype="S"
            )
            tracker_dataset.attrs["time_origin"] = (
                "seconds relative to the first event processed by the tracker"
            )
