import h5py 
import os
import argparse


if __name__ == "__main__":
    with h5py.File("./track_logs/11-09-2026-09-36-33.h5", "r") as f:
        print(list(f.keys()))

        for key in f:
            print("Group:", key)
            print("  decoded:", f[key]["decoded"].shape)
            print("  tracker:", f[key]["tracker"].shape)
            # print("  decoded data:", f[key]["decoded"][:,0])
            # print("  tracker data:", f[key]["tracker"][:,0])