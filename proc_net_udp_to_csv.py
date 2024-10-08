#!/usr/bin/python3

import os
import re
import sys

COLUMNS = [
    "sl",
    "local_address",
    "local_port",
    "rem_address",
    "rem_port",
    "st",
    "tx_queue",
    "rx_queue",
    "tr",
    "tm->when",
    "retrnsmt",
    "uid",
    "timeout",
    "inode",
    "ref",
    "pointer",
    "drops",
]


def convert_line(line: str):
    split = re.split(r"[:\s]+", line.strip())
    if len(split) != len(COLUMNS):
        raise ValueError("Length mismatch")
    split = [str(int(value, 16)) for value in split]
    return split


def convert(src_filename: str, dst_filename: str):
    with open(src_filename, "r") as f:
        lines = f.readlines()

    rows = [convert_line(line) for line in lines]

    with open(dst_filename, "w") as f:
        f.write(",".join(COLUMNS) + "\n")
        for row in rows:
            f.write(",".join(row) + "\n")


if __name__ == "__main__":
    args = sys.argv
    if len(args) == 1:
        print(f"Usage: {args[0]} [file1 [file2 ...]]")
        sys.exit(1)

    files_to_process = args[1:]
    for filename in files_to_process:
        out_filename = os.path.splitext(filename)[0] + ".csv"
        convert(filename, out_filename)
