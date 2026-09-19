"""Pick the checkpoint a resume should start from, across every run directory of an experiment.

A supervised run is a chain of directories -- each resume makes a new one -- so "the latest
checkpoint" is not "the latest checkpoint in the latest directory". UniFP's own
`get_load_path(load_run=-1)` takes the alphabetically last directory, which is the wrong one
whenever a restart died before saving anything (and, as its own TODO says, whenever the month
changes). This scans them all and orders by the iteration number in the filename, which is
globally increasing because `current_learning_iteration` is restored on load.

Prints "<run dir name> <iteration>" for the newest, or nothing if there is none.
`--skip N` returns the Nth-newest instead, so a supervisor can fall back past a checkpoint that
was being written when the process died.
"""

import argparse
import os
import re


def checkpoints(log_root):
    """[(iteration, run dir name)] for every model file under `log_root`, newest first."""
    found = []
    if not os.path.isdir(log_root):
        return found
    for run in os.listdir(log_root):
        run_path = os.path.join(log_root, run)
        if not os.path.isdir(run_path):
            continue
        for name in os.listdir(run_path):
            match = re.fullmatch(r"model_(\d+)\.pt", name)
            if match and os.path.getsize(os.path.join(run_path, name)) > 0:
                found.append((int(match.group(1)), run))
    return sorted(found, reverse=True)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log_root")
    p.add_argument("--skip", type=int, default=0)
    a = p.parse_args()
    found = checkpoints(a.log_root)
    if a.skip < len(found):
        iteration, run = found[a.skip]
        print(f"{run} {iteration}")


if __name__ == "__main__":
    main()
