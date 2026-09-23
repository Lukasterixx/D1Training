"""Write the `run.json` that `dashboard.py record` reads, for a UniFP run directory.

UniFP's runner writes only TensorBoard events and checkpoints. `evidence/record.py` will record
a run from events alone, but then the record carries no seed, no environment count, no commit
and no robot mass -- which is most of what makes a run comparable to another one later.
This writes the same fields `run_position_only.py` writes on the Isaac Lab side, so runs from
the two stacks line up in the dashboard.

Used by `launch_training.py`; also runnable on its own to backfill a directory:

    python run_metadata.py <run dir> --mode train --num-envs 4096 --iterations 60000 \
        --status finished --notes "..."
"""

import argparse
import json
import os
import subprocess
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git(*args, default=""):
    try:
        return subprocess.check_output(["git", "-C", REPO, *args], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except Exception:
        return default


def urdf_mass(urdf_path):
    """Nominal articulation mass: every `<inertial>` in the asset, before randomisation.

    Comparable with `articulation_mass_kg` on the Isaac Lab side, which is where the 18.171 kg
    cross-check between the two stacks comes from.
    """
    if not os.path.exists(urdf_path):
        return None
    root = ET.parse(urdf_path).getroot()
    return round(sum(float(link.find("inertial").find("mass").get("value"))
                     for link in root.findall("link") if link.find("inertial") is not None), 6)


def write(run_dir, mode="train", seed=None, status="running", error=None, num_envs=None,
          iterations=None, steps=None, urdf=None, notes="", command=None, extra=None):
    meta = {
        "mode": mode,
        "seed": seed,
        "status": status,
        "error": error,
        "arguments": {"num_envs": num_envs, "iterations": iterations, "steps": steps},
        "git_commit": _git("rev-parse", "HEAD"),
        "git_status": _git("status", "--porcelain"),
        "articulation_mass_kg": urdf_mass(urdf) if urdf else None,
        "stack": "isaacgym_preview4/unifp",
        "unifp_commit": "68847a070f88d731058c3d8476929bc3b205f5bd",
        "command": command or " ".join(os.sys.argv),
        "notes": notes,
    }
    meta.update(extra or {})
    # The runner only creates its log directory when the SummaryWriter opens, inside learn(),
    # so at launch time it does not exist yet.
    os.makedirs(run_dir, exist_ok=True)
    path = os.path.join(run_dir, "run.json")
    with open(path, "w") as handle:
        json.dump(meta, handle, indent=2)
        handle.write("\n")
    return path


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dir")
    p.add_argument("--mode", default="train")
    p.add_argument("--seed", type=int)
    p.add_argument("--status", default="finished")
    p.add_argument("--error")
    p.add_argument("--num-envs", type=int)
    p.add_argument("--iterations", type=int)
    p.add_argument("--urdf", default=os.path.expanduser(
        "~/thesis_b_legacy/UniFP/resources/robots/go2d1/go2d1.urdf"))
    p.add_argument("--notes", default="")
    p.add_argument("--command", default="")
    a = p.parse_args()
    print(write(a.run_dir, mode=a.mode, seed=a.seed, status=a.status, error=a.error,
                num_envs=a.num_envs, iterations=a.iterations, urdf=a.urdf, notes=a.notes,
                command=a.command or None))


if __name__ == "__main__":
    main()
