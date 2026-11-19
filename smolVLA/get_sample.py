#!/usr/bin/env python3
"""Save one frame of a LeRobot dataset as inputs for run_smolvla.py.

Downloads a single episode (not the whole dataset), then writes each camera image
as a PNG, plus the joint state, the task and the recorded actions that followed,
and prints the run_smolvla.py command that uses them.

    python get_sample.py                                     # lerobot/svla_so101_pickplace, episode 0, frame 0
    python get_sample.py --episode 3 --frame 120 --out sample3

Exit codes: 0 ok, 1 bad arguments, 2 the dataset failed to load.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path

import numpy as np

from lerobot.utils.constants import ACTION, OBS_STATE

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as err:
    print(f"error: {err}\nInstall the dataset support with:  pip install \"lerobot[dataset]>=0.7\"",
          file=sys.stderr)
    sys.exit(1)

DEFAULT_DATASET = "lerobot/svla_so101_pickplace"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", default=DEFAULT_DATASET,
                   help=f"Hugging Face dataset repo id (default {DEFAULT_DATASET})")
    p.add_argument("--root", default=None, help="local dataset directory instead of the download cache")
    p.add_argument("--episode", type=int, default=0, help="episode index (default 0)")
    p.add_argument("--frame", type=int, default=0, help="frame index within the episode (default 0)")
    p.add_argument("--out", default="sample", help="output directory (default ./sample)")
    p.add_argument("--actions", type=int, default=50,
                   help="how many recorded actions to save from this frame on (default 50, one chunk)")
    return p.parse_args()


def to_uint8_hwc(image) -> np.ndarray:
    """Dataset images come as float CxHxW in [0, 1]; PNG wants uint8 HxWxC."""
    arr = image.numpy() if hasattr(image, "numpy") else np.asarray(image)
    if arr.ndim == 3 and arr.shape[0] in (1, 3):
        arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        arr = np.clip(arr * 255.0 + 0.5, 0, 255).astype(np.uint8)
    return arr


def main() -> int:
    args = parse_args()
    if args.episode < 0 or args.frame < 0 or args.actions < 1:
        print("error: --episode and --frame must be >= 0 and --actions >= 1", file=sys.stderr)
        return 1

    print(f"loading episode {args.episode} of {args.dataset} ...")
    try:
        ds = LeRobotDataset(args.dataset, root=args.root, episodes=[args.episode])
    except Exception as err:  # third-party loaders report every failure by raising
        print(f"error: cannot load {args.dataset}: {type(err).__name__}: {err}", file=sys.stderr)
        return 2

    length = len(ds)
    print(f"{ds.meta.total_episodes} episodes in the dataset, this one has {length} frames at {ds.fps} fps, "
          f"robot {ds.meta.robot_type}")
    if args.frame >= length:
        print(f"error: frame {args.frame} is past the end of the episode ({length} frames)", file=sys.stderr)
        return 1

    item = ds[args.frame]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    from PIL import Image  # installed with lerobot

    command = ["python", "run_smolvla.py"]
    for key in ds.meta.camera_keys:
        name = key.rsplit(".", 1)[-1]
        path = out / f"{name}.png"
        img = to_uint8_hwc(item[key])
        Image.fromarray(img).save(path)
        command += ["--image", f"{name}={path}"]
        print(f"  {key:32s} -> {path} ({img.shape[1]}x{img.shape[0]})")

    state = item[OBS_STATE].numpy().astype(np.float64)
    names = ds.meta.features[OBS_STATE].get("names")
    np.savetxt(out / "state.txt", state[None], fmt="%.4f")
    print(f"  state {names}:\n    {np.array2string(state, precision=3, suppress_small=True)}")

    task = item.get("task", "")
    (out / "task.txt").write_text(task + "\n")
    print(f"  task: {task!r}")

    # The actions recorded from this frame on: what a good policy should roughly predict.
    last = min(args.frame + args.actions, length)
    recorded = np.stack([ds[i][ACTION].numpy() for i in range(args.frame, last)])
    np.savetxt(out / "actions.txt", recorded, fmt="%.4f")
    np.set_printoptions(precision=3, suppress=True, linewidth=120)
    print(f"  recorded actions {recorded.shape} saved to {out / 'actions.txt'}; first 5:")
    print(recorded[:5])

    command += ["--state", *(f"{v:.4f}" for v in state), "--task", task, "--compare", str(out / "actions.txt")]
    print("\nrun it with:\n  " + shlex.join(command))
    return 0


if __name__ == "__main__":
    sys.exit(main())