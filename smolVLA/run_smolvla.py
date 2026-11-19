#!/usr/bin/env python3
"""Load SmolVLA and run it on one observation.

Standalone: it needs only LeRobot (with the smolvla extra), not ROS or the twin.
It loads a checkpoint, prints what the policy expects, builds an observation
from image files and a joint vector (or synthetic ones), and prints the action
chunk the policy returns, with timings.

    python run_smolvla.py                                   # lerobot/smolvla_base, synthetic inputs
    python run_smolvla.py --image scene=scene.png --image wrist=wrist.png \
        --state 0 -30 90 0 90 0 --task "pick up the red cube"
    python run_smolvla.py ... --compare sample/actions.txt        # vs what was recorded
    python run_smolvla.py --policy ./outputs/train/ec66/checkpoints/last/pretrained_model

Exit codes: 0 ok, 1 bad arguments or inputs, 2 the checkpoint failed to load.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import torch

from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.utils.constants import ACTION, OBS_STATE

DEFAULT_POLICY = "lerobot/smolvla_base"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--policy", default=DEFAULT_POLICY,
                   help=f"Hugging Face repo id or local checkpoint directory (default {DEFAULT_POLICY})")
    p.add_argument("--device", default=None, help="cuda (also AMD ROCm), xpu (Intel), mps or cpu (default: the first available)")
    p.add_argument("--task", default="pick up the cube", help="language instruction")
    p.add_argument("--image", action="append", default=[], metavar="NAME=FILE",
                   help="an image for one camera; repeat for each camera. Without any, synthetic images are used")
    p.add_argument("--map", action="append", default=[], metavar="NAME=KEY",
                   help="send camera NAME to policy input KEY (e.g. scene=observation.images.camera1). "
                        "Unmapped cameras fill the policy's image inputs in order")
    p.add_argument("--state", type=float, nargs="+", default=None,
                   help="joint state, in the units the policy was trained with (default: zeros)")
    p.add_argument("--repeat", type=int, default=3, help="timed inferences after the warm-up one (default 3)")
    p.add_argument("--compare", default=None, metavar="FILE",
                   help="recorded actions (one per line, e.g. actions.txt from get_sample.py) "
                        "to compare the predicted chunk against")
    p.add_argument("--seed", type=int, default=0, help="seed for the flow-matching noise (default 0)")
    return p.parse_args()


def pick_device(name: str | None) -> torch.device:
    if name:
        return torch.device(name)
    if torch.cuda.is_available():  # NVIDIA (CUDA) and AMD (ROCm) builds both report "cuda"
        return torch.device("cuda")
    if hasattr(torch, "xpu") and torch.xpu.is_available():  # Intel
        return torch.device("xpu")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def synchronize(device: torch.device) -> None:
    """Wait for queued GPU work, so the timings measure the inference itself."""
    if device.type == "cuda":
        torch.cuda.synchronize()
    elif device.type == "xpu":
        torch.xpu.synchronize()


def split_pairs(items: list[str], what: str) -> dict[str, str] | None:
    pairs = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep or not name or not value:
            print(f"error: {what} '{item}' is not NAME=VALUE", file=sys.stderr)
            return None
        pairs[name] = value
    return pairs


def load_image(path: str) -> np.ndarray | None:
    """Return an HxWx3 uint8 RGB array, or None when the file can't be read."""
    from PIL import Image  # installed with lerobot

    try:
        with Image.open(path) as img:
            return np.array(img.convert("RGB"), dtype=np.uint8)  # a writable copy
    except OSError as err:
        print(f"error: cannot read image {path}: {err}", file=sys.stderr)
        return None


def synthetic_image(shape: tuple[int, ...], index: int) -> np.ndarray:
    """A gradient so the image isn't constant; shape is the policy's (C, H, W)."""
    _, h, w = shape
    y, x = np.mgrid[0:h, 0:w]
    img = np.stack([(x * 255 // max(w - 1, 1)), (y * 255 // max(h - 1, 1)),
                    np.full_like(x, (60 * index) % 256)], axis=-1)
    return img.astype(np.uint8)


def describe(policy: SmolVLAPolicy) -> None:
    cfg = policy.config
    print("policy inputs:")
    for key, feat in cfg.input_features.items():
        print(f"  {key:40s} {feat.type.name:8s} {tuple(feat.shape)}")
    print("policy outputs:")
    for key, feat in cfg.output_features.items():
        print(f"  {key:40s} {feat.type.name:8s} {tuple(feat.shape)}")
    print(f"chunk: {cfg.chunk_size} steps predicted, {cfg.n_action_steps} executed before replanning")
    print(f"images resized (with padding) to {cfg.resize_imgs_with_padding}, "
          f"empty cameras: {cfg.empty_cameras}")


def main() -> int:
    args = parse_args()
    images = split_pairs(args.image, "--image")
    mapping = split_pairs(args.map, "--map")
    if images is None or mapping is None:
        return 1

    device = pick_device(args.device)
    print(f"loading {args.policy} on {device} ...")
    t0 = time.perf_counter()
    try:
        policy = SmolVLAPolicy.from_pretrained(args.policy)
        policy.to(device)
        policy.eval()
        # The processors hold the normalisation statistics and the tokenizer of this checkpoint.
        preprocess, postprocess = make_pre_post_processors(
            policy.config, args.policy,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
    except Exception as err:  # third-party loaders report every failure by raising
        print(f"error: cannot load {args.policy}: {type(err).__name__}: {err}", file=sys.stderr)
        return 2
    print(f"loaded in {time.perf_counter() - t0:.1f} s, "
          f"{sum(p.numel() for p in policy.parameters()) / 1e6:.0f} M parameters")
    describe(policy)

    cfg = policy.config
    image_keys = list(cfg.image_features)
    state_feat = cfg.input_features.get(OBS_STATE)
    action_dim = cfg.output_features[ACTION].shape[0]

    # --- images: explicit --map first, then the remaining cameras in order --------------
    observation: dict[str, np.ndarray] = {}
    if images:
        free_keys = [k for k in image_keys if k not in mapping.values()]
        for name, path in images.items():
            key = mapping.get(name)
            if key is None:
                if not free_keys:
                    print(f"error: no policy image input left for camera '{name}'", file=sys.stderr)
                    return 1
                key = free_keys.pop(0)
            if key not in image_keys:
                print(f"error: '{key}' is not an image input of this policy", file=sys.stderr)
                return 1
            img = load_image(path)
            if img is None:
                return 1
            observation[key] = img
            print(f"camera {name} -> {key} {img.shape[1]}x{img.shape[0]}")
        missing = [k for k in image_keys if k not in observation]
        if missing:
            # SmolVLA skips image inputs that are absent (at least one is required),
            # so fewer cameras than the checkpoint was trained with still runs; it just sees less.
            print(f"no image for {missing}: the policy will run without them")
    else:
        for i, key in enumerate(image_keys):
            observation[key] = synthetic_image(tuple(cfg.input_features[key].shape), i)
        print("using synthetic images")

    # --- state ---------------------------------------------------------------------------
    if state_feat is not None:
        dim = state_feat.shape[0]
        state = np.zeros(dim, dtype=np.float32) if args.state is None else np.asarray(args.state, np.float32)
        if state.shape[0] != dim:
            print(f"error: the policy expects a state of {dim} values, got {state.shape[0]}", file=sys.stderr)
            return 1
        observation[OBS_STATE] = state
    elif args.state is not None:
        print("note: this policy has no state input; --state ignored")

    # --- inference -----------------------------------------------------------------------
    torch.manual_seed(args.seed)
    frame = prepare_observation_for_inference(dict(observation), device, args.task, "ec66")
    batch = preprocess(frame)

    def infer() -> tuple[torch.Tensor, float]:
        policy.reset()
        synchronize(device)
        t0 = time.perf_counter()
        out = policy.predict_action_chunk(batch)  # (1, chunk_size, action_dim), normalised
        synchronize(device)
        return out, time.perf_counter() - t0

    chunk, first = infer()  # the first call includes one-off setup (kernels, caches)
    times = []
    for _ in range(args.repeat):
        chunk, dt = infer()
        times.append(dt)
    print(f"first inference: {1000 * first:.0f} ms (warm-up)")
    if times:
        print(f"inference: {1000 * min(times):.0f} ms best, {1000 * np.mean(times):.0f} ms mean "
              f"over {len(times)} runs")

    actions = postprocess(chunk)  # back to the units of the training data
    actions = actions.squeeze(0).cpu().numpy()
    if device.type == "cuda":
        print(f"gpu memory peak: {torch.cuda.max_memory_allocated() / 2**20:.0f} MiB")

    np.set_printoptions(precision=3, suppress=True, linewidth=120)
    print(f"action chunk {actions.shape} (first 5 steps):")
    print(actions[:5])
    if args.compare:
        try:
            recorded = np.loadtxt(args.compare, ndmin=2)
        except (OSError, ValueError) as err:
            print(f"error: cannot read {args.compare}: {err}", file=sys.stderr)
            return 1
        if recorded.shape[1] != actions.shape[1]:
            print(f"error: {args.compare} has {recorded.shape[1]} values per line, "
                  f"the policy outputs {actions.shape[1]}", file=sys.stderr)
            return 1
        n = min(len(recorded), len(actions))
        err = actions[:n] - recorded[:n]
        print(f"compared with {n} recorded actions from {args.compare}:")
        print(f"  rms error per joint: {np.sqrt(np.mean(err ** 2, axis=0))}")
        print(f"  first step error:    {err[0]}")
        print(f"  last step error:     {err[-1]}")
    if action_dim != actions.shape[1]:
        print(f"warning: output has {actions.shape[1]} values, config says {action_dim}")
    return 0


if __name__ == "__main__":
    sys.exit(main())