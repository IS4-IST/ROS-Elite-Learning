# run_smolvla.py

Loads a SmolVLA checkpoint and runs it once on an observation, without ROS or the twin.
Use it to check that the model loads on your machine, see what inputs it expects, and
measure how long one inference takes.

## Setup

Python 3.12, in a virtual environment:

    python3 -m venv ~/venvs/smolvla
    source ~/venvs/smolvla/bin/activate
    pip install "lerobot[smolvla]>=0.7"

The first run downloads the checkpoint and the SmolVLM2 backbone from Hugging Face
(about 2 GB together) into `~/.cache/huggingface`.

## Usage

    python run_smolvla.py
    python run_smolvla.py --image scene=scene.png --image wrist=wrist.png \
        --state 0 -30 90 0 90 0 --task "pick up the red cube"
    python run_smolvla.py --policy outputs/train/ec66/checkpoints/last/pretrained_model

| Option | Meaning |
|---|---|
| `--policy` | Hugging Face repo id or local checkpoint directory (default `lerobot/smolvla_base`) |
| `--device` | `cuda`, `cpu` or `mps` (default: `cuda` if available) |
| `--task` | the language instruction |
| `--image NAME=FILE` | an image for one camera, repeated per camera; without any, synthetic images are used |
| `--map NAME=KEY` | send camera `NAME` to policy input `KEY`; unmapped cameras fill the inputs in order |
| `--state` | the joint state, in the units the policy was trained with (default zeros) |
| `--repeat` | timed inferences after the warm-up one (default 3) |
| `--seed` | seed for the sampling noise |

It prints the policy's inputs and outputs, which camera went to which input, the
warm-up and steady inference times, and the first five steps of the action chunk.
Exit codes: 0 ok, 1 bad arguments or inputs, 2 the checkpoint didn't load.

## What to expect

- `lerobot/smolvla_base` is the pretrained base model, not a policy for the EC66. It loads
  and runs, but its actions mean nothing for this arm until it is fine-tuned on EC66
  demonstrations. Use it to check the setup and timing.
- The input names (`observation.images.camera1`, ...) and the state size come from the
  checkpoint. A fine-tune on your own dataset will use your dataset's names instead.
- Missing cameras are allowed (SmolVLA skips absent image inputs) as long as there is at least one.
- Each call returns a chunk of 50 actions; at 30 fps that covers about 1.7 s.
