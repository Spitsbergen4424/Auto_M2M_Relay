#!/usr/bin/env bash
# Launches (or resumes) headless ML-Agents training inside tmux so it survives
# SSH disconnects. Run ON the VM from the folder holding config.yaml and the
# Linux build. Usage:
#   bash run_training.sh <run_id> [path/to/env.x86_64] [config.yaml]
set -euo pipefail

RUN_ID="${1:?Usage: run_training.sh <run_id> [env] [config]}"
ENV_PATH="${2:-./GFSX_Simulator.x86_64}"
CONFIG="${3:-./config.yaml}"
SESSION="train_${RUN_ID}"
MINICONDA_DIR="$HOME/miniconda3"

[ -f "$CONFIG" ] || { echo "Config not found: $CONFIG"; exit 1; }
[ -f "$ENV_PATH" ] || { echo "Env build not found: $ENV_PATH"; exit 1; }
chmod +x "$ENV_PATH"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already running. Attach with: tmux attach -t $SESSION"
    exit 0
fi

# --resume if a prior checkpoint for this run_id exists, otherwise fresh.
RESUME=""
[ -d "results/$RUN_ID" ] && RESUME="--resume" && echo "Existing results/$RUN_ID -> resuming."

# A regular Linux Player build still initializes a graphics device and segfaults
# on a headless VM (no GPU, no X). Wrap it in a virtual framebuffer so the env
# process the trainer spawns has a DISPLAY to bind to.
XVFB=""
if command -v xvfb-run >/dev/null; then
    XVFB="xvfb-run -a"
else
    echo "WARNING: xvfb-run not found; a Player build will likely SIGSEGV headless."
fi

# num-envs stays 1: the scene already holds 40 agents, and the staged curriculum
# keeps its state per-process (see SuccessGatedCurriculum).
CMD="source $MINICONDA_DIR/etc/profile.d/conda.sh && conda activate mlagents && \
$XVFB mlagents-learn '$CONFIG' --run-id='$RUN_ID' --env='$ENV_PATH' \
--num-envs=1 --no-graphics $RESUME"

tmux new-session -d -s "$SESSION" "$CMD; echo; echo '=== training exited, press enter ==='; read"
echo "Started in tmux session '$SESSION'."
echo "  Watch live : tmux attach -t $SESSION   (detach: Ctrl-b then d)"
echo "  TensorBoard: tensorboard --logdir results --host 0.0.0.0 --port 6006"
