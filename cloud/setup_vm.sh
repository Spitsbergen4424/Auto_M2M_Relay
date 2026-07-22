#!/usr/bin/env bash
# Provisions a fresh Ubuntu VM (Yandex Cloud or any provider) with the exact
# ML-Agents 1.1.0 / Python 3.10.12 stack from Manual/P1. CPU-only PyTorch:
# this workload is Unity-physics bound, a GPU would sit mostly idle.
# Idempotent enough to re-run. Usage:  bash setup_vm.sh
set -euo pipefail

MINICONDA_DIR="$HOME/miniconda3"
ENV_NAME="mlagents"

echo "=== [1/5] System packages ==="
sudo apt-get update -y
# Headless Unity players need these shared libs even with --no-graphics.
sudo apt-get install -y wget bzip2 tmux libgtk-3-0 libnss3 libxss1 \
    libasound2t64 libglu1-mesa 2>/dev/null || \
sudo apt-get install -y wget bzip2 tmux libgtk-3-0 libnss3 libxss1 \
    libasound2 libglu1-mesa

echo "=== [2/5] Miniconda ==="
if [ ! -d "$MINICONDA_DIR" ]; then
    wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
    bash /tmp/miniconda.sh -b -p "$MINICONDA_DIR"
    rm /tmp/miniconda.sh
fi
# shellcheck disable=SC1091
source "$MINICONDA_DIR/etc/profile.d/conda.sh"

echo "=== [3/5] Python env ($ENV_NAME) ==="
if ! conda env list | grep -q "^$ENV_NAME "; then
    conda create -n "$ENV_NAME" python=3.10.12 -y
fi
conda activate "$ENV_NAME"

echo "=== [4/5] PyTorch (CPU) + grpcio ==="
pip3 install "torch~=2.2.1" --index-url https://download.pytorch.org/whl/cpu
# Prebuilt grpcio avoids the C++ build failure documented in Manual/P1 step 5.
conda install "grpcio=1.48.2" -c conda-forge -y

echo "=== [5/5] ML-Agents ==="
python -m pip install mlagents==1.1.0
# pkg_resources was dropped from modern setuptools; ml-agents 1.1.0 still needs it.
pip install "setuptools<70"

echo
echo "Verifying..."
python -c "import torch; print('torch', torch.__version__)"
mlagents-learn --help >/dev/null && echo "mlagents-learn OK"
echo
echo "Done. Activate later with:  source $MINICONDA_DIR/etc/profile.d/conda.sh && conda activate $ENV_NAME"
