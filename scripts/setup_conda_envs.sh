#!/usr/bin/env bash
#
# Create the two Python environments this repo uses, as conda environments:
#
#   chatscene-v2/   Scenic 2.x (editable ./Scenic) - route-driven train_scenario / eval
#                   (scenic_runner_v2.py, run_eval_v2*.py, generate_scenic_route_pickle_v2.py)
#   chatscene-v3/   Scenic 3.x (PyPI) - route generation (generate_scenic_route_pickle.py)
#
# Both are created as conda prefix envs inside the repo root, at the same paths
# the old venvs used, and each gets a small bin/activate shim, so env.scenic2.sh /
# env.scenic3.sh keep working unchanged:
#
#   source ~/yungloon/ChatScene-main/chatscene-v2/bin/activate   # == conda activate <repo>/chatscene-v2
#
# Each env gets:
#   - python 3.10 plus the conda packages jpeg=9e and libtiff=4.5.1. CARLA 0.9.15's
#     libcarla links against libtiff.so.5, which Ubuntu 24.04 no longer ships;
#     <env>/carla_compat_libs/ symlinks those two libraries and the env scripts put
#     it on LD_LIBRARY_PATH.
#   - the pinned packages in requirements_v2.txt / requirements_v3.txt (torch from
#     the PyTorch index: cu128 for v2, cu117 for v3)
#   - the CARLA 0.9.15 wheel from $CARLA_ROOT/PythonAPI/carla/dist (falls back to
#     `pip install carla==0.9.15` from PyPI if that wheel isn't there)
#   - safebench (this repo) installed editable, and for v2 the repo's Scenic/ editable
#
# Usage:
#   scripts/setup_conda_envs.sh [v2] [v3] [options]
#
# With no env names, both v2 and v3 are created.
#
# Options:
#   --carla-root PATH   CARLA install whose PythonAPI wheel to use
#                       (default: $CARLA_ROOT, else ~/yungloon/fail2drive/f2d_carla)
#   --force             Delete and recreate an env directory that already exists
#                       (by default an existing chatscene-v2/ or chatscene-v3/ is
#                       left alone and the script stops)
#
# Examples:
#   scripts/setup_conda_envs.sh                                  # both envs
#   scripts/setup_conda_envs.sh v3 --carla-root /opt/CARLA_0.9.15
#
# Afterwards, check the paths in env.scenic2.sh / env.scenic3.sh (CARLA_ROOT in
# particular) match this machine, then e.g.:
#   source env.scenic3.sh && python -c "import carla, scenic, torch"

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Anaconda's main channel: the jpeg/libtiff builds there are the ones the CARLA
# compat libs were taken from (libtiff 4.5.1 linked against libjpeg.so.9).
CONDA_CHANNEL="https://repo.anaconda.com/pkgs/main"
PYTHON_VERSION="3.10"

ENVS=()
CARLA_ROOT_ARG="${CARLA_ROOT:-$HOME/yungloon/fail2drive/f2d_carla}"
FORCE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    v2|v3) ENVS+=("$1"); shift ;;
    --carla-root) CARLA_ROOT_ARG="$2"; shift 2 ;;
    --force) FORCE=1; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ ${#ENVS[@]} -eq 0 ]] && ENVS=(v2 v3)

CONDA_BIN="${CONDA_EXE:-$(command -v conda || true)}"
if [[ -z "$CONDA_BIN" ]]; then
  echo "conda not found: install Miniconda/Anaconda or put conda on PATH." >&2
  exit 1
fi

log() { echo -e "\n>>> $*"; }

setup_env() {
  local v="$1"
  local prefix="$REPO_ROOT/chatscene-$v"
  local req="$REPO_ROOT/requirements_$v.txt"
  local py="$prefix/bin/python"

  if [[ -e "$prefix" ]]; then
    if [[ "$FORCE" -eq 1 ]]; then
      log "[$v] --force: removing existing $prefix"
      rm -rf "$prefix"
    else
      echo "$prefix already exists. Remove it or re-run with --force to recreate it." >&2
      exit 1
    fi
  fi

  log "[$v] Creating conda env at $prefix"
  "$CONDA_BIN" create -y -p "$prefix" --override-channels -c "$CONDA_CHANNEL" \
    "python=$PYTHON_VERSION" pip "jpeg=9e" "libtiff=4.5.1"

  log "[$v] Linking CARLA compat libs (libtiff.so.5, libjpeg.so.9)"
  mkdir -p "$prefix/carla_compat_libs"
  ln -sf ../lib/libtiff.so.5 "$prefix/carla_compat_libs/libtiff.so.5"
  ln -sf ../lib/libjpeg.so.9 "$prefix/carla_compat_libs/libjpeg.so.9"

  # --no-deps: the requirements files are complete freezes of the working venvs,
  # which each carry one declared-but-harmless conflict pip's resolver rejects
  # (v2: moviepy wants decorator<5; v3: verifai wants scenic>=3.1.1).
  log "[$v] Installing pinned requirements from $(basename "$req")"
  "$py" -m pip install --upgrade pip
  "$py" -m pip install --no-deps -r "$req"

  log "[$v] Installing CARLA 0.9.15 Python API"
  local wheel
  wheel="$(ls "$CARLA_ROOT_ARG"/PythonAPI/carla/dist/carla-0.9.15-cp310-*.whl 2>/dev/null | head -1 || true)"
  if [[ -n "$wheel" ]]; then
    "$py" -m pip install --no-deps "$wheel"
  else
    echo "No CARLA wheel under $CARLA_ROOT_ARG/PythonAPI/carla/dist; using carla==0.9.15 from PyPI."
    "$py" -m pip install --no-deps "carla==0.9.15"
  fi

  if [[ "$v" == "v2" ]]; then
    log "[$v] Installing Scenic 2.x (editable, $REPO_ROOT/Scenic)"
    "$py" -m pip install --no-deps -e "$REPO_ROOT/Scenic"
  fi

  log "[$v] Installing safebench (editable, $REPO_ROOT)"
  "$py" -m pip install --no-deps -e "$REPO_ROOT"

  log "[$v] Writing $prefix/bin/activate shim"
  cat > "$prefix/bin/activate" <<EOF
# Generated by scripts/setup_conda_envs.sh so that \`source <env>/bin/activate\`
# (as used by env.scenic2.sh / env.scenic3.sh) activates this conda env.
eval "\$('$CONDA_BIN' shell.bash hook)"
conda activate '$prefix'
EOF

  log "[$v] Verifying imports"
  LD_LIBRARY_PATH="$prefix/carla_compat_libs:${LD_LIBRARY_PATH:-}" PYTHONPATH="$REPO_ROOT" \
    "$py" -c "import carla, scenic, torch, safebench; print('carla OK | scenic', scenic.__version__ if hasattr(scenic, '__version__') else '', '| torch', torch.__version__, '| cuda', torch.cuda.is_available())"
}

for v in "${ENVS[@]}"; do
  setup_env "$v"
done

log "Done: ${ENVS[*]}. Activate with: source env.scenic2.sh  /  source env.scenic3.sh"
