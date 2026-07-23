#!/usr/bin/env bash
#
# Run SafeBench's train_scenario (or eval) over one or more scenic bench
# configs, in small chunks of scenario ids, restarting the CARLA server
# between chunks.
#
# CARLA degrades (and eventually crashes / stops loading worlds) after many
# scenarios have been run back-to-back against one long-lived server process.
# scripts/run_eval_batch.py already isolates each scenario in its own Python
# subprocess, but the CARLA server on the other end is shared for the whole
# batch. This wrapper splits the discovered scenario ids into chunks of
# --chunk-size and kills/relaunches the server after every chunk, so no CARLA
# instance ever serves more than that many scenarios.
#
# It also owns the server's lifecycle end-to-end: if nothing is listening on
# --port when it starts, it launches CARLA itself.
#
# Usage:
#   scripts/run_train_scenario_batch.sh [config1.yaml config2.yaml ...] [options]
#
# With no positional configs, the five Chat2Scenic_results_kimi26_think_250
# route-driven modality configs are used.
#
# Examples:
#   # Everything (134 scenarios across 5 modalities), CARLA restart every 5
#   scripts/run_train_scenario_batch.sh
#
#   # Preview the chunking/commands without touching CARLA
#   scripts/run_train_scenario_batch.sh --dry-run
#
#   # One modality, a subset of ids
#   scripts/run_train_scenario_batch.sh \
#     eval_scenic_Chat2Scenic_kimi26_think_250_image-only_route_driven.yaml \
#     --scenario_range 1-5
#
# Each scenario runs under `timeout --scenario-timeout`. SafebenchScenicDataLoader.
# generate_scene() (safebench/scenario/scenario_data_loader.py) resamples in an
# UNBOUNDED while-loop until setSimulation() succeeds, so a scenario whose spawn
# point is permanently blocked ("Unable to spawn object unnamed Car") spins
# forever and stalls the whole batch. The timeout turns that into a recorded
# failure and moves on.
#
# Options:
#   --chunk-size N        Scenarios per CARLA lifetime (default: 5)
#   --scenario-timeout T  Per-scenario wall-clock limit, `timeout` syntax
#                         (default: 15m; 0 disables). A healthy scenario needs
#                         roughly 8-10min for sample_num=50, so keep headroom.
#   --skip-ids "..."      Ids to never run, space/comma separated. Bare "7"
#                         skips id 7 in every config; "image-only:7" skips it
#                         only in the config whose bench_id is image-only.
#   --mode NAME           run_eval.py mode (default: train_scenario)
#   --test_policy NAME    Ego policy (default: sac)
#   --agent_cfg NAME      Agent config (default: adv_scenic.yaml)
#   --route_id N          Route id (default: 0)
#   --device NAME         torch device (default: cpu)
#   --port N              CARLA RPC port (default: 2005)
#   --tm-port N           Traffic manager port (default: 8005)
#   --scenario_ids "..."  Space-separated subset of ids (applied to every config)
#   --scenario_range A-B  Inclusive id range (applied to every config)
#   --no-resume           Re-run scenarios that already have a scenario_<id>.json
#   --env-script PATH     Sourced before running (default: env.scenic3.sh)
#   --carla-root PATH     CARLA install dir (default: $HOME/yungloon/CARLA_0.9.15)
#   --log PATH            Log file (default: /tmp/train_scenario_batch_<timestamp>.log)
#   --dry-run             Print resolved chunks/commands, start nothing
#   -h, --help            Show this help and exit

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

CHUNK_SIZE=5
SCENARIO_TIMEOUT="15m"
SKIP_IDS=""
MODE="train_scenario"
TEST_POLICY="sac"
AGENT_CFG="adv_scenic.yaml"
ROUTE_ID=0
DEVICE="cpu"
PORT=2005
TM_PORT=8005
SCENARIO_IDS=""
SCENARIO_RANGE=""
RESUME=1
ENV_SCRIPT="env.scenic3.sh"
CARLA_ROOT="$HOME/yungloon/CARLA_0.9.15"
LOGFILE=""
DRY_RUN=0

DEFAULT_CONFIGS=(
  eval_scenic_Chat2Scenic_kimi26_think_250_image-only_route_driven.yaml
  eval_scenic_Chat2Scenic_kimi26_think_250_text-image_route_driven.yaml
  eval_scenic_Chat2Scenic_kimi26_think_250_text-only_route_driven.yaml
  eval_scenic_Chat2Scenic_kimi26_think_250_text-video_route_driven.yaml
  eval_scenic_Chat2Scenic_kimi26_think_250_video-only_route_driven.yaml
)

usage() {
  sed -n '2,50p' "${BASH_SOURCE[0]}"
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

# Positional config names come first, before any --options.
CONFIGS=()
while [ "$#" -gt 0 ] && [[ "$1" != --* ]]; do
  CONFIGS+=("$1")
  shift
done

while [ "$#" -gt 0 ]; do
  case "$1" in
    --chunk-size) CHUNK_SIZE="$2"; shift 2 ;;
    --scenario-timeout|--scenario_timeout) SCENARIO_TIMEOUT="$2"; shift 2 ;;
    --skip-ids|--skip_ids) SKIP_IDS="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --test_policy|--test-policy) TEST_POLICY="$2"; shift 2 ;;
    --agent_cfg|--agent-cfg) AGENT_CFG="$2"; shift 2 ;;
    --route_id|--route-id) ROUTE_ID="$2"; shift 2 ;;
    --device) DEVICE="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --tm-port|--tm_port) TM_PORT="$2"; shift 2 ;;
    --scenario_ids|--scenario-ids) SCENARIO_IDS="$2"; shift 2 ;;
    --scenario_range|--scenario-range) SCENARIO_RANGE="$2"; shift 2 ;;
    --no-resume) RESUME=0; shift ;;
    --env-script) ENV_SCRIPT="$2"; shift 2 ;;
    --carla-root) CARLA_ROOT="$2"; shift 2 ;;
    --log) LOGFILE="$2"; shift 2 ;;
    --dry-run|--dry_run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [ "${#CONFIGS[@]}" -eq 0 ]; then
  CONFIGS=("${DEFAULT_CONFIGS[@]}")
fi

if ! [[ "$CHUNK_SIZE" =~ ^[0-9]+$ ]] || [ "$CHUNK_SIZE" -lt 1 ]; then
  echo "Error: --chunk-size must be a positive integer (got: $CHUNK_SIZE)" >&2
  exit 1
fi

for cfg in "${CONFIGS[@]}"; do
  if [ ! -f "safebench/scenario/config/$cfg" ]; then
    echo "Error: scenario config not found: safebench/scenario/config/$cfg" >&2
    exit 1
  fi
done

[ -n "$LOGFILE" ] || LOGFILE="/tmp/train_scenario_batch_$(date +%Y%m%d_%H%M%S).log"
: > "$LOGFILE"

if [ -f "$ENV_SCRIPT" ]; then
  # Env/venv activation scripts reference vars (e.g. LD_LIBRARY_PATH) that may
  # legitimately be unset before first activation - relax nounset for sourcing.
  set +u
  # shellcheck disable=SC1090
  source "$ENV_SCRIPT"
  set -u
else
  echo "Warning: env script not found: $ENV_SCRIPT (assuming environment is already set up)" >&2
fi

# ---------------------------------------------------------------------------
# CARLA lifecycle (same approach as scripts/generate_route_batch.sh): the
# server is matched by its -carla-rpc-port=$PORT argument, so this never
# touches another port's / another user's CARLA instance on a shared machine.
# ---------------------------------------------------------------------------

carla_is_up() {
  timeout 2 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null
}

stop_carla() {
  echo "Stopping CARLA server on port $PORT..." >&2
  pkill -f -- "-carla-rpc-port=$PORT" 2>/dev/null || true
  local waited=0
  while carla_is_up; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge 30 ]; then
      pkill -9 -f -- "-carla-rpc-port=$PORT" 2>/dev/null || true
      break
    fi
  done
  # The port closing doesn't guarantee the old Unreal Engine process has
  # finished releasing GPU/VRAM/render-context resources yet - give it a
  # buffer before anything tries to start a new instance on the same GPU.
  echo "Cooling down for 10s before restart..." >&2
  sleep 10
}

start_carla() {
  if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
    echo "Error: CARLA launcher not found/executable: $CARLA_ROOT/CarlaUE4.sh" >&2
    return 1
  fi
  echo "Starting CARLA server on port $PORT (root: $CARLA_ROOT)..." >&2
  (
    cd "$CARLA_ROOT" || exit 1
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
      ./CarlaUE4.sh -quality-level=High -nosound -RenderOffScreen \
      -carla-rpc-port="$PORT" \
      > "/tmp/carla_server_${PORT}.log" 2>&1 &
    disown
  )

  local waited=0
  until carla_is_up; do
    sleep 2
    waited=$((waited + 2))
    if [ "$waited" -ge 120 ]; then
      echo "Error: CARLA did not start listening on port $PORT within 120s (see /tmp/carla_server_${PORT}.log)" >&2
      return 1
    fi
  done
  # The RPC port can open slightly before the world-loading subsystem is ready
  # to serve requests - a short grace period avoids the first post-restart
  # scenario spuriously hitting "CARLA could not load world".
  sleep 8
  echo "CARLA is up on port $PORT." >&2
}

restart_carla() {
  stop_carla
  start_carla
}

# ---------------------------------------------------------------------------
# Scenario discovery: reuse run_eval_batch.py's own resolution (legacy
# scenario_NNN.scenic pattern first, scenario_id_manifest.json fallback), then
# apply the id subset, the skip list, and the resume filter.
# ---------------------------------------------------------------------------

# discover_ids <cfg> -> space-separated scenario ids on stdout
discover_ids() {
  local cfg="$1"
  python - "$cfg" "$SCENARIO_IDS" "$SCENARIO_RANGE" "$RESUME" "$SKIP_IDS" <<'PY'
import os
import os.path as osp
import sys

sys.path.insert(0, os.getcwd())
sys.path.insert(0, osp.join(os.getcwd(), "scripts"))

from run_eval_batch import discover_scenario_ids, resolve_scenario_ids

import yaml

cfg, ids_arg, range_arg, resume, skip_arg = sys.argv[1:6]
root = os.getcwd()

ids = discover_scenario_ids(root, cfg)
explicit = [int(x) for x in ids_arg.split()] if ids_arg.strip() else None
ids = resolve_scenario_ids(ids, explicit, range_arg or None)

with open(osp.join(root, "safebench/scenario/config", cfg)) as f:
    conf = yaml.safe_load(f)
bench_id = str(conf.get("bench_id", ""))
bench_dir = osp.join(root, conf["scenic_dir"], bench_id)

# Skip list: bare "7" applies everywhere, "image-only:7" only to that bench.
skip = set()
for tok in skip_arg.replace(",", " ").split():
    if ":" in tok:
        bench, _, num = tok.rpartition(":")
        if bench != bench_id:
            continue
        skip.add(int(num))
    else:
        skip.add(int(tok))
ids = [i for i in ids if i not in skip]

if resume == "1":
    ids = [i for i in ids if not osp.isfile(osp.join(bench_dir, f"scenario_{i}.json"))]

print(" ".join(str(i) for i in ids))
PY
}

# bench_dir_for <cfg> -> absolute path of scenic_dir/<bench_id>
bench_dir_for() {
  local cfg="$1"
  python - "$cfg" <<'PY'
import os, os.path as osp, sys, yaml

cfg = sys.argv[1]
root = os.getcwd()
with open(osp.join(root, "safebench/scenario/config", cfg)) as f:
    conf = yaml.safe_load(f)
print(osp.join(root, conf["scenic_dir"], str(conf.get("bench_id", ""))))
PY
}

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

echo "Configs:     ${#CONFIGS[@]}"
echo "Mode:        $MODE"
echo "Policy:      $TEST_POLICY (device: $DEVICE)"
echo "Chunk size:  $CHUNK_SIZE (CARLA restarts after every chunk)"
echo "Timeout:     $([ "$SCENARIO_TIMEOUT" = "0" ] && echo 'none' || echo "$SCENARIO_TIMEOUT per scenario")"
[ -n "$SKIP_IDS" ] && echo "Skip ids:    $SKIP_IDS"
echo "Ports:       carla=$PORT tm=$TM_PORT"
echo "Resume:      $([ "$RESUME" -eq 1 ] && echo 'on (skip ids with an existing scenario_<id>.json)' || echo off)"
echo "Log:         $LOGFILE"
echo

TOTAL_ATTEMPTED=0
SUMMARY_LINES=()
CARLA_STARTED=0

for cfg in "${CONFIGS[@]}"; do
  ids_str="$(discover_ids "$cfg")" || {
    echo "Error: discovery failed for $cfg" >&2
    SUMMARY_LINES+=("$cfg: DISCOVERY FAILED")
    continue
  }
  read -r -a IDS <<< "$ids_str"
  bench_dir="$(bench_dir_for "$cfg")"

  echo "=== $cfg ==="
  echo "  bench dir: $bench_dir"
  echo "  ${#IDS[@]} scenario(s) to run: ${IDS[*]:-<none>}"

  if [ "${#IDS[@]}" -eq 0 ]; then
    SUMMARY_LINES+=("$cfg: nothing to do (all done or none discovered)")
    echo
    continue
  fi

  TOTAL_ATTEMPTED=$((TOTAL_ATTEMPTED + ${#IDS[@]}))

  chunk_index=0
  for ((i = 0; i < ${#IDS[@]}; i += CHUNK_SIZE)); do
    chunk=("${IDS[@]:i:CHUNK_SIZE}")
    chunk_index=$((chunk_index + 1))

    echo "  [chunk $chunk_index] ids: ${chunk[*]}"

    # One scenario per run_eval.py process, each under its own timeout. This is
    # what run_eval_batch.py does internally, unrolled here so the timeout
    # applies per scenario rather than per chunk.
    for id in "${chunk[@]}"; do
      cmd=(python scripts/run_eval.py
        --scenario_cfg "$cfg"
        --mode "$MODE"
        --agent_cfg "$AGENT_CFG"
        --test_policy "$TEST_POLICY"
        --route_id "$ROUTE_ID"
        --port "$PORT" --tm_port "$TM_PORT"
        --device "$DEVICE"
        --scenario_id "$id")
      if [ "$SCENARIO_TIMEOUT" != "0" ]; then
        cmd=(timeout -k 30 "$SCENARIO_TIMEOUT" "${cmd[@]}")
      fi

      echo "    scenario_id=$id: ${cmd[*]}"

      if [ "$DRY_RUN" -eq 1 ]; then
        continue
      fi

      if [ "$CARLA_STARTED" -eq 0 ]; then
        if carla_is_up; then
          echo "CARLA already listening on port $PORT - reusing it." >&2
        else
          start_carla || { echo "Error: could not start CARLA; aborting." >&2; exit 1; }
        fi
        CARLA_STARTED=1
      fi

      {
        echo
        echo "=== $cfg | chunk $chunk_index | scenario_id=$id | $(date -Is) ==="
      } >> "$LOGFILE"

      "${cmd[@]}" 2>&1 | tee -a "$LOGFILE"
      rc="${PIPESTATUS[0]}"

      if [ "$rc" -eq 124 ] || [ "$rc" -eq 137 ]; then
        echo "    scenario_id=$id TIMED OUT after $SCENARIO_TIMEOUT - killed, restarting CARLA" | tee -a "$LOGFILE"
        # The simulation was killed mid-run, so the server is left holding
        # actors/state - always take a fresh one rather than waiting for the
        # end-of-chunk restart.
        restart_carla || echo "Warning: CARLA restart failed after timeout on scenario $id" >&2
      elif [ "$rc" -ne 0 ]; then
        echo "    scenario_id=$id failed with exit code $rc" | tee -a "$LOGFILE"
      fi
    done

    if [ "$DRY_RUN" -eq 1 ]; then
      continue
    fi

    # Fresh server for the next chunk (and for the next config's first chunk).
    restart_carla || echo "Warning: CARLA restart failed after chunk $chunk_index of $cfg" >&2
  done

  if [ "$DRY_RUN" -eq 0 ]; then
    done_ids=()
    missing_ids=()
    for id in "${IDS[@]}"; do
      if [ -f "$bench_dir/scenario_${id}.json" ]; then
        done_ids+=("$id")
      else
        missing_ids+=("$id")
      fi
    done
    SUMMARY_LINES+=("$cfg: ${#done_ids[@]}/${#IDS[@]} produced scenario_<id>.json; missing: ${missing_ids[*]:-none}")
  else
    SUMMARY_LINES+=("$cfg: ${#IDS[@]} scenario(s) planned (dry run)")
  fi
  echo
done

echo "=== Summary ==="
for line in "${SUMMARY_LINES[@]}"; do
  echo "  $line"
done
echo "  total scenarios attempted: $TOTAL_ATTEMPTED"
echo "  log: $LOGFILE"

if [ "$DRY_RUN" -eq 1 ]; then
  echo "  (dry run - nothing was executed, CARLA untouched)"
fi
