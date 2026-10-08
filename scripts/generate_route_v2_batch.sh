#!/usr/bin/env bash
#
# Scenic 2.x counterpart of scripts/generate_route_batch.sh. Loops
# scripts/generate_scenic_route_pickle_v2.py over a bench directory's .scenic
# files, one fresh Python process per file (via --scenic-file).
#
# --scenic-dir batch mode reuses one Python process across every file, and
# Scenic keeps a fair amount of process-global state (scenic.syntax.veneer's
# running-scenario registry, the road-network cache, etc.) that isn't fully
# torn down by ScenicSimulator.endSimulation()/destroy() between files. In
# practice this means only the FIRST file in a --scenic-dir batch ever
# succeeds; every file after it fails with a generic, content-independent
# AssertionError. Running --scenic-file in a loop gives each file a fresh
# interpreter (and fresh CARLA client connection), which avoids this
# entirely - at the cost of a few seconds of process/connect overhead per
# file instead of paying it once for the whole batch.
#
# Differences from generate_route_batch.sh (the Scenic 3.x version):
#   - Runs generate_scenic_route_pickle_v2.py (safebench.util.scenic_utils_v2)
#     instead of generate_scenic_route_pickle.py.
#   - Sources env.scenic2.sh by default (the Scenic 2.x venv) instead of
#     env.scenic3.sh.
#   - The v2 generator has no --carla-timeout flag, so it is not passed.
#
# Usage:
#   scripts/generate_route_v2_batch.sh <bench-dir> [file1.scenic file2.scenic ...] [options]
#
# If no filenames are given, every *.scenic file directly inside <bench-dir>
# is processed. Filenames are relative to <bench-dir> and must be listed
# immediately after it, before any --options.
#
# Examples:
#   # Whole modality directory (safe to re-run - --resume skips finished ones)
#   scripts/generate_route_v2_batch.sh \
#     safebench/scenario/scenic_data/chatscene_kimi26think/chatscene_kimik26think_R1/image-only
#
#   # Just a curated list of known-executable files
#   scripts/generate_route_v2_batch.sh \
#     safebench/scenario/scenic_data/chatscene_kimi26think/chatscene_kimik26think_R1/video-only \
#     BLO_3.scenic HORM_3.scenic
#
# Options:
#   --bench-id NAME          Bench id passed through (default: basename of bench-dir)
#   --out-pickle PATH        Pickle path (default: <bench-dir>/scenic_route.pickle)
#   --port N                 CARLA port (default: 2005)
#   --tm-port N              Traffic manager port (default: 8005)
#   --max-scene-attempts N   (default: 200)
#   --warmup-ticks N         (default: 20)
#   --max-record-seconds N   (default: 20)
#   --carla-timeout N        (default: 60) CARLA RPC client timeout, seconds
#   --file-timeout N         (default: 60) per-file wall-clock cap, seconds;
#                            a file exceeding it is killed and marked failed
#   --no-video               Disable --record-video
#   --no-resume              Force regeneration of already-completed scenarios
#   --env-script PATH        Sourced before running (default: env.scenic2.sh)
#   --log PATH               Log file (default: /tmp/<bench-id>_v2_batch_loop_<timestamp>.log)
#   --restart-carla-every N  Kill and relaunch the CARLA server every N files
#                            processed (default: 0 = never restart). Use this
#                            if CARLA tends to crash/degrade after running many
#                            scenarios back-to-back in one long-lived process.
#   --carla-root PATH        CARLA install dir for restarts (default: ~/yungloon/fail2drive/f2d_carla)
#   -h, --help               Show this help and exit

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BENCH_ID=""
OUT_PICKLE=""
PORT=2005
TM_PORT=8005
MAX_SCENE_ATTEMPTS=200
WARMUP_TICKS=20
MAX_RECORD_SECONDS=20
CARLA_TIMEOUT=60
FILE_TIMEOUT=60
RECORD_VIDEO=1
RESUME_FLAG=""
ENV_SCRIPT="env.scenic2.sh"
LOGFILE=""
RESTART_CARLA_EVERY=0
CARLA_ROOT="$HOME/yungloon/fail2drive/f2d_carla"

usage() {
  sed -n '2,62p' "${BASH_SOURCE[0]}"
}

if [ "$#" -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  usage
  exit 0
fi

BENCH_DIR="$1"
shift

# Filenames (if any) come immediately after bench-dir, before any --options.
FILES=()
while [ "$#" -gt 0 ] && [[ "$1" != --* ]]; do
  FILES+=("$1")
  shift
done

while [ "$#" -gt 0 ]; do
  case "$1" in
    --bench-id) BENCH_ID="$2"; shift 2 ;;
    --out-pickle) OUT_PICKLE="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --tm-port) TM_PORT="$2"; shift 2 ;;
    --max-scene-attempts) MAX_SCENE_ATTEMPTS="$2"; shift 2 ;;
    --warmup-ticks) WARMUP_TICKS="$2"; shift 2 ;;
    --max-record-seconds) MAX_RECORD_SECONDS="$2"; shift 2 ;;
    --carla-timeout) CARLA_TIMEOUT="$2"; shift 2 ;;
    --file-timeout) FILE_TIMEOUT="$2"; shift 2 ;;
    --no-video) RECORD_VIDEO=0; shift ;;
    --no-resume) RESUME_FLAG="--no-resume"; shift ;;
    --env-script) ENV_SCRIPT="$2"; shift 2 ;;
    --log) LOGFILE="$2"; shift 2 ;;
    --restart-carla-every) RESTART_CARLA_EVERY="$2"; shift 2 ;;
    --carla-root) CARLA_ROOT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [ ! -d "$BENCH_DIR" ]; then
  echo "Error: not a directory: $BENCH_DIR" >&2
  exit 1
fi
BENCH_DIR="${BENCH_DIR%/}"

[ -n "$BENCH_ID" ] || BENCH_ID="$(basename "$BENCH_DIR")"
[ -n "$OUT_PICKLE" ] || OUT_PICKLE="$BENCH_DIR/scenic_route.pickle"
[ -n "$LOGFILE" ] || LOGFILE="/tmp/${BENCH_ID}_v2_batch_loop_$(date +%Y%m%d_%H%M%S).log"
: > "$LOGFILE"

if [ "${#FILES[@]}" -eq 0 ]; then
  while IFS= read -r -d '' f; do
    FILES+=("$(basename "$f")")
  done < <(find "$BENCH_DIR" -maxdepth 1 -name '*.scenic' -print0 | sort -z)
fi

if [ "${#FILES[@]}" -eq 0 ]; then
  echo "Error: no .scenic files found in $BENCH_DIR" >&2
  exit 1
fi

if [ -f "$ENV_SCRIPT" ]; then
  # Env/venv activation scripts often reference vars (e.g. LD_LIBRARY_PATH)
  # that may be legitimately unset before first activation - relax nounset
  # just for sourcing them.
  set +u
  # shellcheck disable=SC1090
  source "$ENV_SCRIPT"
  set -u
else
  echo "Warning: env script not found: $ENV_SCRIPT (assuming environment is already set up)" >&2
fi

if ! timeout 2 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
  echo "Error: nothing is listening on 127.0.0.1:$PORT - is the CARLA server running?" >&2
  exit 1
fi

# Only used when --restart-carla-every > 0. Kills/relaunches the CARLA server
# on $PORT specifically (matched by its -carla-rpc-port=$PORT command-line
# argument, so this never touches another user's or another port's CARLA
# instance on a shared machine).
stop_carla() {
  echo "Stopping CARLA server on port $PORT..." >&2
  pkill -f -- "-carla-rpc-port=$PORT" 2>/dev/null || true
  local waited=0
  while timeout 1 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; do
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
  until timeout 2 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; do
    sleep 2
    waited=$((waited + 2))
    if [ "$waited" -ge 120 ]; then
      echo "Error: CARLA did not start listening on port $PORT within 120s (see /tmp/carla_server_${PORT}.log)" >&2
      return 1
    fi
  done
  # The RPC port can open slightly before the world-loading subsystem is
  # actually ready to serve requests - a short grace period avoids the first
  # post-restart file spuriously hitting "CARLA could not load world".
  sleep 8
  echo "CARLA is back up on port $PORT." >&2
}

restart_carla() {
  stop_carla
  start_carla
}

# Per-file liveness guard. CARLA can segfault (Signal 11) under sustained
# load - running hundreds of sims back-to-back with video recording - or be
# left unstable after a per-file timeout SIGKILLs a Scenic process mid-sim.
# Without this the batch keeps firing files at a dead server and every one
# fails. Called before each file: if the RPC port isn't accepting
# connections, (re)start the server so the next file gets a live CARLA.
ensure_carla() {
  if timeout 3 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
    return 0
  fi
  echo "--- CARLA not responding on port $PORT; (re)starting server ---" >> "$LOGFILE"
  echo "CARLA not responding on port $PORT - (re)starting..." >&2
  stop_carla 2>/dev/null || true
  start_carla
}

VIDEO_FLAG=""
[ "$RECORD_VIDEO" -eq 1 ] && VIDEO_FLAG="--record-video"

ERR_FILE="$BENCH_DIR/scenic_route_errors.json"
rm -f "$ERR_FILE"

echo "Bench dir:   $BENCH_DIR"
echo "Bench id:    $BENCH_ID"
echo "Out pickle:  $OUT_PICKLE"
echo "Files:       ${#FILES[@]}"
echo "Log:         $LOGFILE"
echo

FAILED_FILES=()
PROCESSED_COUNT=0
for fn in "${FILES[@]}"; do
  f="$BENCH_DIR/$fn"
  if [ ! -f "$f" ]; then
    echo "Warning: skipping missing file: $f" >&2
    continue
  fi

  echo "=== $fn ===" >> "$LOGFILE"
  # Make sure CARLA is alive before spending this file's time budget on it
  # (recovers from a mid-run segfault or a previous timeout that destabilized
  # the server). Done outside the timeout wrapper so restart time is not
  # charged against this file's --file-timeout.
  ensure_carla
  # Per-file wall-clock cap: some Scenic files hang in scene sampling (e.g. a
  # 4-way-intersection require loop that rarely satisfies), which would
  # otherwise stall the whole batch indefinitely. `timeout --signal=KILL`
  # after a grace SIGTERM guarantees the process (and its CARLA connection)
  # is torn down so the next file gets a fresh interpreter. Exit code 124
  # (SIGTERM) or 137 (SIGKILL) => this file timed out.
  timeout --kill-after=15s "${FILE_TIMEOUT}s" \
    python scripts/generate_scenic_route_pickle_v2.py \
    --scenic-file "$f" \
    --out-pickle "$OUT_PICKLE" \
    --bench-id "$BENCH_ID" \
    --key-mode numeric_only \
    --max-scene-attempts "$MAX_SCENE_ATTEMPTS" \
    --warmup-ticks "$WARMUP_TICKS" \
    --max-record-seconds "$MAX_RECORD_SECONDS" \
    --carla-timeout "$CARLA_TIMEOUT" \
    --port "$PORT" --tm-port "$TM_PORT" \
    $VIDEO_FLAG $RESUME_FLAG >> "$LOGFILE" 2>&1
  rc=$?

  if [ "$rc" -eq 124 ] || [ "$rc" -eq 137 ]; then
    echo "--- TIMEOUT for $fn (exceeded ${FILE_TIMEOUT}s, killed) ---" >> "$LOGFILE"
    rm -f "$ERR_FILE"
    FAILED_FILES+=("$fn (timeout)")
    # A SIGKILL mid-simulation can leave CARLA in synchronous mode with
    # orphaned actors; reset it so the next file starts from a clean world.
    restart_carla
  elif [ -f "$ERR_FILE" ]; then
    echo "--- error for $fn ---" >> "$LOGFILE"
    cat "$ERR_FILE" >> "$LOGFILE"
    echo >> "$LOGFILE"
    rm -f "$ERR_FILE"
    FAILED_FILES+=("$fn")
  fi

  PROCESSED_COUNT=$((PROCESSED_COUNT + 1))
  if [ "$RESTART_CARLA_EVERY" -gt 0 ] && [ $((PROCESSED_COUNT % RESTART_CARLA_EVERY)) -eq 0 ]; then
    echo "--- restarting CARLA after $PROCESSED_COUNT file(s) ---" >> "$LOGFILE"
    restart_carla
  fi
done
echo "=== LOOP DONE ===" >> "$LOGFILE"

ENTRY_COUNT="$(python3 -c "
import pickle, os
p = '$OUT_PICKLE'
print(len(pickle.load(open(p, 'rb'))) if os.path.exists(p) else 0)
")"

echo
echo "=== Summary ==="
echo "Requested:  ${#FILES[@]}"
echo "Failed:     ${#FAILED_FILES[@]}"
echo "Pickle now has $ENTRY_COUNT total scenario(s) for bench_id=$BENCH_ID."
echo "Full log: $LOGFILE"
if [ "${#FAILED_FILES[@]}" -gt 0 ]; then
  echo
  echo "Failed files (see \"--- error for <file> ---\" in the log for details):"
  printf '  %s\n' "${FAILED_FILES[@]}"
  exit 1
fi
exit 0
