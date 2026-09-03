#!/bin/bash
# One-command workflow:  scrutiny → sim → open the results.
#
#   ./run.sh                          # verify, ask for test values, run, open
#   ./run.sh --defaults               # same but skip the questions
#   ./run.sh -l "ttc-tires" -n "..."  # extra args go straight to run_sim.py
#   ./run.sh --maneuver corner_exit --corner-throttle 65
#
# By default the sim ASKS which values to test at (speeds, steer angles,
# throttle) — Enter keeps each default. --defaults (or piping stdin) skips
# the questions; the same knobs also exist as flags (run_sim.py --help).
#
# Step 1 runs verify.py (78 independent physics checks, ~35 s). If ANY check
# fails, the sim does NOT run — results from a sim that fails its own
# physics audit aren't worth having. Step 2 records a labeled run under
# runs/. Step 3 opens that run's plots (one Preview window), replay videos
# (QuickTime), and the run folder (Finder).

cd "$(dirname "$0")" || exit 1
PY=.venv/bin/python

echo "══ 1/3  scrutiny — verify.py ══════════════════════════════════"
if ! $PY verify.py; then
    echo ""
    echo "✗ VERIFICATION FAILED — the sim was NOT run."
    echo "  The failed checks are listed above. Fix (or understand) them"
    echo "  before trusting any result this sim produces."
    exit 1
fi

echo ""
echo "══ 2/3  simulation — run_sim.py ═══════════════════════════════"
ASK="--ask"
ARGS=()
for a in "$@"; do
    case "$a" in
        --defaults) ASK="" ;;          # skip the questions
        --ask)      ;;                 # already the default here
        *)          ARGS+=("$a") ;;
    esac
done
[ -t 0 ] || ASK=""                     # not a terminal: never block on input
$PY run_sim.py $ASK "${ARGS[@]}" || exit 1

echo ""
echo "══ 3/3  opening results ═══════════════════════════════════════"
RUN_ID=$(readlink runs/latest 2>/dev/null)
RUN_DIR="runs/${RUN_ID:-latest}"

open "$RUN_DIR"                                       # Finder: run folder
compgen -G "$RUN_DIR/plots/*.png"  > /dev/null && open "$RUN_DIR"/plots/*.png
compgen -G "$RUN_DIR/replay/*.mp4" > /dev/null && open "$RUN_DIR"/replay/*.mp4

echo "opened: $RUN_DIR  (plots → Preview, replays → QuickTime)"
echo "read:   $RUN_DIR/summary.md   $RUN_DIR/CHANGES.md"
