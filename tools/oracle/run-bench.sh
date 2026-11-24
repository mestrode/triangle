#!/usr/bin/env bash
#
# run-bench.sh - single entry point for the performance gate.
# Wayfinder ticket #11: Performance gate and benchmark suite.
#
#   tools/oracle/run-bench.sh --candidate BIN                benchmark vs oracle
#   tools/oracle/run-bench.sh --candidate BIN --gate 1.05    apply tolerance band (the band from #11)
#   tools/oracle/run-bench.sh --candidate <oracle-path>     noise-floor probe
#   tools/oracle/run-bench.sh --candidate BIN --threads N    one MT variant
#
# Builds the pinned oracle and the corpus if missing (same recipes as
# run-harness.sh), then delegates to bench.py, which times whole-process wall
# time over the harness corpus: interleaved oracle/candidate runs, warmup
# first, median per side, per-run and geometric-mean ratios. The method, the
# timed set (sizes m and l), and the tolerance band are specified in ticket
# #11; see bench.py's docstring and tools/oracle/README.md.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"

ORACLE="$SCRIPT_DIR/work/oracle/triangle-oracle"
if [ ! -x "$ORACLE" ]; then
  bash "$SCRIPT_DIR/build-oracle.sh"
fi

MANIFEST="$SCRIPT_DIR/work/corpus/manifest.json"
if [ ! -f "$MANIFEST" ]; then
  python3 "$SCRIPT_DIR/generate_corpus.py" --quiet
fi

exec python3 "$SCRIPT_DIR/bench.py" "$@"
