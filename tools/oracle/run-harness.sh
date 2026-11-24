#!/usr/bin/env bash
#
# run-harness.sh - single entry point for the differential oracle harness.
#
# Wayfinder ticket #13: Differential oracle harness and byte-parity corpus.
#
#   tools/oracle/run-harness.sh                       self-check (default):
#                                                     build the pinned oracle if
#                                                     missing, (re)generate the
#                                                     corpus if missing, then
#                                                     validate the harness:
#                                                     corpus determinism, oracle
#                                                     self-determinism, and the
#                                                     injected-divergence demo.
#
#   tools/oracle/run-harness.sh --candidate BIN       differential run: compare
#                                                     BIN against the oracle
#                                                     byte-for-byte over the
#                                                     corpus (this is the gate
#                                                     for the Rust port).
#   tools/oracle/run-harness.sh --rebuild             rebuild the oracle first
#   --case GLOB / --family GLOB                       narrow the run (kernel
#                                                     bisection during porting)
#
# Everything lands under tools/oracle/work/ (git-ignored). See
# tools/oracle/README.md for the wiring: stage-1 verification, idiomatization
# rungs, the permanent serial-path CI gate, and the corpus hand-off to the
# performance gate (#11).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"

REBUILD=0
if [ "${1:-}" = "--rebuild" ]; then
  REBUILD=1
  shift
fi

ORACLE="$SCRIPT_DIR/work/oracle/triangle-oracle"

if [ ! -x "$ORACLE" ] || [ "$REBUILD" = "1" ]; then
  bash "$SCRIPT_DIR/build-oracle.sh"
fi

MANIFEST="$SCRIPT_DIR/work/corpus/manifest.json"
if [ ! -f "$MANIFEST" ]; then
  python3 "$SCRIPT_DIR/generate_corpus.py"
fi

exec python3 "$SCRIPT_DIR/diff_harness.py" "$@"
