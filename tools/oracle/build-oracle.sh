#!/usr/bin/env bash
#
# build-oracle.sh - pinned C oracle build recipe for the differential harness.
#
# Part of wayfinder ticket #13 (Differential oracle harness and byte-parity
# corpus). The flags are pinned per the oracle determinism findings (research
# ticket #5, docs/research/oracle-determinism.md on branch research/
# oracle-determinism) and ADR docs/adr/0001-port-strategy.md:
#
#   -O2 (or -O3, via TRIANGLE_ORACLE_OPT), -ffp-contract=off, no -march/-mtune
#   or -native overrides, 64-bit target. aCute is compiled IN (no NO_ACUTE).
#
# The recipe is independent of the CMake build on purpose: CMake adds its own
# per-compiler FP flags and does not pin -ffp-contract, so the oracle links its
# exact flag set itself. The two headers CMake would configure for the build
# (triangle_version.h, triangle_export.h) are generated locally.
#
# Outputs:
#   work/oracle/triangle-oracle   the pinned oracle binary
#   work/oracle/build-info.txt    compiler, flags, sources, binary sha256
#
# The build is deterministic for a given source tree, compiler and flag set:
# rebuilds on the same machine reproduce the same binary sha256.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

CC="${CC:-cc}"
OPT="${TRIANGLE_ORACLE_OPT:--O2}"

case "$OPT" in
  -O2|-O3) ;;
  *) echo "build-oracle.sh: TRIANGLE_ORACLE_OPT must be -O2 or -O3 (got: $OPT)" >&2; exit 2 ;;
esac

OUT="${TRIANGLE_ORACLE_OUT:-$SCRIPT_DIR/work/oracle}"
SRC="$REPO_ROOT/src"
GEN="$OUT/include"

mkdir -p "$GEN"

# Generated header 1: version (mirrors src/Triangle/triangle_version.h.in
# configured with version 1.7.0 and TRIANGLE_ENABLE_ACUTE, as in src/CMakeLists.txt).
cat > "$GEN/triangle_version.h" <<'EOF'
/* This file is automatically created by tools/oracle/build-oracle.sh.
 * Please make changes only in its source file triangle_version.h.in.
 */

#ifndef TRIANGLE_VERSION_H
#define TRIANGLE_VERSION_H

#define TRIANGLE_ENABLE_ACUTE

#ifdef TRIANGLE_ENABLE_ACUTE
#define TRIANGLE_VERSION "1.7.0_acute"
#else
#define TRIANGLE_VERSION "1.7.0"
#endif
#define TRIANGLE_VERSION_MAJOR 1
#define TRIANGLE_VERSION_MINOR 7
#define TRIANGLE_VERSION_PATCH 0

#define TRIANGLE_VERSION_INT ( ( TRIANGLE_VERSION_MAJOR * 100 + TRIANGLE_VERSION_MINOR ) * 100 + TRIANGLE_VERSION_PATCH )

#endif
EOF

# Generated header 2: export macros (the oracle is a statically linked binary;
# mirror the define_generate_export_header() result for a hidden-visibility
# static build).
cat > "$GEN/triangle_export.h" <<'EOF'
/* This file is automatically created by tools/oracle/build-oracle.sh. */

#ifndef TRIANGLE_EXPORT_H
#define TRIANGLE_EXPORT_H

#define TRIANGLE_EXPORT
#define TRIANGLE_NO_EXPORT

#endif
EOF

SOURCES=(
  "$SRC/examples/triangle-cli/main.c"
  "$SRC/examples/triangle-cli/util.c"
  "$SRC/Triangle/triangle.c"
  "$SRC/Triangle/triangle_io.c"
  "$SRC/Triangle/triangle_api.c"
  "$SRC/Triangle/triangle_helper.c"
  "$SRC/Triangle/acute.c"
  "$SRC/Triangle/eps_writer.c"
  "$SRC/Triangle/predicates.c"
)

FLAGS=(-O2)
if [ "$OPT" = "-O3" ]; then FLAGS=(-O3); fi
FLAGS+=(-ffp-contract=off)

{
  echo "Pinned C oracle build recipe (wayfinder #13)"
  echo "compiler: $("$CC" --version | head -1)"
  echo "target arch: $(uname -m)"
  echo "flags: ${FLAGS[*]}"
  echo "sources:"
  for src in "${SOURCES[@]}"; do
    echo "  $(basename "$src") sha256=$(sha256sum "$src" | cut -d' ' -f1)"
  done
} > "$OUT/build-info.txt"

# shellcheck disable=SC2086
"$CC" "${FLAGS[@]}" \
  -I"$SRC/Triangle" -I"$SRC/Triangle/private" -I"$GEN" \
  -o "$OUT/triangle-oracle" "${SOURCES[@]}" \
  -lm

# The binary must run and report the aCute version.
"$OUT/triangle-oracle" -h | head -1 | grep -q "1.7.0_acute" \
  || { echo "build-oracle.sh: oracle binary does not report 1.7.0_acute" >&2; exit 3; }

echo "sha256:$(sha256sum "$OUT/triangle-oracle" | cut -d' ' -f1)" >> "$OUT/build-info.txt"

echo "[build-oracle] oracle ready: $OUT/triangle-oracle"
echo "[build-oracle] flags: ${FLAGS[*]} (compiler: $("$CC" --version | head -1))"
echo "[build-oracle] sha256: $(sha256sum "$OUT/triangle-oracle" | cut -d' ' -f1)"
