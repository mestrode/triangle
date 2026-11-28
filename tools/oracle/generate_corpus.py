#!/usr/bin/env python3
"""Deterministic corpus generator for the differential oracle harness.

Wayfinder ticket #13: Differential oracle harness and byte-parity corpus.

Generates the corpus the differential harness runs on: .node (point set) and
.poly (PSLG) inputs crossed with the full option family (plain Delaunay, -p
constrained, -q quality with and without the aCute -U max-angle switch, area
constraints, and the -r refine families reading pre-existing meshes; the -r
extension is issue #68). Everything is seeded; regeneration is
byte-identical, and the manifest records the sha256 of every generated
input file so callers (the harness, CI, or the performance gate #11) can
verify the corpus.

Case classes
  grid          integer lattice (collinear and cocircular degeneracy stress),
                outer ring plus a centered square hole ring
  random        random point set in (0,1)^2 with four boundary corners
  pslg_hole     unit square with a square hole ring and random interior
  pslg_concave  L-shaped domain (concave corner) with random interior
  pslg_wall     unit square with an interior wall segment (open slit)

Size classes: s, m, l, plus an opt-in xl (via --sizes, for the
performance gate's MT variants, ticket #11; default output unchanged).
The area-constrained families use a per-case area
target = domain_area / area_target_elements, so element counts (and runtimes)
stay comparable across size classes.

Layout (regenerated, never committed):
  <out>/<case>/input.node   vertices as a .node point set
  <out>/<case>/input.poly   the same vertices as a .poly PSLG
  <out>/<case>/rbase-plain/ -r base mesh from the plain family
                            (input.node + input.ele, sealed oracle output)
  <out>/<case>/rbase-p/     -r base mesh from the p family
                            (input.node + input.ele + input.poly)
  <out>/manifest.json       cases, per-family argv, and input hashes

The 'plain' and 'q20' families run on the .node input (Delaunay / quality
refinement of the point set); the -p families run on the .poly input.

Determinism: Python's Mersenne Twister is platform-independent; coordinates
are printed with %.17g, which round-trips binary64 exactly through the C
reader (the same format the C writers use). Two runs of this script produce
identical bytes; the harness self-check verifies this by regenerating into a
temp directory and comparing hashes.

Usage: python3 tools/oracle/generate_corpus.py [--out DIR] [--sizes s,m,l]
                                            [--oracle BIN] [--quiet]
"""

import argparse
import hashlib
import json
import os
import random
import shutil
import subprocess
import tempfile

MANIFEST_VERSION = 2

# Per-case-class seeds; fixed integers, never derived from time or machine state.
SEEDS = {"grid": 11, "random": 22, "pslg_hole": 33, "pslg_concave": 44, "pslg_wall": 55}
SIZE_INDEX = {"s": 0, "m": 1, "l": 2, "xl": 3}

# Size class parameters: (grid n, random points, pslg interior points,
# area target elements).
SIZE_PARAMS = {
    "s": (10, 40, 15, 200),
    "m": (20, 300, 150, 3000),
    "l": (36, 1200, 600, 30000),
    # xl is opt-in (--sizes xl) for the performance gate's MT variants
    # (ticket #11): a heavier single-mesh workload class so parallel scaling
    # is measurable. It is NOT part of the sealed 120-run harness contract.
    "xl": (100, 100000, 50000, 1000000),
}

# Option families, per the strategy decision (ticket #6, ADR 0001): the full
# family from the start. 'U' is the aCute max-angle switch (compiled in, see
# build-oracle.sh); {area} interpolates the per-case area target. Each entry:
# (tag, options template, input kind, expected output file extensions).
FAMILIES = [
    ("plain", "", "node", ["node", "ele"]),
    ("q20", "-q20", "node", ["node", "ele"]),
    ("p", "-p", "poly", ["node", "ele", "poly"]),
    ("pq20", "-pq20", "poly", ["node", "ele", "poly"]),
    ("pq20U120", "-pq20U120", "poly", ["node", "ele", "poly"]),
    ("pa", "-pa{area}", "poly", ["node", "ele", "poly"]),
    ("pq20a", "-pq20a{area}", "poly", ["node", "ele", "poly"]),
    ("pen", "-pen", "poly", ["node", "ele", "poly", "edge", "neigh"]),
    # The -r refine families (issue #68): they read a pre-existing mesh as
    # input, staged from the sealed base meshes (RBASES below). The option
    # strings follow the non-r counterparts exactly: -r reconstructs,
    # -rq20 adds quality refinement on the existing mesh, -ra{area} is the
    # fixed-area form of pa, and -rpq20U120 is the aCute variant over a
    # segmented (p) base mesh.
    ("r", "-r", "rplain", ["node", "ele"]),
    ("rq20", "-rq20", "rplain", ["node", "ele"]),
    ("ra", "-ra{area}", "rplain", ["node", "ele"]),
    ("rpq20U120", "-rpq20U120", "rp", ["node", "ele", "poly"]),
]

# -r base-mesh wiring (issue #68): input kind -> (base family tag, argv input
# token, files staged into every run dir). The base meshes are built by one
# pinned-oracle run per case over the base family's own corpus invocation
# (plain over the .node input; p over the .poly input, where segments are
# required) and sealed into the corpus as <case>/rbase-<tag>/<staged names>,
# so the -r runs read the standard 'input.*' names and write the standard
# 'input.1.*' outputs.
RBASES = {
    "rplain": ("plain", "input.node", ["input.node", "input.ele"]),
    "rp": ("p", "input.poly", ["input.node", "input.ele", "input.poly"]),
}


def fmt(x):
    """Format a coordinate exactly like the C writers: %.17g round-trips binary64."""
    return "%.17g" % x


def ring_segments(ring):
    """Closed-ring segment list over consecutive ring vertices."""
    return [(ring[i], ring[(i + 1) % len(ring)]) for i in range(len(ring))]


def interior_points(n, seed, inside):
    """Draw n deterministic interior points accepted by the predicate 'inside'."""
    rng = random.Random(seed)
    pts = []
    guard = 0
    while len(pts) < n:
        guard += 1
        if guard > 1000 * (n + 100):
            raise RuntimeError("interior_points: rejection sampling made no progress")
        x = 0.02 + 0.96 * rng.random()
        y = 0.02 + 0.96 * rng.random()
        if inside(x, y):
            pts.append((x, y))
    return pts


def case_grid(n):
    """Lattice domain [0,n-1]^2 minus an open centered square hole ring."""
    a = max(1, n // 3)
    b = n - 1 - a
    if b - a < 2:
        a, b = 1, 2
    outer = ([(i, 0) for i in range(n)] +
             [(n - 1, j) for j in range(1, n)] +
             [(i, n - 1) for i in range(n - 2, -1, -1)] +
             [(0, j) for j in range(n - 2, 0, -1)])
    inner = ([(a, j) for j in range(a, b + 1)] +
             [(i, b) for i in range(a + 1, b + 1)] +
             [(b, j) for j in range(b - 1, a - 1, -1)] +
             [(i, a) for i in range(b - 1, a, -1)])

    ring = set(outer) | set(inner)

    def in_hole(x, y):
        return a < x < b and a < y < b

    pts = []
    for p in outer + inner:
        pts.append((float(p[0]), float(p[1]), 1))
    for i in range(n):
        for j in range(n):
            if not in_hole(i, j) and (i, j) not in ring:
                pts.append((float(i), float(j), 0))
    id_of = {(p[0], p[1]): k + 1 for k, p in enumerate(pts)}
    seg_pairs = [(id_of[(float(p[0]), float(p[1]))], id_of[(float(q[0]), float(q[1]))])
                 for ring_ in (outer, inner)
                 for (p, q) in ring_segments(ring_)]
    # Hole marker deliberately off all ring diagonals (avoids on-edge
    # degeneracies the corpus is not meant to test).
    cx = (a + b) / 2.0
    hole = (cx + 0.117, cx - 0.083)
    return pts, seg_pairs, [hole], float(n - 1) * float(n - 1)


def case_random(n, seed):
    """Random point set with four boundary corners; no holes (convex hull)."""
    pts = [(0.0, 0.0, 1), (1.0, 0.0, 1), (1.0, 1.0, 1), (0.0, 1.0, 1)]
    pts += [(x, y, 0) for (x, y) in interior_points(n, seed, lambda x, y: True)]
    ring = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    id_of = {(p[0], p[1]): k + 1 for k, p in enumerate(pts)}
    seg_pairs = [(id_of[p], id_of[q]) for (p, q) in ring_segments(ring)]
    return pts, seg_pairs, [], 1.0


def case_pslg_hole(n, seed):
    """Unit square with a centered square hole ring and random interior."""
    ring_outer = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    ring_inner = [(0.4, 0.4), (0.6, 0.4), (0.6, 0.6), (0.4, 0.6)]

    def inside(x, y):
        return not (0.4 < x < 0.6 and 0.4 < y < 0.6)

    pts = [(x, y, 1) for (x, y) in ring_outer + ring_inner]
    pts += [(x, y, 0) for (x, y) in interior_points(n, seed, inside)]
    id_of = {(p[0], p[1]): k + 1 for k, p in enumerate(pts)}
    seg_pairs = [(id_of[p], id_of[q])
                 for ring in (ring_outer, ring_inner)
                 for (p, q) in ring_segments(ring)]
    hole = (0.5137, 0.5271)
    return pts, seg_pairs, [hole], 1.0


def case_pslg_concave(n, seed):
    """L-shaped domain (concave corner) with random interior."""
    ring = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.6), (0.6, 0.6), (0.6, 1.0), (0.0, 1.0)]

    def inside(x, y):
        return not (x > 0.6 and y > 0.6)

    pts = [(x, y, 1) for (x, y) in ring]
    pts += [(x, y, 0) for (x, y) in interior_points(n, seed, inside)]
    id_of = {(p[0], p[1]): k + 1 for k, p in enumerate(pts)}
    seg_pairs = [(id_of[p], id_of[q]) for (p, q) in ring_segments(ring)]
    return pts, seg_pairs, [], 0.6  # unit square minus the 0.4 x 0.4 corner


def case_pslg_wall(n, seed):
    """Unit square with an interior wall (slit) segment and random interior."""
    ring = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    wall_a = (0.5, 0.3)
    wall_b = (0.5, 0.7)

    def inside(x, y):
        # Keep points strictly off the wall line so the segment never passes
        # through a vertex (well-formed PSLG by construction).
        return not (abs(x - 0.5) < 0.01 and 0.29 < y < 0.71)

    pts = [(x, y, 1) for (x, y) in ring] + [(wall_a[0], wall_a[1], 1), (wall_b[0], wall_b[1], 1)]
    pts += [(x, y, 0) for (x, y) in interior_points(n, seed, inside)]
    id_of = {(p[0], p[1]): k + 1 for k, p in enumerate(pts)}
    seg_pairs = [(id_of[p], id_of[q]) for (p, q) in ring_segments(ring)]
    seg_pairs.append((id_of[wall_a], id_of[wall_b]))
    return pts, seg_pairs, [], 1.0


def build_case(kind, size):
    n_grid, n_random, n_pslg, area_target = SIZE_PARAMS[size]
    seed = SEEDS[kind] * 100 + SIZE_INDEX[size]
    if kind == "grid":
        return case_grid(n_grid) + (area_target,)
    if kind == "random":
        return case_random(n_random, seed) + (area_target,)
    if kind == "pslg_hole":
        return case_pslg_hole(n_pslg, seed) + (area_target,)
    if kind == "pslg_concave":
        return case_pslg_concave(n_pslg, seed) + (area_target,)
    if kind == "pslg_wall":
        return case_pslg_wall(n_pslg, seed) + (area_target,)
    raise ValueError("unknown kind: %s" % kind)


def write_node(path, pts):
    lines = ["%d 2 0 1" % len(pts)]
    for k, (x, y, marker) in enumerate(pts, 1):
        lines.append(" %d %s %s %d" % (k, fmt(x), fmt(y), marker))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_poly(path, pts, seg_pairs, holes):
    lines = ["%d 2 0 1" % len(pts)]
    for k, (x, y, marker) in enumerate(pts, 1):
        lines.append(" %d %s %s %d" % (k, fmt(x), fmt(y), marker))
    lines.append("%d 1" % len(seg_pairs))
    for k, (p, q) in enumerate(seg_pairs, 1):
        lines.append(" %d %d %d 1" % (k, p, q))
    lines.append("%d" % len(holes))
    for k, (x, y) in enumerate(holes, 1):
        lines.append(" %d %s %s" % (k, fmt(x), fmt(y)))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=None,
                        help="output directory (default: tools/oracle/work/corpus)")
    parser.add_argument("--quiet", action="store_true", help="suppress per-case output")
    parser.add_argument("--oracle", default=None,
                        help="pinned oracle binary used to build the -r base "
                             "meshes (default: tools/oracle/work/oracle/"
                             "triangle-oracle; built by build-oracle.sh, which "
                             "run-harness.sh and run-bench.sh run before "
                             "corpus generation)")
    parser.add_argument("--sizes", default="s,m,l",
                        help="comma-separated size classes to generate "
                             "(default s,m,l). 'xl' is opt-in: a heavier "
                             "class for the performance-gate MT variants "
                             "(ticket #11); the default s,m,l output IS the "
                             "sealed 120-run differential-harness contract")
    args = parser.parse_args()

    sizes = [t_.strip() for t_ in args.sizes.split(",") if t_.strip()]
    if not sizes or any(t_ not in SIZE_INDEX for t_ in sizes):
        parser.error("--sizes must be a comma-separated list drawn from %s"
                     % ", ".join(SIZE_INDEX))

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out = args.out or os.path.join(script_dir, "work", "corpus")
    oracle = args.oracle or os.path.join(script_dir, "work", "oracle",
                                        "triangle-oracle")
    if not (os.path.isfile(oracle) and os.access(oracle, os.X_OK)):
        parser.error("the -r base meshes need the pinned oracle at %s - "
                     "run tools/oracle/build-oracle.sh first (run-harness.sh "
                     "and run-bench.sh do), or pass --oracle" % oracle)
    os.makedirs(out, exist_ok=True)

    manifest_cases = []
    for kind in ("grid", "random", "pslg_hole", "pslg_concave", "pslg_wall"):
        for size in sizes:
            case_id = "%s_%s" % (kind, size)
            case_dir = os.path.join(out, case_id)
            os.makedirs(case_dir, exist_ok=True)
            pts, seg_pairs, holes, area, area_target = build_case(kind, size)
            node_path = os.path.join(case_dir, "input.node")
            poly_path = os.path.join(case_dir, "input.poly")
            write_node(node_path, pts)
            write_poly(poly_path, pts, seg_pairs, holes)

            area_value = area / area_target
            # -r base meshes (issue #68): one pinned-oracle run per case over
            # each base family; the outputs are sealed into the corpus
            # (rbase-<tag>/) and hashed into the manifest, so regeneration
            # reproduces them byte-identically (oracle determinism, ticket
            # #5). The invocation mirrors the harness contract (symlink
            # 'candidate', fixed argv): the writers embed the command line
            # in the output trailer.
            rbase = {}
            base_kinds = {}
            for _tag, _options, kind_input, _expected in FAMILIES:
                if kind_input in RBASES and RBASES[kind_input][0] not in base_kinds:
                    base_kinds[RBASES[kind_input][0]] = kind_input
            for base_tag in sorted(base_kinds):
                kind_input = base_kinds[base_tag]
                _base_tag, _argv_input, staged = RBASES[kind_input]
                base_opts = next(f[1] for f in FAMILIES if f[0] == base_tag)
                base_kind_input = next(f[2] for f in FAMILIES if f[0] == base_tag)
                base_input = "input." + base_kind_input
                base_argv = ([base_opts] if base_opts else []) + [base_input]
                base_dir = os.path.join(case_dir, "rbase-" + base_tag)
                if os.path.exists(base_dir):
                    shutil.rmtree(base_dir)
                os.makedirs(base_dir)
                tmp = tempfile.mkdtemp(prefix="corpus-rbase-")
                try:
                    shutil.copy(os.path.join(case_dir, base_input),
                                os.path.join(tmp, base_input))
                    os.symlink(os.path.abspath(oracle),
                               os.path.join(tmp, "candidate"))
                    proc = subprocess.run(["./candidate"] + base_argv, cwd=tmp,
                                          stdout=subprocess.DEVNULL,
                                          stderr=subprocess.STDOUT)
                    if proc.returncode != 0:
                        raise RuntimeError("oracle exited %d building the %s "
                                           "base mesh of %s"
                                           % (proc.returncode, base_tag, case_id))
                    files = {}
                    for name in staged:
                        out_name = "input.1." + name.split(".", 1)[1]
                        src = os.path.join(tmp, out_name)
                        if not os.path.isfile(src) or os.path.getsize(src) == 0:
                            raise RuntimeError("oracle produced no %s for the "
                                               "%s base mesh of %s"
                                               % (out_name, base_tag, case_id))
                        shutil.copy(src, os.path.join(base_dir, name))
                        files[name] = sha256_file(os.path.join(base_dir, name))
                    rbase[base_tag] = {"dir": "rbase-" + base_tag,
                                       "files": files}
                finally:
                    shutil.rmtree(tmp, ignore_errors=True)
            families = []
            for tag, options, kind_input, expected in FAMILIES:
                options = options.replace("{area}", fmt(area_value))
                if kind_input in RBASES:
                    base_tag, input_file, staged = RBASES[kind_input]
                    argv = ([options] if options else []) + [input_file]
                    families.append({
                        "tag": tag,
                        "options": options,
                        "input": input_file,
                        "argv": argv,
                        "expected": expected,
                        "rbase": base_tag,
                        "staged": staged,
                    })
                else:
                    input_file = "input." + kind_input
                    argv = ([options] if options else []) + [input_file]
                    families.append({
                        "tag": tag,
                        "options": options,
                        "input": input_file,
                        "argv": argv,
                        "expected": expected,
                    })
            manifest_cases.append({
                "id": case_id,
                "kind": kind,
                "size": size,
                "area": fmt(area_value),
                "points": len(pts),
                "segments": len(seg_pairs),
                "holes": len(holes),
                "input": {
                    "node": {"file": "input.node", "sha256": sha256_file(node_path)},
                    "poly": {"file": "input.poly", "sha256": sha256_file(poly_path)},
                },
                "rbase": rbase,
                "families": families,
            })
            if not args.quiet:
                print("[corpus] %s: %d points, %d segments, %d holes, area %s"
                      % (case_id, len(pts), len(seg_pairs), len(holes), fmt(area_value)))

    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "generator": "tools/oracle/generate_corpus.py",
        "description": "Differential harness corpus (wayfinder #13; -r extension #68)",
        "cases": manifest_cases,
    }
    manifest_path = os.path.join(out, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")

    total_runs = sum(len(c["families"]) for c in manifest_cases)
    print("[corpus] %d cases, %d runs (case x option family)" % (len(manifest_cases), total_runs))
    print("[corpus] manifest: %s" % manifest_path)


if __name__ == "__main__":
    main()
