# collide pair visualizer

A self-contained web view of one collision pair at a time: the two primitives,
the emitted contact points from the reference and optimized solvers, and the
separation distance between them.

## Run

```bash
# from the repo root, after `make -f Makefile.optimized optimized` and a run
# that produced the two results files (see "data" below):
cd viz
python3 -m http.server 8000      # any static server; ES modules need an origin
# open http://localhost:8000/index.html
```

Controls: drag to orbit, scroll to zoom, right-drag to pan, ←/→ to step pairs,
`frame` to re-fit the camera. Use the file pickers at the bottom to load a
different `pairs.txt` / results pair.

## Data (all regenerable — not solver source)

The page fetches three files from its own directory:

| file               | source                                                         |
|--------------------|----------------------------------------------------------------|
| `pairs.txt`        | `performance-test/pairs.txt` (shapes + pair indices)           |
| `results_ref.txt`  | `build/perf_harness  shapes.bin pairs.bin results_ref.txt`     |
| `results_opt.txt`  | `build/perf_harness_opt shapes_optimized.bin pairs_optimized.bin results_opt.txt` |

`pairs.txt` format: `PRIM <type> <pos.xyz> <rot[9] row-major> <radius> <length>
<half_extent.xyz> <vert_count> [verts.xyz...]`, then `PAIR <a> <b>` index lines.
Results format (one line per pair): `idx flag dist alpha p1.xyz p2.xyz`.

Geometry conventions match the solver: world = `rot * body + pos` (row-major
`rot`); capsule axis is local **+X** with segment length = `length`; polytope
verts are body-frame and hulled in world space; scaling/contact center is the
body origin (`pos`).

## Screenshots

```bash
PLAYWRIGHT_PKG="$(npm root -g)/playwright" node shoot.mjs [pair ...]
# writes viz/shots/pair-NNNN.png; default sample covers all type combos plus
# the two far-from-origin float-precision pairs (117, 805).
```
