
# Convex Primitive Collision — Pair Visualizer

Instructions for an LLM agent. Build a small web tool that renders **one
collision pair at a time** so a human can confirm, by eye, that the emitted
contact points lie on the shapes' surfaces and that the separation distance is
correct. You must **build, run, capture screenshots, and inspect them** until
the renders are demonstrably valid. "Done" is the acceptance checklist at the
bottom, met **with real screenshots and command output as evidence**. Never
report a step complete without running it; never fabricate output.

## What this is for

The optimized solver emits, per pair, a colliding flag, a signed distance, and
two contact points `p1`/`p2`. Numbers in a results file can't be eyeballed for
geometric sanity. This tool turns one pair into a framed 3D view: the two
primitives, the reference contacts, the optimized contacts, and the line whose
length is the reported distance. It is a **verification aid**, not solver code
— it shares nothing with `src-optimized/` and must never be on a timed path.

## The data is the thing — start here

Read the actual files before designing anything. There are three inputs, all
plain text, all regenerable.

### Input 1 — shapes + pairs (`performance-test/pairs.txt`)

```
collide-pairs-v1 <n_pairs> <n_prims>
PRIM <type> <pos.x pos.y pos.z> <rot[9] row-major> <radius> <length> <he.x he.y he.z> <vcount> [vx vy vz ...]
...                     (n_prims PRIM lines)
PAIR <a> <b>            (n_pairs PAIR lines; a,b index into the prim array)
```

Field layout of a `PRIM` line (token indices, 0-based, after the `PRIM` tag at
0): `type@1`, `pos@2..4`, `rot@5..13` (row-major 3×3), `radius@14`,
`length@15`, `half_extent@16..18`, `vcount@19`, then `vcount` vertices of 3
floats each at `20..`. `type`: `0`=sphere, `1`=box, `2`=capsule, `3`=polytope.
Non-polytope lines have `vcount=0` and stop at token 19.

### Input 2 & 3 — solver outputs (one line per pair)

```
<idx> <flag> <dist> <alpha> <p1.x p1.y p1.z> <p2.x p2.y p2.z>
```

`flag` 0/1 = separated/colliding; `dist` meters (+ separated, − penetration);
`p1` on primitive `a`, `p2` on primitive `b`; all world-space meters. Produce
the two files (reference + optimized) with the existing harness:

```bash
make -f Makefile.optimized optimized        # builds harnesses + bin inputs
./build/perf_harness     build/shapes.bin           build/pairs.bin           viz/results_ref.txt
./build/perf_harness_opt build/shapes_optimized.bin build/pairs_optimized.bin viz/results_opt.txt
cp performance-test/pairs.txt viz/pairs.txt
```

### Data facts that constrain the design (verify them yourself)

- **World coordinates reach several km from the origin** (`|pos|` up to ~1.1e4
  m) while shapes are ~1–5 m. Rendering at those coordinates in float32 (WebGL)
  jitters. **Constraint to exploit:** a single pair is local — translate every
  point by a per-pair pivot (midpoint of the two body origins) before handing
  it to the renderer, so geometry sits near the origin. Keep the pivot in JS
  doubles.
- `n_pairs` ~1000; only **two** primitives are ever drawn at once. There is no
  performance problem here — do not build spatial structures, instancing, or
  LOD. Solve the one-pair case straight-line.
- Contact points are stored as `float32` in the results, so far-from-origin
  pairs carry ~0.5–1 mm of coordinate noise. The view only needs to be visually
  faithful; do not present sub-mm differences as defects.

## Geometry conventions — MUST match the solver exactly

Get these from `src/collide.c` (`make_vshape`) and confirm; a wrong convention
makes a contact point float off the surface and invalidates the whole tool.

- **Transform:** `world = rot · body + pos`, with `rot` row-major (`rot[3*i +
  j]` is row `i`, col `j`). Build the object matrix from `rot`+`pos`.
- **Sphere:** center `pos`, radius `radius`.
- **Box:** half-extents `half_extent`, centered at `pos`, oriented by `rot`.
- **Capsule:** axis is body-frame **+X** (`= rot · [1,0,0]ᵀ` = first column of
  `rot`); segment half-length `hl = length/2`; radius `radius`. (Most mesh
  libraries default the capsule axis to +Y — rotate the geometry −90° about Z
  to put it on +X before applying the object matrix.)
- **Polytope:** `vcount` body-frame vertices; transform each to world (`rot·v +
  pos`), then build a convex hull from the world points. Do not apply the
  object matrix again afterwards.
- Scaling/contact center is the body origin `pos` for sphere/box/capsule
  (vertex centroid for polytope) — but the tool reads contact points straight
  from the results file, so it never recomputes a center; it only needs
  `pos`+`rot`(+`verts`) to draw.

## The transform (the whole machine)

```
text files ─► parse ─► {prims[], pairs[]}, results_ref[], results_opt[]
select pair i ─► (A,B) = prims[pairs[i]] ─► pivot = (A.pos+B.pos)/2
  ├─ build mesh A (color A), mesh B (color B), each translucent + wireframe
  ├─ markers: ref p1,p2 (one color); opt p1,p2 (another); all minus pivot
  ├─ dashed line p1→p2 (length = reported distance)
  ├─ frame camera to the pair's bounding box
  └─ info panel: types, flag, ref dist, opt dist, dist Δ, contact Δ (mm), |pos|
```

Marker radius should scale to the pair's on-screen size (e.g. ~1% of the
bounding-box diagonal) so it's visible but doesn't swallow the contact.

## Technology and self-containment

- A single `viz/index.html` ES-module page. **Vendor** the 3D library locally
  (e.g. `viz/vendor/`) and resolve it with an import map — **no CDN at
  runtime**, so the tool works offline and headless runs aren't flaky. Use a
  maintained WebGL library with ready-made box/sphere/capsule geometry, a
  convex-hull geometry helper for polytopes, and orbit controls.
- Load data by `fetch('./pairs.txt')` etc. (ES modules + fetch need a real HTTP
  origin — a `file://` open will not work), and **also** offer `<input
  type=file>` pickers as a fallback. Support `?pair=N` in the URL.
- Expose a `window.__ready` flag set true once the first pair has rendered, so
  a headless driver can wait on it deterministically.
- Enable `preserveDrawingBuffer` on the renderer so screenshots are reliable.

## Screenshots (the verification step — not optional)

Drive the page headlessly and capture PNGs. A headless Chromium driver is
available; ES modules need an origin, so serve the directory and navigate to
it:

```bash
PLAYWRIGHT_PKG="$(npm root -g)/playwright" node viz/shoot.mjs        # default sample
PLAYWRIGHT_PKG="$(npm root -g)/playwright" node viz/shoot.mjs 7 117  # specific pairs
# → viz/shots/pair-NNNN.png
```

The driver must: start a tiny static file server over the `viz/` dir, launch
the browser, `goto(...?pair=N)`, `waitForFunction(() => window.__ready)`, pause
briefly for control damping/render to settle, then `screenshot`. Log the info
panel text alongside each shot.

**Sample set must cover every primitive-type combination** present in the data
(sphere–sphere, box–sphere, capsule–box, capsule–capsule, box–polytope,
polytope–sphere, …) **plus at least one separated pair, one colliding pair, and
one far-from-origin pair** (where the float floor shows). Then **open the PNGs
and look at them** — confirm each contact dot sits on a surface, the dashed
line spans the gap for separated pairs, colliding pairs interpenetrate, and all
four shape types render with correct orientation. Fix the generator (geometry
convention, pivot, parse offsets) when a render is wrong — do not adjust the
artifact.

## Constraints

- **Never modify** the reference (`src/`, `test/`,
  `performance-test/pairs.txt`, reference harness) or the optimized solver to
  suit the viewer.
- The three data files and `shots/` are **regenerable** — `.gitignore` them.
  The committed artifact is the page, the headless driver, the vendored
  library, and a short `viz/README.md` with run instructions.
- This tool is never timed and never imported by the solver or harness.

## Acceptance checklist

- [ ] `viz/index.html` renders one pair at a time, framed, with orbit/zoom/pan,
      pair stepping (prev/next, numeric entry, `?pair=N`), and a `frame` action.
- [ ] All four primitive types render with the solver's exact conventions
      (row-major `rot`, capsule axis +X, polytope hull in world space), verified
      against screenshots.
- [ ] Reference and optimized contacts are both shown and visually distinct; a
      dashed line connects `p1`–`p2`; the info panel shows types, flag, both
      distances, distance Δ, per-point contact Δ (mm), and `|pos|`.
- [ ] Per-pair pivot translation is applied so far-from-origin pairs render
      without float32 jitter.
- [ ] The page is self-contained: library vendored locally, no runtime CDN;
      loads via fetch from its own directory; `window.__ready` exposed.
- [ ] A headless driver produces PNGs for a sample covering **every** type
      combination plus a separated, a colliding, and a far-from-origin pair; the
      PNGs were opened and confirmed valid.
- [ ] No reference or solver file changed; data files and `shots/` are
      git-ignored; `viz/README.md` documents how to run it.
