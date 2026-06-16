# Convex Primitive Collision Detection — Reference Implementation

Instructions for an LLM agent. You must **implement, build, test, run, and
verify** this project end to end. "Done" means every item in the Acceptance
Checklist at the bottom passes with real command output as evidence. Never
report a step as complete without having run it; never fabricate results.

## Goal

Create a reference solution in C for convex primitive collision detection,
based on the method described in `{project-root}/documents/2207.00669.pdf`.
Read the paper first (extract text or upload it to a document-capable model);
the implementation must follow its algorithm, not a generic substitute.

## Fixed requirements (do not relax any of these)

- Language: C (C99 or C11). Build with gcc. Dependencies: libc and libm only.
- Layout: `src/` for the library, `test/` for tests, `performance-test/` for
  the performance harness, `run-performance-test` script at project root.
- Units: meters.
- Input domain: coordinates within ±8,192 m (±8 km) of the origin on every
  axis. Out-of-range input must be **rejected with an explicit error** at the
  API boundary — not clamped, not silently accepted.
- Primitive size: each input primitive's axis-aligned bounding box extent must
  be between 0.1 m and 250 m on every axis. Sizes outside this range must be
  **rejected with an explicit error** at the API boundary, same policy as
  out-of-range coordinates.
- Resolution: results must be correct to 1 mm. The domain is sized so that
  32-bit float suffices: float32 ULP is ~0.49 mm in [4,096, 8,192) m and ~0.98
  mm at the 8,192 m edge (2^13 × 2^-23 m), both within 1 mm — use `float` for
  positions and distances.
- Single-threaded only. No threads, no SIMD intrinsics required.
- Primitives (pinned set; edit this list if a different set is wanted): sphere,
  box, capsule, convex polytope (point cloud / hull). All handled through one
  support-function path per the paper — no per-pair special-case solvers except
  where the paper itself specifies one.
- Validation: a **secondary, independent method** must verify every result. It
  must share no solver code with the primary path (e.g. dense
  sampling/brute-force support comparison, or an independently implemented
  baseline algorithm). Any disagreement beyond 1 mm tolerance is a test
  failure.

## Phase 1 — Plan (write before coding)

Write `src/README.md` containing, briefly:

1. The input and output data: struct layouts for each primitive, the query pair
   format, the result format (colliding flag, distance/penetration, witness
   points). Flat structs, explicit fields, no hidden pointers.
2. The batch contract: the primary API takes **arrays** of query pairs and
   produces arrays of results (`collide_pairs(pairs, count, results)`); a
   single query is `count = 1`. Use indices, not pointers, to reference
   primitives from pairs.
3. The algorithm from the paper, in your own words, 5–10 lines.
4. The secondary validation method and why it is independent.
5. Every fact you need but could not confirm from the paper or this file, as a
   line: `ASSUMPTION: <fact> — affects <decision>`.

## Phase 2 — Implement

- `src/`: the collision library. Deterministic: same input bytes → same output
  bytes, every run.
- `test/`: correctness tests. Required coverage: - known analytic cases for
  every primitive pair type (touching, separated,
    deeply penetrating);
  - corner cases at the domain edge (±8 km) verifying 1 mm correctness;
  - out-of-range coordinates and out-of-range primitive sizes (below 0.1 m
    or above 250 m extent) verifying explicit rejection;
  - every test case checked by **both** the primary solver and the secondary
    validator, and the two compared against each other.
- Provide a `Makefile` (or single build script) that builds library, tests, and
  performance test with `-Wall -Wextra -Werror -O2`.

## Phase 3 — Build and test

Run the build and the test suite. Fix all warnings and failures. Record the
exact commands and their output. A clean build and a fully passing test run are
prerequisites for Phase 4.

## Phase 4 — Performance test

- `performance-test/` contains its own source and data.
- Generate input data for **1000 collision test pairs once**, statically; store
  it in a file committed under `performance-test/`. Generation is a separate
  tool/step and is never re-run by the timing harness.
- The harness must **not modify the input data**; it uses it directly. Verify
  this: checksum the input file before and after a run and fail if it changed.
- The harness **may** pre-convert the input file into a binary format better
  suited for loading. Pre-conversion time is **excluded** from the reported
  time; only time spent inside the collision calls across all 1000 pairs is
  measured and reported.
- Outputs, both required: 1. A text results file, one line per pair, fixed
  format (pair index,
     colliding flag, distance to a fixed number of decimals) — suitable for
     `diff`.
  2. The total collision time, printed to stdout.
- `run-performance-test` (project root, executable): builds if needed,
  pre-converts if needed, runs the harness, prints the time, writes the results
  file.
- Single-threaded throughput is the only metric. Do not report latency claims,
  projections, or any unmeasured performance statement; report only the
  measured total time, as a measurement on this machine.

## Phase 5 — Verify (mandatory, with evidence)

Run and capture output for all of the following:

1. Clean build from scratch (`make clean && make`), zero warnings.
2. Full test suite, all passing, including primary-vs-validator agreement.
3. `run-performance-test` twice; the two results files must be
   **byte-identical** (`diff` empty) and the input data file unchanged.
4. Validator agreement on all 1000 performance pairs (run validation over the
   same input once, outside the timed path).

Report what was verified with the actual commands and outputs, and state
explicitly anything that was not verified and why.

## Acceptance checklist

- [ ] `src/README.md` plan exists with data layouts, batch contract, and
      labeled ASSUMPTIONs.
- [ ] Builds clean with `-Wall -Wextra -Werror`; libc/libm only.
- [ ] All tests pass; ±8 km / 1 mm cases, size-range (0.1–250 m) rejection,
      and out-of-range coordinate rejection covered.
- [ ] Secondary validator is code-independent and agrees on every test and
      all 1000 performance pairs.
- [ ] Performance input generated once, stored in file, never modified by
      the harness (checksum-verified).
- [ ] Pre-conversion (if used) excluded from the reported time.
- [ ] Results text file is deterministic and diff-stable across runs.
- [ ] Total collision time printed; single-threaded; no unmeasured
      performance claims anywhere.
- [ ] Verification evidence (commands + output) included in the final
      report.
