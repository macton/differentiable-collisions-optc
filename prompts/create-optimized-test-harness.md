# Convex Primitive Collision Detection — Optimized Test Harness (Scaffold)

Instructions for an LLM agent. Build the **test, comparison, and measurement
scaffold** that `create-optimized.md`'s optimization loop will use to prove an
optimized solver is correct and fast. In this task the optimized library is a
**verbatim copy of the reference** `src/`, so the scaffold must demonstrate the
trivial identity baseline with real command output: **no measurable speedup
(~1.0x, ≈0% gain)**, **0.000 mm maximum distance deviation from the
reference**, **100% of pairs matching within the distance tolerance**,
**identical collision count**, and the full reference test suite passing
against the copy. Never report a step as complete without running it; never
fabricate results or measurements.

If the scaffold reports anything other than the identity baseline above when
both libraries are byte-identical copies, the scaffold is wrong — fix it before
finishing.

## Why this exists

`create-optimized.md` requires, on every iteration, that you compare an
optimized solver against the reference under a tolerance criterion, measure a
median-of-5 speedup, run the full reference test suite against the optimized
library, run the independent validator, and run a generalization check on
alternate-seed inputs. That machinery must exist and be proven correct
**before** any real optimization is attempted — otherwise a passing report
proves nothing. This task builds that machinery and proves it against the one
input for which the answer is known exactly: an identical copy.

The data-oriented point: the core of this scaffold is a **results-file
comparator** — a pure data transform. Input: two results files in the fixed
reference format. Output: a match verdict (collision-count equality + per-pair
distance within tolerance) and a max-deviation number. When the two producing
libraries are identical, that transform must yield PASS with zero deviation.
Build and verify that transform first; everything else is plumbing around it.

## Read first (do not modify any of these)

- `create-reference.md` and `create-optimized.md` (same directory) — the
  contracts this scaffold serves. The optimized solver's fixed constraints,
  tolerance criterion, measurement protocol, and generalization check are
  defined there; reproduce them faithfully, do not invent variants.
- `../README.md` — project overview and measured reference baseline.
- `../run-performance-test`, `../Makefile`.
- `../src/collide.h`, `../src/collide.c` — the reference library (the copy's
  starting point).
- `../test/test_main.c`, `../test/validator.c`, `../test/validator.h` — the
  full correctness suite and the independent validator.
- `../performance-test/`: `gen_pairs.c`, `pairs_io.c/.h`, `perf_main.c`,
  `validate_main.c`, `pairs.txt` (committed input), `results.txt`.

The committed input `../performance-test/pairs.txt`, the reference `src/`,
`test/`, and the reference harness are the comparison baseline. **Never modify
them.** Do not overwrite `performance-test/pairs.txt`. Do not alter existing
reference build targets.

## Fixed facts to reproduce (from create-optimized.md — do not relax)

- **Results-file format** (one line per pair, already produced by the reference
  harness): `"%04u %u %.6f\n"` = pair index, colliding flag (0/1), distance in
  meters to 6 decimals.
- **Tolerance / match criterion** (both must hold): (1) the colliding flag is
  identical for every pair, so the **collision count and the exact set of
  colliding pairs are identical**; and (2) for every pair the per-pair distance
  agrees within the **hybrid tolerance** |d_opt − d_ref| ≤ 1 mm + 0.1%·|d_ref|
  + 5e-4·|c1−c2|/alpha² — an absolute floor (tight near contact, where a pure
  relative test would explode as distance→0) plus a relative term for large
  separations/penetrations. The results file's per-pair distance is the
  quantity the comparator checks. A `diff` is **not** the comparator — the
  comparator must implement the count + hybrid-tolerance check. Contact points
  are NOT matched against the reference (non-unique); a separate
  `validate_contacts` certifies them for geometric validity instead.
- **Independent validator**: must agree within the hybrid tolerance **1 mm +
  0.1%·|d|** on every pair, using `test/validator.c` unchanged (shared with the
  reference; not reweakened).
- **Measurement protocol**: run a harness 5 times back-to-back, take the
  **median** total collision time. WSL2 is noisy; single runs are not evidence.
  Speedup = reference median ÷ optimized median.
- **Pre-processing is excluded from timed collision time** and must be a
  general transform over any valid input file.
- **No overfitting**: nothing keyed to the committed 1000 pairs.

## Deliverables (create exactly these; start from copies)

    src-optimized/                 verbatim copy of src/ (collide.c, collide.h);
                                   the library create-optimized.md will edit
    performance-test-optimized/    copy of the harness/comparison sources,
                                   retargeted to link src-optimized; holds the
                                   optimized results files (NOT a copy of
                                   pairs.txt — both harnesses read the one
                                   committed input)
    run-performance-test-optimized project-root script, same contract as
                                   run-performance-test, building/running the
                                   optimized harness and writing
                                   performance-test-optimized/results.txt

Build wiring: add optimized build targets that compile `src-optimized/` and
`performance-test-optimized/` into `build/` under **distinct** object and
binary names (e.g. `build/collide_opt.o`, `build/perf_harness_opt`,
`build/test_runner_opt`, `build/perf_validate_opt`, `build/perf_gen_seed`,
`build/compare_results`). Do this **without changing any existing reference
target**. Either add new `.PHONY` targets to the root `Makefile` or add a
separate `Makefile.optimized`; either way `-std=c11 -Wall -Wextra -Werror -O2`,
libc/libm only, single-threaded.

### The comparator (build this first, it is the core)

`performance-test-optimized/compare_results.c` → `build/compare_results`.

- Usage: `compare_results <reference-results> <optimized-results>`.
- Reads both files in the fixed format above.
- **Out-of-range / malformed behavior is explicit and loud**: if the two files
  have a different number of pairs, if a line is malformed, or if a pair index
  is out of order/missing, print a specific error and exit non-zero. Do not
  silently skip.
- Checks: (a) per-pair colliding flag identical → derive reference count,
  optimized count, and number of pairs whose flag differs; (b) per-pair `|d_opt
  − d_ref|`, tracking the maximum (absolute and relative) and the count
  exceeding the hybrid tolerance `1 mm + 0.1%·|d_ref| + 5e-4·|c1−c2|/alpha²`.
- Prints, in a fixed parseable form: number of pairs, reference collision
  count, optimized collision count, flag-mismatch count, **max |distance diff|
  in mm and %**, the distance tolerance, and the number of pairs over it.
- Exit 0 only when flag-mismatch count is 0 **and** no pair exceeds the hybrid
  distance tolerance (identical collision count is implied by zero flag
  mismatches). Exit non-zero otherwise.

This is the comparator `create-optimized.md` refers to; it will be reused by
that loop. Treat it as the contract boundary.

### Optimized harness and tests (copies retargeted to src-optimized)

- Copy `performance-test/perf_main.c`, `pairs_io.c`, `pairs_io.h`,
  `validate_main.c` into `performance-test-optimized/`, changing only the
  include path so they resolve `src-optimized/collide.h` (the library contract)
  and link `build/collide_opt.o`. Keep the harness logic identical to the
  reference for this scaffold — the optimization loop will change it later. Do
  **not** weaken the timing, checksum, or determinism behavior.
- The optimized timing harness reads the committed
  `../performance-test/pairs.txt` (read-only, checksum-verified before/after,
  same as the reference), pre-converts to a binary under `build/` (excluded
  from timing), and writes `performance-test-optimized/results.txt`.
- Optimized test runner: compile `test/test_main.c` + `test/validator.c`
  against `src-optimized/collide.o` into `build/test_runner_opt`. This runs the
  **full reference correctness suite** (same cases, same validator cross-check)
  against the optimized library. Do not copy-and-weaken the tests.
- Optimized validator harness: compile `validate_main.c` + `test/validator.c`
  against `src-optimized/collide.o` into `build/perf_validate_opt`; agrees
  within 1 mm + 0.1% (+ alpha-conditioning) on every pair of an input set.

### Alternate-seed generation (for the generalization check)

The reference `gen_pairs.c` has a fixed seed and fixed count and must not be
modified. Create `performance-test-optimized/gen_pairs_seed.c` (a copy of
`gen_pairs.c`) that takes the **seed** (and optionally pair count) as
command-line arguments and writes to a caller-named path, so alternate input
sets can be generated **without touching the committed input**. Build it as
`build/perf_gen_seed`. It must produce only in-domain pairs exactly as the
reference generator does (re-roll rejects). Write alternate sets under `build/`
or `/tmp` only.

### Measurement script

`performance-test-optimized/measure-speedup.sh` (or equivalent at a stated
path): runs a given harness binary on a given input 5 times, parses the printed
total collision time, prints the 5 samples and the **median**; computes speedup
= reference median ÷ optimized median. This is the single measurement protocol
used everywhere.

### Top-level proof driver

`prove-optimized-harness.sh` at project root: runs the whole scaffold end to
end on the identity copy and prints a summary. Steps, each with real output:

1. Clean build of reference and optimized targets (`-Wall -Wextra -Werror`,
   zero warnings).
2. `./run-performance-test` → `performance-test/results.txt` (reference).
3. `./run-performance-test-optimized` →
   `performance-test-optimized/results.txt`.
4. `build/compare_results performance-test/results.txt
   performance-test-optimized/results.txt` → expect: equal collision counts, 0
   flag mismatches, **max distance diff 0.000 mm**, 0 pairs over tolerance,
   PASS. (Because the libraries are identical, the two files are in fact
   byte-identical here; the comparator still applies the tolerance criterion,
   since later optimized versions will not be byte-identical.)
5. `build/test_runner_opt` → full reference suite passes against the copy.
6. `build/perf_validate_opt performance-test/pairs.txt` → validator agrees
   within 1 mm + 0.1% (+ alpha-conditioning) on all pairs (here exactly).
7. Median-of-5 for both harnesses on the committed input via the measurement
   script → report both medians and the speedup factor. Expect **≈1.0x (no
   measurable gain)**; state explicitly that any deviation from 1.0 is pure
   measurement noise on this WSL2 host (identical code cannot be faster than
   itself) — do not dress noise up as a speedup.
8. Generalization check: generate **≥2** alternate-seed inputs with
   `build/perf_gen_seed` under `build/`; for each, run the reference harness
   and the optimized harness and run `compare_results` → expect PASS (identical
   counts, 0.000 mm) on every set; report each set's speedup too.
9. Determinism: run the optimized harness twice on the same input; the two
   `results.txt` files must be **byte-identical** (`diff` empty).
10. Print a final summary block: speedup ≈1.0x (≈0% gain), max distance
    deviation 0.000 mm, 100% of pairs within the distance tolerance, collision counts
    equal, full test suite pass, validator agreement, determinism confirmed,
    committed input checksum unchanged.

Save the summary to `performance-test-optimized/HARNESS-BASELINE.md` (the
identity baseline, with the actual commands and output as evidence).

## Done means (identity baseline, with evidence)

All of these, shown with real command output:

- Builds clean: `-Wall -Wextra -Werror`, libc/libm only, single-threaded.
- `compare_results` on committed input: collision counts equal, 0 flag
  mismatches, max distance diff **0.000 mm**, 0 pairs over the distance
  tolerance, PASS; and the two results files are byte-identical here.
- `test_runner_opt`: full reference suite passes against the copy.
- `perf_validate_opt`: agrees within 1 mm + 0.1% (+ alpha-conditioning) on all
  pairs (here exactly).
- Speedup ≈1.0x reported as **no measurable gain**, with the 5 samples and
  medians shown; noise labeled as noise, not claimed as speedup.
- Generalization: ≥2 alternate-seed sets, each PASS under the comparator,
  speedups reported.
- Determinism: two optimized runs byte-identical.
- Committed `performance-test/pairs.txt` checksum unchanged; reference `src/`,
  `test/`, and reference harness untouched.

## Hand-off to create-optimized.md

State explicitly in `HARNESS-BASELINE.md` that the optimization loop in
`create-optimized.md` reuses, unchanged: `build/compare_results` as the
tolerance comparator, `measure-speedup.sh` as the median-of-5 protocol,
`build/test_runner_opt` / `build/perf_validate_opt` as the correctness and
validator gates, `build/perf_gen_seed` for the Phase 4 generalization check,
and `run-performance-test-optimized` as the optimized run contract. The
optimization work then edits only `src-optimized/` (and, where the
pre-processing layout requires it, `performance-test-optimized/`), with this
identity baseline as iteration-0 evidence that the measurement pipeline itself
is sound.

## Acceptance checklist

- [ ] `src-optimized/` is a verbatim copy of `src/` (collide.c, collide.h)
      at start; reference `src/` untouched.
- [ ] `performance-test-optimized/` holds the retargeted harness sources and
      its own results files; committed `pairs.txt` not copied or modified.
- [ ] `run-performance-test-optimized` exists at project root, same contract
      as `run-performance-test`, checksum-verifies the committed input.
- [ ] Optimized build targets are separate from reference targets; clean
      build with `-Wall -Wextra -Werror`, libc/libm only.
- [ ] `build/compare_results` implements the count + hybrid (1 mm + 0.1% +
  conditioning) distance tolerance
      criterion (not a `diff`), with explicit loud failure on malformed or
      mismatched-length inputs; exit code reflects PASS/FAIL.
- [ ] `build/test_runner_opt` runs the full reference suite (not a weakened
      copy) against the optimized library and passes.
- [ ] `build/perf_validate_opt` agrees within 1 mm + 0.1% (+
  alpha-conditioning) on all pairs.
- [ ] `build/perf_gen_seed` generates in-domain alternate-seed inputs
      without touching the committed input.
- [ ] `measure-speedup.sh` implements the median-of-5 protocol and a speedup
      ratio.
- [ ] `prove-optimized-harness.sh` runs all steps and reports the identity
      baseline: ≈1.0x (no measurable gain), 0.000 mm max deviation, 100%
      within the distance tolerance, equal collision counts, suite pass, validator
      agreement, determinism, checksum unchanged.
- [ ] `HARNESS-BASELINE.md` records the proof with real command output and
      the hand-off note to `create-optimized.md`.
- [ ] No unmeasured performance claim anywhere; noise labeled as noise.
