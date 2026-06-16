# Optimized Harness — Identity Baseline (iteration 0)

This records the proof that the optimized **test, comparison, and measurement
scaffold** is sound, run against the one input for which the answer is known
exactly: `src-optimized/` is a **verbatim copy** of `src/`. Identical code
cannot be faster than itself and cannot disagree with itself, so the correct
baseline is **no measurable speedup (~1.0x)** and **0.000 mm** deviation. The
scaffold reports exactly that. If it ever reported otherwise on byte-identical
libraries, the scaffold would be wrong.

All numbers below are real command output captured by
`prove-optimized-harness.sh` into `performance-test-optimized/proof-run.log`
(proof run dated 2026-06-14T05:56:45Z). Re-run with `./prove-optimized-harness.sh`.

## Precondition (verbatim copy)

    $ diff -q src/collide.c src-optimized/collide.c   # (no output: identical)
    $ diff -q src/collide.h src-optimized/collide.h   # (no output: identical)

## Build (clean, zero warnings)

`-std=c11 -Wall -Wextra -Werror -O2`, libc/libm only, single-threaded.
Reference targets via `make all`; optimized targets via
`make -f Makefile.optimized optimized`, into distinct names:
`build/collide_opt.o`, `build/perf_harness_opt`, `build/test_runner_opt`,
`build/perf_validate_opt`, `build/perf_gen_seed`, `build/compare_results`.
No existing reference target is modified.

## Step 4 — comparator on committed input (the core transform)

    $ ./build/compare_results performance-test/results.txt performance-test-optimized/results.txt
    pairs 1000
    ref_collisions 217
    opt_collisions 217
    flag_mismatches 0
    max_distance_diff_mm 0.000000
    pairs_over_tolerance 0
    PASS        (exit 0)

The two results files are also byte-identical here (`diff` empty), as expected
for an identity copy. The comparator still applies the count + hybrid (1 mm + 0.1%)
tolerance criterion (not a `diff`), because later optimized versions will not
be byte-identical.

## Step 5 — full reference suite against the optimized library

    $ ./build/test_runner_opt
    ...
    178 passed, 0 failed       (exit 0)

The full reference correctness suite (`test/test_main.c` + `test/validator.c`,
unchanged) compiled against `build/collide_opt.o`.

## Step 6 — independent validator within 1 mm + 0.1%

    $ ./build/perf_validate_opt performance-test/pairs.txt
    validated 1000 pairs: 217 colliding, max |dist diff| = 0.000002432 m, distance failures = 0, flag mismatches = 0

Max validator disagreement ~2.4 µm, well within the 1 mm bound.

## Step 7 — median-of-5 speedup on the committed input

    reference samples: 0.329580 0.332494 0.323951 0.328585 0.327819 s   median 0.328585 s
    optimized samples: 0.328840 0.331068 0.332232 0.332047 0.330269 s   median 0.331068 s
    speedup (ref median / opt median): 0.9925x

The final-summary independent re-measure gave reference median 0.322542 s,
optimized median 0.323326 s, **0.9976x**. **This is not a speedup.** Identical
code cannot be faster than itself; sub-percent deviation from 1.0x is the WSL2
measurement-noise floor. It is reported as **no measurable gain (≈0%)**.

## Step 8 — generalization on alternate-seed inputs (≥2)

Three alternate in-domain sets generated with `build/perf_gen_seed` under
`build/` (committed input never touched):

| set | seed                 | colliding | comparator        | speedup (noise) |
|-----|----------------------|-----------|-------------------|-----------------|
| 1   | 0xA5A5A5A5DEADBEEF   | 196       | PASS, 0.000 mm    | 1.0032x         |
| 2   | 0x0123456789ABCDEF   | 220       | PASS, 0.000 mm    | 0.9839x         |
| 3   | 0xFEEDFACECAFEF00D   | 232       | PASS, 0.000 mm    | 0.9984x         |

Every alternate set: equal collision counts, 0 flag mismatches, 0.000 mm max
deviation, 0 pairs over tolerance, PASS. Speedups are noise around 1.0x.

`build/perf_gen_seed` is verified to reproduce the committed `pairs.txt`
**byte-for-byte** when given the reference seed `0x123456789ABCDEF7`, and to
produce distinct in-domain sets for distinct seeds.

## Step 9 — determinism

    $ ./build/perf_harness_opt performance-test/pairs.txt build/det_a.bin build/det_a.res
    $ ./build/perf_harness_opt performance-test/pairs.txt build/det_b.bin build/det_b.res
    $ diff -q build/det_a.res build/det_b.res        # (no output: byte-identical)

## Checksum

Committed `performance-test/pairs.txt` sha256 unchanged before and after:

    d8b048ee2c2955509dd356afc6367b1629859d9047f2dace24d88ce42848fd4d

Reference `src/`, `test/`, and the reference harness are untouched.

## Final summary (identity baseline)

- collision counts: reference == optimized (equal)
- flag mismatches: 0
- max distance deviation: **0.000 mm**
- pairs within the distance tolerance: **100%**
- full test suite: **178 passed, 0 failed** against the copy
- independent validator: agrees within 1 mm + 0.1% (max ~2.4 µm)
- determinism: two optimized runs byte-identical
- speedup: **≈1.0x — no measurable gain** (noise on this WSL2 host)
- committed input checksum: unchanged

## Comparator out-of-range / malformed behavior (explicit and loud)

`build/compare_results` exits non-zero with a specific message on: differing
pair counts, malformed line, bad colliding flag, out-of-order/missing index.
PASS (exit 0) only when flag-mismatch count is 0 AND no pair exceeds the hybrid distance tolerance (1 mm + 0.1%).

## Hand-off to create-optimized.md

The optimization loop in `create-optimized.md` reuses, **unchanged**:

- `build/compare_results` — the tolerance comparator (count + hybrid 1 mm + 0.1%).
- `performance-test-optimized/measure-speedup.sh` — the median-of-5 protocol;
  speedup = reference median ÷ optimized median.
- `build/test_runner_opt` — full reference correctness suite gate.
- `build/perf_validate_opt` — independent validator gate (1 mm).
- `build/perf_gen_seed` — Phase 4 generalization input generator.
- `run-performance-test-optimized` — the optimized run contract
  (checksum-verifies the committed input, pre-conversion excluded from timing).

The optimization work edits only `src-optimized/` (and, where the
pre-processing layout requires it, `performance-test-optimized/`). This
identity baseline is iteration-0 evidence that the measurement pipeline itself
is sound: on byte-identical libraries it yields PASS, 0.000 mm, ≈1.0x.
