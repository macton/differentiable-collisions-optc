
# Convex Primitive Collision Detection — Optimized Implementation

Instructions for an LLM agent. You must **implement, build, test, measure, and
iterate** until the performance target is reached or you can prove no further
optimization opportunities remain. "Done" means one of the two exit criteria at
the bottom is met **with real command output as evidence**. Never report a step
as complete without having run it; never fabricate results or measurements.

## Context

A working reference implementation already exists, and so does the full **test,
comparison, and measurement harness** you will use to prove your work. Before
writing any code:

- Read `create-reference.md` (same directory) and `../README.md` for the
  reference's requirements, layout, and measured baseline.
- Read the reference source under `src/`, the test suite under `test/`, and the
  harness under `performance-test/`.
- Read `../performance-test-optimized/HARNESS-BASELINE.md` — the
  test/comparison/measurement scaffold is **already built and proven** against
  an identity copy (iteration-0 evidence that the measurement pipeline is
  sound). You **reuse** it; you do not rebuild it.
- Read the paper `{project-root}/documents/2207.00669.pdf` as needed to
  understand what the solver computes.

Reference baseline on the development machine: ~0.33 s total collision time for
the committed 1000-pair input. Your baseline is whatever you **measure** on
this machine, not that number.

## The test harness already exists — use it, do not rebuild it

A previous task built and verified the full scaffold; its identity baseline
(≈1.0x, 0.000 mm deviation, suite pass, validator agreement, determinism) is
recorded in `performance-test-optimized/HARNESS-BASELINE.md`. `src-optimized/`
already exists as a **verbatim copy of `src/`** — that copy is your starting
point.

**Your job is to edit `src-optimized/` (the solver), plus only the build and
data pre-transformation steps your new code actually needs. Do not recreate,
weaken, or fork any harness piece.** Reuse these exactly as they are:

- `run-performance-test-optimized` — builds the optimized targets, pre-converts
  the committed input (excluded from timing), runs the optimized timing
  harness, checksum-verifies `performance-test/pairs.txt`, and writes
  `performance-test-optimized/results.txt`.
- `build/compare_results <ref-results> <opt-results>` — the tolerance
  comparator (collision-count equality + per-pair distance under the hybrid
  tolerance `|Δ| ≤ 1 mm + 0.1%·|d_ref| + 5e-4·|c1−c2|/alpha²`). Prints a fixed
  parseable report and exits 0 only on PASS. **This is the matcher; never
  substitute a `diff`.** Contact points are NOT matched against the reference
  here (they are non-unique); they are certified for geometric validity by
  `validate_contacts`.
- `performance-test-optimized/measure-speedup.sh` — the median-of-5 protocol.
  `measure-speedup.sh <ref-harness> <opt-harness> <input.txt>` prints both
  medians and `speedup = ref median ÷ opt median`.
- `build/test_runner_opt` — the **full reference correctness suite**
  (`test/test_main.c` + `test/validator.c`, unchanged) against the optimized
  library.
- `build/perf_validate_opt <pairs.txt>` — the independent validator (agrees
  within the hybrid tolerance `1 mm + 0.1%·|d| + 5e-4·|c1−c2|/alpha²` on every
  pair).
- `build/perf_gen_seed <out.txt> <seed> [npairs]` — alternate-seed in-domain
  input generator for the Phase 4 generalization check (writes under `build/`
  or `/tmp`; with the reference seed it reproduces the committed input
  byte-for-byte).
- `Makefile.optimized` (targets: `collide_opt.o`, `perf_harness_opt`,
  `test_runner_opt`, `perf_validate_opt`, `perf_gen_seed`, `compare_results`) —
  the optimized build. **Extend it only** if your code adds a new source file
  or a pre-processing step; never touch a reference target.

If you add a separate **pre-processing** step (allowed and free, excluded from
the timed collision time), wire it into `Makefile.optimized` and
`run-performance-test-optimized` as new build/run steps, and keep it a
**general transform** over any valid input file (see "No overfitting"). Do not
modify `src/`, `test/`, `performance-test/`, or the committed
`performance-test/pairs.txt`.

## Goal

A platform-specific optimized version of the collision solver with a target of
**100x improvement** in measured total collision time over the reference, on
this machine, producing results that **match the reference within tolerance**
(identical collision count; per-pair distance within the tolerance `1 mm +
0.1%·|d_ref| + 5e-4·|c1−c2|/alpha²`; contact points certified geometrically
valid rather than matched to the reference — see the contract below).

## Fixed constraints (inherited from the reference — do not relax any)

- Exactly the same functional constraints as the reference: meters; every world
  AABB corner within ±8,192 m per axis; AABB extent within [0.1, 250] m per
  axis; out-of-range input **rejected with an explicit error**, never clamped
  or silently accepted; results correct to 1 mm; same four primitives through
  one solver path; deterministic — same input bytes → same output bytes, every
  run.
- **Single-threaded only.** No threads. SIMD within one thread is allowed and
  expected.
- C (C99/C11), gcc, libc + libm only. Build with `-Wall -Wextra -Werror`.
- **Platform-specific optimization is the point**: x86_64 intrinsics, SSE / AVX
  / FMA, inline assembly, `-march=native`, cache-conscious layout — anything
  **this host actually supports**. Inspect the host first (`/proc/cpuinfo`,
  `gcc -march=native -Q --help=target`, cache sizes) and record what you found;
  do not assume a feature exists.
- **Results must match the reference within tolerance.** For any valid input
  set, compare the optimized harness's results file (same fixed format: pair
  index, colliding flag, distance, alpha, contact points) against the reference
  harness's results file on the same input, **using `build/compare_results`**
  (a tolerance comparison, never a byte `diff`). Results **match** when
  **both** conditions hold: (1) the **collision count is identical** — the
  optimized run flags exactly the same set of pairs as colliding as the
  reference (same count, same pairs); and (2) **every pair's distance is within
  the hybrid tolerance** `|d_opt − d_ref| ≤ 1 mm + 0.1%·|d_ref| +
  5e-4·|c1−c2|/alpha²` — an absolute floor that stays tight near contact (where
  a pure relative test would explode as distance→0) plus a relative term for
  large separations and deep penetrations (where the solver carries fixed
  *relative* precision and demanding 0.5 mm absolute out of a multi-metre value
  is over-specified). An optimization that changes the collision count, or
  moves any distance outside that band, is rejected, however fast.
- **Contact points are certified for validity, not matched.** A face/edge
  contact has many equally-valid witness points, so optimized contacts need not
  equal the reference's. `build/validate_contacts` independently certifies each
  emitted contact lies on both shapes' surfaces and is separated by the
  reported distance (coordinate-aware float tolerance). The contact-vs-
  reference deviation `compare_results` prints is informational only.
- The independent validator (`build/perf_validate_opt`) must also agree within
  the hybrid tolerance `1 mm + 0.1%·|d| + 5e-4·|c1−c2|/alpha²` on every pair.
- **Build-stage precompute is allowed and free, but capped.** Build-stage
  preparation lives in exactly two files, and these are the **only** files the
  build-stage transforms may use:
    - `performance-test-optimized/build_optimized_shapes.c`
      (`shapes.bin` → `shapes_optimized.bin`) — per-shape work: validation,
      face construction, world-space geometry, SoA/aligned layout, precomputed
      per-shape constants.
    - `performance-test-optimized/build_optimized_pairs.c`
      (`pairs.bin` → `pairs_optimized.bin`) — pair-list work: reordering,
      bucketing by pair type, alignment.
The time to RUN these transforms is **excluded from the collision time used for
speedup** (the runtime consumes their output directly). **But the total
precompute time across both transforms must be ≤ 60 minutes** — the prove
harness times it, fails the run if it exceeds the cap, and reports it in the
final summary, so precompute cannot become unbounded. Each transform must be a
**general transform** of *any* valid input — see "No overfitting" below.
- **The two transforms are isolated on purpose.** `build_optimized_shapes` sees
  only the shapes and `build_optimized_pairs` sees only the pairs; neither sees
  both. So the build stage **cannot** precompute collision answers (an answer
  needs shapes AND pairs together, which meet only at runtime). Do not defeat
  this: do not merge the two transforms or feed one's input to the other.

## No overfitting (this will be graded on different data)

The optimized solver will later be evaluated on a **different input pair set**
with the same constraints and format. Therefore:

- No precomputed answers, no caching of results keyed to the committed data, no
  constants derived from the specific 1000 pairs.
- Do not tune thresholds, bucket sizes, or branch layouts to this dataset's
  particular distribution (e.g. its 217/1000 collision rate) beyond what the
  stated constraints guarantee for all inputs.
- Partitioning by pair *type* (sphere–sphere, box–capsule, ...) and processing
  each bucket straight-line is encouraged — that exploits the constraint set,
  not this dataset.
- The generalization check in Phase 4 is mandatory and exists to catch exactly
  this failure mode.

## Layout (do not modify the reference)

Leave `src/`, `test/`, `performance-test/pairs.txt`, and the reference harness
untouched — they are your comparison baseline. The optimized scaffold below
**already exists**; you **modify only** the optimized library and the two
build-stage transforms, plus the build wiring they require:

    src-optimized/                optimized solver/runtime — EDIT THIS; add
                                  README.md plan + OPTIMIZATION-LOG.md
    performance-test-optimized/build_optimized_shapes.c
                                  build-stage SHAPE transform — EDIT THIS
                                  (sees only shapes; precompute per-shape data)
    performance-test-optimized/build_optimized_pairs.c
                                  build-stage PAIR transform — EDIT THIS
                                  (sees only pairs; reorder/bucket the pair list)
    performance-test-optimized/   harness, comparator, measure script (exist) —
                                  edit only if a new precompute layout requires it
    run-performance-test-optimized   optimized run script at project root (exists)
    Makefile.optimized            optimized build targets (exists; extend only for
                                  a new source file or build step)

`build/test_runner_opt` already runs the **full reference correctness suite**
(same cases, same validator cross-check) against the optimized library — keep
it that way; do not write a weaker copy.

## Phase 1 — Plan and baseline (before optimizing)

1. Write `src-optimized/README.md`: host CPU features and cache sizes as
   measured; where the reference spends its time — profile **where the cycles
   go**, not just how many times each routine is called (`perf record` / `perf
   stat` if available, else `gprof` or coarse manual timers around solver
   stages — state which you used). A call-count profile is not enough on its
   own: it makes throughput, data-layout, and SIMD wins **invisible**, because
   those remove no calls — they compress the cycles spent *per* call. Then a
   candidate list of optimizations **ranked by expected payoff**, where payoff
   is the **fraction of measured runtime a change touches × the speedup you
   expect on that fraction** (Amdahl) — *not* merely the count of operations it
   removes. Note implementation effort separately if you like, but **do not
   rank by it** — payoff is the ordering key. Mark every unverifiable fact as
   `ASSUMPTION: <fact> — affects <decision>`.
2. The **measurement protocol** already exists as
   `performance-test-optimized/measure-speedup.sh` (run the harness 5 times
   back-to-back, report the median total collision time; WSL2 is noisy, so
   single runs are not evidence). **Use it for every number in this project —
   do not invent a second protocol.**
3. Measure the reference baseline with that script and record it
   (`measure-speedup.sh ./build/perf_harness performance-test/pairs.txt`).

## Phase 2 — Implement the optimized solver

`src-optimized/` already starts as a correct port (verbatim copy of `src/`), so
begin from it and optimize. This rule is about what you *keep*, not what you
*attempt*: never **commit or carry forward** a broken state across iterations —
every kept iteration ends with a clean build (`-Wall -Wextra -Werror`,
libc/libm only) and a passing test suite. Attempting a hard change that ends up
broken is fine and expected; you simply revert it. Do not let "keep the tree
clean" talk you out of trying a high-payoff change.

## Phase 3 — Iterate (the core loop)

Repeat until an exit criterion (below) is met. Each iteration:

1. **Pick** the candidate with the **highest expected payoff** — the fraction
   of measured runtime it touches × the speedup you expect on that fraction
   (Amdahl) — *not* the safest or simplest one, and *not* merely the one that
   eliminates the most operations. Your candidate list must span **two kinds**
   of change, not one: **(a) work removal** — skip, precompute into the free
   pre-processing step, fewer iterations, cheaper math; and **(b) throughput /
   data layout** — batch across pairs in SoA form, partition pairs by type so
   each batch runs one branch-free path, align to vector width, and use SIMD
   (SSE/AVX/FMA) over the loops that dominate runtime. A throughput change
   removes no calls, so a call-count profile ranks it ≈0 — rank it instead by
   runtime-fraction × expected lane/pipeline speedup, or you will never select
   it. Implementation risk is **not** a selection criterion here: a failed
   attempt costs about one iteration and a logged negative result (step 6),
   nothing more. Do not keep choosing small, low-risk specializations: if the
   hottest family is the hardest, that is exactly where the target lives —
   attempt it. Only break a near-tie in payoff by preferring the simpler
   change. If you are about to skip the top-payoff candidate because it seems
   hard or risky, that is the signal to do it next, not to defer it. Then run
   the simplification pass on the change you picked — prefer removing work over
   doing the same work faster **when the payoff is comparable**; but
   "same work faster" over a loop that dominates runtime (vectorizing it,
   making it branch-free) is itself high payoff, not a lesser category — do not
   demote it.
2. **Implement** that one change in `src-optimized/` (and, if it needs a new
   pre-processing layout, in `performance-test-optimized/` +
   `Makefile.optimized` + `run-performance-test-optimized`).
3. **Gate on correctness**, in order, using the existing harness; a failure at
   any gate means fix or revert before measuring: - clean build (`make -f
   Makefile.optimized optimized`), full test suite
     passes (`build/test_runner_opt`);
   - `./run-performance-test-optimized`, then
     `build/compare_results performance-test/results.txt
     performance-test-optimized/results.txt` reports **PASS** (identical
     collision count, distance within 1 mm + 0.1% (+ alpha-conditioning)), committed input
     checksum unchanged;
   - validator agrees within 1 mm + 0.1% (+ alpha-conditioning) on all pairs
     (`build/perf_validate_opt performance-test/pairs.txt`);
   - two consecutive optimized runs produce byte-identical results files.
4. **Measure** with the protocol (`measure-speedup.sh ./build/perf_harness
   ./build/perf_harness_opt performance-test/pairs.txt`, median of 5).
5. **Record** in `src-optimized/OPTIMIZATION-LOG.md`: hypothesis, change,
   before/after medians, speedup vs reference so far, keep/revert decision, and
   the **cost of this hypothesis**: total wall-clock time (LLM time + any shell
   time) and total tokens (in + out) spent on it. Measure both by diffing the
   `utc` and `tokens_in_total`/`tokens_out_total` fields of the
   `nagent-turn-status` lines that bracket the work for this hypothesis (first
   turn that began it through the turn that recorded it); state the two
   endpoints you diffed.
6. **Keep or revert — and make every kept state durable.** Keep a change if it
   passed every gate and either **(a)** measurably helps, **(b)** is a pure
   simplification (less code, same speed), or **(c)** is an **enabling transform**
   (next paragraph) that is correctness-preserving and net-neutral. When you keep,
   **commit it immediately**: `git add` the changed `src-optimized/` files (plus any
   build / pre-processing files and `OPTIMIZATION-LOG.md`) and `git commit`. That
   commit is now your baseline. Otherwise **revert to that last
   committed baseline** (`git checkout -- <changed files>`) — which is safe
   *precisely because* every kept state was committed. Keep the log entry either
   way; a rejected hypothesis with its measurement is a result. Never let a kept
   improvement live only in the uncommitted working tree: the working tree is not
   durable state, and a later failed attempt's revert (`git checkout`) will
   silently destroy any kept-but-uncommitted gains. Committing after each green
   gate is what turns "revert" into a safe operation instead of a destructive one.
7. **Update** the candidate list: remove what was tried, add anything the
   measurement revealed.

**Enabling transforms broaden the reachable space — keep them even when neutral.**
Some changes don't speed anything up by themselves but *make other optimizations
possible*: converting a branchy hot path to **branch-free** (partition by
state / pair-type, then run each partition straight-line), converting AoS to
**SoA**, **aligning** to vector width. A branchy, AoS, scalar loop cannot be
vectorized; the same loop made branch-free and SoA can. Treat these as
first-class moves, not detours: if such a transform preserves correctness (all
gates pass) and is net performance-neutral, **keep and commit it** (step 6c) —
it is paying you in *optionality*, not cycles, and removing the branches is
itself the prerequisite that makes SIMD, partitioned batch passes, and
straight-line specialization reachable on the next iteration. When a transform
is a declared prerequisite for a specific follow-on optimization, judge the
**arc** (the foundation plus the change built on it) by its *end state*, not by
the foundation's standalone delta — do not revert the foundation merely because
it had not paid off by itself yet. Bound this honestly (DOD: complexity requires
evidence): keep a neutral enabling transform only when you can **name the
concrete optimization it unlocks** and intend to attempt that next — "might help
someday" is not a reason to carry complexity.

Never stack multiple speculative changes into one measurement; you must be able
to attribute the effect of each change. (A declared enabling-transform arc is
the one exception, and only because each step in it is still gated and committed
separately: keep the foundation and the change built on it as two commits, each
passing all correctness gates, so the speed effect is still attributed to the
step that produced it.)

## Phase 4 — Generalization check (mandatory before claiming any exit)

1. Generate at least 2 alternate input sets with different PRNG seeds using
   `build/perf_gen_seed <out.txt> <seed> [npairs]`, written under `build/` or
   `/tmp` — **never overwrite the committed `performance-test/pairs.txt`**.
2. For each alternate set: run the reference harness and the optimized harness
   on it and compare with `build/compare_results`; the results must match under
   the tolerance criterion above (identical collision count, distance within 1
   mm + 0.1%), and the measured speedup (via `measure-speedup.sh`) must be
   reported alongside the committed-input speedup.
3. If speedup collapses or results diverge on alternate data, the optimization
   is overfit or wrong: diagnose, fix or revert, and return to Phase 3.

(`prove-optimized-harness.sh` at the project root runs all of these gates end
to end and is a convenient way to re-confirm the whole pipeline after a
change.)

## Exit criteria (exactly one must be met, with evidence)

- **TARGET REACHED**: median measured total collision time on the committed
  input is ≥100x faster than the measured reference baseline, all Phase 3
  correctness gates pass, and the Phase 4 generalization check passes.
- **NO FURTHER OPPORTUNITIES**: the candidate list is empty — every candidate
  was either (a) implemented and kept, (b) implemented, measured, and reverted
  with its numbers in the log, or (c) rejected with a stated cost reason (e.g.
  "requires threads — prohibited", "breaks result identity — measured
  divergence"). Report the best achieved speedup as the result. "I ran out of
  ideas" without a log of tried-and-measured candidates is not this criterion.

Do not stop for any other reason. Do not claim the target from a projection or
a single noisy run.

## Final report (whichever exit is taken)

- Measured reference baseline and final optimized time (protocol medians), and
  the resulting speedup factor — on this machine, no projections.
- The optimization log: what was tried, what each change measured, what was
  kept.
- Generalization results on the alternate input sets.
- Verification evidence: build output, test output, the tolerance comparison
  against reference results (collision count match and per-pair hybrid 1 mm +
  0.1% distance check), checksum checks, determinism check — actual commands
  and output.
- Anything not verified, stated explicitly, with why.

## Acceptance checklist

- [ ] `src-optimized/README.md` exists: host inspection results, reference
      profile, ranked candidate list, labeled ASSUMPTIONs.
- [ ] The existing measurement protocol (`measure-speedup.sh`) was used for
      every number in the report; no second protocol was invented.
- [ ] Reference baseline measured on this machine (not quoted from docs).
- [ ] Builds clean with `-Wall -Wextra -Werror`; libc/libm only;
      single-threaded.
- [ ] Full reference test suite passes against the optimized library
      (`build/test_runner_opt`); explicit rejection behavior unchanged.
- [ ] Optimized results match reference results under the tolerance
      criterion via `build/compare_results` (identical collision count;
      distance within 1 mm + 0.1% (+ alpha-conditioning)) on the committed input **and** on
      ≥2 alternate-seed input sets.
- [ ] Independent validator agrees within 1 mm + 0.1% (+ alpha-conditioning) on
  all pairs
      (`build/perf_validate_opt`).
- [ ] Committed input file never modified (checksum evidence); alternate
      sets generated with `build/perf_gen_seed` and written elsewhere.
- [ ] Pre-processing excluded from reported time; pre-processor works on
      any valid input file; any new pre-processing/build step is wired into
      `Makefile.optimized` and `run-performance-test-optimized`, not into the
      reference targets.
- [ ] `OPTIMIZATION-LOG.md` has one entry per iteration with before/after
      measurements, keep/revert decisions, and per-hypothesis cost (total
      wall-clock time = LLM + shell, and total tokens spent).
- [ ] Only `src-optimized/` (and, where a new layout requires it,
      `performance-test-optimized/` + the optimized build/run wiring) was
      changed; no harness piece was weakened or forked.
- [ ] Exit criterion explicitly identified as TARGET REACHED or NO FURTHER
      OPPORTUNITIES, with the evidence that criterion demands.
- [ ] No unmeasured performance claim anywhere in code, comments, log, or
      report.
