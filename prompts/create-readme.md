# create-readme.md — instructions to generate the root README.md

You are writing `README.md` in the root of this project. This document tells you
what it must contain, in what order, in whose voice, and — most importantly —
which facts to ground in the actual repository instead of inventing.

There is already a `README.md` in the root (a technical reference: build, run,
API). You are **replacing** it with a README that tells the story of how the
optimization was done, while folding the essential "how to build and run"
material into a later section (or linking it). Read the existing `README.md`
first and reuse its accurate technical content; do not lose the build/run/API
facts, just stop leading with them.

## Voice and tone

Write in the **first person, as Mike Acton** ("I decided…", "I wrote…", "My
target was…"). Straightforward, plain, technical. State things directly. Do
**not** be snarky, dismissive, or mean — not about the model, not about the
paper, not about anyone's code. Do not write marketing hype, and do not be
falsely modest either. The register is: an engineer explaining what was done,
why, and what it cost. No exclamation points, no "amazing"/"incredible", no
emoji. Prefer short declarative sentences. When something is uncertain or
unverified, say so.

## Ground every claim — do not fabricate

This is the most important rule. Every number, filename, and behavioral claim in
the README must come from the repository, not from your assumptions.

- **Read these before writing:**
  - `documents/2207.00669.pdf` — the paper (Tracy, Howell, Manchester,
    "Differentiable Collision Detection for a Set of Convex Primitives,"
    arXiv:2207.00669). Read enough to describe what the solver computes.
  - `src/` — the reference C implementation; `src/README.md` if present.
  - `prompts/create-reference.md`, `prompts/create-optimized-test-harness.md`,
    `prompts/create-optimized.md`, `prompts/create-visualizer.md` — the four
    instruction documents that drove each phase. Read each so you describe it
    accurately.
  - `src-optimized/OPTIMIZATION-LOG.md` — the per-hypothesis history (kept and
    rejected). This is the source for the "what was optimized" section.
  - `prove-optimized-harness.sh` and `performance-test-optimized/HARNESS-BASELINE.md`
    — the proof harness and its baseline.
  - `context/data-oriented-design.md` — the operating rules injected into every
    optimization conversation. (Part of "what the human contributed.")
  - `viz/` and `viz/shots/*.png` — the visualizer and its screenshots.
- **The headline speedup is a measured value, not a guess.** Obtain it by
  running `./prove-optimized-harness.sh` (its FINAL SUMMARY prints the
  committed-input median-of-5 speedup and gate verdicts) or, if you cannot run
  it, by reading the latest gate-passing figure recorded in
  `OPTIMIZATION-LOG.md`. State it as a multiple (e.g. "~95×") as the primary
  framing; you may also give the time reduction. Always state the conditions:
  committed 1000-pair input, median-of-5, single thread, and the machine
  (gcc/WSL2, dev machine). Do **not** hard-code a number you did not verify, and
  do not present a committed-input figure as a universal result.
- If a fact you want is not in the repo, leave it out. Never invent
  measurements, dates, or quotes.

## Required structure

Use this order unless a clearly better narrative emerges; the lede and the roles
section are not negotiable.

1. **Title + lede (do not bury it).** Open by naming the paper and that this
   repo contains (a) a faithful reference C implementation of it and (b) an
   optimized implementation that is **~<MEASURED>× faster** on the committed
   benchmark. One or two sentences. Embed **one representative screenshot** from
   `viz/shots/` here so a reader immediately sees what kind of problem this is
   (convex primitives in contact). Keep the lede tight. State here that this is
   a **narrow-phase** solver: it assumes the caller has already run a cheap
   broadphase and culled non-overlapping pairs, so only AABB-overlapping pairs
   are queried — and the committed benchmark reflects that (all 1000 pairs are
   AABB-overlapping, so the timing measures narrow-phase work, not trivial
   rejection).

2. **What this project is for.** Two purposes, stated plainly: (1) to provide
   the actual optimized collision routines, and (2) to demonstrate, concretely
   and reproducibly, how an LLM was used to do the optimization — what the human
   did, what the model did, and what kept the whole thing honest. Say early and
   explicitly that **the model under test here was GPT-5.5** (one run, one
   model; this is a case study, not a benchmark of models).

3. **The roles (the heart of the README).** Make it unambiguous who/what did
   what. Use four clear roles; a short subsection or a table each:
   - **Me (the human).** Defined the problem and the output contract. Set the
     **100× target** — explain its provenance honestly: I reviewed the reference
     code and made a judgment call about what I believed was achievable on this
     hardware; it was not derived from a formal bound. Wrote the four
     instruction documents. Encoded my engineering approach as operating rules
     (`context/data-oriented-design.md`) injected into every conversation. Made
     course corrections and decided what to keep.
   - **GPT-5.5 (the model).** Generated the reference implementation, the test
     harness, and the optimized solver from the instruction documents; proposed
     and implemented each optimization hypothesis; kept the optimization log.
   - **The test harness (ground truth).** The reference-vs-optimized comparator,
     the independent validator (shares no solver code), the contact-point
     certifier, the determinism check, and the median-of-5 timing protocol.
     Nothing is claimed without it; it is what makes "faster" trustworthy and
     "correct" checkable. Explain the **match contract**: the optimized output
     is not bit-identical to the reference — it must match the collision flags
     exactly and every distance within the documented tolerance
     (`1 mm + 0.1%·|d| + conditioning`); contact points are certified for
     geometric validity, not matched.
   - **nagent (the LLM harness).** The data-oriented agent loop that actually
     ran the optimization. Link it: https://github.com/macton/nagent . Describe
     in a few sentences what makes it relevant here: the working state lives in
     inspectable files, not hidden in the model; the model acts only through a
     fixed set of structured tags; and **`prove-optimized-harness.sh` is wired
     to run once per turn (`nagent --hook-per-run ./prove-optimized-harness.sh`)
     so the model sees the real, measured gate status every turn** instead of
     reasoning from its own recollection. Note that the run used model
     `gpt-5.5`.

3a. **Why the grounding matters (short — a few sentences, no drama).** Make the
   point that the structured-state design and the per-turn proof are not
   ceremony. An LLM left to optimize on its own tends to drift: it reasons from
   its own recollection of the last result instead of a fresh measurement, it
   can lose a good change that was never committed, and it can report a result
   it did not actually run. Keeping the working state in inspectable files,
   committing every kept gain, and injecting the real gate/speedup status every
   turn are what convert "the model says it is faster and correct" into
   "measured faster, gates pass, and committed." State this plainly and move on;
   do not recount specific incidents or assign blame.

4. **Methodology — four documents, four phases.** Walk through how the repo was
   built, one phase per instruction document. For each, link the file and say in
   2–4 sentences what it specifies and what it produced:
   1. `prompts/create-reference.md` → the reference implementation in `src/`
      (a faithful port of the paper's problem (10) solve and contacts from
      eq. (24)).
   2. `prompts/create-optimized-test-harness.md` → the test, comparison, and
      measurement harness: its requirements and constraints, the grounding
      scripts that keep every version honest, the committed fixed input, the
      independent validator. Emphasize that the harness was built and proven
      against an identity copy **before** any optimization, so the measurement
      pipeline itself is trusted.
   3. `prompts/create-optimized.md` → the optimization instructions: how I
      approach optimizing (data first, state the cost, remove work, the
      simplification pass, batch/branch-free/layout as first-class levers),
      turned into instructions the model iterates on. This is where nagent and
      `prove-optimized-harness.sh` come in (cross-reference the roles section).
   4. `prompts/create-visualizer.md` → the visualizer (see its own section).

5. **What was optimized — and what was rejected.** Mine `OPTIMIZATION-LOG.md`.
   Do **not** paste the log. Summarize it for a reader: a readable list or table
   of the **kept** optimizations (what each did and roughly the speedup arc it
   contributed — e.g. removing the barrier solve for a GJK/bisection alpha,
   per-type SAT specializations, build-stage precompute excluded from timing,
   single-precision with re-centering, analytic contact witnesses, iteration-
   count reductions), and a shorter list of notable **rejected** trials (what
   was tried and why it was dropped — regressed, or broke the tolerance/flag
   contract). The point a reader should take away: progress was incremental,
   measured, and reversible; dead ends were recorded, not hidden. If the log
   records per-hypothesis cost (tokens, wall-clock), mention that the cost of
   each step was tracked.

6. **The visualizer.** Describe `viz/` (per the `create-visualizer.md` spec):
   what it renders (one query pair at a time, the two primitives, the reported
   contact points / distance), how to run it, and that the screenshots in
   `viz/shots/` were produced by it. Embed two or three more screenshots here.
   Use real paths to the PNGs that exist in `viz/shots/`.

7. **Honest results and limits.** State the measured speedup and gate results
   with their conditions. Be explicit about generalization: report what the
   alternate-seed runs measured versus the committed input, and do not claim a
   generalized result the data does not support. Note the results are
   machine-specific (name the machine/compiler) and single-threaded. Restate the
   tolerance contract here if it helps. This section is where the README earns
   trust by not overclaiming.

8. **Reproduce it yourself.** Concise, copy-pasteable: build (`make clean &&
   make`, `make -f Makefile.optimized optimized`), run the tests, run
   `./run-performance-test` / `./prove-optimized-harness.sh`, and run the
   visualizer. Pull the exact commands from the existing `README.md` and the
   scripts; verify they are current. Keep this practical and short — link to the
   technical detail rather than duplicating all of it.

9. **Citation and license.** Cite the paper properly (authors, title,
   arXiv:2207.00669) and point at `documents/2207.00669.pdf`. Include the
   license if one exists in the repo; if not, say nothing rather than invent
   one.

## Screenshots

At least one in the lede, two or three in the visualizer section. Reference only
files that actually exist under `viz/shots/` (e.g. `viz/shots/pair-0000.png`) —
check first. Use plain Markdown image syntax with short, accurate alt text. Do
not invent captions describing geometry you have not looked at; open the images
or describe them generically ("a sample query pair from the benchmark set").

## Length and format

Aim for something a person will actually read end to end — roughly 150–300 lines
of Markdown. Use headings, a little bulleting, and one or two small tables (the
roles; the kept/rejected optimizations). GitHub-flavored Markdown. Code/commands
in fenced blocks. No badge spam.

## Final self-check before you finish

- [ ] The lede names the paper, the reference, and the **measured** speedup with
      its conditions — and a screenshot is visible immediately.
- [ ] GPT-5.5 is named early; the four roles (me / model / test harness / nagent)
      are each clearly delineated, including what `prove-optimized-harness.sh`
      run-per-turn does and the nagent link.
- [ ] The 100× target is explained as my judgment call from reading the
      reference, not a derived bound.
- [ ] Every speedup/number was taken from the harness or the log, with stated
      conditions; nothing is fabricated; generalization limits are stated.
- [ ] The four `create-*.md` documents are each linked and accurately described.
- [ ] Kept and rejected optimizations are summarized from the log (not pasted),
      conveying measured, reversible, recorded progress.
- [ ] The match/tolerance contract is explained (not bit-identical; flags exact,
      distance within tolerance, contacts certified for validity).
- [ ] Build/run/visualizer commands are present and verified against the repo.
- [ ] First person, Mike Acton, straightforward, not snarky. Read it once aloud
      in that voice and fix anything that sounds like marketing or like a put-down.
