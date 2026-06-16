# C Reference vs. Original Julia — Collision-Detection Comparison

> **Status: reference documentation.** This is an archival record of a one-time
> comparison between the C reference implementation in `src/` (included in this
> project) and the **original** Julia implementation it was derived from
> (DifferentiableCollisions.jl, *not* included in this project). The comparison
> was performed in a separate, throwaway harness; that harness is **not** part
> of this project. This document preserves the methodology and the measured
> results so the equivalence can be cited without re-running anything.

## What was compared

- **C reference** — `src/collide.c` / `src/collide.h` (in this project): a
  single-file C11 reference implementation of optimization-based convex
  collision detection (sphere, box, capsule, polytope), double precision,
  libc/libm only.
- **Original Julia** — Kevin Tracy's **DifferentiableCollisions.jl** (DCOL), the
  published research artifact, at upstream commit `3d29f76`
  (<https://github.com/kevin-tracy/DifferentiableCollisions.jl>). Not vendored
  or required here; named only for provenance.

Both implement the same paper —

> K. Tracy, T. A. Howell, Z. Manchester, *Differentiable Collision Detection
> for a Set of Convex Primitives*, arXiv:2207.00669.

Per pair, both solve the same convex conic program for the **minimum uniform
scaling α** that must be applied to both shapes (about a per-shape scaling
center) until they touch: `α < 1` ⇒ penetrating, `α > 1` ⇒ separated, with a
contact point recovered on each shape (paper eq. 24).

**Headline result.** On a 1000-pair test set the two implementations **agree**:
identical collision decisions (756/756, 0 flag mismatches), all distances within
tolerance (max 1.79 mm / 0.35 %), and every Julia contact independently
certified geometrically valid. A timing profile found the Julia path's apparent
~84 s cost to be **100 % one-time JIT compilation**; its actual per-pair collide
was **~21× faster** than the (untuned) double-precision C reference.

---

## 1. The two implementations side by side

| aspect              | C reference (`src/`)                              | Original Julia (DCOL)                              |
|---------------------|---------------------------------------------------|---------------------------------------------------|
| Problem solved      | min uniform scaling α (paper prob. 10)            | same                                              |
| Solver              | log-barrier interior-point Newton                 | specialized SOCP primal-dual interior point       |
| Arithmetic          | double                                            | double (`StaticArrays`)                            |
| Primitives          | sphere, box, capsule, polytope (point cloud ≤32)  | sphere, capsule, cylinder, cone, polytope, …       |
| Polytope input      | point cloud → quickhull halfspaces                | halfspaces (`A x ≤ b`) directly                    |
| Scaling center      | sphere/box/capsule: origin; polytope: vertex centroid | the primitive's frame origin `r`              |
| Compilation         | ahead-of-time (`make`), once                      | JIT, specialize-on-first-use per type pair        |
| Output              | α, signed distance, two contact points, status    | α and the intersection point `x*`                 |
| Purpose             | readable reference                                | fast differentiable research artifact             |

The C reference returns more than DCOL does directly (signed distance + both
contact points); the comparison harness reconstructed those from DCOL's
`(α, x*)` using the **same formulas** `src/collide.c` uses, so the two outputs
are directly comparable (see §2.3).

---

## 2. How the comparison was done

### 2.1 Shared input, results-level comparison

Both implementations were driven over the **same input geometry**: a fixed
1000-pair set in the project's `collide-pairs-v1` text format (the same set the
reference's own performance test uses). The C path produced its standard
fixed-format results file; a small Julia driver consumed the identical input,
called DCOL, and wrote results in the **exact same fixed format**
(`%04u %u %.6f %.9f %.6f %.6f %.6f %.6f %.6f %.6f` = index, colliding(0/1),
distance, α, contact `p1` xyz, contact `p2` xyz).

The two results files were then compared by two language-agnostic tools (they
read only results files and the shape data — no solver code):

- **Distance/flag comparator** — count + hybrid tolerance per pair:
  `|d_julia − d_ref| ≤ 1 mm + 0.1 %·|d_ref| + 5e-4·|c1−c2|/α²`. The third term
  is a *conditioning* term: distance = `|c1−c2|·(1 − 1/α)`, so a fixed α error
  produces a distance error scaling as `1/α²` at deep penetration. The pass
  criterion is flag equality on every pair **and** every distance within
  tolerance.
- **Independent contact certifier** — each emitted contact point must lie on the
  primitive surface and be separation-consistent; shares no solver code with
  either implementation, so it validates *geometric* correctness rather than
  matching one implementation against the other.

Because contact points are non-unique (a face/edge contact has many equally
valid witness points), the gate is on flags + distance; contacts are certified
for validity rather than required to match byte-for-byte.

### 2.2 Mapping each primitive C → DCOL

Verified line-by-line against DCOL's `problem_matrices.jl`:

| C type   | DCOL primitive                                    | scaling center (DCOL `r`) |
|----------|---------------------------------------------------|---------------------------|
| sphere   | `Sphere(R)`                                       | `pos`                     |
| box      | `Polytope(A=[+I; −I], b=[he; he])`                | `pos`                     |
| capsule  | `Capsule(R, L)` (segment on body +x, full `L`)    | `pos`                     |
| polytope | `Polytope(hull halfspaces about vertex centroid)` | world vertex centroid     |

- **sphere**: DCOL `|x − r| ≤ αR` ≡ the C sphere constraint.
- **box**: DCOL `A·Qᵀ·(x − r) ≤ α·b` with `A=[+I;−I]`, `b=half-extents`,
  `r=pos` ≡ the C box (which the reference itself treats as a 6-halfspace
  polytope scaled about its center).
- **capsule**: DCOL places segment endpoints at `±α·L/2` along body +x with
  radius `R` ≡ the C capsule parameterization.
- **polytope** (the only nontrivial case): the C reference scales a point cloud
  about its **vertex centroid** using quickhull halfspaces about that centroid.
  The harness reproduced this — it computed the convex-hull H-representation of
  `(verts − centroid)` in the body frame **independently** (double description,
  a different implementation than the C quickhull), set DCOL
  `r = pos + rot·centroid` (world centroid) and `q` = body rotation. Then DCOL's
  `A·Qᵀ·(x − r) ≤ α·b` equals the reference's world-frame
  `A_world·(x − c) ≤ α·b` exactly.

**Rotation** was converted from the C row-major world-from-body matrix to a
scalar-first quaternion (Shepperd's method) and **verified per call** that
DCOL's quaternion→matrix reproduced the original matrix to 1e-4 — eliminating
any quaternion convention ambiguity.

### 2.3 Derived outputs computed exactly as `src/collide.c`

DCOL's `proximity` returns `(α, x*)`. The harness formed the rest identically to
the C reference (`src/collide.c` ≈ lines 607–629):

    colliding = α < 1
    distance  = |c_a − c_b| · (1 − 1/α)         (c = scaling centers)
    p_i       = c_i + (x* − c_i)/α               (paper eq. 24)
    coincident-center policy (α < 1e-7 = CP_ALPHA_EPS):
        colliding = 1, distance = 0, p1 = p2 = c_a

DCOL was run with `pdip_tol = 1e-10` to approach the C reference's 1e-10 duality
gap, so the solvers' α values agree well within the tolerance budget.

### 2.4 Timing profile method

DCOL's `proximity` is generic, so the **first** solve of each distinct
primitive-type pair triggers a fresh LLVM specialization (JIT). To separate that
compilation — which has **no equivalent in the ahead-of-time C build** — from
the actual collide, all primitives were built once, then:

- **Cold pass**: first solve of every pair (collide + JIT). JIT time was read
  from Julia's cumulative-compile-time counter (the same one `@time` uses for
  its "% compilation") — measured, not inferred.
- **Warm passes**: the loop again, all specializations cached → **pure
  collide**. Cross-check: cold − warm ≈ the JIT counter.
- **Shape build** timed warm separately: the one-time per-shape cost the Julia
  solve loop *excludes* and the C collide call *includes*.

C reference timing was the median of 7 runs of its performance harness.

---

## 3. Results — correctness

Distance/flag comparator (reference vs. Julia):

    pairs 1000
    ref_collisions 756
    julia_collisions 756
    flag_mismatches 0
    max_distance_diff_mm 1.788000
    max_distance_diff_rel_pct 0.353357
    distance_tol abs 1.0000 mm + rel 0.100 % + cond 5.0e-04*|c1-c2|/alpha^2
    pairs_over_tolerance 0
    max_contact_diff_mm 4.261812
    contacts_matching_reference 997
    PASS

Independent contact certifier on the Julia output:

    pairs_checked 1000
    valid_contacts 1000
    invalid_contacts 0
    max_surface_dev_mm 0.007473
    max_separation_error_mm 0.001493
    PASS

- **Identical collision decisions**: 756 collisions each, **0 flag
  mismatches** — the two solvers decide α ≷ 1 the same way on every pair.
- **Distances agree** within tolerance on all 1000 pairs; worst case 1.79 mm at
  deep penetration (the `1/α²` conditioning regime the tolerance accounts for).
- **Contacts**: 997/1000 match the reference contact within 0.5 mm; the other 3
  differ only because a face/edge contact has many equally-valid witness points
  — all 1000 are certified on-surface to 0.007 mm and separation-consistent to
  0.001 mm.

**Conclusion: the C reference and the original Julia implementation produce the
same collision results on this set.**

---

## 4. Results — timing profile

| phase                                   |        total | per unit            |
|-----------------------------------------|-------------:|---------------------|
| **Julia JIT compilation** (cold, 1×)    | **84.75 s**  | 92 type-pairs, **100.0 % of cold** |
| Julia warm shape build (incl. hulls)    |     0.149 s  | 74.7 µs/shape (excluded from solve) |
| **Julia pure collide** (warm, best/3)   |  **0.0130 s** | **12.96 µs/pair**  |
| **C reference collide call** (median/7) |  **0.2783 s** | **278.3 µs/pair** (incl. per-batch precompute) |

C min/median: 0.2736 / 0.2783 s. Julia warm passes: 0.0139 / 0.0133 / 0.0130 s.
(Measured on x86-64 WSL2, gcc `-O3 -march=native`, Julia 1.10.5, single thread.)

1. **A single Julia run is essentially all compilation.** The cold pass took
   84.75 s and **100.0 %** of it was JIT — the arithmetic of colliding 1000
   pairs is 0.013 s, lost beneath an 84-second compile of 92 type-pair
   specializations. This one-time cost has no analogue in C, which is compiled
   once before it runs. (It is also why a naive single-run measurement appears
   to show "~84 s collision time": that number is compilation, not collision.)

2. **Warm, the collide is fast.** In steady state — the regime DCOL targets
   (e.g. calling proximity every step of a trajectory optimization over fixed
   shapes) — Julia solved a pair in **12.96 µs** vs **278.3 µs** for the C
   reference: **~21× faster per solve**. This is expected: DCOL is a speed-tuned
   `StaticArrays` SOCP solver, whereas `src/` is a readable double-precision
   reference (≈8 outer × ≤30 inner Newton steps over up to ~125 constraint
   rows).

**Fairness caveats.** C's 278 µs *includes* its per-batch precompute (validating
2000 primitives + building polytope hull faces); Julia's 12.96 µs *excludes*
shape construction (done once up front, 74.7 µs/shape, heavier because the hull
went through a generic polyhedral library). So 278 µs is an upper bound on C's
pure solve and 12.96 µs is Julia's pure solve; even crediting all 278 µs to C's
solver, Julia's compiled solver wins ~21×. Both are single-threaded on the same
host and input; the profile's purpose is the **compilation-vs-collide split**,
not a tuned benchmark.

---

## 5. Provenance

- C reference under test: `src/collide.c` / `src/collide.h` (this project).
- Julia implementation: DifferentiableCollisions.jl, upstream commit `3d29f76`;
  Julia 1.10.5, run with `pdip_tol = 1e-10`.
- Input: the project's fixed 1000-pair `collide-pairs-v1` set; both
  implementations consumed identical geometry, and the input's checksum was
  verified unchanged across the comparison run.
- The comparison harness (Julia driver, profiler, and the results-level
  comparator/certifier) lived in a separate project and is intentionally not
  carried here; this document is the durable record of its findings.
