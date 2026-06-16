# Convex Primitive Collision Detection — Plan (Phase 1)

Tier 2 (new subsystem). Reference implementation of:

> K. Tracy, T. A. Howell, Z. Manchester, "Differentiable Collision Detection
> for a Set of Convex Primitives," arXiv:2207.00669.

The paper computes collision information for a pair of convex shapes by
solving a small convex conic optimization problem for the **minimum uniform
scaling alpha** applied to both shapes (about their own reference origins)
until they intersect. alpha < 1 means penetration, alpha > 1 means separated,
and the optimal intersection point x* maps to a contact point on each shape.

Platform: x86-64 Linux (WSL2), gcc 11, single thread, libc+libm only.
Units: meters. API positions/distances: float32 (domain ±8192 m makes float
ULP <= ~0.98 mm at the edge, within the 1 mm requirement). Solver internals:
double (see ASSUMPTIONS).

## 1. Data: inputs and outputs (flat structs, no hidden pointers)

```c
typedef enum {
  CP_SPHERE   = 0,  /* radius                                  */
  CP_BOX      = 1,  /* half_extent[3]                          */
  CP_CAPSULE  = 2,  /* radius + length L, segment on body x    */
  CP_POLYTOPE = 3   /* vert_count body-frame points (hull)     */
} cp_type;

#define CP_MAX_POLY_VERTS 32

typedef struct {              /* one convex primitive            */
  float    pos[3];            /* r: body origin in world, meters */
  float    rot[9];            /* WQB row-major world-from-body   */
  uint32_t type;              /* cp_type                         */
  uint32_t vert_count;        /* CP_POLYTOPE only, 4..32, else 0 */
  float    radius;            /* sphere, capsule                 */
  float    length;            /* capsule: segment length L       */
  float    half_extent[3];    /* box                             */
  float    verts[CP_MAX_POLY_VERTS][3]; /* polytope, body frame  */
} cp_prim;

typedef struct { uint32_t a, b; } cp_pair;   /* indices into prim array */

typedef enum {
  CP_OK              = 0,
  CP_ERR_COORD_RANGE = 1,  /* world AABB corner outside ±8192 m       */
  CP_ERR_SIZE_RANGE  = 2,  /* world AABB extent < 0.1 m or > 250 m    */
  CP_ERR_BAD_INDEX   = 3,  /* pair index >= prim_count                */
  CP_ERR_BAD_PRIM    = 4,  /* bad type/vert_count/non-orthonormal rot */
  CP_ERR_NO_CONVERGE = 5   /* solver failed (explicit, never silent)  */
} cp_status;

typedef struct {
  uint32_t status;     /* cp_status; fields below valid only if CP_OK */
  uint32_t colliding;  /* 1 iff alpha < 1                             */
  float    alpha;      /* minimum uniform scaling (paper's output)    */
  float    distance;   /* +|p1-p2| if alpha>=1, -|p1-p2| if alpha<1   */
  float    p1[3];      /* contact point on primitive a (eq. 24)       */
  float    p2[3];      /* contact point on primitive b (eq. 24)       */
} cp_result;
```

## 2. Batch contract (primary API)

```c
void cp_collide_pairs(const cp_prim *prims, uint32_t prim_count,
                      const cp_pair *pairs, uint32_t pair_count,
                      cp_result    *results);
```

- Arrays in, arrays out; `results[i]` answers `pairs[i]`. A single query is
  `pair_count = 1`. Pairs reference primitives by **index**, not pointer.
- Deterministic: same input bytes -> same output bytes, every run.
- Batch passes: (1) validate every primitive once (AABB, ranges, rotation);
  (2) precompute polytope hull faces once per polytope; (3) solve pairs.
- Ownership: caller owns all arrays; results fully overwritten; library
  allocates only internal scratch for the duration of the call.
- Out-of-range input is **rejected** per pair via `status` — never clamped.

## 3. Algorithm (from the paper, our words)

For shapes S1, S2 solve:  minimize alpha over (x, alpha, aux)
subject to x in S1(alpha), x in S2(alpha), alpha >= 0, where S(alpha) is the
shape scaled by alpha about its body origin. Membership constraints (paper
Sec. III-A): polytope/box `A*BQW*(x-r) <= alpha*b` (linear, eq. 11); sphere
`|x-r| <= alpha*R` (second-order cone, eq. 21); capsule `|x-(r+gamma*bx)| <=
alpha*R`, `-alpha*L/2 <= gamma <= alpha*L/2` (SOC + linear, eqs. 12-13, aux
variable gamma). One generic path handles every pair: assemble that pair's
constraint rows and solve the same conic program with a log-barrier
interior-point Newton method (double precision); n <= 6 unknowns (x, alpha,
up to two capsule gammas). A strictly feasible start is constructed
analytically (x0 = midpoint of origins, alpha0 large enough for strictness).
Outer loop t *= 20 until gap bound nu/t < 1e-10, so alpha error is orders of
magnitude below 1 mm at domain scale. Contact points: p_i = r_i +
(x*-r_i)/alpha* (eq. 24); colliding iff alpha* < 1.

Boxes are 6-halfspace polytopes (the paper has no separate box; eq. 11
covers it with A = [+-I], b = half extents). Polytope point clouds are
converted once per batch to halfspaces (quickhull) about the **vertex
centroid** as scaling center, which guarantees the origin is strictly
interior, preserving the paper's feasibility guarantee.

Cost statement: per pair, one dense Newton solve with n <= 6 and at most
~125 constraint rows (two 60-face polytopes); ~8 outer x <= 30 inner Newton
steps, each O(rows * n^2). Throughput on this machine is whatever the Phase 4
harness measures — no other performance claim is made.

## 4. Secondary validation (independent)

GJK distance (support-function based, double precision) + **bisection on
alpha**: for candidate alpha, scale both shapes about the same centers and
ask GJK whether the scaled shapes intersect; bisect to the alpha where the
gap reaches zero; recover x* from GJK witness points at the last separated
alpha and map contact points via the same eq. (24). Independence: shares only
the `cp_prim` data definitions; no solver, assembly, hull, or barrier code is
shared. It is a different algorithm family (support sampling — polytopes use
raw vertices, never the quickhull faces the primary path uses). Any
disagreement beyond 1 mm in `distance` (or witness points, where contact is
unique) is a test failure.

## 5. Explicit boundary behavior

- World AABB of each primitive computed exactly per type; any corner
  coordinate outside ±8192 m -> CP_ERR_COORD_RANGE. Extent on any axis
  < 0.1 m or > 250 m -> CP_ERR_SIZE_RANGE. Rejected, never clamped.
- Rotation matrix checked orthonormal to 1e-3 -> else CP_ERR_BAD_PRIM.
- CP_POLYTOPE with vert_count outside 4..32 -> CP_ERR_BAD_PRIM.
- Coincident origins: if alpha* < 1e-7, eq. 24 divides by ~0; policy:
  colliding = 1, p1 = p2 = r_a, distance = 0. Explicit, documented here.
- Solver non-convergence -> CP_ERR_NO_CONVERGE (never a wrong answer).

## 6. ASSUMPTIONS

- ASSUMPTION: the task's "one support-function path" maps to the paper's one
  uniform constraint-based conic path (the paper's method is set-membership
  constraints, not GJK-style support calls) — affects overall architecture.
- ASSUMPTION: "distance" output is defined as the signed gap between the
  paper's eq.-24 contact points (positive separated, negative penetrating);
  the paper returns alpha + contact points, not Euclidean min distance —
  affects result format and what the validator compares.
- ASSUMPTION: degenerate contact sets (e.g. exactly parallel face-face) make
  witness points non-unique; validation compares `distance` always, witness
  points only for generically-posed cases — affects test comparisons.
- ASSUMPTION: the ±8192 m domain applies to every world AABB corner of a
  primitive, not just its origin — affects validation pass.
- ASSUMPTION: double precision inside the solver is permitted; the float32
  requirement governs API positions/distances — affects 1 mm accuracy.
- ASSUMPTION: polytope scaling center = vertex centroid (paper requires the
  scaling origin strictly inside for feasibility; arbitrary clouds don't
  guarantee that about their frame origin) — affects alpha and contact
  values for polytopes; validator uses the same convention.
- ASSUMPTION: vert_count <= 32 suffices for "convex polytope" here — affects
  struct layout (fixed flat array, no pointers).
- ASSUMPTION: touching exactly (alpha == 1) counts as not colliding — affects
  the `colliding` flag only, not distance.

Not built (deliberately): derivatives/differentiability (not required by the
task), warm starting, broadphase, cylinder/cone/padded-polygon (not in the
pinned primitive set). No parameters or extension points exist for them.
