/* src/collide.c
 *
 * Reference implementation of optimization-based collision detection from:
 *   K. Tracy, T. A. Howell, Z. Manchester,
 *   "Differentiable Collision Detection for a Set of Convex Primitives,"
 *   arXiv:2207.00669.
 *
 * Per pair we solve the paper's problem (10):
 *     minimize    alpha
 *     subject to  x in S1(alpha),  x in S2(alpha),  alpha >= 0
 * where S(alpha) is the primitive uniformly scaled by alpha about its
 * scaling center. Set membership uses the paper's constraints:
 *   polytope/box: A*BQW*(x - r) <= alpha*b           (eq. 11, linear)
 *   capsule:      |x - (r + gamma*bx)| <= alpha*R,
 *                 -alpha*L/2 <= gamma <= alpha*L/2   (eqs. 12-13, SOC+lin)
 *   sphere:       |x - r| <= alpha*R                 (eq. 21, SOC)
 * One identical conic path for every pair type: assemble that pair's
 * constraint rows and solve with a log-barrier interior-point Newton
 * method in float precision (n <= 6 unknowns: x, alpha, up to two
 * capsule gammas). Contact points use eq. (24): p = c + (x* - c)/alpha*.
 *
 * colliding := alpha* < 1.
 * distance  := |c1 - c2| * (1 - 1/alpha*)  ==  signed |p1 - p2|
 * (identity: p1 - p2 = (c1 - c2)(1 - 1/alpha), independent of x*).
 *
 * Scaling centers: body origin r for sphere/box/capsule (per paper);
 * vertex centroid for polytope point clouds so the center is strictly
 * interior (feasibility guarantee; ASSUMPTION in src/README.md).
 *
 * Determinism: fixed iteration order, fixed iteration bounds, no
 * time/rand dependence; same input bytes -> same output bytes.
 *
 * Explicit failure policy: any validation failure or solver
 * non-convergence is reported in cp_result.status; results are never
 * silently clamped or approximated. The library allocates nothing: the caller
 * provides the per-batch scratch buffer (see cp_collide_scratch_bytes); a
 * missing/undersized buffer rejects the whole batch with CP_ERR_NO_CONVERGE.
 */
#include "collide.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define CP_UNUSED_FUNC __attribute__((unused))
#define CP_FORCE_INLINE inline __attribute__((always_inline))
#else
#define CP_UNUSED_FUNC
#define CP_FORCE_INLINE inline
#endif

/* SIMD vector ops from the CEngine math library (C:/CEngine). Absolute quoted
 * path so the angle-bracket <math.h> above is never shadowed and no extra -I is
 * needed; SIMD.h's only dependency is IntFloat.h (plain stdint typedefs), so the
 * libc/libm-only link is unchanged. The solver keeps every 3-vector stored as
 * float[3] (the vshape/cp_shape tables serialize as POD blobs, so field layouts
 * must not change) — SIMD is applied inside the hot geometry kernels.
 *
 * vld3 loads exactly three lanes with w=0: Vec3Load/_mm_loadu_ps would read 16
 * bytes from a 12-byte float[3] and trip -Werror=array-bounds. VecSetR compiles
 * to lane inserts with no over-read. */
/* CEngine SIMD math (C:/CEngine). Absolute quoted path so the angle-bracket
 * <math.h> above is never shadowed and no -I is needed. The headers are
 * inline/macro only (no extra link deps). The pragma block suppresses warnings
 * from third-party header bodies (type-punned BitCast, an unused-function with a
 * paren/promotion nit) without relaxing the flags for our own code. Provides
 * Vec3DotfV, Vec3Cross, Vec3Store, Max3, Min3, Sqrtf, VecMin/Max, etc. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#include "C:/CEngine/Math/Math.h"
#pragma GCC diagnostic pop

/* Every internal 3-vector is a v128f (xyz; w = 0). float[3] survives only at the
 * public ABI boundary (cp_prim / cp_result). vld3 is the one helper not already
 * in CEngine: a SAFE 3-float load. It reads exactly 12 bytes (loadl_pi for xy +
 * a lane insert for z); Vec3Load/_mm_loadu_ps would read 16 bytes from a
 * 12-byte float[3] (over-read, -Werror=array-bounds), and a VecSetR of the three
 * lanes lets -O3 re-coalesce into the same 16-byte movups. */
static CP_FORCE_INLINE v128f vld3(const float p[3]) {
  v128f v = VecLoadLo64(p, VecZero());
  VecSetZ(v, p[2]);
  return v;
}

#define CP_DOMAIN_MAX 8192.0
#define CP_EXTENT_MIN 0.1
#define CP_EXTENT_MAX 250.0
#define CP_MAX_FACES  64
#define CP_MAX_POLY_EDGES 96
#define CP_MAX_ROWS   (2 * CP_MAX_FACES + 5)
#define CP_NVAR_MAX   6
#define CP_ALPHA_EPS  1e-6   /* below this, centers coincide: see policy */
#define CP_GAP_TOL    1e-6  /* barrier duality-gap bound on alpha error */

/* ------------------------------------------------------------------ */
/* validated, precomputed world-space form of one primitive            */
typedef struct {
  uint32_t status;
  uint32_t type;
  v128f   c;                    /* scaling center, world (xyz; w=0)   */
  float   R;                    /* sphere/capsule radius              */
  v128f   axis;                 /* capsule axis (world, unit)         */
  float   hl;                   /* capsule half segment length        */
  int      nface;                /* box: 6; polytope: hull faces       */
  v128f   fa[CP_MAX_FACES];     /* unit outward normals, world        */
  float   fb[CP_MAX_FACES];     /* a.(x - c) <= alpha*fb, fb > 0      */
  int      nedge;                /* polytope: unique hull edge dirs    */
  v128f   edge[CP_MAX_POLY_EDGES];
} cp_shape;

typedef struct {
  uint32_t status;
  uint32_t type;
  v128f   c;
} cp_shape_lite;

static float d3dot(const float a[3], const float b[3]) {
  return Vec3DotfV(vld3(a), vld3(b));
}

static void d3cross(const float a[3], const float b[3], float o[3]) {
  Vec3Store(o, Vec3Cross(vld3(a), vld3(b)));
}

/* rotation must be orthonormal (3e-3 tol) with det ~ +1 */
static int rot_ok(const float rotf[9]) {
  float q[3][3];
  int i, j;
  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j)
      q[i][j] = rotf[3 * i + j];
  for (i = 0; i < 3; ++i) {
    float n = q[i][0] * q[i][0] + q[i][1] * q[i][1] + q[i][2] * q[i][2];
    if (fabsf(n - 1.0f) > 3e-3)
      return 0;
  }
  for (i = 0; i < 3; ++i) {
    for (j = i + 1; j < 3; ++j) {
      float d = q[i][0] * q[j][0] + q[i][1] * q[j][1] + q[i][2] * q[j][2];
      if (fabsf(d) > 3e-3)
        return 0;
    }
  }
  {
    float det = q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1])
               - q[0][1] * (q[1][0] * q[2][2] - q[1][2] * q[2][0])
               + q[0][2] * (q[1][0] * q[2][1] - q[1][1] * q[2][0]);
    if (det < 0.5)
      return 0;
  }
  return 1;
}

/* Brute-force convex hull halfspaces for <= 32 world-space points about
 * center c: every vertex triple whose plane has all points on one side
 * is a face; offsets are taken as the max over all points so every
 * vertex satisfies a.(w - c) <= b exactly. Duplicate planes are merged.
 * Returns 0 on degeneracy (flat cloud, center not strictly interior,
 * face overflow) -> caller rejects with CP_ERR_BAD_PRIM. O(n^4), run
 * once per primitive per batch, never inside the per-pair solve. */
static int CP_UNUSED_FUNC poly_faces(int n, const float w[][3],
                                     const float c[3], float scale,
                                     cp_shape *s) {
  const float tol = 1e-6 * (scale + 1.0);
  int i, j, k, m, f;
  s->nface = 0;
  for (i = 0; i < n; ++i) {
    for (j = i + 1; j < n; ++j) {
      for (k = j + 1; k < n; ++k) {
        float e1[3], e2[3], nm[3], len, dmax, dmin, b;
        int face, dup;
        for (m = 0; m < 3; ++m) {
          e1[m] = w[j][m] - w[i][m];
          e2[m] = w[k][m] - w[i][m];
        }
        d3cross(e1, e2, nm);
        len = sqrtf(d3dot(nm, nm));
        if (len <= 1e-6 * (scale * scale + 1.0))
          continue;
        nm[0] /= len;
        nm[1] /= len;
        nm[2] /= len;
        dmax = -1e30f;
        dmin = 1e30f;
        for (m = 0; m < n; ++m) {
          float d = nm[0] * (w[m][0] - w[i][0])
                   + nm[1] * (w[m][1] - w[i][1])
                   + nm[2] * (w[m][2] - w[i][2]);
          if (d > dmax) dmax = d;
          if (d < dmin) dmin = d;
        }
        face = 0;
        if (dmax <= tol) {
          face = 1;
        } else if (dmin >= -tol) {
          nm[0] = -nm[0];
          nm[1] = -nm[1];
          nm[2] = -nm[2];
          face = 1;
        }
        if (!face)
          continue;
        b = -1e30f;
        for (m = 0; m < n; ++m) {
          float d = nm[0] * (w[m][0] - c[0])
                   + nm[1] * (w[m][1] - c[1])
                   + nm[2] * (w[m][2] - c[2]);
          if (d > b) b = d;
        }
        if (b < 1e-6 * (scale + 1.0))
          return 0; /* center not strictly interior */
        dup = 0;
        for (f = 0; f < s->nface; ++f) {
          if (Vec3DotfV(vld3(nm), s->fa[f]) > 1.0 - 1e-6 &&
              fabsf(b - s->fb[f]) <= 1e-6 * (1.0 + b)) {
            dup = 1;
            break;
          }
        }
        if (!dup) {
          if (s->nface >= CP_MAX_FACES)
            return 0;
          s->fa[s->nface] = vld3(nm);
          s->fb[s->nface] = b;
          ++s->nface;
        }
      }
    }
  }
  return s->nface >= 4;
}

static int CP_UNUSED_FUNC poly_edges(int n, const float w[][3],
                                     float scale, cp_shape *s) {
  const float tol = 2e-5f * (scale + 1.0f);
  int i, j, f, e;
  s->nedge = 0;
  for (i = 0; i < n; ++i) {
    for (j = i + 1; j < n; ++j) {
      int common = 0, dup = 0;
      float dir[3], len;
      for (f = 0; f < s->nface; ++f) {
        float di = Vec3DotfV(s->fa[f], vld3(w[i])) - s->fb[f];
        float dj = Vec3DotfV(s->fa[f], vld3(w[j])) - s->fb[f];
        if (fabsf(di) <= tol && fabsf(dj) <= tol) {
          ++common;
          if (common >= 2)
            break;
        }
      }
      if (common < 2)
        continue;
      dir[0] = w[j][0] - w[i][0];
      dir[1] = w[j][1] - w[i][1];
      dir[2] = w[j][2] - w[i][2];
      len = sqrtf(d3dot(dir, dir));
      if (len <= 1e-8f * (scale + 1.0f))
        continue;
      dir[0] /= len;
      dir[1] /= len;
      dir[2] /= len;
      for (e = 0; e < s->nedge; ++e) {
        if (fabsf(Vec3DotfV(vld3(dir), s->edge[e])) > 1.0f - 1e-6f) {
          dup = 1;
          break;
        }
      }
      if (!dup) {
        if (s->nedge >= CP_MAX_POLY_EDGES)
          return 0;
        s->edge[s->nedge] = vld3(dir);
        ++s->nedge;
      }
    }
  }
  return s->nedge > 0;
}

/* Validate one primitive and build its world-space solver form.
 * Check order (explicit): rotation/params (BAD_PRIM), then coordinate
 * range (COORD_RANGE), then extent range (SIZE_RANGE). */
static void CP_UNUSED_FUNC build_shape(const cp_prim *p, cp_shape *s) {
  float r[3], Q[3][3], mn[3], mx[3], c3[3];
  int i, j;
  memset(s, 0, sizeof *s);
  s->type = p->type;
  s->status = CP_OK;
  if (!rot_ok(p->rot)) {
    s->status = CP_ERR_BAD_PRIM;
    return;
  }
  for (i = 0; i < 3; ++i) {
    r[i] = p->pos[i];
    for (j = 0; j < 3; ++j)
      Q[i][j] = p->rot[3 * i + j];
  }
  switch (p->type) {
  case CP_SPHERE: {
    if (!(p->radius > 0.0f)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    s->R = p->radius;
    for (i = 0; i < 3; ++i) {
      c3[i] = r[i];
      mn[i] = r[i] - s->R;
      mx[i] = r[i] + s->R;
    }
  } break;
  case CP_BOX: {
    float h[3];
    for (i = 0; i < 3; ++i) {
      h[i] = p->half_extent[i];
      if (!(p->half_extent[i] > 0.0f)) {
        s->status = CP_ERR_BAD_PRIM;
        return;
      }
    }
    s->nface = 6;
    for (j = 0; j < 3; ++j) { /* +/- world columns of Q (eq. 11 form) */
      v128f col = VecSetR(Q[0][j], Q[1][j], Q[2][j], 0.0f);
      s->fa[2 * j] = col;
      s->fa[2 * j + 1] = VecNeg(col);
      s->fb[2 * j] = h[j];
      s->fb[2 * j + 1] = h[j];
    }
    for (i = 0; i < 3; ++i) {
      float e = fabsf(Q[i][0]) * h[0] + fabsf(Q[i][1]) * h[1]
               + fabsf(Q[i][2]) * h[2];
      c3[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
    }
  } break;
  case CP_CAPSULE: {
    float axis3[3];
    if (!(p->radius > 0.0f) || !(p->length > 0.0f)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    s->R = p->radius;
    s->hl = 0.5 * (float)p->length;
    for (i = 0; i < 3; ++i)
      axis3[i] = Q[i][0]; /* bx = WQB * [1,0,0]^T */
    s->axis = VecSetR(axis3[0], axis3[1], axis3[2], 0.0f);
    for (i = 0; i < 3; ++i) {
      float e = s->hl * fabsf(axis3[i]) + s->R;
      c3[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
    }
  } break;
  case CP_POLYTOPE: {
    /* Fit the hull halfspaces in the CENTROID-LOCAL frame, not world space.
     * The verts sit at km-scale world positions (|coord| up to 8 km), so
     * forming edges/normals/offsets from world verts in float32 suffers
     * catastrophic cancellation (~mm error) — a contact snapped onto those
     * planes lands ~mm off the true hull, which the double-precision contact
     * certifier flags. Working from wl = Q*body_vert (meter scale, no km term)
     * minus its own centroid keeps every quantity at meter scale, so fa/fb are
     * precise. fb = fa.(vert - centroid) matches the runtime snap, which
     * evaluates fa.(p - s->c) against the world centroid s->c. This is build-
     * stage work, excluded from the timed solve. */
    float w[CP_MAX_POLY_VERTS][3];   /* verts relative to the local centroid */
    float lc[3], origin[3];          /* local centroid; fit origin (= 0)     */
    float scale;
    int n = (int)p->vert_count;
    if (n < 4 || n > CP_MAX_POLY_VERTS) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    for (i = 0; i < 3; ++i) {
      mn[i] = 1e30f;
      mx[i] = -1e30f;
      c3[i] = 0.0f;
      lc[i] = 0.0f;
    }
    for (j = 0; j < n; ++j) {
      for (i = 0; i < 3; ++i) {
        float wl = Q[i][0] * p->verts[j][0]
                 + Q[i][1] * p->verts[j][1]
                 + Q[i][2] * p->verts[j][2];     /* body-rotated, meter scale */
        float ww = r[i] + wl;                     /* world vert (km scale)     */
        w[j][i] = wl;
        lc[i] += wl;
        c3[i] += ww;
        if (ww < mn[i]) mn[i] = ww;
        if (ww > mx[i]) mx[i] = ww;
      }
    }
    for (i = 0; i < 3; ++i) {
      lc[i] /= (float)n;
      c3[i] /= (float)n;
    }
    for (j = 0; j < n; ++j)
      for (i = 0; i < 3; ++i)
        w[j][i] -= lc[i];                         /* center on local centroid  */
    scale = 0.0f;
    for (i = 0; i < 3; ++i)
      if (mx[i] - mn[i] > scale)
        scale = mx[i] - mn[i];
    origin[0] = origin[1] = origin[2] = 0.0f;
    if (!poly_faces(n, w, origin, scale, s)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    if (!poly_edges(n, w, scale, s)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
  } break;
  default:
    s->status = CP_ERR_BAD_PRIM;
    return;
  }
  s->c = vld3(c3);
  for (i = 0; i < 3; ++i) {
    if (mn[i] < -CP_DOMAIN_MAX || mx[i] > CP_DOMAIN_MAX) {
      s->status = CP_ERR_COORD_RANGE;
      return;
    }
  }
  for (i = 0; i < 3; ++i) {
    float ext = mx[i] - mn[i];
    if (ext < CP_EXTENT_MIN || ext > CP_EXTENT_MAX) {
      s->status = CP_ERR_SIZE_RANGE;
      return;
    }
  }
}

static void build_shape_lite(const cp_prim *p, cp_shape_lite *s) {
  float r[3], Q[3][3], mn[3], mx[3], c3[3];
  int i, j;
  memset(s, 0, sizeof *s);
  s->type = p->type;
  s->status = CP_OK;
  if (!rot_ok(p->rot)) {
    s->status = CP_ERR_BAD_PRIM;
    return;
  }
  for (i = 0; i < 3; ++i) {
    r[i] = p->pos[i];
    for (j = 0; j < 3; ++j)
      Q[i][j] = p->rot[3 * i + j];
  }
  switch (p->type) {
  case CP_SPHERE: {
    float radius;
    if (!(p->radius > 0.0f)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    radius = p->radius;
    for (i = 0; i < 3; ++i) {
      c3[i] = r[i];
      mn[i] = r[i] - radius;
      mx[i] = r[i] + radius;
    }
  } break;
  case CP_BOX: {
    float h[3];
    for (i = 0; i < 3; ++i) {
      h[i] = p->half_extent[i];
      if (!(p->half_extent[i] > 0.0f)) {
        s->status = CP_ERR_BAD_PRIM;
        return;
      }
    }
    for (i = 0; i < 3; ++i) {
      float e = fabsf(Q[i][0]) * h[0] + fabsf(Q[i][1]) * h[1]
               + fabsf(Q[i][2]) * h[2];
      c3[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
    }
  } break;
  case CP_CAPSULE: {
    float radius;
    float hl;
    if (!(p->radius > 0.0f) || !(p->length > 0.0f)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    radius = p->radius;
    hl = 0.5 * (float)p->length;
    for (i = 0; i < 3; ++i) {
      float e = hl * fabsf(Q[i][0]) + radius;
      c3[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
    }
  } break;
  case CP_POLYTOPE: {
    int n = (int)p->vert_count;
    if (n < 4 || n > CP_MAX_POLY_VERTS) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    for (i = 0; i < 3; ++i) {
      mn[i] = 1e30f;
      mx[i] = -1e30f;
      c3[i] = 0.0;
    }
    for (j = 0; j < n; ++j) {
      for (i = 0; i < 3; ++i) {
        float w = r[i] + Q[i][0] * p->verts[j][0]
                        + Q[i][1] * p->verts[j][1]
                        + Q[i][2] * p->verts[j][2];
        if (w < mn[i]) mn[i] = w;
        if (w > mx[i]) mx[i] = w;
        c3[i] += w;
      }
    }
    for (i = 0; i < 3; ++i)
      c3[i] /= (float)n;
  } break;
  default:
    s->status = CP_ERR_BAD_PRIM;
    return;
  }
  s->c = vld3(c3);
  for (i = 0; i < 3; ++i) {
    if (mn[i] < -CP_DOMAIN_MAX || mx[i] > CP_DOMAIN_MAX) {
      s->status = CP_ERR_COORD_RANGE;
      return;
    }
  }
  for (i = 0; i < 3; ++i) {
    float ext = mx[i] - mn[i];
    if (ext < CP_EXTENT_MIN || ext > CP_EXTENT_MAX) {
      s->status = CP_ERR_SIZE_RANGE;
      return;
    }
  }
}

/* ------------------------------------------------------------------ */
/* one pair's conic program over u = (x[3], alpha, gamma_a?, gamma_b?) */
typedef struct {
  int    nvar, nlin, nsoc;
  float lw[CP_MAX_ROWS][CP_NVAR_MAX]; /* linear rows: lw.u + ld <= 0  */
  float ld[CP_MAX_ROWS];
  float sA[2][3][CP_NVAR_MAX];        /* SOC: |sA.u + sb| <= alpha*sR */
  float sb[2][3];
  float sR[2];
} cp_prob;

static void add_lin(cp_prob *pr, const float w[CP_NVAR_MAX], float d) {
  memcpy(pr->lw[pr->nlin], w, sizeof(float) * CP_NVAR_MAX);
  pr->ld[pr->nlin] = d;
  ++pr->nlin;
}

/* cl = shape scaling center in the pair-local frame */
static void add_shape_constraints(cp_prob *pr, const cp_shape *s,
                                  const float cl[3]) {
  int i;
  if (s->type == CP_SPHERE || s->type == CP_CAPSULE) {
    int k = pr->nsoc++;
    memset(pr->sA[k], 0, sizeof pr->sA[k]);
    pr->sA[k][0][0] = 1.0;
    pr->sA[k][1][1] = 1.0;
    pr->sA[k][2][2] = 1.0;
    for (i = 0; i < 3; ++i)
      pr->sb[k][i] = -cl[i];
    pr->sR[k] = s->R;
    if (s->type == CP_CAPSULE) {
      int g = pr->nvar++;
      float w[CP_NVAR_MAX], axis3[3];
      Vec3Store(axis3, s->axis);
      for (i = 0; i < 3; ++i)
        pr->sA[k][i][g] = -axis3[i];
      memset(w, 0, sizeof w);
      w[3] = -s->hl;
      w[g] = 1.0; /*  gamma - alpha*L/2 <= 0  (eq. 13) */
      add_lin(pr, w, 0.0);
      w[g] = -1.0; /* -gamma - alpha*L/2 <= 0  (eq. 13) */
      add_lin(pr, w, 0.0);
    }
  } else { /* box / polytope halfspaces (eq. 11) */
    for (i = 0; i < s->nface; ++i) {
      float w[CP_NVAR_MAX], fa3[3];
      memset(w, 0, sizeof w);
      Vec3Store(fa3, s->fa[i]);
      w[0] = fa3[0];
      w[1] = fa3[1];
      w[2] = fa3[2];
      w[3] = -s->fb[i];
      add_lin(pr, w, -Vec3DotfV(s->fa[i], vld3(cl)));
    }
  }
}

static int p_feas(const cp_prob *pr, const float u[]) {
  int i, j, k;
  for (i = 0; i < pr->nlin; ++i) {
    float f = pr->ld[i];
    for (j = 0; j < pr->nvar; ++j)
      f += pr->lw[i][j] * u[j];
    if (f >= 0.0)
      return 0;
  }
  for (k = 0; k < pr->nsoc; ++k) {
    float tr = u[3] * pr->sR[k];
    float v[3], s2;
    if (tr <= 0.0)
      return 0;
    for (i = 0; i < 3; ++i) {
      v[i] = pr->sb[k][i];
      for (j = 0; j < pr->nvar; ++j)
        v[i] += pr->sA[k][i][j] * u[j];
    }
    s2 = tr * tr - d3dot(v, v);
    if (s2 <= 0.0)
      return 0;
  }
  return 1;
}

/* barrier objective; only called on strictly feasible u */
static float p_phi(const cp_prob *pr, float tpar, const float u[]) {
  int i, j, k;
  float phi = tpar * u[3];
  for (i = 0; i < pr->nlin; ++i) {
    float f = pr->ld[i];
    for (j = 0; j < pr->nvar; ++j)
      f += pr->lw[i][j] * u[j];
    phi -= logf(-f);
  }
  for (k = 0; k < pr->nsoc; ++k) {
    float tr = u[3] * pr->sR[k];
    float v[3];
    for (i = 0; i < 3; ++i) {
      v[i] = pr->sb[k][i];
      for (j = 0; j < pr->nvar; ++j)
        v[i] += pr->sA[k][i][j] * u[j];
    }
    phi -= logf(tr * tr - d3dot(v, v));
  }
  return phi;
}

static void p_grad_hess(const cp_prob *pr, float tpar, const float u[],
                        float g[CP_NVAR_MAX],
                        float H[CP_NVAR_MAX][CP_NVAR_MAX]) {
  int a, b, i, j, k;
  int n = pr->nvar;
  for (a = 0; a < n; ++a) {
    g[a] = 0.0;
    for (b = 0; b < n; ++b)
      H[a][b] = 0.0;
  }
  g[3] = tpar;
  for (i = 0; i < pr->nlin; ++i) {
    float f = pr->ld[i], inv;
    for (j = 0; j < n; ++j)
      f += pr->lw[i][j] * u[j];
    inv = 1.0 / (-f);
    for (a = 0; a < n; ++a) {
      float wa = pr->lw[i][a];
      if (wa == 0.0)
        continue;
      g[a] += wa * inv;
      for (b = 0; b < n; ++b)
        H[a][b] += wa * pr->lw[i][b] * inv * inv;
    }
  }
  for (k = 0; k < pr->nsoc; ++k) {
    float R = pr->sR[k];
    float v[3], gs[CP_NVAR_MAX], s, tr;
    tr = u[3] * R;
    for (i = 0; i < 3; ++i) {
      v[i] = pr->sb[k][i];
      for (j = 0; j < n; ++j)
        v[i] += pr->sA[k][i][j] * u[j];
    }
    s = tr * tr - d3dot(v, v);
    for (a = 0; a < n; ++a) {
      gs[a] = -2.0 * (v[0] * pr->sA[k][0][a] + v[1] * pr->sA[k][1][a]
                      + v[2] * pr->sA[k][2][a]);
    }
    gs[3] += 2.0 * R * R * u[3];
    for (a = 0; a < n; ++a)
      g[a] -= gs[a] / s;
    for (a = 0; a < n; ++a) {
      for (b = 0; b < n; ++b) {
        float ata = pr->sA[k][0][a] * pr->sA[k][0][b]
                   + pr->sA[k][1][a] * pr->sA[k][1][b]
                   + pr->sA[k][2][a] * pr->sA[k][2][b];
        H[a][b] += gs[a] * gs[b] / (s * s) + 2.0 * ata / s;
      }
    }
    H[3][3] -= 2.0 * R * R / s;
  }
}

/* solve H dx = -g, H symmetric positive definite (small ridge added) */
static int solve_spd(int n, float H[CP_NVAR_MAX][CP_NVAR_MAX],
                     const float g[CP_NVAR_MAX], float dx[CP_NVAR_MAX]) {
  float L[CP_NVAR_MAX][CP_NVAR_MAX];
  float y[CP_NVAR_MAX];
  int i, j, k;
  for (i = 0; i < n; ++i)
    for (j = 0; j < n; ++j)
      L[i][j] = H[i][j];
  for (i = 0; i < n; ++i)
    L[i][i] += 1e-6 * (fabsf(L[i][i]) + 1.0);
  for (i = 0; i < n; ++i) {
    for (j = 0; j <= i; ++j) {
      float sum = L[i][j];
      for (k = 0; k < j; ++k)
        sum -= L[i][k] * L[j][k];
      if (i == j) {
        if (sum <= 0.0)
          return 0;
        L[i][i] = sqrtf(sum);
      } else {
        L[i][j] = sum / L[j][j];
      }
    }
  }
  for (i = 0; i < n; ++i) {
    float sum = -g[i];
    for (k = 0; k < i; ++k)
      sum -= L[i][k] * y[k];
    y[i] = sum / L[i][i];
  }
  for (i = n - 1; i >= 0; --i) {
    float sum = y[i];
    for (k = i + 1; k < n; ++k)
      sum -= L[k][i] * dx[k];
    dx[i] = sum / L[i][i];
  }
  return 1;
}

/* damped Newton to the barrier central point; returns final decrement^2 */
static float newton_center(const cp_prob *pr, float tpar, float u[]) {
  float lam2 = 1e30f;
  int it, j;
  for (it = 0; it < 100; ++it) {
    float g[CP_NVAR_MAX], H[CP_NVAR_MAX][CP_NVAR_MAX], dx[CP_NVAR_MAX];
    float un[CP_NVAR_MAX], phi0, step;
    int ok;
    p_grad_hess(pr, tpar, u, g, H);
    if (!solve_spd(pr->nvar, H, g, dx))
      return lam2;
    lam2 = 0.0;
    for (j = 0; j < pr->nvar; ++j)
      lam2 -= g[j] * dx[j];
    if (lam2 < 1e-6)
      return lam2 < 0.0 ? 0.0 : lam2;
    phi0 = p_phi(pr, tpar, u);
    step = 1.0;
    ok = 0;
    while (step > 1e-6) {
      for (j = 0; j < pr->nvar; ++j)
        un[j] = u[j] + step * dx[j];
      if (p_feas(pr, un) &&
          p_phi(pr, tpar, un) <= phi0 - 0.25 * step * lam2) {
        ok = 1;
        break;
      }
      step *= 0.5;
    }
    if (!ok)
      return lam2;
    memcpy(u, un, sizeof(float) * (size_t)pr->nvar);
  }
  return lam2;
}

static void CP_UNUSED_FUNC solve_pair(const cp_shape *sa, const cp_shape *sb,
                                      cp_result *res) {
  cp_prob pr;
  float mid[3], ca[3], cb[3], u[CP_NVAR_MAX];
  float req, mnu, tpar, alpha, dvec[3], dist;
  int i, k, tries;
  memset(&pr, 0, sizeof pr);
  pr.nvar = 4;
  for (i = 0; i < 3; ++i) {
    mid[i] = 0.5 * (sa->c[i] + sb->c[i]);
    ca[i] = sa->c[i] - mid[i];
    cb[i] = sb->c[i] - mid[i];
  }
  { /* alpha >= 0 (problem (10)); strict alpha > 0 under the barrier */
    float w[CP_NVAR_MAX];
    memset(w, 0, sizeof w);
    w[3] = -1.0;
    add_lin(&pr, w, 0.0);
  }
  add_shape_constraints(&pr, sa, ca);
  add_shape_constraints(&pr, sb, cb);

  /* strictly feasible start: x = 0 (midpoint), gammas = 0, alpha big */
  memset(u, 0, sizeof u);
  req = 0.1;
  for (i = 0; i < pr.nlin; ++i) {
    if (pr.lw[i][3] < 0.0) {
      float need = pr.ld[i] / (-pr.lw[i][3]);
      if (need > req)
        req = need;
    }
  }
  for (k = 0; k < pr.nsoc; ++k) {
    float need = sqrtf(d3dot(pr.sb[k], pr.sb[k])) / pr.sR[k];
    if (need > req)
      req = need;
  }
  u[3] = 2.0 * req + 1.0;
  tries = 0;
  while (!p_feas(&pr, u) && tries++ < 60)
    u[3] *= 2.0;
  if (!p_feas(&pr, u)) {
    res->status = CP_ERR_NO_CONVERGE;
    return;
  }

  /* barrier path: t *= 20 until gap bound (nlin + 2*nsoc)/t < CP_GAP_TOL */
  mnu = (float)pr.nlin + 2.0 * (float)pr.nsoc;
  tpar = 1.0;
  for (;;) {
    float lam2 = newton_center(&pr, tpar, u);
    if (lam2 > 1e-6) {
      res->status = CP_ERR_NO_CONVERGE;
      return;
    }
    if (mnu / tpar < CP_GAP_TOL)
      break;
    tpar *= 20.0;
  }

  alpha = u[3];
  for (i = 0; i < 3; ++i)
    dvec[i] = sa->c[i] - sb->c[i];
  res->status = CP_OK;
  res->alpha = (float)alpha;
  if (alpha < CP_ALPHA_EPS) {
    /* coincident scaling centers: eq. (24) is 0/0 here. Explicit
     * policy (src/README.md): colliding, zero distance, p1=p2=c1. */
    res->colliding = 1;
    res->distance = 0.0f;
    for (i = 0; i < 3; ++i) {
      res->p1[i] = (float)sa->c[i];
      res->p2[i] = (float)sa->c[i];
    }
    return;
  }
  res->colliding = (alpha < 1.0) ? 1u : 0u;
  dist = sqrtf(d3dot(dvec, dvec)) * (1.0 - 1.0 / alpha);
  res->distance = (float)dist;
  for (i = 0; i < 3; ++i) {
    float xw = u[i] + mid[i];
    res->p1[i] = (float)(sa->c[i] + (xw - sa->c[i]) / alpha);
    res->p2[i] = (float)(sb->c[i] + (xw - sb->c[i]) / alpha);
  }
}

/* test/validator.c — independent validation path.
 *
 * Algorithm family: GJK distance (Gilbert-Johnson-Keerthi, support-point
 * sampling on the Minkowski difference, Ericson-style simplex reduction)
 * plus bisection on the uniform scaling alpha. The scaled shape S(alpha)
 * is sampled directly through its support function:
 *     sup_{S(alpha)}(d) = c + alpha * (sup_S(d) - c)
 * with the same scaling-center convention as the primary path (body origin;
 * vertex centroid for polytopes). Polytopes use RAW vertices — never the
 * halfspace/hull representation the primary solver builds.
 *
 * No code is shared with src/collide.c. Single precision (float) throughout:
 * each pair is solved in a frame re-centered on shape A's scaling center, so
 * the working coordinates are meter-scale (not the km-scale world positions),
 * which keeps float32 precise; convergence/degeneracy tolerances are set for
 * float epsilon (~1.2e-7). See OPTIMIZATION-LOG.md H1 for the precision study.
 */
#include <math.h>
#include <string.h>

typedef struct {
  int    type;
  v128f  c;        /* scaling center, world (xyz; w=0)                      */
  v128f  r;        /* body origin, world                                   */
  v128f  ax[3];    /* world body axes = columns of WQB (ax[0] = body-x,    */
                   /*   the capsule axis); ax[j] = world dir of local +j   */
  float  R, hl;
  v128f  h;        /* box half extents                                     */
  int    nv;
  v128f  w[CP_MAX_POLY_VERTS]; /* polytope verts, world                    */
} vshape;


static void make_vshape(const cp_prim *p, vshape *s) {
  int j;
  memset(s, 0, sizeof *s);
  s->type = (int)p->type;
  /* world body axes = columns of WQB (row-major rot[3i+j]) */
  s->ax[0] = VecSetR(p->rot[0], p->rot[3], p->rot[6], 0.0f);
  s->ax[1] = VecSetR(p->rot[1], p->rot[4], p->rot[7], 0.0f);
  s->ax[2] = VecSetR(p->rot[2], p->rot[5], p->rot[8], 0.0f);
  s->r = vld3(p->pos);
  s->R = p->radius;
  s->hl = 0.5f * (float)p->length;
  s->h = vld3(p->half_extent);
  if (p->type == CP_POLYTOPE) {
    v128f acc = VecZero();
    s->nv = (int)p->vert_count;
    for (j = 0; j < s->nv; ++j) {
      v128f vb = vld3(p->verts[j]);   /* body vert */
      v128f ww = VecAdd(s->r, VecAdd(VecAdd(VecMul(s->ax[0], VecSplatX(vb)),
                                            VecMul(s->ax[1], VecSplatY(vb))),
                                     VecMul(s->ax[2], VecSplatZ(vb))));
      s->w[j] = ww;
      acc = VecAdd(acc, ww);
    }
    s->c = VecMulf(acc, 1.0f / (float)s->nv);
  } else {
    s->c = s->r;
  }
}

/* support of S(alpha) = c + alpha*(S - c); d is the (unnormalized) direction */
static CP_FORCE_INLINE v128f sup_scaled(const vshape *s, float alpha, v128f d) {
  switch (s->type) {
  case CP_SPHERE: {
    float n = sqrtf(Vec3DotfV(d, d));
    if (n < 1e-30f)
      return VecAdd(s->c, VecSetR(alpha * s->R, 0.0f, 0.0f, 0.0f));
    return VecAdd(s->c, VecMulf(d, alpha * s->R / n));
  }
  case CP_BOX: {
    /* local support sign = sign(Q^T d) per axis; world = sum ax[j]*dl[j] */
    float hx = VecGetX(s->h), hy = VecGetY(s->h), hz = VecGetZ(s->h);
    float dl0 = alpha * ((Vec3DotfV(s->ax[0], d) >= 0.0f) ? hx : -hx);
    float dl1 = alpha * ((Vec3DotfV(s->ax[1], d) >= 0.0f) ? hy : -hy);
    float dl2 = alpha * ((Vec3DotfV(s->ax[2], d) >= 0.0f) ? hz : -hz);
    return VecAdd(s->c, VecAdd(VecAdd(VecMulf(s->ax[0], dl0),
                                      VecMulf(s->ax[1], dl1)),
                               VecMulf(s->ax[2], dl2)));
  }
  case CP_CAPSULE: {
    v128f ax = s->ax[0];
    float t = alpha * ((Vec3DotfV(ax, d) >= 0.0f) ? s->hl : -s->hl);
    float n = sqrtf(Vec3DotfV(d, d));
    v128f o = VecAdd(s->c, VecMulf(ax, t));
    if (n >= 1e-30f)
      o = VecAdd(o, VecMulf(d, alpha * s->R / n));
    return o;
  }
  default: { /* CP_POLYTOPE: raw vertices */
    int best = 0, j;
    float bd = -1e30f;
    for (j = 0; j < s->nv; ++j) {
      float dd = Vec3DotfV(s->w[j], d);
      if (dd > bd) { bd = dd; best = j; }
    }
    return VecAdd(s->c, VecMulf(VecSub(s->w[best], s->c), alpha));
  }
  }
}

/* ---- closest point to origin on simplex, with reduction (Ericson) ---- */

static float closest_seg(v128f a, v128f b) {
  v128f ab = VecSub(b, a);
  float dd = Vec3DotfV(ab, ab), t;
  if (dd < 1e-30f)
    return 0.0f;
  t = -Vec3DotfV(a, ab) / dd;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t;
}

/* closest point to origin on triangle abc; bary[3] out; returns mask of
 * vertices kept (bit0=a, bit1=b, bit2=c) */
static int closest_tri(v128f a, v128f b, v128f c, float bary[3]) {
  v128f ab = VecSub(b, a), ac = VecSub(c, a), bp, cp_;
  float d1, d2, d3, d4, d5, d6, va, vb, vc, denom, v, w;
  d1 = -Vec3DotfV(ab, a);
  d2 = -Vec3DotfV(ac, a);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    bary[0] = 1.0f; bary[1] = 0.0f; bary[2] = 0.0f;
    return 1;
  }
  bp = VecNeg(b);
  d3 = Vec3DotfV(ab, bp);
  d4 = Vec3DotfV(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) {
    bary[0] = 0.0f; bary[1] = 1.0f; bary[2] = 0.0f;
    return 2;
  }
  vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    v = d1 / (d1 - d3);
    bary[0] = 1.0f - v; bary[1] = v; bary[2] = 0.0f;
    return 3;
  }
  cp_ = VecNeg(c);
  d5 = Vec3DotfV(ab, cp_);
  d6 = Vec3DotfV(ac, cp_);
  if (d6 >= 0.0f && d5 <= d6) {
    bary[0] = 0.0f; bary[1] = 0.0f; bary[2] = 1.0f;
    return 4;
  }
  vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    w = d2 / (d2 - d6);
    bary[0] = 1.0f - w; bary[1] = 0.0f; bary[2] = w;
    return 5;
  }
  va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    bary[0] = 0.0f; bary[1] = 1.0f - w; bary[2] = w;
    return 6;
  }
  denom = 1.0f / (va + vb + vc);
  v = vb * denom;
  w = vc * denom;
  bary[0] = 1.0f - v - w; bary[1] = v; bary[2] = w;
  return 7;
}

typedef struct {
  int    n;
  v128f p[4]; /* Minkowski-difference points              */
  v128f a[4]; /* support point on A for each vertex       */
} simplex;

/* bary blend of three v128f points -> v128f */
static CP_FORCE_INLINE v128f bary3(v128f p0, v128f p1, v128f p2,
                                   const float bary[3]) {
  return VecAdd(VecAdd(VecMulf(p0, bary[0]), VecMulf(p1, bary[1])),
                VecMulf(p2, bary[2]));
}

/* Reduce the simplex to the minimal face supporting the closest point. Writes
 * the closest point to *v and, using the same barycentric weights, the witness
 * on A (the contact location in scaled space) to *xa. The a[] support points
 * are reduced in lockstep with p[]. Returns 1 if the origin is enclosed. */
static int simplex_closest(simplex *sx, v128f *v, v128f *xa) {
  if (sx->n == 1) {
    *v = sx->p[0];
    *xa = sx->a[0];
    return 0;
  }
  if (sx->n == 2) {
    float t = closest_seg(sx->p[0], sx->p[1]);
    *v  = VecAdd(sx->p[0], VecMulf(VecSub(sx->p[1], sx->p[0]), t));
    *xa = VecAdd(sx->a[0], VecMulf(VecSub(sx->a[1], sx->a[0]), t));
    if (t <= 0.0f) {
      sx->n = 1;
    } else if (t >= 1.0f) {
      sx->p[0] = sx->p[1];
      sx->a[0] = sx->a[1];
      sx->n = 1;
    }
    return 0;
  }
  if (sx->n == 3) {
    float bary[3];
    v128f q[3], qa[3];
    int mask = closest_tri(sx->p[0], sx->p[1], sx->p[2], bary);
    int k = 0, j;
    *v  = bary3(sx->p[0], sx->p[1], sx->p[2], bary);
    *xa = bary3(sx->a[0], sx->a[1], sx->a[2], bary);
    for (j = 0; j < 3; ++j) {
      if (mask & (1 << j)) {
        q[k] = sx->p[j];
        qa[k] = sx->a[j];
        ++k;
      }
    }
    for (j = 0; j < k; ++j) {
      sx->p[j] = q[j];
      sx->a[j] = qa[j];
    }
    sx->n = k;
    return 0;
  }
  { /* tetrahedron */
    v128f bestv = VecZero(), bestxa = VecZero();
    float bestd = 1e30f;
    v128f keep[3], keepa[3];
    float mx = 0.0f;
    int bestk = 0, found_outside = 0, f, j;
    static const int faces[4][3] = {
      {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}
    };
    for (f = 0; f < 4; ++f) {
      float m = Max3(VecFabs(sx->p[f]));
      if (m > mx) mx = m;
    }
    for (f = 0; f < 4; ++f) {
      int i0 = faces[f][0], i1 = faces[f][1], i2 = faces[f][2];
      v128f a = sx->p[i0];
      v128f b = sx->p[i1];
      v128f c = sx->p[i2];
      v128f d = sx->p[6 - i0 - i1 - i2];
      v128f ab = VecSub(b, a), ac = VecSub(c, a), nrm = Vec3Cross(ab, ac);
      v128f ad = VecSub(d, a);
      float nlen = sqrtf(Vec3DotfV(nrm, nrm));
      float so = -Vec3DotfV(nrm, a);   /* origin side  */
      float sd = Vec3DotfV(nrm, ad);   /* 4th-pt side  */
      float outside;
      /* degenerate (flat) tetra: cannot trust the enclosure test; treat
       * the face as a candidate instead of skipping it */
      if (fabsf(sd) <= 1e-6 * nlen * mx + 1e-18) {
        outside = 1.0;
      } else if (so * sd < 0.0) {
        outside = 1.0;
      } else {
        outside = 0.0;
      }
      if (outside != 0.0) {
        float bary[3], dd;
        v128f cv, cxa;
        int mask = closest_tri(a, b, c, bary), gi[3], k = 0;
        found_outside = 1;
        gi[0] = i0; gi[1] = i1; gi[2] = i2;
        cv  = bary3(a, b, c, bary);
        cxa = bary3(sx->a[i0], sx->a[i1], sx->a[i2], bary);
        dd = Vec3DotfV(cv, cv);
        if (dd < bestd) {
          bestd = dd;
          bestv = cv;
          bestxa = cxa;
          for (j = 0; j < 3; ++j) {
            if (mask & (1 << j)) {
              keep[k] = sx->p[gi[j]];
              keepa[k] = sx->a[gi[j]];
              ++k;
            }
          }
          bestk = k;
        }
      }
    }
    if (!found_outside)
      return 1;
    for (j = 0; j < bestk; ++j) {
      sx->p[j] = keep[j];
      sx->a[j] = keepa[j];
    }
    sx->n = bestk;
    *v = bestv;
    *xa = bestxa;
    return 0;
  }
}

/* Distance between S_a(alpha) and S_b(alpha); 0 means intersecting. If xstar is
 * non-NULL it receives the witness point on A (the contact location in scaled
 * space) computed from the same simplex reduction used for the distance. */
static float gjk_dist(const vshape *sa, const vshape *sb, float alpha,
                       v128f *xstar) {
  simplex sx;
  v128f v = VecZero(), xa, d0, pa, pb;
  int it;
  d0 = VecSub(sb->c, sa->c);
  if (Vec3DotfV(d0, d0) <= 1e-30f)
    d0 = VecSetR(1.0f, 0.0f, 0.0f, 0.0f);
  pa = sup_scaled(sa, alpha, d0);
  pb = sup_scaled(sb, alpha, VecNeg(d0));
  sx.p[0] = VecSub(pa, pb);
  sx.a[0] = pa;
  xa = pa;
  sx.n = 1;
  for (it = 0; it < 11; ++it) {
    v128f w;
    float vv, vw;
    if (simplex_closest(&sx, &v, &xa)) {
      if (xstar) *xstar = xa;
      return 0.0f;
    }
    vv = Vec3DotfV(v, v);
    if (vv < 1e-12f) {
      if (xstar) *xstar = xa;
      return 0.0f;
    }
    pa = sup_scaled(sa, alpha, VecNeg(v));
    pb = sup_scaled(sb, alpha, v);
    w = VecSub(pa, pb);
    vw = Vec3DotfV(v, w);
    if (vv - vw <= 1e-6f * vv + 1e-12f) {
      if (xstar) *xstar = xa;
      return sqrtf(vv);
    }
    /* duplicate support point => no progress: converged */
    {
      int dup = 0, j;
      for (j = 0; j < sx.n; ++j) {
        v128f e = VecSub(w, sx.p[j]);
        if (Vec3DotfV(e, e) < 1e-12f) {
          dup = 1;
          break;
        }
      }
      if (dup) {
        if (xstar) *xstar = xa;
        return sqrtf(vv);
      }
    }
    sx.p[sx.n] = w;
    sx.a[sx.n] = pa;
    ++sx.n;
  }
  if (xstar) *xstar = xa;
  return sqrtf(Vec3DotfV(v, v));
}


/* world vector -> box-local coords: (ax0.v, ax1.v, ax2.v) = Q^T v */
static CP_FORCE_INLINE v128f box_local(const vshape *box, v128f v) {
  return VecSetR(Vec3DotfV(box->ax[0], v), Vec3DotfV(box->ax[1], v),
                 Vec3DotfV(box->ax[2], v), 0.0f);
}
/* box-local point -> world: c + ax0*lx + ax1*ly + ax2*lz = c + Q*local */
static CP_FORCE_INLINE v128f box_world(const vshape *box, v128f local) {
  return VecAdd(box->c, VecAdd(VecAdd(VecMul(box->ax[0], VecSplatX(local)),
                                      VecMul(box->ax[1], VecSplatY(local))),
                               VecMul(box->ax[2], VecSplatZ(local))));
}

static int sphere_box_predicate_ok(const vshape *sphere, const vshape *box,
                                   float alpha) {
  v128f gap;
  float lhs, rhs, eps;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  /* gap = max(|Q^T(c_s - c_b)| - alpha*h, 0) per axis; lhs = |gap|^2 */
  gap = box_local(box, VecSub(sphere->c, box->c));
  gap = VecMax(VecSub(VecFabs(gap), VecMulf(box->h, alpha)), VecZero());
  lhs = Vec3DotfV(gap, gap);
  rhs = alpha * sphere->R;
  rhs *= rhs;
  eps = 1e-6 * (1.0 + lhs + rhs);
  return lhs <= rhs + eps;
}

static float sphere_box_alpha_fallback(const vshape *sphere,
                                        const vshape *box) {
  float lo, hi;
  int i;
  if (sphere_box_predicate_ok(sphere, box, 0.0))
    return 0.0;
  lo = 0.0;
  hi = 1.0;
  for (i = 0; i < 60 && isfinite(hi); ++i) {
    if (sphere_box_predicate_ok(sphere, box, hi))
      break;
    hi *= 2.0;
  }
  if (!isfinite(hi) || !sphere_box_predicate_ok(sphere, box, hi))
    return INFINITY;
  for (i = 0; i < 80; ++i) {
    float mid = 0.5 * (lo + hi);
    if (sphere_box_predicate_ok(sphere, box, mid))
      hi = mid;
    else
      lo = mid;
    if (hi - lo <= 1e-6 * (1.0 + hi))
      break;
  }
  return hi;
}

static float sphere_box_alpha(const vshape *sphere, const vshape *box) {
  v128f a, h;
  float best;
  int i, mask;
  /* a = |Q^T(c_s - c_b)| per axis; h = box half extents (both kept in v128f, */
  /* lane-indexed by VecGetN since the search enumerates axis subsets) */
  a = VecFabs(box_local(box, VecSub(sphere->c, box->c)));
  h = box->h;
  if (sphere_box_predicate_ok(sphere, box, 0.0))
    return 0.0;
  best = INFINITY;
  for (mask = 1; mask < 8; ++mask) {
    float D2 = 0.0, H = 0.0, H2 = 0.0;
    float A, C, disc;
    int r;
    for (i = 0; i < 3; ++i) {
      if (mask & (1 << i)) {
        float ai = VecGetN(a, i), hi = VecGetN(h, i);
        D2 += ai * ai;
        H += ai * hi;
        H2 += hi * hi;
      }
    }
    A = H2 - sphere->R * sphere->R;
    C = D2;
    disc = H * H - A * C;
    for (r = 0; r < 2; ++r) {
      float alpha;
      int active_ok;
      if (fabsf(A) <= 1e-6 * (1.0 + fabsf(H2) + sphere->R * sphere->R)) {
        if (r != 0 || H <= 0.0)
          continue;
        alpha = C / (2.0 * H);
      } else {
        float root_disc, q;
        if (disc < -1e-6 * (1.0 + H * H + fabsf(A * C)))
          continue;
        root_disc = sqrtf(disc < 0.0 ? 0.0 : disc);
        /* Stable roots (H >= 0): large root q/A, small root C/q (Vieta), to
         * avoid catastrophic cancellation in H - sqrt(disc) — see the
         * sphere_capsule path and OPTIMIZATION-LOG.md H1. */
        q = H + root_disc;
        if (r == 0)
          alpha = (q != 0.0) ? C / q : (H - root_disc) / A;
        else
          alpha = q / A;
      }
      if (!(alpha >= 0.0) || !isfinite(alpha) || alpha >= best)
        continue;
      active_ok = 1;
      for (i = 0; i < 3; ++i) {
        float ai = VecGetN(a, i), hi = VecGetN(h, i);
        float gap = ai - alpha * hi;
        float tol = 1e-6 * (1.0 + ai + fabsf(alpha * hi));
        if ((mask & (1 << i)) != 0) {
          if (gap < -tol)
            active_ok = 0;
        } else if (gap > tol) {
          active_ok = 0;
        }
      }
      if (active_ok && sphere_box_predicate_ok(sphere, box, alpha))
        best = alpha;
    }
  }
  if (isfinite(best))
    return best;
  return sphere_box_alpha_fallback(sphere, box);
}

static int sphere_capsule_predicate_ok(const vshape *sphere,
                                       const vshape *capsule,
                                       float alpha) {
  v128f d;
  float x, p2, gap, lhs, rhs, R, eps;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  d = VecSub(sphere->c, capsule->c);
  x = fabsf(Vec3DotfV(d, capsule->ax[0]));
  p2 = Vec3DotfV(d, d) - x * x;
  if (p2 < 0.0)
    p2 = 0.0;
  gap = x - alpha * capsule->hl;
  if (gap < 0.0)
    gap = 0.0;
  lhs = p2 + gap * gap;
  R = sphere->R + capsule->R;
  rhs = alpha * R;
  rhs *= rhs;
  eps = 1e-6 * (1.0 + lhs + rhs);
  return lhs <= rhs + eps;
}

static float sphere_capsule_alpha_fallback(const vshape *sphere,
                                            const vshape *capsule) {
  float lo, hi;
  int i;
  if (sphere_capsule_predicate_ok(sphere, capsule, 0.0))
    return 0.0;
  lo = 0.0;
  hi = 1.0;
  for (i = 0; i < 60 && isfinite(hi); ++i) {
    if (sphere_capsule_predicate_ok(sphere, capsule, hi))
      break;
    hi *= 2.0;
  }
  if (!isfinite(hi) || !sphere_capsule_predicate_ok(sphere, capsule, hi))
    return INFINITY;
  for (i = 0; i < 80; ++i) {
    float mid = 0.5 * (lo + hi);
    if (sphere_capsule_predicate_ok(sphere, capsule, mid))
      hi = mid;
    else
      lo = mid;
    if (hi - lo <= 1e-6 * (1.0 + hi))
      break;
  }
  return hi;
}

static float sphere_capsule_alpha(const vshape *sphere,
                                   const vshape *capsule) {
  v128f d;
  float x, p2, R, hl, best;
  d = VecSub(sphere->c, capsule->c);
  x = fabsf(Vec3DotfV(d, capsule->ax[0]));
  p2 = Vec3DotfV(d, d) - x * x;
  if (p2 < 0.0)
    p2 = 0.0;
  R = sphere->R + capsule->R;
  hl = capsule->hl;
  if (sphere_capsule_predicate_ok(sphere, capsule, 0.0))
    return 0.0;
  best = INFINITY;

  if (R > 0.0) {
    float alpha = sqrtf(p2) / R;
    float tol = 1e-6 * (1.0 + x + fabsf(alpha * hl));
    if (isfinite(alpha) && x <= alpha * hl + tol &&
        sphere_capsule_predicate_ok(sphere, capsule, alpha)) {
      best = alpha;
    }
  }

  {
    float A = hl * hl - R * R;
    float H = x * hl;
    float C = x * x + p2;
    float disc = H * H - A * C;
    int r;
    for (r = 0; r < 2; ++r) {
      float alpha;
      if (fabsf(A) <= 1e-6 * (1.0 + hl * hl + R * R)) {
        if (r != 0 || H <= 0.0)
          continue;
        alpha = C / (2.0 * H);
      } else {
        float root_disc, q;
        if (disc < -1e-6 * (1.0 + H * H + fabsf(A * C)))
          continue;
        root_disc = sqrtf(disc < 0.0 ? 0.0 : disc);
        /* Numerically stable roots. H = x*hl >= 0, so q = H + sqrt(disc) never
         * cancels; the large root is q/A and the small root is C/q (Vieta:
         * C/q == (H - sqrt(disc))/A exactly, but without the catastrophic
         * cancellation of H - sqrt(disc) when A is small and disc ~ H^2 — that
         * cancellation is what made the float root wrong enough that
         * predicate_ok rejected it; see OPTIMIZATION-LOG.md H1). */
        q = H + root_disc;
        if (r == 0)
          alpha = (q != 0.0) ? C / q : (H - root_disc) / A;
        else
          alpha = q / A;
      }
      if (alpha >= 0.0 && isfinite(alpha) && alpha < best) {
        float tol = 1e-6 * (1.0 + x + fabsf(alpha * hl));
        if (x >= alpha * hl - tol &&
            sphere_capsule_predicate_ok(sphere, capsule, alpha)) {
          best = alpha;
        }
      }
    }
  }

  if (hl > 0.0) {
    float alpha = x / hl;
    if (alpha >= 0.0 && isfinite(alpha) && alpha < best &&
        sphere_capsule_predicate_ok(sphere, capsule, alpha)) {
      best = alpha;
    }
  }

  if (isfinite(best))
    return best;
  return sphere_capsule_alpha_fallback(sphere, capsule);
}

static float sphere_sphere_alpha(const vshape *a, const vshape *b) {
  v128f delta = VecSub(a->c, b->c);
  float d = sqrtf(Vec3DotfV(delta, delta)), r;
  if (d <= 0.0)
    return 0.0;
  r = a->R + b->R;
  return d / r;
}

static float clamp01(float x) {
  if (x < 0.0)
    return 0.0;
  if (x > 1.0)
    return 1.0;
  return x;
}

static float segment_segment_dist2_scaled_capsules(const vshape *a,
                                                    const vshape *b,
                                                    float alpha) {
  v128f pa0, pb0, d1, d2, r, cd;
  float aa, bb, cc, ee, ff, denom, s, t;
  v128f ua = VecMulf(a->ax[0], alpha * a->hl);
  v128f ub = VecMulf(b->ax[0], alpha * b->hl);
  pa0 = VecSub(a->c, ua);
  pb0 = VecSub(b->c, ub);
  d1 = VecMulf(ua, 2.0f);   /* pa1 - pa0 */
  d2 = VecMulf(ub, 2.0f);   /* pb1 - pb0 */
  r = VecSub(pa0, pb0);
  aa = Vec3DotfV(d1, d1);
  ee = Vec3DotfV(d2, d2);
  if (aa <= 1e-30 && ee <= 1e-30)
    return Vec3DotfV(r, r);
  if (aa <= 1e-30) {
    s = 0.0;
    t = clamp01(Vec3DotfV(d2, r) / ee);
  } else {
    cc = Vec3DotfV(d1, r);
    if (ee <= 1e-30) {
      t = 0.0;
      s = clamp01(-cc / aa);
    } else {
      bb = Vec3DotfV(d1, d2);
      ff = Vec3DotfV(d2, r);
      denom = aa * ee - bb * bb;
      if (denom > 1e-18 * (aa * ee + 1.0))
        s = clamp01((bb * ff - cc * ee) / denom);
      else
        s = 0.0;
      t = (bb * s + ff) / ee;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-cc / aa);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((bb - cc) / aa);
      }
    }
  }
  cd = VecSub(VecAdd(r, VecMulf(d1, s)), VecMulf(d2, t));
  return Vec3DotfV(cd, cd);
}

static int capsule_capsule_predicate_ok(const vshape *a, const vshape *b,
                                        float alpha) {
  float dist2, R, rhs, coord, len, eps;
  v128f ua, ub, m;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  dist2 = segment_segment_dist2_scaled_capsules(a, b, alpha);
  R = alpha * (a->R + b->R);
  rhs = R * R;
  ua = VecMulf(a->ax[0], alpha * a->hl);
  ub = VecMulf(b->ax[0], alpha * b->hl);
  m = VecMax(VecMax(VecFabs(VecSub(a->c, ua)), VecFabs(VecAdd(a->c, ua))),
             VecMax(VecFabs(VecSub(b->c, ub)), VecFabs(VecAdd(b->c, ub))));
  coord = Max3(m);
  len = alpha * (a->hl + b->hl + a->R + b->R);
  eps = 1e-6 * (1.0 + dist2 + rhs + coord * coord + len * len);
  return dist2 <= rhs + eps;
}

static float capsule_capsule_alpha(const vshape *a, const vshape *b) {
  float lo, hi;
  int i;
  if (capsule_capsule_predicate_ok(a, b, 0.0))
    return 0.0;
  lo = 0.0;
  hi = 1.0;
  for (i = 0; i < 60 && isfinite(hi); ++i) {
    if (capsule_capsule_predicate_ok(a, b, hi))
      break;
    hi *= 2.0;
  }
  if (!isfinite(hi) || !capsule_capsule_predicate_ok(a, b, hi))
    return INFINITY;
  for (i = 0; i < 60; ++i) {
    float mid = 0.5 * (lo + hi);
    if (capsule_capsule_predicate_ok(a, b, mid))
      hi = mid;
    else
      lo = mid;
    if (hi - lo <= 1e-6 * (1.0 + hi))
      break;
  }
  return hi;
}

static float segment_aabb_dist2(v128f p0, v128f p1, v128f e) {
  v128f v = VecSub(p1, p0);
  float cand[8], best;
  int nc, i, j;

  nc = 0;
  cand[nc++] = 0.0;
  cand[nc++] = 1.0;
  for (i = 0; i < 3; ++i) {
    float vi = VecGetN(v, i), p0i = VecGetN(p0, i), ei = VecGetN(e, i);
    if (fabsf(vi) > 1e-30) {
      float t = (ei - p0i) / vi;
      if (t >= 0.0 && t <= 1.0)
        cand[nc++] = t;
      t = (-ei - p0i) / vi;
      if (t >= 0.0 && t <= 1.0)
        cand[nc++] = t;
    }
  }

  for (i = 1; i < nc; ++i) {
    float x = cand[i];
    j = i - 1;
    while (j >= 0 && cand[j] > x) {
      cand[j + 1] = cand[j];
      --j;
    }
    cand[j + 1] = x;
  }

  {
    int out = 0;
    for (i = 0; i < nc; ++i) {
      if (out == 0 || fabsf(cand[i] - cand[out - 1]) > 1e-6)
        cand[out++] = cand[i];
    }
    nc = out;
  }

  best = INFINITY;
  for (i = 0; i < nc; ++i) {
    /* gap = max(|p0 + t v| - e, 0) per axis; d2 = |gap|^2 */
    v128f gap = VecMax(VecSub(VecFabs(VecAdd(p0, VecMulf(v, cand[i]))), e),
                       VecZero());
    float d2 = Vec3DotfV(gap, gap);
    if (d2 < best)
      best = d2;
  }

  for (i = 0; i + 1 < nc; ++i) {
    float lo = cand[i], hi = cand[i + 1];
    float mid, A, B, C;
    if (hi <= lo)
      continue;
    mid = 0.5 * (lo + hi);
    A = 0.0;
    B = 0.0;
    C = 0.0;
    for (j = 0; j < 3; ++j) {
      float vj = VecGetN(v, j), p0j = VecGetN(p0, j), ej = VecGetN(e, j);
      float q = p0j + mid * vj;
      if (q > ej) {
        float m = vj;
        float c = p0j - ej;
        A += m * m;
        B += m * c;
        C += c * c;
      } else if (q < -ej) {
        float m = -vj;
        float c = -p0j - ej;
        A += m * m;
        B += m * c;
        C += c * c;
      }
    }
    if (A > 0.0) {
      float t = -B / A;
      if (t > lo && t < hi) {
        float d2 = A * t * t + 2.0 * B * t + C;
        if (d2 < best)
          best = d2;
      }
    }
  }

  return best;
}

static int box_capsule_predicate_ok(const vshape *box,
                                    const vshape *capsule,
                                    float alpha) {
  v128f cl, axl, hu, p0, p1, e;
  float dist2, R, rhs, coord, len, eps;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  cl  = box_local(box, VecSub(capsule->c, box->c)); /* capsule center, box-local */
  axl = box_local(box, capsule->ax[0]);             /* capsule axis,   box-local */
  hu  = VecMulf(axl, alpha * capsule->hl);
  p0 = VecSub(cl, hu);
  p1 = VecAdd(cl, hu);
  e  = VecMulf(box->h, alpha);
  dist2 = segment_aabb_dist2(p0, p1, e);
  coord = Max3(VecMax(VecMax(VecFabs(p0), VecFabs(p1)), e));
  len = alpha * (capsule->hl + capsule->R)
      + VecGetX(e) + VecGetY(e) + VecGetZ(e);
  R = alpha * capsule->R;
  rhs = R * R;
  eps = 1e-6 * (1.0 + dist2 + rhs + coord * coord + len * len);
  return dist2 <= rhs + eps;
}

static float box_capsule_alpha(const vshape *box, const vshape *capsule) {
  float lo, hi;
  int i;
  if (box_capsule_predicate_ok(box, capsule, 0.0))
    return 0.0;
  lo = 0.0;
  hi = 1.0;
  for (i = 0; i < 60 && isfinite(hi); ++i) {
    if (box_capsule_predicate_ok(box, capsule, hi))
      break;
    hi *= 2.0;
  }
  if (!isfinite(hi) || !box_capsule_predicate_ok(box, capsule, hi))
    return INFINITY;
  for (i = 0; i < 80; ++i) {
    float mid = 0.5 * (lo + hi);
    if (box_capsule_predicate_ok(box, capsule, mid))
      hi = mid;
    else
      lo = mid;
    if (hi - lo <= 1e-6 * (1.0 + hi))
      break;
  }
  return hi;
}

static int sphere_poly_predicate_ok(const vshape *sphere, const vshape *poly,
                                    float alpha) {
  vshape point;
  float q[3], d, eps;
  int i;
  if (!(alpha > CP_ALPHA_EPS) || !isfinite(alpha))
    return 0;
  point = *sphere;
  point.R = 0.0;
  for (i = 0; i < 3; ++i) {
    q[i] = poly->c[i] + (sphere->c[i] - poly->c[i]) / alpha;
    if (!isfinite(q[i]))
      return 0;
    point.c[i] = q[i];
    point.r[i] = q[i];
  }
  d = gjk_dist(&point, poly, 1.0, NULL);
  if (!isfinite(d))
    return 0;
  eps = 1e-6 * (1.0 + fabsf(d) + fabsf(sphere->R));
  return d <= sphere->R + eps;
}

static float sphere_poly_alpha(const vshape *sphere, const vshape *poly) {
  float lo, hi;
  int i, found_hi;
  lo = 1e-6;
  if (sphere_poly_predicate_ok(sphere, poly, lo))
    return 0.0;
  hi = 1.0;
  found_hi = 0;
  for (i = 0; i < 60 && isfinite(hi); ++i) {
    if (sphere_poly_predicate_ok(sphere, poly, hi)) {
      found_hi = 1;
      break;
    }
    hi *= 2.0;
  }
  if (!found_hi)
    return INFINITY;
  if (hi > 1.0)
    lo = hi * 0.5;
  for (i = 0; i < 15; ++i) {
    float mid = 0.5 * (lo + hi);
    if (sphere_poly_predicate_ok(sphere, poly, mid))
      hi = mid;
    else
      lo = mid;
    if (hi - lo <= 1e-6 * hi)
      break;
  }
  return 0.5 * (lo + hi);
}

static int capsule_poly_predicate_ok(const vshape *capsule, const vshape *poly,
                                     float alpha) {
  vshape seg;
  float q[3], d, eps;
  int i;
  if (!(alpha > CP_ALPHA_EPS) || !isfinite(alpha))
    return 0;
  seg = *capsule;
  seg.R = 0.0;
  for (i = 0; i < 3; ++i) {
    q[i] = poly->c[i] + (capsule->c[i] - poly->c[i]) / alpha;
    if (!isfinite(q[i]))
      return 0;
    seg.c[i] = q[i];
    seg.r[i] = q[i];
  }
  d = gjk_dist(&seg, poly, 1.0, NULL);
  if (!isfinite(d))
    return 0;
  eps = 1e-6 * (1.0 + fabsf(d) + fabsf(capsule->R));
  return d <= capsule->R + eps;
}

static float capsule_poly_alpha(const vshape *capsule, const vshape *poly) {
  float lo, hi;
  int i, found_hi;
  lo = 1e-6;
  if (capsule_poly_predicate_ok(capsule, poly, lo))
    return 0.0;
  hi = 1.0;
  found_hi = 0;
  for (i = 0; i < 60 && isfinite(hi); ++i) {
    if (capsule_poly_predicate_ok(capsule, poly, hi)) {
      found_hi = 1;
      break;
    }
    hi *= 2.0;
  }
  if (!found_hi)
    return INFINITY;
  if (hi > 1.0)
    lo = hi * 0.5;
  for (i = 0; i < 15; ++i) {
    float mid = 0.5 * (lo + hi);
    if (capsule_poly_predicate_ok(capsule, poly, mid))
      hi = mid;
    else
      lo = mid;
    if (hi - lo <= 1e-6 * hi)
      break;
  }
  return 0.5 * (lo + hi);
}

static int box_poly_face_axes_jit(const vshape *poly,
                                  const v128f *poly_rel,
                                  v128f *axes) {
  float scale = 0.0;
  int i, j, k, m, count;
  count = 0;
  for (i = 0; i < poly->nv; ++i) {
    for (j = i + 1; j < poly->nv; ++j) {
      float s = Max3(VecFabs(VecSub(poly_rel[j], poly_rel[i])));
      if (s > scale) scale = s;
    }
  }
  for (i = 0; i < poly->nv; ++i) {
    for (j = i + 1; j < poly->nv; ++j) {
      for (k = j + 1; k < poly->nv; ++k) {
        v128f e1 = VecSub(poly_rel[j], poly_rel[i]);
        v128f e2 = VecSub(poly_rel[k], poly_rel[i]);
        v128f nm = Vec3Cross(e1, e2);
        float len = sqrtf(Vec3DotfV(nm, nm)), dmax, dmin;
        int face, dup, f;
        if (!isfinite(len))
          return -1;
        if (len <= 1e-6 * (scale * scale + 1.0))
          continue;
        nm = VecMulf(nm, 1.0f / len);
        dmax = -1e30f;
        dmin = 1e30f;
        for (m = 0; m < poly->nv; ++m) {
          float d = Vec3DotfV(nm, VecSub(poly_rel[m], poly_rel[i]));
          if (!isfinite(d))
            return -1;
          if (d > dmax) dmax = d;
          if (d < dmin) dmin = d;
        }
        face = 0;
        if (dmax <= 1e-6 * (scale + 1.0)) {
          face = 1;
        } else if (dmin >= -1e-6 * (scale + 1.0)) {
          nm = VecNeg(nm);
          face = 1;
        }
        if (!face)
          continue;
        dup = 0;
        for (f = 0; f < count; ++f) {
          if (Vec3DotfV(nm, axes[f]) > 1.0 - 1e-6) {
            dup = 1;
            break;
          }
        }
        if (!dup) {
          if (count >= CP_MAX_FACES)
            return -1;
          axes[count] = nm;
          ++count;
        }
      }
    }
  }
  return count >= 4 ? count : -1;
}

static float box_poly_axis_alpha_asym(const vshape *box,
                                       const vshape *poly,
                                       const v128f *poly_rel,
                                       v128f center_delta,
                                       v128f L) {
  float len2, rb, pmin, pmax, delta, denom, candidate;
  int j;
  len2 = Vec3DotfV(L, L);
  if (!isfinite(len2))
    return INFINITY;
  if (len2 < 1e-12)
    return -1.0; /* skip near-zero axes; L need not be normalized */

  rb = 0.0;
  for (j = 0; j < 3; ++j) {
    float d = fabsf(Vec3DotfV(box->ax[j], L));
    if (!isfinite(d))
      return INFINITY;
    rb += VecGetN(box->h, j) * d;
  }
  if (!isfinite(rb))
    return INFINITY;

  pmin = INFINITY;
  pmax = -INFINITY;
  (void)poly;
  for (j = 0; j < poly->nv; ++j) {
    float proj = Vec3DotfV(poly_rel[j], L);
    if (!isfinite(proj))
      return INFINITY;
    if (proj < pmin)
      pmin = proj;
    if (proj > pmax)
      pmax = proj;
  }

  delta = Vec3DotfV(center_delta, L); /* P - B */
  if (!isfinite(delta))
    return INFINITY;

  candidate = 0.0;
  if (delta > 0.0) {
    denom = rb - pmin;
    if (!isfinite(denom) || !(denom > 0.0))
      return INFINITY;
    candidate = delta / denom;
  } else if (delta < 0.0) {
    denom = rb + pmax;
    if (!isfinite(denom) || !(denom > 0.0))
      return INFINITY;
    candidate = -delta / denom;
  }
  return isfinite(candidate) ? candidate : INFINITY;
}

static int box_poly_alpha_asym(const vshape *box, const vshape *poly,
                               const cp_shape *poly_shape,
                               float *alpha_out) {
  float best = 0.0;
  v128f poly_axes[CP_MAX_FACES];
  v128f poly_rel[CP_MAX_POLY_VERTS], center_delta;
  int poly_axis_count;
  int used = 0;
  int i, j, k, f, e;

  center_delta = VecSub(poly->c, box->c);
  for (i = 0; i < poly->nv; ++i)
    poly_rel[i] = VecSub(poly->w[i], poly->c);

#define USE_BOX_POLY_AXIS(axis_) do {                                      \
    float axis_alpha__ = box_poly_axis_alpha_asym(box, poly, poly_rel,     \
                                                  center_delta, (axis_));   \
    if (axis_alpha__ == INFINITY)                                          \
      return 0;                                                            \
    if (axis_alpha__ >= 0.0) {                                             \
      if (axis_alpha__ > best)                                             \
        best = axis_alpha__;                                               \
      ++used;                                                              \
    }                                                                      \
  } while (0)

  for (j = 0; j < 3; ++j)
    USE_BOX_POLY_AXIS(box->ax[j]);

  if (poly_shape != NULL && poly_shape->status == CP_OK &&
      poly_shape->type == CP_POLYTOPE && poly_shape->nface >= 4) {
    poly_axis_count = poly_shape->nface;
    for (f = 0; f < poly_axis_count; ++f)
      USE_BOX_POLY_AXIS(poly_shape->fa[f]);
  } else {
    poly_axis_count = box_poly_face_axes_jit(poly, poly_rel, poly_axes);
    if (poly_axis_count < 0)
      return 0;
    for (f = 0; f < poly_axis_count; ++f)
      USE_BOX_POLY_AXIS(poly_axes[f]);
  }

  for (k = 0; k < 3; ++k) {
    v128f box_axis = box->ax[k];
    if (poly_shape != NULL && poly_shape->status == CP_OK &&
        poly_shape->type == CP_POLYTOPE && poly_shape->nedge > 0) {
      for (e = 0; e < poly_shape->nedge; ++e)
        USE_BOX_POLY_AXIS(Vec3Cross(box_axis, poly_shape->edge[e]));
    } else {
      for (i = 0; i < poly->nv; ++i) {
        for (j = i + 1; j < poly->nv; ++j)
          USE_BOX_POLY_AXIS(Vec3Cross(box_axis, VecSub(poly_rel[j], poly_rel[i])));
      }
    }
  }

#undef USE_BOX_POLY_AXIS

  if (!used)
    return 0;
  *alpha_out = best;
  return 1;
}



static float box_box_alpha(const vshape *a, const vshape *b) {
  /* a_axes[i] = a->ax[i] (world box axis i); likewise b_axes */
  v128f d = VecSub(b->c, a->c);
  float alpha = 0.0;
  int j, k;
  for (k = 0; k < 15; ++k) {
    v128f L;
    float len2, num, ra, rb, denom, candidate;
    if (k < 3) {
      L = a->ax[k];
    } else if (k < 6) {
      L = b->ax[k - 3];
    } else {
      L = Vec3Cross(a->ax[(k - 6) / 3], b->ax[(k - 6) % 3]);
      len2 = Vec3DotfV(L, L);
      if (!isfinite(len2))
        return INFINITY;
      if (len2 < 1e-12)
        continue;
    }
    if (!isfinite(VecGetX(L)) || !isfinite(VecGetY(L)) || !isfinite(VecGetZ(L)))
      return INFINITY;
    num = fabsf(Vec3DotfV(d, L));
    if (!isfinite(num))
      return INFINITY;
    ra = 0.0;
    rb = 0.0;
    for (j = 0; j < 3; ++j) {
      float da = fabsf(Vec3DotfV(a->ax[j], L));
      float db = fabsf(Vec3DotfV(b->ax[j], L));
      if (!isfinite(da) || !isfinite(db))
        return INFINITY;
      ra += VecGetN(a->h, j) * da;
      rb += VecGetN(b->h, j) * db;
    }
    denom = ra + rb;
    if (!isfinite(denom) || !(denom > 1e-30))
      return INFINITY;
    candidate = num / denom;
    if (!isfinite(candidate))
      return INFINITY;
    if (candidate > alpha)
      alpha = candidate;
  }
  return alpha;
}

static int pair_alpha_specialized(const vshape *sa, const vshape *sb,
                                  float *alpha_out) {
  float alpha;
  if (sa->type == CP_SPHERE && sb->type == CP_SPHERE) {
    alpha = sphere_sphere_alpha(sa, sb);
  } else if (sa->type == CP_SPHERE && sb->type == CP_BOX) {
    alpha = sphere_box_alpha(sa, sb);
  } else if (sa->type == CP_BOX && sb->type == CP_SPHERE) {
    alpha = sphere_box_alpha(sb, sa);
  } else if (sa->type == CP_BOX && sb->type == CP_BOX) {
    alpha = box_box_alpha(sa, sb);
  } else if (sa->type == CP_SPHERE && sb->type == CP_CAPSULE) {
    alpha = sphere_capsule_alpha(sa, sb);
  } else if (sa->type == CP_CAPSULE && sb->type == CP_SPHERE) {
    alpha = sphere_capsule_alpha(sb, sa);
  } else if (sa->type == CP_BOX && sb->type == CP_CAPSULE) {
    alpha = box_capsule_alpha(sa, sb);
  } else if (sa->type == CP_CAPSULE && sb->type == CP_BOX) {
    alpha = box_capsule_alpha(sb, sa);
  } else if (sa->type == CP_CAPSULE && sb->type == CP_CAPSULE) {
    alpha = capsule_capsule_alpha(sa, sb);
  } else if (sa->type == CP_SPHERE && sb->type == CP_POLYTOPE) {
    alpha = sphere_poly_alpha(sa, sb);
  } else if (sa->type == CP_POLYTOPE && sb->type == CP_SPHERE) {
    alpha = sphere_poly_alpha(sb, sa);
  } else if (sa->type == CP_CAPSULE && sb->type == CP_POLYTOPE) {
    alpha = capsule_poly_alpha(sa, sb);
  } else if (sa->type == CP_POLYTOPE && sb->type == CP_CAPSULE) {
    alpha = capsule_poly_alpha(sb, sa);
  } else {
    return 0;
  }
  if (!isfinite(alpha))
    return 0;
  *alpha_out = alpha;
  return 1;
}

/* Closest points c1 (on segment P0-P1) and c2 (on segment Q0-Q1). Ericson,
 * Real-Time Collision Detection; clamps to the segments and handles a
 * (near-)zero-length segment or parallel pair without dividing by zero. */
static void closest_seg_seg(v128f P0, v128f P1, v128f Q0, v128f Q1,
                            v128f *c1, v128f *c2) {
  v128f d1 = VecSub(P1, P0), d2 = VecSub(Q1, Q0), r = VecSub(P0, Q0);
  float a = Vec3DotfV(d1, d1), e = Vec3DotfV(d2, d2), f = Vec3DotfV(d2, r), s, t;
  if (a <= 1e-12 && e <= 1e-12) { s = 0.0; t = 0.0; }
  else if (a <= 1e-12) { s = 0.0; t = f / e; if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0; }
  else {
    float c = Vec3DotfV(d1, r);
    if (e <= 1e-12) { t = 0.0; s = -c / a; if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0; }
    else {
      float b = Vec3DotfV(d1, d2), denom = a * e - b * b;
      s = (denom > 1e-12 || denom < -1e-12) ? (b * f - c * e) / denom : 0.0;
      if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0;
      t = (b * s + f) / e;
      if (t < 0.0)      { t = 0.0; s = -c / a;       if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0; }
      else if (t > 1.0) { t = 1.0; s = (b - c) / a;  if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0; }
    }
  }
  *c1 = VecAdd(P0, VecMulf(d1, s));
  *c2 = VecAdd(Q0, VecMulf(d2, t));
}

static int radius_poly_witness(const vshape *radius_shape,
                               const vshape *poly,
                               float alpha,
                               v128f *xstar) {
  vshape core;
  v128f ypoly, q, x;
  float d, eps;
  if (!(alpha > CP_ALPHA_EPS) || !isfinite(alpha) ||
      radius_shape->type == CP_POLYTOPE || poly->type != CP_POLYTOPE)
    return 0;
  core = *radius_shape;
  core.R = 0.0f;
  q = VecAdd(poly->c, VecMulf(VecSub(radius_shape->c, poly->c), 1.0f / alpha));
  if (!isfinite(VecGetX(q)) || !isfinite(VecGetY(q)) || !isfinite(VecGetZ(q)))
    return 0;
  core.c = q;
  core.r = q;
  d = gjk_dist(poly, &core, 1.0f, &ypoly);
  if (!isfinite(d))
    return 0;
  eps = 1e-3f; /* accept <=1 mm radius-poly witness slack; fallback otherwise */
  if (d > radius_shape->R + eps)
    return 0;
  x = VecAdd(poly->c, VecMulf(VecSub(ypoly, poly->c), alpha));
  if (!isfinite(VecGetX(x)) || !isfinite(VecGetY(x)) || !isfinite(VecGetZ(x)))
    return 0;
  *xstar = x;
  return 1;
}

static float segment_aabb_closest_local(v128f p0, v128f p1, v128f e,
                                        v128f *seg_out, v128f *box_out) {
  v128f v = VecSub(p1, p0), q;
  float cand[8], best, best_t;
  int nc, i, j;

  nc = 0;
  cand[nc++] = 0.0f;
  cand[nc++] = 1.0f;
  for (i = 0; i < 3; ++i) {
    float vi = VecGetN(v, i), p0i = VecGetN(p0, i), ei = VecGetN(e, i);
    if (fabsf(vi) > 1e-30f) {
      float t = (ei - p0i) / vi;
      if (t >= 0.0f && t <= 1.0f)
        cand[nc++] = t;
      t = (-ei - p0i) / vi;
      if (t >= 0.0f && t <= 1.0f)
        cand[nc++] = t;
    }
  }

  for (i = 1; i < nc; ++i) {
    float x = cand[i];
    j = i - 1;
    while (j >= 0 && cand[j] > x) {
      cand[j + 1] = cand[j];
      --j;
    }
    cand[j + 1] = x;
  }

  {
    int out = 0;
    for (i = 0; i < nc; ++i) {
      if (out == 0 || fabsf(cand[i] - cand[out - 1]) > 1e-6f)
        cand[out++] = cand[i];
    }
    nc = out;
  }

  best = INFINITY;
  best_t = 0.0f;
  for (i = 0; i < nc; ++i) {
    v128f gap = VecMax(VecSub(VecFabs(VecAdd(p0, VecMulf(v, cand[i]))), e),
                       VecZero());
    float d2 = Vec3DotfV(gap, gap);
    if (d2 < best) {
      best = d2;
      best_t = cand[i];
    }
  }

  for (i = 0; i + 1 < nc; ++i) {
    float lo = cand[i], hi = cand[i + 1];
    float mid, A, B, C;
    if (hi <= lo)
      continue;
    mid = 0.5f * (lo + hi);
    A = 0.0f;
    B = 0.0f;
    C = 0.0f;
    for (j = 0; j < 3; ++j) {
      float vj = VecGetN(v, j), p0j = VecGetN(p0, j), ej = VecGetN(e, j);
      float qj = p0j + mid * vj;
      if (qj > ej) {
        float m = vj;
        float c = p0j - ej;
        A += m * m;
        B += m * c;
        C += c * c;
      } else if (qj < -ej) {
        float m = -vj;
        float c = -p0j - ej;
        A += m * m;
        B += m * c;
        C += c * c;
      }
    }
    if (A > 0.0f) {
      float t = -B / A;
      if (t > lo && t < hi) {
        float d2 = A * t * t + 2.0f * B * t + C;
        if (d2 < best) {
          best = d2;
          best_t = t;
        }
      }
    }
  }

  q = VecAdd(p0, VecMulf(v, best_t));
  *seg_out = q;
  *box_out = VecClamp(q, VecNeg(e), e);   /* clamp each lane to [-e, e] */
  return best;
}

static int box_capsule_witness(const vshape *box, const vshape *capsule,
                               float alpha, v128f *xstar) {
  v128f cl, axl, hu, p0, p1, e, segp, boxp, dvec, x;
  float dist2, dist, R, eps;
  if (!(alpha > CP_ALPHA_EPS) || !isfinite(alpha) ||
      box->type != CP_BOX || capsule->type != CP_CAPSULE)
    return 0;
  cl  = box_local(box, VecSub(capsule->c, box->c));
  axl = box_local(box, capsule->ax[0]);
  hu  = VecMulf(axl, alpha * capsule->hl);
  p0 = VecSub(cl, hu);
  p1 = VecAdd(cl, hu);
  e  = VecMulf(box->h, alpha);
  dist2 = segment_aabb_closest_local(p0, p1, e, &segp, &boxp);
  if (!isfinite(dist2))
    return 0;
  dvec = VecSub(boxp, segp);
  dist = sqrtf(Vec3DotfV(dvec, dvec));
  R = alpha * capsule->R;
  eps = 1e-5f * (1.0f + fabsf(dist) + fabsf(R));
  if (!(dist > 1e-6f) || dist > R + eps || dist < R - eps)
    return 0;
  x = box_world(box, boxp);
  if (!isfinite(VecGetX(x)) || !isfinite(VecGetY(x)) || !isfinite(VecGetZ(x)))
    return 0;
  *xstar = x;
  return 1;
}

/* Closed-form witness x* (the touch point of the two alpha-scaled shapes) for
 * sphere-involving pairs, avoiding the iterative gjk_dist witness. x* is the
 * closest point on the OTHER shape's alpha-scaled boundary to the sphere
 * centre: at the solution alpha it lies on BOTH scaled boundaries, so eq.(24)
 * lands a contact on each surface. Returns 1 with xstar filled, or 0 to fall
 * back to GJK (non-sphere pair, or a degenerate config: sphere centre inside
 * the scaled box, or on the capsule axis). The other shape's closest feature
 * must be closed-form, so polytopes are excluded (their closest point needs
 * the hull GJK). See OPTIMIZATION-LOG.md H3. */
static int analytic_witness(const vshape *sa, const vshape *sb, float alpha,
                            v128f *xstar) {
  const vshape *sph, *oth;
  v128f from, d;
  float len;
  /* No sphere: capsule-capsule is closed-form (closest points between the two
   * scaled segment cores, then offset by radius — the radius-shape analogue of
   * sphere-sphere). Other non-sphere pairs (box-box, box-poly, poly-poly) fall
   * back to GJK. */
  if (sa->type != CP_SPHERE && sb->type != CP_SPHERE) {
    if (sa->type == CP_CAPSULE && sb->type == CP_CAPSULE) {
      v128f hua = VecMulf(sa->ax[0], alpha * sa->hl);
      v128f hub = VecMulf(sb->ax[0], alpha * sb->hl);
      v128f PA0 = VecSub(sa->c, hua), PA1 = VecAdd(sa->c, hua);
      v128f QB0 = VecSub(sb->c, hub), QB1 = VecAdd(sb->c, hub), c1, c2, n;
      closest_seg_seg(PA0, PA1, QB0, QB1, &c1, &c2);
      n = VecSub(c2, c1);
      len = sqrtf(Vec3DotfV(n, n));
      if (len < 1e-6) return 0;          /* cores intersect: let GJK handle */
      *xstar = VecAdd(c1, VecMulf(n, alpha * sa->R / len));
      return 1;
    }
    if (sa->type == CP_BOX && sb->type == CP_CAPSULE)
      return box_capsule_witness(sa, sb, alpha, xstar);
    if (sa->type == CP_CAPSULE && sb->type == CP_BOX)
      return box_capsule_witness(sb, sa, alpha, xstar);
    if (sa->type == CP_CAPSULE && sb->type == CP_POLYTOPE)
      return radius_poly_witness(sa, sb, alpha, xstar);
    if (sa->type == CP_POLYTOPE && sb->type == CP_CAPSULE)
      return radius_poly_witness(sb, sa, alpha, xstar);
    return 0;
  }
  if (sa->type == CP_SPHERE) { sph = sa; oth = sb; }
  else                       { sph = sb; oth = sa; }
  from = sph->c;

  if (oth->type == CP_SPHERE) {
    d = VecSub(oth->c, from);
    len = sqrtf(Vec3DotfV(d, d));
    if (len < 1e-6) return 0;
    *xstar = VecSub(oth->c, VecMulf(d, alpha * oth->R / len));
    return 1;
  }
  if (oth->type == CP_BOX) {
    /* closest point on the alpha-scaled box to the sphere centre, box-local */
    v128f loc = box_local(oth, VecSub(from, oth->c));
    v128f lim = VecMulf(oth->h, alpha);
    v128f cl = VecClamp(loc, VecNeg(lim), lim);
    if ((VecMovemask(VecCmpEq(cl, loc)) & 0x7) == 0x7)
      return 0;                          /* sphere centre inside scaled box */
    *xstar = box_world(oth, cl);
    return 1;
  }
  if (oth->type == CP_CAPSULE) {
    v128f ax = oth->ax[0], sp;
    float t = Vec3DotfV(VecSub(from, oth->c), ax), lim = alpha * oth->hl;
    if (t > lim) t = lim; else if (t < -lim) t = -lim;
    sp = VecAdd(oth->c, VecMulf(ax, t));
    d = VecSub(from, sp);
    len = sqrtf(Vec3DotfV(d, d));
    if (len < 1e-6) return 0;           /* sphere centre on the capsule axis */
    *xstar = VecAdd(sp, VecMulf(d, alpha * oth->R / len));
    return 1;
  }
  if (oth->type == CP_POLYTOPE)
    return radius_poly_witness(sph, oth, alpha, xstar);
  return 0;
}

/* Core solve over already-built world-space shapes. The per-shape build
 * (make_vshape) is NOT done here, so this is exactly what the precomputed-shape
 * runtime times. opt_val_collide below is the raw-primitive wrapper that builds
 * the vshapes first (used by the non-precomputed paths). */
static int opt_val_collide_v(const vshape *pa, const vshape *pb,
                             const cp_shape *sha, const cp_shape *shb,
                             float *alpha_out, float *dist_out,
                             v128f *xstar_out) {
  vshape sa = *pa, sb = *pb;
  float lo, hi, alpha;
  v128f cd = VecSub(sa.c, sb.c);
  int i;

  if (sa.type == CP_BOX && sb.type == CP_POLYTOPE &&
      box_poly_alpha_asym(&sa, &sb, shb, &alpha))
    goto done;
  if (sa.type == CP_POLYTOPE && sb.type == CP_BOX &&
      box_poly_alpha_asym(&sb, &sa, sha, &alpha))
    goto done;
  if (pair_alpha_specialized(&sa, &sb, &alpha))
    goto done;

  lo = 1e-6;
  if (gjk_dist(&sa, &sb, lo, NULL) <= 0.0) {
    alpha = 0.0; /* scaling centers (effectively) coincident: alpha -> 0 */
    goto done;
  }
  hi = 1.0;
  i = 0;
  while (gjk_dist(&sa, &sb, hi, NULL) > 0.0) {
    hi *= 2.0;
    if (++i > 60)
      return 1; /* no intersection found at huge alpha: invalid input */
  }
  if (hi > 1.0)
    lo = hi * 0.5;
  for (i = 0; i < 16; ++i) {
    float mid = 0.5 * (lo + hi);
    if (gjk_dist(&sa, &sb, mid, NULL) > 0.0)
      lo = mid;
    else
      hi = mid;
    if (hi - lo <= 1e-6 * hi)
      break;
  }
  alpha = 0.5 * (lo + hi);

done:
  *alpha_out = alpha;
  if (alpha <= CP_ALPHA_EPS) {
    *dist_out = 0.0;
    /* coincident centers: contact point is the (shared) scaling center */
    *xstar_out = sa.c;
  } else {
    /* The witness is a barycentric combination of A's support points on the
     * boundary of the scaled shape, so unscaling it by alpha (eq. 24) lands the
     * contact exactly on the original surface — but only if it is evaluated
     * where the scaled shapes are (barely) separated. At the solution alpha they
     * touch/overlap; step to the touch boundary from below by bisection (tight,
     * so the contact stays on-surface to well under tolerance), then read the
     * witness there. Distance still uses the true alpha. */
    *dist_out = sqrtf(Vec3DotfV(cd, cd)) * (1.0 - 1.0 / alpha);
    /* Closed-form witness for supported type partitions. Known-unsupported
     * partitions (box-box, box-poly, poly-box, poly-poly) go directly to the
     * GJK witness path instead of entering analytic_witness only to return 0. */
    {
      int try_analytic = (sa.type == CP_SPHERE || sb.type == CP_SPHERE ||
          (sa.type == CP_CAPSULE && sb.type == CP_CAPSULE) ||
          (sa.type == CP_BOX && sb.type == CP_CAPSULE) ||
          (sa.type == CP_CAPSULE && sb.type == CP_BOX) ||
          (sa.type == CP_CAPSULE && sb.type == CP_POLYTOPE) ||
          (sa.type == CP_POLYTOPE && sb.type == CP_CAPSULE));
      if (try_analytic && analytic_witness(&sa, &sb, alpha, xstar_out)) {
        /* xstar set analytically */
      } else if (gjk_dist(&sa, &sb, alpha, xstar_out) <= 0.0) {
      float a_lo = alpha * (1.0 - 1e-6), a_hi = alpha;
      int t;
      for (t = 0; t < 40 && gjk_dist(&sa, &sb, a_lo, NULL) <= 0.0; ++t)
        a_lo = alpha - (alpha - a_lo) * 2.0;
      for (t = 0; t < 4; ++t) {
        float am = 0.5 * (a_lo + a_hi);
        if (am <= a_lo || am >= a_hi)
          break; /* float precision exhausted: further gjk_dist calls are wasted */
        if (gjk_dist(&sa, &sb, am, NULL) > 0.0)
          a_lo = am;
        else
          a_hi = am;
      }
        gjk_dist(&sa, &sb, a_lo, xstar_out);
      }
    }
  }
  return 0;
}

/* Raw-primitive wrapper: build the world-space vshapes, then solve. */

/* Snap a contact point exactly onto a shape's surface, removing the small
 * residual the float32 witness leaves. Sphere/capsule: project back to the
 * exact radius (a purely radial error). Box/polytope: project onto the convex
 * halfspace boundary fa.(x-c) <= fb; this relies on the planes being accurate,
 * which holds because box fb are exact half-extents and polytope fa/fb are fit
 * in the centroid-local frame at build time (see build_shape) rather than from
 * km-scale world verts. */
static v128f snap_to_surface(const cp_shape *s, v128f c, v128f p) {
  if (s->type == CP_SPHERE) {
    v128f d = VecSub(p, c);
    float n = sqrtf(Vec3DotfV(d, d));
    if (n > 1e-6)
      p = VecAdd(c, VecMulf(d, s->R / n));
  } else if (s->type == CP_CAPSULE) {
    v128f rel = VecSub(p, c), cp, d;
    float t = Vec3DotfV(rel, s->axis), n;
    if (t > s->hl) t = s->hl; else if (t < -s->hl) t = -s->hl;
    cp = VecAdd(c, VecMulf(s->axis, t));
    d = VecSub(p, cp);
    n = sqrtf(Vec3DotfV(d, d));
    if (n > 1e-6)
      p = VecAdd(cp, VecMulf(d, s->R / n));
  } else if (s->nface > 0) {
    /* Box/polytope: project onto the convex boundary fa.(x-c) <= fb. A face
     * contact sits on one face; an edge/corner contact sits where several faces
     * meet, and projecting onto a single face plane overshoots past the edge.
     * Iterate "project onto the most-violated face" — for the orthogonal box
     * faces this converges onto the edge/corner in a few passes; it drives the
     * point onto the boundary (max signed distance -> 0) for any convex hull.
     * The planes must be accurate for this to land on the true surface: box fb
     * are exact half-extents, and polytope fa/fb are fit in the centroid-local
     * frame at build time (see build_shape) so they carry no km-scale
     * cancellation error. */
    int iter;
    for (iter = 0; iter < 8; ++iter) {
      int imax = 0, i;
      float dmax = -1e30f;
      for (i = 0; i < s->nface; ++i) {
        float d = Vec3DotfV(s->fa[i], VecSub(p, c)) - s->fb[i];
        if (d > dmax) { dmax = d; imax = i; }
      }
      /* Project onto the nearest/most-violated face plane (moves the point
       * outward onto the boundary when interior, inward when exterior). */
      p = VecSub(p, VecMulf(s->fa[imax], dmax));
      if (dmax > -1e-6 && dmax < 1e-6)
        break;
    }
  }
  return p;
}

/* Solve one pair and fill its result. The pair is re-centered on shape A's
 * scaling center first: the shape geometry is defined relative to that center
 * (faces fa.(x-c)<=fb, capsule segment c +/- hl*axis, sphere |x-c|<=R), so only
 * the centers, body origins, and polytope world verts move, and the witness/
 * contacts come back in the re-centered frame and shift back by piv. This keeps
 * every coordinate the float32 solver touches at meter scale instead of the
 * km-scale world positions (see OPTIMIZATION-LOG.md H1). sA/sB carry the
 * halfspace form + radii used by the box/poly path and surface snap; either may
 * be NULL (the lite path has no precomputed cp_shape and does not snap). The
 * vshapes are taken by value so the re-centering does not touch the caller's
 * tables. */
static void solve_pair_vshape(vshape A, vshape B,
                              const cp_shape *sA, const cp_shape *sB,
                              cp_result *out) {
  cp_result r;
  v128f piv, xstar = VecZero();
  float alpha = 0.0, dist = 0.0;
  int j;
  memset(&r, 0, sizeof r);
  /* re-center the whole pair on A's scaling center (meter-scale working frame) */
  piv = A.c;
  A.c = VecSub(A.c, piv); A.r = VecSub(A.r, piv);
  B.c = VecSub(B.c, piv); B.r = VecSub(B.r, piv);
  for (j = 0; j < A.nv; ++j) A.w[j] = VecSub(A.w[j], piv);
  for (j = 0; j < B.nv; ++j) B.w[j] = VecSub(B.w[j], piv);
  if (opt_val_collide_v(&A, &B, sA, sB, &alpha, &dist, &xstar)) {
    r.status = CP_ERR_NO_CONVERGE;
    *out = r;
    return;
  }
  r.status = CP_OK;
  r.alpha = alpha;
  r.distance = dist;
  r.colliding = (alpha < 1.0f) ? 1u : 0u;
  if (alpha <= CP_ALPHA_EPS) {
    v128f w = VecAdd(A.c, piv);
    Vec3Store(r.p1, w);
    Vec3Store(r.p2, w);
    r.distance = 0.0f;
    r.colliding = 1u;
  } else {
    /* eq. (24): map the witness back onto each shape (re-centered frame), snap
     * radius/convex shapes onto their surface, then shift back to world. */
    v128f q1 = VecAdd(A.c, VecDivf(VecSub(xstar, A.c), alpha));
    v128f q2 = VecAdd(B.c, VecDivf(VecSub(xstar, B.c), alpha));
    if (sA)
      q1 = snap_to_surface(sA, VecSub(sA->c, piv), q1);
    if (sB)
      q2 = snap_to_surface(sB, VecSub(sB->c, piv), q2);
    Vec3Store(r.p1, VecAdd(q1, piv));
    Vec3Store(r.p2, VecAdd(q2, piv));
  }
  *out = r;
}

/* ------------------------------------------------------------------ */
/* Precomputed-shape table: one record per primitive holding the validation
 * status, the full cp_shape (faces, used by the box-poly path), and the
 * world-space vshape the solver runs on. Built once in the build stage. */
typedef struct {
  uint32_t status;
  vshape   v;
  cp_shape s;
} cp_pshape;

struct cp_vshapes {
  uint32_t  count;
  cp_pshape shapes[]; /* flexible array; flat POD blob, serializes by copy */
};

size_t cp_vshapes_bytes(uint32_t prim_count) {
  size_t max_count =
      ((size_t)-1 - sizeof(struct cp_vshapes)) / sizeof(cp_pshape);
  if ((size_t)prim_count > max_count)
    return 0; /* size would overflow */
  return sizeof(struct cp_vshapes) + sizeof(cp_pshape) * (size_t)prim_count;
}

cp_vshapes *cp_vshapes_create(const cp_prim *prims, uint32_t prim_count,
                              void *buf, size_t buf_bytes) {
  struct cp_vshapes *out = (struct cp_vshapes *)buf;
  size_t need = cp_vshapes_bytes(prim_count);
  uint32_t i;
  if (need == 0 || buf == NULL || buf_bytes < need)
    return NULL;
  out->count = prim_count;
  for (i = 0; i < prim_count; ++i) {
    build_shape(&prims[i], &out->shapes[i].s);          /* validation + faces */
    out->shapes[i].status = out->shapes[i].s.status;
    make_vshape(&prims[i], &out->shapes[i].v);          /* world-space solve geometry */
  }
  return out;
}

const void *cp_vshapes_blob(const cp_vshapes *shapes, size_t *nbytes) {
  if (nbytes)
    *nbytes = sizeof(struct cp_vshapes) +
              (size_t)shapes->count * sizeof(cp_pshape);
  return shapes;
}

const cp_vshapes *cp_vshapes_from_blob(const void *data, size_t nbytes) {
  const struct cp_vshapes *src = (const struct cp_vshapes *)data;
  size_t need;
  if (nbytes < sizeof(struct cp_vshapes))
    return NULL;
  need = sizeof(struct cp_vshapes) + (size_t)src->count * sizeof(cp_pshape);
  if (need != nbytes)
    return NULL;
  /* The blob IS the table layout (flat POD); view it in place, no copy. The
   * caller owns `data` and keeps it alive while the table is used. */
  return src;
}

void cp_collide_pairs_vshapes(const cp_vshapes *shapes, const cp_pair *pairs,
                              uint32_t pair_count, cp_result *results) {
  uint32_t i;
  if (!results)
    return;
  if (shapes == NULL) {
    for (i = 0; i < pair_count; ++i) {
      memset(&results[i], 0, sizeof results[i]);
      results[i].status = CP_ERR_BAD_PRIM;
    }
    return;
  }
  for (i = 0; i < pair_count; ++i) {
    cp_result r;
    const cp_pshape *sa, *sb;
    memset(&r, 0, sizeof r);
    if (pairs[i].a >= shapes->count || pairs[i].b >= shapes->count) {
      r.status = CP_ERR_BAD_INDEX; results[i] = r; continue;
    }
    sa = &shapes->shapes[pairs[i].a];
    sb = &shapes->shapes[pairs[i].b];
    if (sa->status != CP_OK) { r.status = sa->status; results[i] = r; continue; }
    if (sb->status != CP_OK) { r.status = sb->status; results[i] = r; continue; }
    /* everything (faces, vshape) is precomputed in the table — no per-pair build */
    solve_pair_vshape(sa->v, sb->v, &sa->s, &sb->s, &results[i]);
  }
}

/* ------------------------------------------------------------------ */
/* optimized public API                                                */
size_t cp_collide_scratch_bytes(uint32_t prim_count) {
  return (size_t)prim_count * sizeof(cp_shape_lite);
}

void cp_collide_pairs(const cp_prim *prims, uint32_t prim_count,
                      const cp_pair *pairs, uint32_t pair_count,
                      cp_result *results, void *scratch, size_t scratch_bytes) {
  cp_shape_lite *shapes = (cp_shape_lite *)scratch;
  uint32_t i;
  if (!results)
    return;
  if (prim_count > 0 &&
      (scratch == NULL || scratch_bytes < (size_t)prim_count * sizeof(cp_shape_lite))) {
    /* explicit policy: a missing/undersized caller buffer rejects the batch */
    for (i = 0; i < pair_count; ++i) {
      memset(&results[i], 0, sizeof results[i]);
      results[i].status = CP_ERR_NO_CONVERGE;
    }
    return;
  }
  for (i = 0; i < prim_count; ++i)
    build_shape_lite(&prims[i], &shapes[i]);
  for (i = 0; i < pair_count; ++i) {
    cp_result r;
    vshape va, vb;
    memset(&r, 0, sizeof r);
    if (pairs[i].a >= prim_count || pairs[i].b >= prim_count) {
      r.status = CP_ERR_BAD_INDEX; results[i] = r; continue;
    }
    if (shapes[pairs[i].a].status != CP_OK) {
      r.status = shapes[pairs[i].a].status; results[i] = r; continue;
    }
    if (shapes[pairs[i].b].status != CP_OK) {
      r.status = shapes[pairs[i].b].status; results[i] = r; continue;
    }
    /* lite path: no precomputed cp_shape, so no halfspace fast path or surface
     * snap — the vshape alone drives the solve (sA/sB NULL). */
    make_vshape(&prims[pairs[i].a], &va);
    make_vshape(&prims[pairs[i].b], &vb);
    solve_pair_vshape(va, vb, NULL, NULL, &results[i]);
  }
}
