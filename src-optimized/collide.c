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
  float   c[3];                 /* scaling center, world              */
  float   R;                    /* sphere/capsule radius              */
  float   axis[3];              /* capsule axis (world, unit)         */
  float   hl;                   /* capsule half segment length        */
  int      nface;                /* box: 6; polytope: hull faces       */
  float   fa[CP_MAX_FACES][3];  /* unit outward normals, world        */
  float   fb[CP_MAX_FACES];     /* a.(x - c) <= alpha*fb, fb > 0      */
  int      nedge;                /* polytope: unique hull edge dirs    */
  float   edge[CP_MAX_POLY_EDGES][3];
} cp_shape;

typedef struct {
  uint32_t status;
  uint32_t type;
  float   c[3];
} cp_shape_lite;

static float d3dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void d3cross(const float a[3], const float b[3], float o[3]) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
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
          if (d3dot(nm, s->fa[f]) > 1.0 - 1e-6 &&
              fabsf(b - s->fb[f]) <= 1e-6 * (1.0 + b)) {
            dup = 1;
            break;
          }
        }
        if (!dup) {
          if (s->nface >= CP_MAX_FACES)
            return 0;
          s->fa[s->nface][0] = nm[0];
          s->fa[s->nface][1] = nm[1];
          s->fa[s->nface][2] = nm[2];
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
        float di = d3dot(s->fa[f], w[i]) - s->fb[f];
        float dj = d3dot(s->fa[f], w[j]) - s->fb[f];
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
        if (fabsf(d3dot(dir, s->edge[e])) > 1.0f - 1e-6f) {
          dup = 1;
          break;
        }
      }
      if (!dup) {
        if (s->nedge >= CP_MAX_POLY_EDGES)
          return 0;
        s->edge[s->nedge][0] = dir[0];
        s->edge[s->nedge][1] = dir[1];
        s->edge[s->nedge][2] = dir[2];
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
  float r[3], Q[3][3], mn[3], mx[3];
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
      s->c[i] = r[i];
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
      for (i = 0; i < 3; ++i) {
        s->fa[2 * j][i] = Q[i][j];
        s->fa[2 * j + 1][i] = -Q[i][j];
      }
      s->fb[2 * j] = h[j];
      s->fb[2 * j + 1] = h[j];
    }
    for (i = 0; i < 3; ++i) {
      float e = fabsf(Q[i][0]) * h[0] + fabsf(Q[i][1]) * h[1]
               + fabsf(Q[i][2]) * h[2];
      s->c[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
    }
  } break;
  case CP_CAPSULE: {
    if (!(p->radius > 0.0f) || !(p->length > 0.0f)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    s->R = p->radius;
    s->hl = 0.5 * (float)p->length;
    for (i = 0; i < 3; ++i)
      s->axis[i] = Q[i][0]; /* bx = WQB * [1,0,0]^T */
    for (i = 0; i < 3; ++i) {
      float e = s->hl * fabsf(s->axis[i]) + s->R;
      s->c[i] = r[i];
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
      s->c[i] = 0.0f;
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
        s->c[i] += ww;
        if (ww < mn[i]) mn[i] = ww;
        if (ww > mx[i]) mx[i] = ww;
      }
    }
    for (i = 0; i < 3; ++i) {
      lc[i] /= (float)n;
      s->c[i] /= (float)n;
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
  float r[3], Q[3][3], mn[3], mx[3];
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
      s->c[i] = r[i];
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
      s->c[i] = r[i];
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
      s->c[i] = r[i];
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
      s->c[i] = 0.0;
    }
    for (j = 0; j < n; ++j) {
      for (i = 0; i < 3; ++i) {
        float w = r[i] + Q[i][0] * p->verts[j][0]
                        + Q[i][1] * p->verts[j][1]
                        + Q[i][2] * p->verts[j][2];
        if (w < mn[i]) mn[i] = w;
        if (w > mx[i]) mx[i] = w;
        s->c[i] += w;
      }
    }
    for (i = 0; i < 3; ++i)
      s->c[i] /= (float)n;
  } break;
  default:
    s->status = CP_ERR_BAD_PRIM;
    return;
  }
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
      float w[CP_NVAR_MAX];
      for (i = 0; i < 3; ++i)
        pr->sA[k][i][g] = -s->axis[i];
      memset(w, 0, sizeof w);
      w[3] = -s->hl;
      w[g] = 1.0; /*  gamma - alpha*L/2 <= 0  (eq. 13) */
      add_lin(pr, w, 0.0);
      w[g] = -1.0; /* -gamma - alpha*L/2 <= 0  (eq. 13) */
      add_lin(pr, w, 0.0);
    }
  } else { /* box / polytope halfspaces (eq. 11) */
    for (i = 0; i < s->nface; ++i) {
      float w[CP_NVAR_MAX];
      memset(w, 0, sizeof w);
      w[0] = s->fa[i][0];
      w[1] = s->fa[i][1];
      w[2] = s->fa[i][2];
      w[3] = -s->fb[i];
      add_lin(pr, w, -d3dot(s->fa[i], cl));
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
  float c[3];     /* scaling center, world */
  float r[3];     /* body origin, world    */
  float Q[3][3];  /* world-from-body       */
  float R, hl, h[3];
  int    nv;
  float w[CP_MAX_POLY_VERTS][3]; /* polytope verts, world */
} vshape;

static float vdot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vsub(const float a[3], const float b[3], float o[3]) {
  o[0] = a[0] - b[0];
  o[1] = a[1] - b[1];
  o[2] = a[2] - b[2];
}

static void vcrs(const float a[3], const float b[3], float o[3]) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
}

static void make_vshape(const cp_prim *p, vshape *s) {
  int i, j;
  memset(s, 0, sizeof *s);
  s->type = (int)p->type;
  for (i = 0; i < 3; ++i) {
    s->r[i] = p->pos[i];
    for (j = 0; j < 3; ++j)
      s->Q[i][j] = p->rot[3 * i + j];
  }
  s->R = p->radius;
  s->hl = 0.5 * (float)p->length;
  for (i = 0; i < 3; ++i)
    s->h[i] = p->half_extent[i];
  if (p->type == CP_POLYTOPE) {
    s->nv = (int)p->vert_count;
    for (j = 0; j < s->nv; ++j) {
      for (i = 0; i < 3; ++i) {
        s->w[j][i] = s->r[i] + s->Q[i][0] * p->verts[j][0]
                            + s->Q[i][1] * p->verts[j][1]
                            + s->Q[i][2] * p->verts[j][2];
      }
    }
    for (i = 0; i < 3; ++i) {
      float acc = 0.0;
      for (j = 0; j < s->nv; ++j)
        acc += s->w[j][i];
      s->c[i] = acc / (float)s->nv;
    }
  } else {
    for (i = 0; i < 3; ++i)
      s->c[i] = s->r[i];
  }
}

/* support of S(alpha) = c + alpha*(S - c) */
static CP_FORCE_INLINE void sup_scaled(const vshape *s, float alpha, const float d[3],
                                       float out[3]) {
  int i;
  switch (s->type) {
  case CP_SPHERE: {
    float n = sqrtf(vdot(d, d));
    if (n < 1e-30) {
      out[0] = s->c[0] + alpha * s->R;
      out[1] = s->c[1];
      out[2] = s->c[2];
      return;
    }
    for (i = 0; i < 3; ++i)
      out[i] = s->c[i] + alpha * s->R * d[i] / n;
  } break;
  case CP_BOX: {
    float dl0 = s->Q[0][0] * d[0] + s->Q[1][0] * d[1] + s->Q[2][0] * d[2];
    float dl1 = s->Q[0][1] * d[0] + s->Q[1][1] * d[1] + s->Q[2][1] * d[2];
    float dl2 = s->Q[0][2] * d[0] + s->Q[1][2] * d[1] + s->Q[2][2] * d[2];
    dl0 = alpha * ((dl0 >= 0.0) ? s->h[0] : -s->h[0]);
    dl1 = alpha * ((dl1 >= 0.0) ? s->h[1] : -s->h[1]);
    dl2 = alpha * ((dl2 >= 0.0) ? s->h[2] : -s->h[2]);
    for (i = 0; i < 3; ++i)
      out[i] = s->c[i] + s->Q[i][0] * dl0 + s->Q[i][1] * dl1
                     + s->Q[i][2] * dl2;
  } break;
  case CP_CAPSULE: {
    float ax[3], t, n;
    for (i = 0; i < 3; ++i)
      ax[i] = s->Q[i][0];
    t = alpha * ((vdot(ax, d) >= 0.0) ? s->hl : -s->hl);
    n = sqrtf(vdot(d, d));
    for (i = 0; i < 3; ++i) {
      out[i] = s->c[i] + t * ax[i];
      if (n >= 1e-30)
        out[i] += alpha * s->R * d[i] / n;
    }
  } break;
  default: { /* CP_POLYTOPE: raw vertices */
    int best = 0, j;
    float bd = -1e30f;
    for (j = 0; j < s->nv; ++j) {
      float dd = vdot(s->w[j], d);
      if (dd > bd) {
        bd = dd;
        best = j;
      }
    }
    for (i = 0; i < 3; ++i)
      out[i] = s->c[i] + alpha * (s->w[best][i] - s->c[i]);
  } break;
  }
}

/* ---- closest point to origin on simplex, with reduction (Ericson) ---- */

static void closest_seg(const float a[3], const float b[3], float t01[1]) {
  float ab[3], t, dd;
  vsub(b, a, ab);
  dd = vdot(ab, ab);
  if (dd < 1e-30) {
    t01[0] = 0.0;
    return;
  }
  t = -vdot(a, ab) / dd;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  t01[0] = t;
}

/* closest point to origin on triangle abc; bary[3] out; returns mask of
 * vertices kept (bit0=a, bit1=b, bit2=c) */
static int closest_tri(const float a[3], const float b[3],
                       const float c[3], float bary[3]) {
  float ab[3], ac[3], bp[3], cp_[3];
  float d1, d2, d3, d4, d5, d6, va, vb, vc, denom, v, w;
  vsub(b, a, ab);
  vsub(c, a, ac);
  d1 = -vdot(ab, a);
  d2 = -vdot(ac, a);
  if (d1 <= 0.0 && d2 <= 0.0) {
    bary[0] = 1.0; bary[1] = 0.0; bary[2] = 0.0;
    return 1;
  }
  bp[0] = -b[0]; bp[1] = -b[1]; bp[2] = -b[2];
  d3 = vdot(ab, bp);
  d4 = vdot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3) {
    bary[0] = 0.0; bary[1] = 1.0; bary[2] = 0.0;
    return 2;
  }
  vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    v = d1 / (d1 - d3);
    bary[0] = 1.0 - v; bary[1] = v; bary[2] = 0.0;
    return 3;
  }
  cp_[0] = -c[0]; cp_[1] = -c[1]; cp_[2] = -c[2];
  d5 = vdot(ab, cp_);
  d6 = vdot(ac, cp_);
  if (d6 >= 0.0 && d5 <= d6) {
    bary[0] = 0.0; bary[1] = 0.0; bary[2] = 1.0;
    return 4;
  }
  vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    w = d2 / (d2 - d6);
    bary[0] = 1.0 - w; bary[1] = 0.0; bary[2] = w;
    return 5;
  }
  va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    bary[0] = 0.0; bary[1] = 1.0 - w; bary[2] = w;
    return 6;
  }
  denom = 1.0 / (va + vb + vc);
  v = vb * denom;
  w = vc * denom;
  bary[0] = 1.0 - v - w; bary[1] = v; bary[2] = w;
  return 7;
}

typedef struct {
  int    n;
  float p[4][3]; /* Minkowski-difference points              */
  float a[4][3]; /* support point on A for each vertex       */
} simplex;

/* Reduce the simplex to the minimal face supporting the closest point. Writes
 * the closest point to v and, using the same barycentric weights, the witness
 * on A (the contact location in scaled space) to xa. The a[] support points
 * are reduced in lockstep with p[]. Returns 1 if the origin is enclosed. */
static int simplex_closest(simplex *sx, float v[3], float xa[3]) {
  int i;
  if (sx->n == 1) {
    memcpy(v, sx->p[0], sizeof(float) * 3);
    memcpy(xa, sx->a[0], sizeof(float) * 3);
    return 0;
  }
  if (sx->n == 2) {
    float t;
    closest_seg(sx->p[0], sx->p[1], &t);
    for (i = 0; i < 3; ++i) {
      v[i]  = sx->p[0][i] + t * (sx->p[1][i] - sx->p[0][i]);
      xa[i] = sx->a[0][i] + t * (sx->a[1][i] - sx->a[0][i]);
    }
    if (t <= 0.0) {
      sx->n = 1;
    } else if (t >= 1.0) {
      memcpy(sx->p[0], sx->p[1], sizeof sx->p[0]);
      memcpy(sx->a[0], sx->a[1], sizeof sx->a[0]);
      sx->n = 1;
    }
    return 0;
  }
  if (sx->n == 3) {
    float bary[3], q[3][3], qa[3][3];
    int mask = closest_tri(sx->p[0], sx->p[1], sx->p[2], bary);
    int k = 0, j;
    for (i = 0; i < 3; ++i) {
      v[i]  = bary[0] * sx->p[0][i] + bary[1] * sx->p[1][i]
            + bary[2] * sx->p[2][i];
      xa[i] = bary[0] * sx->a[0][i] + bary[1] * sx->a[1][i]
            + bary[2] * sx->a[2][i];
    }
    for (j = 0; j < 3; ++j) {
      if (mask & (1 << j)) {
        memcpy(q[k], sx->p[j], sizeof q[k]);
        memcpy(qa[k], sx->a[j], sizeof qa[k]);
        ++k;
      }
    }
    memcpy(sx->p, q, sizeof(float) * 3 * (size_t)k);
    memcpy(sx->a, qa, sizeof(float) * 3 * (size_t)k);
    sx->n = k;
    return 0;
  }
  { /* tetrahedron */
    float bestv[3] = {0, 0, 0}, bestxa[3] = {0, 0, 0};
    float bestd = 1e30f;
    float keep[3][3], keepa[3][3];
    float mx = 0.0;
    int bestk = 0, found_outside = 0, f, j;
    static const int faces[4][3] = {
      {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}
    };
    for (f = 0; f < 4; ++f)
      for (j = 0; j < 3; ++j)
        if (fabsf(sx->p[f][j]) > mx)
          mx = fabsf(sx->p[f][j]);
    for (f = 0; f < 4; ++f) {
      int i0 = faces[f][0], i1 = faces[f][1], i2 = faces[f][2];
      const float *a = sx->p[i0];
      const float *b = sx->p[i1];
      const float *c = sx->p[i2];
      const float *d = sx->p[6 - i0 - i1 - i2];
      float ab[3], ac[3], nrm[3], ad[3], so, sd, nlen, outside;
      vsub(b, a, ab);
      vsub(c, a, ac);
      vcrs(ab, ac, nrm);
      vsub(d, a, ad);
      nlen = sqrtf(vdot(nrm, nrm));
      so = -vdot(nrm, a);      /* origin side  */
      sd = vdot(nrm, ad);      /* 4th-pt side  */
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
        float bary[3], cv[3], cxa[3], dd;
        int mask = closest_tri(a, b, c, bary), gi[3], k = 0;
        found_outside = 1;
        gi[0] = i0; gi[1] = i1; gi[2] = i2;
        for (j = 0; j < 3; ++j) {
          cv[j]  = bary[0] * a[j] + bary[1] * b[j] + bary[2] * c[j];
          cxa[j] = bary[0] * sx->a[i0][j] + bary[1] * sx->a[i1][j]
                 + bary[2] * sx->a[i2][j];
        }
        dd = vdot(cv, cv);
        if (dd < bestd) {
          bestd = dd;
          memcpy(bestv, cv, sizeof bestv);
          memcpy(bestxa, cxa, sizeof bestxa);
          for (j = 0; j < 3; ++j) {
            if (mask & (1 << j)) {
              memcpy(keep[k], sx->p[gi[j]], sizeof keep[k]);
              memcpy(keepa[k], sx->a[gi[j]], sizeof keepa[k]);
              ++k;
            }
          }
          bestk = k;
        }
      }
    }
    if (!found_outside)
      return 1;
    memcpy(sx->p, keep, sizeof(float) * 3 * (size_t)bestk);
    memcpy(sx->a, keepa, sizeof(float) * 3 * (size_t)bestk);
    sx->n = bestk;
    memcpy(v, bestv, sizeof(float) * 3);
    memcpy(xa, bestxa, sizeof(float) * 3);
    return 0;
  }
}

/* distance between S_a(alpha) and S_b(alpha); 0 means intersecting */
/* Distance between S_a(alpha) and S_b(alpha); 0 means intersecting. If xstar is
 * non-NULL it receives the witness point on A (the contact location in scaled
 * space) computed from the same simplex reduction used for the distance. */
static float gjk_dist(const vshape *sa, const vshape *sb, float alpha,
                       float xstar[3]) {
  simplex sx;
  float v[3], xa[3], d0[3];
  float pa[3], pb[3];
  int it, i;
  vsub(sb->c, sa->c, d0);
  if (vdot(d0, d0) <= 1e-30f) {
    d0[0] = 1.0;
    d0[1] = 0.0;
    d0[2] = 0.0;
  }
  sup_scaled(sa, alpha, d0, pa);
  d0[0] = -d0[0];
  d0[1] = -d0[1];
  d0[2] = -d0[2];
  sup_scaled(sb, alpha, d0, pb);
  vsub(pa, pb, sx.p[0]);
  memcpy(sx.a[0], pa, sizeof sx.a[0]);
  memcpy(xa, pa, sizeof xa);
  sx.n = 1;
  for (it = 0; it < 11; ++it) {
    float w[3], nd[3], vv, vw;
    if (simplex_closest(&sx, v, xa)) {
      if (xstar) memcpy(xstar, xa, sizeof(float) * 3);
      return 0.0;
    }
    vv = vdot(v, v);
    if (vv < 1e-12) {
      if (xstar) memcpy(xstar, xa, sizeof(float) * 3);
      return 0.0;
    }
    for (i = 0; i < 3; ++i)
      nd[i] = -v[i];
    sup_scaled(sa, alpha, nd, pa);
    for (i = 0; i < 3; ++i)
      nd[i] = v[i];
    sup_scaled(sb, alpha, nd, pb);
    vsub(pa, pb, w);
    vw = vdot(v, w);
    if (vv - vw <= 1e-6 * vv + 1e-12) {
      if (xstar) memcpy(xstar, xa, sizeof(float) * 3);
      return sqrtf(vv);
    }
    /* duplicate support point => no progress: converged */
    {
      int dup = 0, j;
      for (j = 0; j < sx.n; ++j) {
        float dx = w[0] - sx.p[j][0], dy = w[1] - sx.p[j][1],
               dz = w[2] - sx.p[j][2];
        if (dx * dx + dy * dy + dz * dz < 1e-12) {
          dup = 1;
          break;
        }
      }
      if (dup) {
        if (xstar) memcpy(xstar, xa, sizeof(float) * 3);
        return sqrtf(vv);
      }
    }
    memcpy(sx.p[sx.n], w, sizeof(float) * 3);
    memcpy(sx.a[sx.n], pa, sizeof sx.a[sx.n]);
    ++sx.n;
  }
  if (xstar) memcpy(xstar, xa, sizeof(float) * 3);
  return sqrtf(vdot(v, v));
}


static int sphere_box_predicate_ok(const vshape *sphere, const vshape *box,
                                   float alpha) {
  float rel[3], d[3], lhs, rhs, eps;
  int i;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  vsub(sphere->c, box->c, rel);
  lhs = 0.0;
  for (i = 0; i < 3; ++i) {
    float gap;
    d[i] = box->Q[0][i] * rel[0] + box->Q[1][i] * rel[1]
         + box->Q[2][i] * rel[2];
    gap = fabsf(d[i]) - alpha * box->h[i];
    if (gap > 0.0)
      lhs += gap * gap;
  }
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
  float rel[3], a[3], h[3], best;
  int i, mask;
  vsub(sphere->c, box->c, rel);
  for (i = 0; i < 3; ++i) {
    float d = box->Q[0][i] * rel[0] + box->Q[1][i] * rel[1]
             + box->Q[2][i] * rel[2];
    a[i] = fabsf(d);
    h[i] = box->h[i];
  }
  if (sphere_box_predicate_ok(sphere, box, 0.0))
    return 0.0;
  best = INFINITY;
  for (mask = 1; mask < 8; ++mask) {
    float D2 = 0.0, H = 0.0, H2 = 0.0;
    float A, C, disc;
    int r;
    for (i = 0; i < 3; ++i) {
      if (mask & (1 << i)) {
        D2 += a[i] * a[i];
        H += a[i] * h[i];
        H2 += h[i] * h[i];
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
        float gap = a[i] - alpha * h[i];
        float tol = 1e-6 * (1.0 + a[i] + fabsf(alpha * h[i]));
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
  float d[3], x, p2, gap, lhs, rhs, R, eps;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  vsub(sphere->c, capsule->c, d);
  x = fabsf(d[0] * capsule->Q[0][0] + d[1] * capsule->Q[1][0]
         + d[2] * capsule->Q[2][0]);
  p2 = vdot(d, d) - x * x;
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
  float d[3], x, p2, R, hl, best;
  vsub(sphere->c, capsule->c, d);
  x = fabsf(d[0] * capsule->Q[0][0] + d[1] * capsule->Q[1][0]
         + d[2] * capsule->Q[2][0]);
  p2 = vdot(d, d) - x * x;
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
  float delta[3], d, r;
  vsub(a->c, b->c, delta);
  d = sqrtf(vdot(delta, delta));
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
  float pa0[3], pa1[3], pb0[3], pb1[3];
  float d1[3], d2[3], r[3], cd[3];
  float aa, bb, cc, ee, ff, denom, s, t;
  int i;
  for (i = 0; i < 3; ++i) {
    float ua = a->Q[i][0];
    float ub = b->Q[i][0];
    pa0[i] = a->c[i] - alpha * a->hl * ua;
    pa1[i] = a->c[i] + alpha * a->hl * ua;
    pb0[i] = b->c[i] - alpha * b->hl * ub;
    pb1[i] = b->c[i] + alpha * b->hl * ub;
    d1[i] = pa1[i] - pa0[i];
    d2[i] = pb1[i] - pb0[i];
    r[i] = pa0[i] - pb0[i];
  }
  aa = vdot(d1, d1);
  ee = vdot(d2, d2);
  if (aa <= 1e-30 && ee <= 1e-30)
    return vdot(r, r);
  if (aa <= 1e-30) {
    s = 0.0;
    t = clamp01(vdot(d2, r) / ee);
  } else {
    cc = vdot(d1, r);
    if (ee <= 1e-30) {
      t = 0.0;
      s = clamp01(-cc / aa);
    } else {
      bb = vdot(d1, d2);
      ff = vdot(d2, r);
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
  for (i = 0; i < 3; ++i)
    cd[i] = r[i] + s * d1[i] - t * d2[i];
  return vdot(cd, cd);
}

static int capsule_capsule_predicate_ok(const vshape *a, const vshape *b,
                                        float alpha) {
  float dist2, R, rhs, coord, len, eps;
  int i;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  dist2 = segment_segment_dist2_scaled_capsules(a, b, alpha);
  R = alpha * (a->R + b->R);
  rhs = R * R;
  coord = 0.0;
  for (i = 0; i < 3; ++i) {
    float ua = alpha * a->hl * a->Q[i][0];
    float ub = alpha * b->hl * b->Q[i][0];
    float vals[4];
    int j;
    vals[0] = a->c[i] - ua;
    vals[1] = a->c[i] + ua;
    vals[2] = b->c[i] - ub;
    vals[3] = b->c[i] + ub;
    for (j = 0; j < 4; ++j) {
      float av = fabsf(vals[j]);
      if (av > coord)
        coord = av;
    }
  }
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

static float segment_aabb_dist2(const float p0[3], const float p1[3],
                                 const float e[3]) {
  float v[3], cand[8], best;
  int nc, i, j;
  for (i = 0; i < 3; ++i)
    v[i] = p1[i] - p0[i];

  nc = 0;
  cand[nc++] = 0.0;
  cand[nc++] = 1.0;
  for (i = 0; i < 3; ++i) {
    if (fabsf(v[i]) > 1e-30) {
      float t = (e[i] - p0[i]) / v[i];
      if (t >= 0.0 && t <= 1.0)
        cand[nc++] = t;
      t = (-e[i] - p0[i]) / v[i];
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
    float d2 = 0.0;
    for (j = 0; j < 3; ++j) {
      float q = p0[j] + cand[i] * v[j];
      float gap = fabsf(q) - e[j];
      if (gap > 0.0)
        d2 += gap * gap;
    }
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
      float q = p0[j] + mid * v[j];
      if (q > e[j]) {
        float m = v[j];
        float c = p0[j] - e[j];
        A += m * m;
        B += m * c;
        C += c * c;
      } else if (q < -e[j]) {
        float m = -v[j];
        float c = -p0[j] - e[j];
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
  float rel[3], center_local[3], axis_local[3];
  float p0[3], p1[3], e[3], dist2, R, rhs, coord, len, eps;
  int i;
  if (!(alpha >= 0.0) || !isfinite(alpha))
    return 0;
  vsub(capsule->c, box->c, rel);
  coord = 0.0;
  len = alpha * (capsule->hl + capsule->R);
  for (i = 0; i < 3; ++i) {
    center_local[i] = box->Q[0][i] * rel[0] + box->Q[1][i] * rel[1]
                    + box->Q[2][i] * rel[2];
    axis_local[i] = box->Q[0][i] * capsule->Q[0][0]
                  + box->Q[1][i] * capsule->Q[1][0]
                  + box->Q[2][i] * capsule->Q[2][0];
    p0[i] = center_local[i] - alpha * capsule->hl * axis_local[i];
    p1[i] = center_local[i] + alpha * capsule->hl * axis_local[i];
    e[i] = alpha * box->h[i];
    if (fabsf(p0[i]) > coord)
      coord = fabsf(p0[i]);
    if (fabsf(p1[i]) > coord)
      coord = fabsf(p1[i]);
    if (e[i] > coord)
      coord = e[i];
    len += e[i];
  }
  dist2 = segment_aabb_dist2(p0, p1, e);
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
                                  const float poly_rel[][3],
                                  float axes[CP_MAX_FACES][3]) {
  float scale = 0.0;
  int i, j, k, m, count;
  count = 0;
  for (i = 0; i < poly->nv; ++i) {
    for (j = i + 1; j < poly->nv; ++j) {
      float dx = fabsf(poly_rel[j][0] - poly_rel[i][0]);
      float dy = fabsf(poly_rel[j][1] - poly_rel[i][1]);
      float dz = fabsf(poly_rel[j][2] - poly_rel[i][2]);
      if (dx > scale) scale = dx;
      if (dy > scale) scale = dy;
      if (dz > scale) scale = dz;
    }
  }
  for (i = 0; i < poly->nv; ++i) {
    for (j = i + 1; j < poly->nv; ++j) {
      for (k = j + 1; k < poly->nv; ++k) {
        float e1[3], e2[3], nm[3], len, dmax, dmin;
        int face, dup, f;
        for (m = 0; m < 3; ++m) {
          e1[m] = poly_rel[j][m] - poly_rel[i][m];
          e2[m] = poly_rel[k][m] - poly_rel[i][m];
        }
        vcrs(e1, e2, nm);
        len = sqrtf(vdot(nm, nm));
        if (!isfinite(len))
          return -1;
        if (len <= 1e-6 * (scale * scale + 1.0))
          continue;
        nm[0] /= len;
        nm[1] /= len;
        nm[2] /= len;
        dmax = -1e30f;
        dmin = 1e30f;
        for (m = 0; m < poly->nv; ++m) {
          float d = nm[0] * (poly_rel[m][0] - poly_rel[i][0])
                   + nm[1] * (poly_rel[m][1] - poly_rel[i][1])
                   + nm[2] * (poly_rel[m][2] - poly_rel[i][2]);
          if (!isfinite(d))
            return -1;
          if (d > dmax) dmax = d;
          if (d < dmin) dmin = d;
        }
        face = 0;
        if (dmax <= 1e-6 * (scale + 1.0)) {
          face = 1;
        } else if (dmin >= -1e-6 * (scale + 1.0)) {
          nm[0] = -nm[0];
          nm[1] = -nm[1];
          nm[2] = -nm[2];
          face = 1;
        }
        if (!face)
          continue;
        dup = 0;
        for (f = 0; f < count; ++f) {
          if (vdot(nm, axes[f]) > 1.0 - 1e-6) {
            dup = 1;
            break;
          }
        }
        if (!dup) {
          if (count >= CP_MAX_FACES)
            return -1;
          axes[count][0] = nm[0];
          axes[count][1] = nm[1];
          axes[count][2] = nm[2];
          ++count;
        }
      }
    }
  }
  return count >= 4 ? count : -1;
}

static float box_poly_axis_alpha_asym(const vshape *box,
                                       const vshape *poly,
                                       const float poly_rel[][3],
                                       const float center_delta[3],
                                       const float L[3]) {
  float len2, rb, pmin, pmax, delta, denom, candidate;
  int i, j;
  len2 = vdot(L, L);
  if (!isfinite(len2))
    return INFINITY;
  if (len2 < 1e-12)
    return -1.0; /* skip near-zero axes; L need not be normalized */

  rb = 0.0;
  for (j = 0; j < 3; ++j) {
    float axis[3];
    float d;
    for (i = 0; i < 3; ++i)
      axis[i] = box->Q[i][j];
    d = fabsf(vdot(axis, L));
    if (!isfinite(d))
      return INFINITY;
    rb += box->h[j] * d;
  }
  if (!isfinite(rb))
    return INFINITY;

  pmin = INFINITY;
  pmax = -INFINITY;
  (void)poly;
  for (j = 0; j < poly->nv; ++j) {
    float proj = vdot(poly_rel[j], L);
    if (!isfinite(proj))
      return INFINITY;
    if (proj < pmin)
      pmin = proj;
    if (proj > pmax)
      pmax = proj;
  }

  delta = vdot(center_delta, L); /* P - B */
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
  float poly_axes[CP_MAX_FACES][3];
  float poly_rel[CP_MAX_POLY_VERTS][3], center_delta[3];
  int poly_axis_count;
  int used = 0;
  int i, j, k, f, e;

  vsub(poly->c, box->c, center_delta);
  for (i = 0; i < poly->nv; ++i)
    vsub(poly->w[i], poly->c, poly_rel[i]);

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

  for (j = 0; j < 3; ++j) {
    float axis[3];
    for (i = 0; i < 3; ++i)
      axis[i] = box->Q[i][j];
    USE_BOX_POLY_AXIS(axis);
  }

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
    float box_axis[3];
    for (i = 0; i < 3; ++i)
      box_axis[i] = box->Q[i][k];
    if (poly_shape != NULL && poly_shape->status == CP_OK &&
        poly_shape->type == CP_POLYTOPE && poly_shape->nedge > 0) {
      for (e = 0; e < poly_shape->nedge; ++e) {
        float axis[3];
        vcrs(box_axis, poly_shape->edge[e], axis);
        USE_BOX_POLY_AXIS(axis);
      }
    } else {
      for (i = 0; i < poly->nv; ++i) {
        for (j = i + 1; j < poly->nv; ++j) {
          float dir[3], axis[3];
          vsub(poly_rel[j], poly_rel[i], dir);
          vcrs(box_axis, dir, axis);
          USE_BOX_POLY_AXIS(axis);
        }
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
  float a_axes[3][3], b_axes[3][3], d[3], alpha;
  int i, j, k;
  for (i = 0; i < 3; ++i) {
    for (j = 0; j < 3; ++j) {
      a_axes[i][j] = a->Q[j][i];
      b_axes[i][j] = b->Q[j][i];
    }
  }
  vsub(b->c, a->c, d);
  alpha = 0.0;
  for (k = 0; k < 15; ++k) {
    float L[3], len2, num, ra, rb, denom, candidate;
    if (k < 3) {
      memcpy(L, a_axes[k], sizeof L);
    } else if (k < 6) {
      memcpy(L, b_axes[k - 3], sizeof L);
    } else {
      vcrs(a_axes[(k - 6) / 3], b_axes[(k - 6) % 3], L);
      len2 = vdot(L, L);
      if (!isfinite(len2))
        return INFINITY;
      if (len2 < 1e-12)
        continue;
    }
    if (!isfinite(L[0]) || !isfinite(L[1]) || !isfinite(L[2]))
      return INFINITY;
    num = fabsf(vdot(d, L));
    if (!isfinite(num))
      return INFINITY;
    ra = 0.0;
    rb = 0.0;
    for (j = 0; j < 3; ++j) {
      float da = fabsf(vdot(a_axes[j], L));
      float db = fabsf(vdot(b_axes[j], L));
      if (!isfinite(da) || !isfinite(db))
        return INFINITY;
      ra += a->h[j] * da;
      rb += b->h[j] * db;
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
static void closest_seg_seg(const float P0[3], const float P1[3],
                            const float Q0[3], const float Q1[3],
                            float c1[3], float c2[3]) {
  float d1[3], d2[3], r[3], a, e, f, s, t;
  int i;
  for (i = 0; i < 3; ++i) { d1[i] = P1[i] - P0[i]; d2[i] = Q1[i] - Q0[i]; r[i] = P0[i] - Q0[i]; }
  a = vdot(d1, d1); e = vdot(d2, d2); f = vdot(d2, r);
  if (a <= 1e-12 && e <= 1e-12) { s = 0.0; t = 0.0; }
  else if (a <= 1e-12) { s = 0.0; t = f / e; if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0; }
  else {
    float c = vdot(d1, r);
    if (e <= 1e-12) { t = 0.0; s = -c / a; if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0; }
    else {
      float b = vdot(d1, d2), denom = a * e - b * b;
      s = (denom > 1e-12 || denom < -1e-12) ? (b * f - c * e) / denom : 0.0;
      if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0;
      t = (b * s + f) / e;
      if (t < 0.0)      { t = 0.0; s = -c / a;       if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0; }
      else if (t > 1.0) { t = 1.0; s = (b - c) / a;  if (s < 0.0) s = 0.0; else if (s > 1.0) s = 1.0; }
    }
  }
  for (i = 0; i < 3; ++i) { c1[i] = P0[i] + s * d1[i]; c2[i] = Q0[i] + t * d2[i]; }
}

static int radius_poly_witness(const vshape *radius_shape,
                               const vshape *poly,
                               float alpha,
                               float xstar[3]) {
  vshape core;
  float ypoly[3], d, eps;
  int i;
  if (!(alpha > CP_ALPHA_EPS) || !isfinite(alpha) ||
      radius_shape->type == CP_POLYTOPE || poly->type != CP_POLYTOPE)
    return 0;
  core = *radius_shape;
  core.R = 0.0f;
  for (i = 0; i < 3; ++i) {
    float q = poly->c[i] + (radius_shape->c[i] - poly->c[i]) / alpha;
    if (!isfinite(q))
      return 0;
    core.c[i] = q;
    core.r[i] = q;
  }
  d = gjk_dist(poly, &core, 1.0f, ypoly);
  if (!isfinite(d))
    return 0;
  eps = 1e-3f; /* accept <=1 mm radius-poly witness slack; fallback otherwise */
  if (d > radius_shape->R + eps)
    return 0;
  for (i = 0; i < 3; ++i) {
    xstar[i] = poly->c[i] + alpha * (ypoly[i] - poly->c[i]);
    if (!isfinite(xstar[i]))
      return 0;
  }
  return 1;
}

static float segment_aabb_closest_local(const float p0[3],
                                        const float p1[3],
                                        const float e[3],
                                        float seg_out[3],
                                        float box_out[3]) {
  float v[3], cand[8], best, best_t;
  int nc, i, j;
  for (i = 0; i < 3; ++i)
    v[i] = p1[i] - p0[i];

  nc = 0;
  cand[nc++] = 0.0f;
  cand[nc++] = 1.0f;
  for (i = 0; i < 3; ++i) {
    if (fabsf(v[i]) > 1e-30f) {
      float t = (e[i] - p0[i]) / v[i];
      if (t >= 0.0f && t <= 1.0f)
        cand[nc++] = t;
      t = (-e[i] - p0[i]) / v[i];
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
    float d2 = 0.0f;
    for (j = 0; j < 3; ++j) {
      float q = p0[j] + cand[i] * v[j];
      float gap = fabsf(q) - e[j];
      if (gap > 0.0f)
        d2 += gap * gap;
    }
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
      float q = p0[j] + mid * v[j];
      if (q > e[j]) {
        float m = v[j];
        float c = p0[j] - e[j];
        A += m * m;
        B += m * c;
        C += c * c;
      } else if (q < -e[j]) {
        float m = -v[j];
        float c = -p0[j] - e[j];
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

  for (i = 0; i < 3; ++i) {
    float q = p0[i] + best_t * v[i];
    seg_out[i] = q;
    if (q > e[i])
      box_out[i] = e[i];
    else if (q < -e[i])
      box_out[i] = -e[i];
    else
      box_out[i] = q;
  }
  return best;
}

static int box_capsule_witness(const vshape *box, const vshape *capsule,
                               float alpha, float xstar[3]) {
  float rel[3], center_local[3], axis_local[3], p0[3], p1[3], e[3];
  float segp[3], boxp[3], dvec[3], dist2, dist, R, eps;
  int i;
  if (!(alpha > CP_ALPHA_EPS) || !isfinite(alpha) ||
      box->type != CP_BOX || capsule->type != CP_CAPSULE)
    return 0;
  vsub(capsule->c, box->c, rel);
  for (i = 0; i < 3; ++i) {
    center_local[i] = box->Q[0][i] * rel[0] + box->Q[1][i] * rel[1]
                    + box->Q[2][i] * rel[2];
    axis_local[i] = box->Q[0][i] * capsule->Q[0][0]
                  + box->Q[1][i] * capsule->Q[1][0]
                  + box->Q[2][i] * capsule->Q[2][0];
    p0[i] = center_local[i] - alpha * capsule->hl * axis_local[i];
    p1[i] = center_local[i] + alpha * capsule->hl * axis_local[i];
    e[i] = alpha * box->h[i];
    if (!isfinite(p0[i]) || !isfinite(p1[i]) || !isfinite(e[i]))
      return 0;
  }
  dist2 = segment_aabb_closest_local(p0, p1, e, segp, boxp);
  if (!isfinite(dist2))
    return 0;
  for (i = 0; i < 3; ++i)
    dvec[i] = boxp[i] - segp[i];
  dist = sqrtf(vdot(dvec, dvec));
  R = alpha * capsule->R;
  eps = 1e-5f * (1.0f + fabsf(dist) + fabsf(R));
  if (!(dist > 1e-6f) || dist > R + eps || dist < R - eps)
    return 0;
  for (i = 0; i < 3; ++i) {
    xstar[i] = box->c[i] + box->Q[i][0] * boxp[0]
                         + box->Q[i][1] * boxp[1]
                         + box->Q[i][2] * boxp[2];
    if (!isfinite(xstar[i]))
      return 0;
  }
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
                            float xstar[3]) {
  const vshape *sph, *oth;
  float from[3], d[3], len;
  int i, j;
  /* No sphere: capsule-capsule is closed-form (closest points between the two
   * scaled segment cores, then offset by radius — the radius-shape analogue of
   * sphere-sphere). Other non-sphere pairs (box-box, box-poly, poly-poly) fall
   * back to GJK. */
  if (sa->type != CP_SPHERE && sb->type != CP_SPHERE) {
    if (sa->type == CP_CAPSULE && sb->type == CP_CAPSULE) {
      float axA[3], axB[3], PA0[3], PA1[3], QB0[3], QB1[3], c1[3], c2[3], n[3];
      for (i = 0; i < 3; ++i) { axA[i] = sa->Q[i][0]; axB[i] = sb->Q[i][0]; }
      for (i = 0; i < 3; ++i) {
        PA0[i] = sa->c[i] - alpha * sa->hl * axA[i];
        PA1[i] = sa->c[i] + alpha * sa->hl * axA[i];
        QB0[i] = sb->c[i] - alpha * sb->hl * axB[i];
        QB1[i] = sb->c[i] + alpha * sb->hl * axB[i];
      }
      closest_seg_seg(PA0, PA1, QB0, QB1, c1, c2);
      for (i = 0; i < 3; ++i) n[i] = c2[i] - c1[i];
      len = sqrtf(vdot(n, n));
      if (len < 1e-6) return 0;          /* cores intersect: let GJK handle */
      for (i = 0; i < 3; ++i) xstar[i] = c1[i] + alpha * sa->R * n[i] / len;
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
  for (i = 0; i < 3; ++i) from[i] = sph->c[i];

  if (oth->type == CP_SPHERE) {
    for (i = 0; i < 3; ++i) d[i] = oth->c[i] - from[i];
    len = sqrtf(vdot(d, d));
    if (len < 1e-6) return 0;
    for (i = 0; i < 3; ++i)
      xstar[i] = oth->c[i] - alpha * oth->R * d[i] / len;
    return 1;
  }
  if (oth->type == CP_BOX) {
    float q[3]; int clamped = 0;
    for (i = 0; i < 3; ++i) q[i] = oth->c[i];
    for (j = 0; j < 3; ++j) {
      float ax[3], e = 0.0, lim = alpha * oth->h[j];
      for (i = 0; i < 3; ++i) ax[i] = oth->Q[i][j];
      for (i = 0; i < 3; ++i) e += (from[i] - oth->c[i]) * ax[i];
      if (e > lim)      { e = lim;  clamped = 1; }
      else if (e < -lim){ e = -lim; clamped = 1; }
      for (i = 0; i < 3; ++i) q[i] += e * ax[i];
    }
    if (!clamped) return 0;            /* sphere centre inside scaled box */
    for (i = 0; i < 3; ++i) xstar[i] = q[i];
    return 1;
  }
  if (oth->type == CP_CAPSULE) {
    float ax[3], t = 0.0, lim = alpha * oth->hl, sp[3];
    for (i = 0; i < 3; ++i) ax[i] = oth->Q[i][0];
    for (i = 0; i < 3; ++i) t += (from[i] - oth->c[i]) * ax[i];
    if (t > lim) t = lim; else if (t < -lim) t = -lim;
    for (i = 0; i < 3; ++i) sp[i] = oth->c[i] + t * ax[i];
    for (i = 0; i < 3; ++i) d[i] = from[i] - sp[i];
    len = sqrtf(vdot(d, d));
    if (len < 1e-6) return 0;          /* sphere centre on the capsule axis */
    for (i = 0; i < 3; ++i) xstar[i] = sp[i] + alpha * oth->R * d[i] / len;
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
                             float xstar_out[3]) {
  vshape sa = *pa, sb = *pb;
  float lo, hi, cd[3], alpha;
  int i;

  for (i = 0; i < 3; ++i)
    cd[i] = sa.c[i] - sb.c[i];

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
    for (i = 0; i < 3; ++i)
      xstar_out[i] = sa.c[i];
  } else {
    /* The witness is a barycentric combination of A's support points on the
     * boundary of the scaled shape, so unscaling it by alpha (eq. 24) lands the
     * contact exactly on the original surface — but only if it is evaluated
     * where the scaled shapes are (barely) separated. At the solution alpha they
     * touch/overlap; step to the touch boundary from below by bisection (tight,
     * so the contact stays on-surface to well under tolerance), then read the
     * witness there. Distance still uses the true alpha. */
    *dist_out = sqrtf(vdot(cd, cd)) * (1.0 - 1.0 / alpha);
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
static void snap_to_surface(const cp_shape *s, const float c[3], float p[3]) {
  int i;
  if (s->type == CP_SPHERE) {
    float d[3], n;
    for (i = 0; i < 3; ++i) d[i] = p[i] - c[i];
    n = sqrtf(vdot(d, d));
    if (n > 1e-6)
      for (i = 0; i < 3; ++i) p[i] = c[i] + s->R * d[i] / n;
  } else if (s->type == CP_CAPSULE) {
    float rel[3], t, cp[3], d[3], n;
    for (i = 0; i < 3; ++i) rel[i] = p[i] - c[i];
    t = vdot(rel, s->axis);
    if (t > s->hl) t = s->hl; else if (t < -s->hl) t = -s->hl;
    for (i = 0; i < 3; ++i) cp[i] = c[i] + t * s->axis[i];
    for (i = 0; i < 3; ++i) d[i] = p[i] - cp[i];
    n = sqrtf(vdot(d, d));
    if (n > 1e-6)
      for (i = 0; i < 3; ++i) p[i] = cp[i] + s->R * d[i] / n;
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
      int imax = 0, j;
      float dmax = -1e30f;
      for (i = 0; i < s->nface; ++i) {
        float rel[3], d;
        for (j = 0; j < 3; ++j) rel[j] = p[j] - c[j];
        d = vdot(s->fa[i], rel) - s->fb[i];
        if (d > dmax) { dmax = d; imax = i; }
      }
      /* Project onto the nearest/most-violated face plane (moves the point
       * outward onto the boundary when interior, inward when exterior). */
      for (i = 0; i < 3; ++i) p[i] -= dmax * s->fa[imax][i];
      if (dmax > -1e-6 && dmax < 1e-6)
        break;
    }
  }
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
  float piv[3], alpha = 0.0, dist = 0.0, xstar[3];
  int j, k;
  memset(&r, 0, sizeof r);
  for (k = 0; k < 3; ++k) piv[k] = A.c[k];
  for (k = 0; k < 3; ++k) {
    A.c[k] -= piv[k]; A.r[k] -= piv[k];
    B.c[k] -= piv[k]; B.r[k] -= piv[k];
  }
  for (j = 0; j < A.nv; ++j) for (k = 0; k < 3; ++k) A.w[j][k] -= piv[k];
  for (j = 0; j < B.nv; ++j) for (k = 0; k < 3; ++k) B.w[j][k] -= piv[k];
  if (opt_val_collide_v(&A, &B, sA, sB, &alpha, &dist, xstar)) {
    r.status = CP_ERR_NO_CONVERGE;
    *out = r;
    return;
  }
  r.status = CP_OK;
  r.alpha = alpha;
  r.distance = dist;
  r.colliding = (alpha < 1.0f) ? 1u : 0u;
  if (alpha <= CP_ALPHA_EPS) {
    for (k = 0; k < 3; ++k) { r.p1[k] = A.c[k] + piv[k]; r.p2[k] = A.c[k] + piv[k]; }
    r.distance = 0.0f;
    r.colliding = 1u;
  } else {
    /* eq. (24): map the witness back onto each shape (re-centered frame), snap
     * radius/convex shapes onto their surface, then shift back to world. */
    float q1[3], q2[3];
    for (k = 0; k < 3; ++k) {
      q1[k] = A.c[k] + (xstar[k] - A.c[k]) / alpha;
      q2[k] = B.c[k] + (xstar[k] - B.c[k]) / alpha;
    }
    if (sA) {
      float cA[3];
      for (k = 0; k < 3; ++k) cA[k] = sA->c[k] - piv[k];
      snap_to_surface(sA, cA, q1);
    }
    if (sB) {
      float cB[3];
      for (k = 0; k < 3; ++k) cB[k] = sB->c[k] - piv[k];
      snap_to_surface(sB, cB, q2);
    }
    for (k = 0; k < 3; ++k) { r.p1[k] = q1[k] + piv[k]; r.p2[k] = q2[k] + piv[k]; }
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
