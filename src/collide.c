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
 * method in double precision (n <= 6 unknowns: x, alpha, up to two
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

#define CP_DOMAIN_MAX 8192.0
#define CP_EXTENT_MIN 0.1
#define CP_EXTENT_MAX 250.0
#define CP_MAX_FACES  64
#define CP_MAX_ROWS   (2 * CP_MAX_FACES + 5)
#define CP_NVAR_MAX   6
#define CP_ALPHA_EPS  1e-7   /* below this, centers coincide: see policy */
#define CP_GAP_TOL    1e-10  /* barrier duality-gap bound on alpha error */

/* ------------------------------------------------------------------ */
/* validated, precomputed world-space form of one primitive            */
typedef struct {
  uint32_t status;
  uint32_t type;
  double   c[3];                 /* scaling center, world              */
  double   R;                    /* sphere/capsule radius              */
  double   axis[3];              /* capsule axis (world, unit)         */
  double   hl;                   /* capsule half segment length        */
  int      nface;                /* box: 6; polytope: hull faces       */
  double   fa[CP_MAX_FACES][3];  /* unit outward normals, world        */
  double   fb[CP_MAX_FACES];     /* a.(x - c) <= alpha*fb, fb > 0      */
} cp_shape;

static double d3dot(const double a[3], const double b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void d3cross(const double a[3], const double b[3], double o[3]) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
}

/* rotation must be orthonormal (3e-3 tol) with det ~ +1 */
static int rot_ok(const float rotf[9]) {
  double q[3][3];
  int i, j;
  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j)
      q[i][j] = rotf[3 * i + j];
  for (i = 0; i < 3; ++i) {
    double n = q[i][0] * q[i][0] + q[i][1] * q[i][1] + q[i][2] * q[i][2];
    if (fabs(n - 1.0) > 3e-3)
      return 0;
  }
  for (i = 0; i < 3; ++i) {
    for (j = i + 1; j < 3; ++j) {
      double d = q[i][0] * q[j][0] + q[i][1] * q[j][1] + q[i][2] * q[j][2];
      if (fabs(d) > 3e-3)
        return 0;
    }
  }
  {
    double det = q[0][0] * (q[1][1] * q[2][2] - q[1][2] * q[2][1])
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
static int poly_faces(int n, const double w[][3], const double c[3],
                      double scale, cp_shape *s) {
  const double tol = 1e-7 * (scale + 1.0);
  int i, j, k, m, f;
  s->nface = 0;
  for (i = 0; i < n; ++i) {
    for (j = i + 1; j < n; ++j) {
      for (k = j + 1; k < n; ++k) {
        double e1[3], e2[3], nm[3], len, dmax, dmin, b;
        int face, dup;
        for (m = 0; m < 3; ++m) {
          e1[m] = w[j][m] - w[i][m];
          e2[m] = w[k][m] - w[i][m];
        }
        d3cross(e1, e2, nm);
        len = sqrt(d3dot(nm, nm));
        if (len <= 1e-10 * (scale * scale + 1.0))
          continue;
        nm[0] /= len;
        nm[1] /= len;
        nm[2] /= len;
        dmax = -1e300;
        dmin = 1e300;
        for (m = 0; m < n; ++m) {
          double d = nm[0] * (w[m][0] - w[i][0])
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
        b = -1e300;
        for (m = 0; m < n; ++m) {
          double d = nm[0] * (w[m][0] - c[0])
                   + nm[1] * (w[m][1] - c[1])
                   + nm[2] * (w[m][2] - c[2]);
          if (d > b) b = d;
        }
        if (b < 1e-6 * (scale + 1.0))
          return 0; /* center not strictly interior */
        dup = 0;
        for (f = 0; f < s->nface; ++f) {
          if (d3dot(nm, s->fa[f]) > 1.0 - 1e-8 &&
              fabs(b - s->fb[f]) <= 1e-6 * (1.0 + b)) {
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

/* Validate one primitive and build its world-space solver form.
 * Check order (explicit): rotation/params (BAD_PRIM), then coordinate
 * range (COORD_RANGE), then extent range (SIZE_RANGE). */
static void build_shape(const cp_prim *p, cp_shape *s) {
  double r[3], Q[3][3], mn[3], mx[3];
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
    double h[3];
    for (i = 0; i < 3; ++i) {
      h[i] = p->half_extent[i];
      if (!(p->half_extent[i] > 0.0f)) {
        s->status = CP_ERR_BAD_PRIM;
        return;
      }
    }
    for (i = 0; i < 3; ++i) {
      double e = fabs(Q[i][0]) * h[0] + fabs(Q[i][1]) * h[1]
               + fabs(Q[i][2]) * h[2];
      s->c[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
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
  } break;
  case CP_CAPSULE: {
    if (!(p->radius > 0.0f) || !(p->length > 0.0f)) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    s->R = p->radius;
    s->hl = 0.5 * (double)p->length;
    for (i = 0; i < 3; ++i)
      s->axis[i] = Q[i][0]; /* bx = WQB * [1,0,0]^T */
    for (i = 0; i < 3; ++i) {
      double e = s->hl * fabs(s->axis[i]) + s->R;
      s->c[i] = r[i];
      mn[i] = r[i] - e;
      mx[i] = r[i] + e;
    }
  } break;
  case CP_POLYTOPE: {
    double w[CP_MAX_POLY_VERTS][3];
    double scale;
    int n = (int)p->vert_count;
    if (n < 4 || n > CP_MAX_POLY_VERTS) {
      s->status = CP_ERR_BAD_PRIM;
      return;
    }
    for (i = 0; i < 3; ++i) {
      mn[i] = 1e300;
      mx[i] = -1e300;
      s->c[i] = 0.0;
    }
    for (j = 0; j < n; ++j) {
      for (i = 0; i < 3; ++i) {
        w[j][i] = r[i] + Q[i][0] * p->verts[j][0]
                       + Q[i][1] * p->verts[j][1]
                       + Q[i][2] * p->verts[j][2];
        if (w[j][i] < mn[i]) mn[i] = w[j][i];
        if (w[j][i] > mx[i]) mx[i] = w[j][i];
        s->c[i] += w[j][i];
      }
    }
    for (i = 0; i < 3; ++i)
      s->c[i] /= (double)n;
    scale = 0.0;
    for (i = 0; i < 3; ++i)
      if (mx[i] - mn[i] > scale)
        scale = mx[i] - mn[i];
    if (!poly_faces(n, w, s->c, scale, s)) {
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
    double ext = mx[i] - mn[i];
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
  double lw[CP_MAX_ROWS][CP_NVAR_MAX]; /* linear rows: lw.u + ld <= 0  */
  double ld[CP_MAX_ROWS];
  double sA[2][3][CP_NVAR_MAX];        /* SOC: |sA.u + sb| <= alpha*sR */
  double sb[2][3];
  double sR[2];
} cp_prob;

static void add_lin(cp_prob *pr, const double w[CP_NVAR_MAX], double d) {
  memcpy(pr->lw[pr->nlin], w, sizeof(double) * CP_NVAR_MAX);
  pr->ld[pr->nlin] = d;
  ++pr->nlin;
}

/* cl = shape scaling center in the pair-local frame */
static void add_shape_constraints(cp_prob *pr, const cp_shape *s,
                                  const double cl[3]) {
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
      double w[CP_NVAR_MAX];
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
      double w[CP_NVAR_MAX];
      memset(w, 0, sizeof w);
      w[0] = s->fa[i][0];
      w[1] = s->fa[i][1];
      w[2] = s->fa[i][2];
      w[3] = -s->fb[i];
      add_lin(pr, w, -d3dot(s->fa[i], cl));
    }
  }
}

static int p_feas(const cp_prob *pr, const double u[]) {
  int i, j, k;
  for (i = 0; i < pr->nlin; ++i) {
    double f = pr->ld[i];
    for (j = 0; j < pr->nvar; ++j)
      f += pr->lw[i][j] * u[j];
    if (f >= 0.0)
      return 0;
  }
  for (k = 0; k < pr->nsoc; ++k) {
    double tr = u[3] * pr->sR[k];
    double v[3], s2;
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
static double p_phi(const cp_prob *pr, double tpar, const double u[]) {
  int i, j, k;
  double phi = tpar * u[3];
  for (i = 0; i < pr->nlin; ++i) {
    double f = pr->ld[i];
    for (j = 0; j < pr->nvar; ++j)
      f += pr->lw[i][j] * u[j];
    phi -= log(-f);
  }
  for (k = 0; k < pr->nsoc; ++k) {
    double tr = u[3] * pr->sR[k];
    double v[3];
    for (i = 0; i < 3; ++i) {
      v[i] = pr->sb[k][i];
      for (j = 0; j < pr->nvar; ++j)
        v[i] += pr->sA[k][i][j] * u[j];
    }
    phi -= log(tr * tr - d3dot(v, v));
  }
  return phi;
}

static void p_grad_hess(const cp_prob *pr, double tpar, const double u[],
                        double g[CP_NVAR_MAX],
                        double H[CP_NVAR_MAX][CP_NVAR_MAX]) {
  int a, b, i, j, k;
  int n = pr->nvar;
  for (a = 0; a < n; ++a) {
    g[a] = 0.0;
    for (b = 0; b < n; ++b)
      H[a][b] = 0.0;
  }
  g[3] = tpar;
  for (i = 0; i < pr->nlin; ++i) {
    double f = pr->ld[i], inv;
    for (j = 0; j < n; ++j)
      f += pr->lw[i][j] * u[j];
    inv = 1.0 / (-f);
    for (a = 0; a < n; ++a) {
      double wa = pr->lw[i][a];
      if (wa == 0.0)
        continue;
      g[a] += wa * inv;
      for (b = 0; b < n; ++b)
        H[a][b] += wa * pr->lw[i][b] * inv * inv;
    }
  }
  for (k = 0; k < pr->nsoc; ++k) {
    double R = pr->sR[k];
    double v[3], gs[CP_NVAR_MAX], s, tr;
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
        double ata = pr->sA[k][0][a] * pr->sA[k][0][b]
                   + pr->sA[k][1][a] * pr->sA[k][1][b]
                   + pr->sA[k][2][a] * pr->sA[k][2][b];
        H[a][b] += gs[a] * gs[b] / (s * s) + 2.0 * ata / s;
      }
    }
    H[3][3] -= 2.0 * R * R / s;
  }
}

/* solve H dx = -g, H symmetric positive definite (small ridge added) */
static int solve_spd(int n, double H[CP_NVAR_MAX][CP_NVAR_MAX],
                     const double g[CP_NVAR_MAX], double dx[CP_NVAR_MAX]) {
  double L[CP_NVAR_MAX][CP_NVAR_MAX];
  double y[CP_NVAR_MAX];
  int i, j, k;
  for (i = 0; i < n; ++i)
    for (j = 0; j < n; ++j)
      L[i][j] = H[i][j];
  for (i = 0; i < n; ++i)
    L[i][i] += 1e-12 * (fabs(L[i][i]) + 1.0);
  for (i = 0; i < n; ++i) {
    for (j = 0; j <= i; ++j) {
      double sum = L[i][j];
      for (k = 0; k < j; ++k)
        sum -= L[i][k] * L[j][k];
      if (i == j) {
        if (sum <= 0.0)
          return 0;
        L[i][i] = sqrt(sum);
      } else {
        L[i][j] = sum / L[j][j];
      }
    }
  }
  for (i = 0; i < n; ++i) {
    double sum = -g[i];
    for (k = 0; k < i; ++k)
      sum -= L[i][k] * y[k];
    y[i] = sum / L[i][i];
  }
  for (i = n - 1; i >= 0; --i) {
    double sum = y[i];
    for (k = i + 1; k < n; ++k)
      sum -= L[k][i] * dx[k];
    dx[i] = sum / L[i][i];
  }
  return 1;
}

/* damped Newton to the barrier central point; returns final decrement^2 */
static double newton_center(const cp_prob *pr, double tpar, double u[]) {
  double lam2 = 1e300;
  int it, j;
  for (it = 0; it < 100; ++it) {
    double g[CP_NVAR_MAX], H[CP_NVAR_MAX][CP_NVAR_MAX], dx[CP_NVAR_MAX];
    double un[CP_NVAR_MAX], phi0, step;
    int ok;
    p_grad_hess(pr, tpar, u, g, H);
    if (!solve_spd(pr->nvar, H, g, dx))
      return lam2;
    lam2 = 0.0;
    for (j = 0; j < pr->nvar; ++j)
      lam2 -= g[j] * dx[j];
    if (lam2 < 1e-13)
      return lam2 < 0.0 ? 0.0 : lam2;
    phi0 = p_phi(pr, tpar, u);
    step = 1.0;
    ok = 0;
    while (step > 1e-13) {
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
    memcpy(u, un, sizeof(double) * (size_t)pr->nvar);
  }
  return lam2;
}

static void solve_pair(const cp_shape *sa, const cp_shape *sb,
                       cp_result *res) {
  cp_prob pr;
  double mid[3], ca[3], cb[3], u[CP_NVAR_MAX];
  double req, mnu, tpar, alpha, dvec[3], dist;
  int i, k, tries;
  memset(&pr, 0, sizeof pr);
  pr.nvar = 4;
  for (i = 0; i < 3; ++i) {
    mid[i] = 0.5 * (sa->c[i] + sb->c[i]);
    ca[i] = sa->c[i] - mid[i];
    cb[i] = sb->c[i] - mid[i];
  }
  { /* alpha >= 0 (problem (10)); strict alpha > 0 under the barrier */
    double w[CP_NVAR_MAX];
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
      double need = pr.ld[i] / (-pr.lw[i][3]);
      if (need > req)
        req = need;
    }
  }
  for (k = 0; k < pr.nsoc; ++k) {
    double need = sqrt(d3dot(pr.sb[k], pr.sb[k])) / pr.sR[k];
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
  mnu = (double)pr.nlin + 2.0 * (double)pr.nsoc;
  tpar = 1.0;
  for (;;) {
    double lam2 = newton_center(&pr, tpar, u);
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
  dist = sqrt(d3dot(dvec, dvec)) * (1.0 - 1.0 / alpha);
  res->distance = (float)dist;
  for (i = 0; i < 3; ++i) {
    double xw = u[i] + mid[i];
    res->p1[i] = (float)(sa->c[i] + (xw - sa->c[i]) / alpha);
    res->p2[i] = (float)(sb->c[i] + (xw - sb->c[i]) / alpha);
  }
}

/* ------------------------------------------------------------------ */
size_t cp_collide_scratch_bytes(uint32_t prim_count) {
  return (size_t)prim_count * sizeof(cp_shape);
}

void cp_collide_pairs(const cp_prim *prims, uint32_t prim_count,
                      const cp_pair *pairs, uint32_t pair_count,
                      cp_result *results, void *scratch, size_t scratch_bytes) {
  cp_shape *shapes = (cp_shape *)scratch;
  uint32_t i;
  if (prim_count > 0 &&
      (scratch == NULL || scratch_bytes < (size_t)prim_count * sizeof(cp_shape))) {
    /* explicit policy: a missing/undersized caller buffer rejects the batch */
    for (i = 0; i < pair_count; ++i) {
      memset(&results[i], 0, sizeof results[i]);
      results[i].status = CP_ERR_NO_CONVERGE;
    }
    return;
  }
  for (i = 0; i < prim_count; ++i)
    build_shape(&prims[i], &shapes[i]);
  for (i = 0; i < pair_count; ++i) {
    cp_result *r = &results[i];
    uint32_t a = pairs[i].a, b = pairs[i].b;
    memset(r, 0, sizeof *r);
    if (a >= prim_count || b >= prim_count) {
      r->status = CP_ERR_BAD_INDEX;
      continue;
    }
    if (shapes[a].status != CP_OK) {
      r->status = shapes[a].status;
      continue;
    }
    if (shapes[b].status != CP_OK) {
      r->status = shapes[b].status;
      continue;
    }
    solve_pair(&shapes[a], &shapes[b], r);
  }
}
