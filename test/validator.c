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
 * No code is shared with src/collide.c. Double precision throughout.
 */
#include "validator.h"

#include <math.h>
#include <string.h>

typedef struct {
  int    type;
  double c[3];     /* scaling center, world */
  double r[3];     /* body origin, world    */
  double Q[3][3];  /* world-from-body       */
  double R, hl, h[3];
  int    nv;
  double w[CP_MAX_POLY_VERTS][3]; /* polytope verts, world */
} vshape;

static double vdot(const double a[3], const double b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vsub(const double a[3], const double b[3], double o[3]) {
  o[0] = a[0] - b[0];
  o[1] = a[1] - b[1];
  o[2] = a[2] - b[2];
}

static void vcrs(const double a[3], const double b[3], double o[3]) {
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
  s->hl = 0.5 * (double)p->length;
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
      double acc = 0.0;
      for (j = 0; j < s->nv; ++j)
        acc += s->w[j][i];
      s->c[i] = acc / (double)s->nv;
    }
  } else {
    for (i = 0; i < 3; ++i)
      s->c[i] = s->r[i];
  }
}

/* support point of the UNSCALED shape in world direction d */
static void sup_base(const vshape *s, const double d[3], double out[3]) {
  int i;
  switch (s->type) {
  case CP_SPHERE: {
    double n = sqrt(vdot(d, d));
    if (n < 1e-300) {
      out[0] = s->r[0] + s->R;
      out[1] = s->r[1];
      out[2] = s->r[2];
      return;
    }
    for (i = 0; i < 3; ++i)
      out[i] = s->r[i] + s->R * d[i] / n;
  } break;
  case CP_BOX: {
    double dl[3];
    for (i = 0; i < 3; ++i) /* d in body frame: Q^T d */
      dl[i] = s->Q[0][i] * d[0] + s->Q[1][i] * d[1] + s->Q[2][i] * d[2];
    for (i = 0; i < 3; ++i)
      dl[i] = (dl[i] >= 0.0) ? s->h[i] : -s->h[i];
    for (i = 0; i < 3; ++i)
      out[i] = s->r[i] + s->Q[i][0] * dl[0] + s->Q[i][1] * dl[1]
                       + s->Q[i][2] * dl[2];
  } break;
  case CP_CAPSULE: {
    double ax[3], t, n;
    for (i = 0; i < 3; ++i)
      ax[i] = s->Q[i][0];
    t = (vdot(ax, d) >= 0.0) ? s->hl : -s->hl;
    n = sqrt(vdot(d, d));
    for (i = 0; i < 3; ++i) {
      out[i] = s->r[i] + t * ax[i];
      if (n >= 1e-300)
        out[i] += s->R * d[i] / n;
    }
  } break;
  default: { /* CP_POLYTOPE: raw vertices */
    int best = 0, j;
    double bd = -1e300;
    for (j = 0; j < s->nv; ++j) {
      double dd = vdot(s->w[j], d);
      if (dd > bd) {
        bd = dd;
        best = j;
      }
    }
    for (i = 0; i < 3; ++i)
      out[i] = s->w[best][i];
  } break;
  }
}

/* support of S(alpha) = c + alpha*(S - c) */
static void sup_scaled(const vshape *s, double alpha, const double d[3],
                       double out[3]) {
  double b[3];
  int i;
  sup_base(s, d, b);
  for (i = 0; i < 3; ++i)
    out[i] = s->c[i] + alpha * (b[i] - s->c[i]);
}

/* ---- closest point to origin on simplex, with reduction (Ericson) ---- */

static void closest_seg(const double a[3], const double b[3], double t01[1]) {
  double ab[3], t, dd;
  vsub(b, a, ab);
  dd = vdot(ab, ab);
  if (dd < 1e-300) {
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
static int closest_tri(const double a[3], const double b[3],
                       const double c[3], double bary[3]) {
  double ab[3], ac[3], bp[3], cp_[3];
  double d1, d2, d3, d4, d5, d6, va, vb, vc, denom, v, w;
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
  double p[4][3]; /* Minkowski-difference points */
} simplex;

/* reduce simplex to the minimal face supporting the closest point; write
 * closest point to v. Returns 1 if origin is enclosed (distance 0). */
static int simplex_closest(simplex *sx, double v[3]) {
  int i;
  if (sx->n == 1) {
    memcpy(v, sx->p[0], sizeof(double) * 3);
    return 0;
  }
  if (sx->n == 2) {
    double t;
    closest_seg(sx->p[0], sx->p[1], &t);
    for (i = 0; i < 3; ++i)
      v[i] = sx->p[0][i] + t * (sx->p[1][i] - sx->p[0][i]);
    if (t <= 0.0) {
      sx->n = 1;
    } else if (t >= 1.0) {
      memcpy(sx->p[0], sx->p[1], sizeof sx->p[0]);
      sx->n = 1;
    }
    return 0;
  }
  if (sx->n == 3) {
    double bary[3], q[3][3];
    int mask = closest_tri(sx->p[0], sx->p[1], sx->p[2], bary);
    int k = 0, j;
    for (i = 0; i < 3; ++i)
      v[i] = bary[0] * sx->p[0][i] + bary[1] * sx->p[1][i]
           + bary[2] * sx->p[2][i];
    for (j = 0; j < 3; ++j) {
      if (mask & (1 << j)) {
        memcpy(q[k], sx->p[j], sizeof q[k]);
        ++k;
      }
    }
    memcpy(sx->p, q, sizeof(double) * 3 * (size_t)k);
    sx->n = k;
    return 0;
  }
  { /* tetrahedron */
    double bestv[3] = {0, 0, 0};
    double bestd = 1e300;
    double keep[3][3];
    double mx = 0.0;
    int bestk = 0, found_outside = 0, f, j;
    static const int faces[4][3] = {
      {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}
    };
    for (f = 0; f < 4; ++f)
      for (j = 0; j < 3; ++j)
        if (fabs(sx->p[f][j]) > mx)
          mx = fabs(sx->p[f][j]);
    for (f = 0; f < 4; ++f) {
      const double *a = sx->p[faces[f][0]];
      const double *b = sx->p[faces[f][1]];
      const double *c = sx->p[faces[f][2]];
      const double *d = sx->p[6 - faces[f][0] - faces[f][1] - faces[f][2]];
      double ab[3], ac[3], nrm[3], ad[3], so, sd, nlen, outside;
      vsub(b, a, ab);
      vsub(c, a, ac);
      vcrs(ab, ac, nrm);
      vsub(d, a, ad);
      nlen = sqrt(vdot(nrm, nrm));
      so = -vdot(nrm, a);      /* origin side  */
      sd = vdot(nrm, ad);      /* 4th-pt side  */
      /* degenerate (flat) tetra: cannot trust the enclosure test; treat
       * the face as a candidate instead of skipping it */
      if (fabs(sd) <= 1e-9 * nlen * mx + 1e-30) {
        outside = 1.0;
      } else if (so * sd < 0.0) {
        outside = 1.0;
      } else {
        outside = 0.0;
      }
      if (outside != 0.0) {
        double bary[3], cv[3], dd;
        int mask = closest_tri(a, b, c, bary), k = 0;
        found_outside = 1;
        for (j = 0; j < 3; ++j)
          cv[j] = bary[0] * a[j] + bary[1] * b[j] + bary[2] * c[j];
        dd = vdot(cv, cv);
        if (dd < bestd) {
          bestd = dd;
          memcpy(bestv, cv, sizeof bestv);
          for (j = 0; j < 3; ++j) {
            if (mask & (1 << j)) {
              memcpy(keep[k], sx->p[faces[f][j]], sizeof keep[k]);
              ++k;
            }
          }
          bestk = k;
        }
      }
    }
    if (!found_outside)
      return 1;
    memcpy(sx->p, keep, sizeof(double) * 3 * (size_t)bestk);
    sx->n = bestk;
    memcpy(v, bestv, sizeof(double) * 3);
    return 0;
  }
}

/* distance between S_a(alpha) and S_b(alpha); 0 means intersecting */
static double gjk_dist(const vshape *sa, const vshape *sb, double alpha) {
  simplex sx;
  double v[3], d0[3] = {1.0, 0.0, 0.0};
  double pa[3], pb[3];
  int it, i;
  sup_scaled(sa, alpha, d0, pa);
  d0[0] = -1.0;
  sup_scaled(sb, alpha, d0, pb);
  vsub(pa, pb, sx.p[0]);
  sx.n = 1;
  for (it = 0; it < 256; ++it) {
    double w[3], nd[3], vv, vw;
    if (simplex_closest(&sx, v))
      return 0.0;
    vv = vdot(v, v);
    if (vv < 1e-22)
      return 0.0;
    for (i = 0; i < 3; ++i)
      nd[i] = -v[i];
    sup_scaled(sa, alpha, nd, pa);
    for (i = 0; i < 3; ++i)
      nd[i] = v[i];
    sup_scaled(sb, alpha, nd, pb);
    vsub(pa, pb, w);
    vw = vdot(v, w);
    if (vv - vw <= 1e-12 * vv + 1e-24)
      return sqrt(vv);
    /* duplicate support point => no progress: converged */
    {
      int dup = 0, j;
      for (j = 0; j < sx.n; ++j) {
        double dx = w[0] - sx.p[j][0], dy = w[1] - sx.p[j][1],
               dz = w[2] - sx.p[j][2];
        if (dx * dx + dy * dy + dz * dz < 1e-24) {
          dup = 1;
          break;
        }
      }
      if (dup)
        return sqrt(vv);
    }
    memcpy(sx.p[sx.n], w, sizeof(double) * 3);
    ++sx.n;
  }
  return sqrt(vdot(v, v));
}

int val_collide(const cp_prim *a, const cp_prim *b,
                double *alpha_out, double *dist_out) {
  vshape sa, sb;
  double lo, hi, cd[3], alpha;
  int i;
  make_vshape(a, &sa);
  make_vshape(b, &sb);
  for (i = 0; i < 3; ++i)
    cd[i] = sa.c[i] - sb.c[i];

  lo = 1e-9;
  if (gjk_dist(&sa, &sb, lo) <= 0.0) {
    /* scaling centers (effectively) coincident: alpha -> 0 */
    *alpha_out = 0.0;
    *dist_out = 0.0;
    return 0;
  }
  hi = 1.0;
  i = 0;
  while (gjk_dist(&sa, &sb, hi) > 0.0) {
    hi *= 2.0;
    if (++i > 60)
      return 1; /* no intersection found at huge alpha: invalid input */
  }
  if (hi > 1.0)
    lo = hi * 0.5;
  for (i = 0; i < 100; ++i) {
    double mid = 0.5 * (lo + hi);
    if (gjk_dist(&sa, &sb, mid) > 0.0)
      lo = mid;
    else
      hi = mid;
    if (hi - lo <= 1e-12 * hi)
      break;
  }
  alpha = 0.5 * (lo + hi);
  *alpha_out = alpha;
  *dist_out = sqrt(vdot(cd, cd)) * (1.0 - 1.0 / alpha);
  return 0;
}
