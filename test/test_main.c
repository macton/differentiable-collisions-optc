/* test/test_main.c — correctness tests.
 *
 * Coverage (per task spec):
 *  - analytic cases for every primitive pair type: separated, touching,
 *    deeply penetrating (sphere, box, capsule, polytope);
 *  - domain-edge cases at +/-8 km verifying 1 mm correctness;
 *  - explicit rejection of out-of-range coordinates and out-of-range
 *    primitive sizes (extent < 0.1 m or > 250 m);
 *  - every CP_OK case is also checked by the independent validator
 *    (test/validator.c) and the two compared within 1 mm;
 *  - batch path == singular path (bit-exact) and determinism (two runs of
 *    the same batch are bit-identical).
 *
 * Note on "distance": both paths report |c1-c2| * (1 - 1/alpha), the signed
 * gap between the paper's eq.-24 contact points (see src/README.md). For
 * the center-aligned analytic cases below this equals the Euclidean gap.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/collide.h"
#include "validator.h"

#define MAX_CASES 64

static const double PI = 3.14159265358979323846;

/* cp_collide_pairs with a transient caller-provided scratch buffer: the library
 * allocates nothing, so the tests supply the working memory it needs. */
static void tcollide(const cp_prim *prims, uint32_t prim_count,
                     const cp_pair *pairs, uint32_t pair_count,
                     cp_result *results) {
  size_t scratch_bytes = cp_collide_scratch_bytes(prim_count);
  void *scratch = scratch_bytes ? malloc(scratch_bytes) : NULL;
  cp_collide_pairs(prims, prim_count, pairs, pair_count, results,
                   scratch, scratch_bytes);
  free(scratch);
}

static int g_pass = 0, g_fail = 0;

static cp_prim   g_prims[2 * MAX_CASES];
static cp_pair   g_pairs[MAX_CASES];
static cp_result g_single[MAX_CASES];
static int       g_ncase = 0;

static void check(int cond, const char *name, const char *what) {
  if (cond) {
    ++g_pass;
    printf("PASS %-28s %s\n", name, what);
  } else {
    ++g_fail;
    printf("FAIL %-28s %s\n", name, what);
  }
}

static void rot_ident(float r[9]) {
  memset(r, 0, 9 * sizeof(float));
  r[0] = r[4] = r[8] = 1.0f;
}

static void rot_z(float r[9], double ang) {
  double c = cos(ang), s = sin(ang);
  r[0] = (float)c;  r[1] = (float)-s; r[2] = 0.0f;
  r[3] = (float)s;  r[4] = (float)c;  r[5] = 0.0f;
  r[6] = 0.0f;      r[7] = 0.0f;      r[8] = 1.0f;
}

static cp_prim mk_sphere(double x, double y, double z, double R) {
  cp_prim p;
  memset(&p, 0, sizeof p);
  p.type = CP_SPHERE;
  p.radius = (float)R;
  p.pos[0] = (float)x; p.pos[1] = (float)y; p.pos[2] = (float)z;
  rot_ident(p.rot);
  return p;
}

static cp_prim mk_box(double x, double y, double z,
                      double hx, double hy, double hz) {
  cp_prim p;
  memset(&p, 0, sizeof p);
  p.type = CP_BOX;
  p.half_extent[0] = (float)hx;
  p.half_extent[1] = (float)hy;
  p.half_extent[2] = (float)hz;
  p.pos[0] = (float)x; p.pos[1] = (float)y; p.pos[2] = (float)z;
  rot_ident(p.rot);
  return p;
}

static cp_prim mk_capsule(double x, double y, double z, double R, double L) {
  cp_prim p;
  memset(&p, 0, sizeof p);
  p.type = CP_CAPSULE;
  p.radius = (float)R;
  p.length = (float)L;
  p.pos[0] = (float)x; p.pos[1] = (float)y; p.pos[2] = (float)z;
  rot_ident(p.rot);
  return p;
}

static cp_prim mk_cube_poly(double x, double y, double z, double h) {
  cp_prim p;
  int i;
  memset(&p, 0, sizeof p);
  p.type = CP_POLYTOPE;
  p.vert_count = 8;
  p.pos[0] = (float)x; p.pos[1] = (float)y; p.pos[2] = (float)z;
  rot_ident(p.rot);
  for (i = 0; i < 8; ++i) {
    p.verts[i][0] = (float)((i & 1) ? h : -h);
    p.verts[i][1] = (float)((i & 2) ? h : -h);
    p.verts[i][2] = (float)((i & 4) ? h : -h);
  }
  return p;
}

static cp_prim mk_tetra(double x, double y, double z, double s) {
  cp_prim p;
  memset(&p, 0, sizeof p);
  p.type = CP_POLYTOPE;
  p.vert_count = 4;
  p.pos[0] = (float)x; p.pos[1] = (float)y; p.pos[2] = (float)z;
  rot_ident(p.rot);
  p.verts[0][0] = (float)s;  p.verts[0][1] = (float)s;  p.verts[0][2] = (float)s;
  p.verts[1][0] = (float)s;  p.verts[1][1] = (float)-s; p.verts[1][2] = (float)-s;
  p.verts[2][0] = (float)-s; p.verts[2][1] = (float)s;  p.verts[2][2] = (float)-s;
  p.verts[3][0] = (float)-s; p.verts[3][1] = (float)-s; p.verts[3][2] = (float)s;
  return p;
}

/* run pair both through primary (single batch) and validator; compare to
 * analytic values when given (NAN skips that comparison). */
static void case_ok(const char *name, cp_prim a, cp_prim b,
                    double exp_alpha, double exp_dist) {
  cp_result r;
  cp_prim prims[2];
  cp_pair pair = { 0, 1 };
  double va = 0.0, vd = 0.0;
  char buf[160];
  prims[0] = a;
  prims[1] = b;
  tcollide(prims, 2, &pair, 1, &r);
  check(r.status == CP_OK, name, "status CP_OK");
  if (r.status != CP_OK)
    return;
  if (exp_dist == exp_dist) {
    snprintf(buf, sizeof buf, "distance %.6f vs analytic %.6f",
             (double)r.distance, exp_dist);
    check(fabs((double)r.distance - exp_dist) <= 1e-3, name, buf);
  }
  if (exp_alpha == exp_alpha) {
    snprintf(buf, sizeof buf, "alpha %.6f vs analytic %.6f",
             (double)r.alpha, exp_alpha);
    check(fabs((double)r.alpha - exp_alpha) <= 1e-3, name, buf);
  }
  if (fabs((double)r.distance) > 1e-3) {
    check(r.colliding == (r.distance < 0.0f ? 1u : 0u), name,
          "colliding flag matches distance sign");
  }
  check(val_collide(&a, &b, &va, &vd) == 0, name, "validator ran");
  snprintf(buf, sizeof buf, "validator dist %.6f vs primary %.6f",
           vd, (double)r.distance);
  check(fabs(vd - (double)r.distance) <= 1e-3, name, buf);
  if (g_ncase < MAX_CASES) {
    g_prims[2 * g_ncase] = a;
    g_prims[2 * g_ncase + 1] = b;
    g_pairs[g_ncase].a = (uint32_t)(2 * g_ncase);
    g_pairs[g_ncase].b = (uint32_t)(2 * g_ncase + 1);
    g_single[g_ncase] = r;
    ++g_ncase;
  }
}

static void case_reject(const char *name, cp_prim a, cp_prim b,
                        uint32_t want) {
  cp_result r;
  cp_prim prims[2];
  cp_pair pair = { 0, 1 };
  char buf[64];
  prims[0] = a;
  prims[1] = b;
  tcollide(prims, 2, &pair, 1, &r);
  snprintf(buf, sizeof buf, "status %u (want %u)", r.status, want);
  check(r.status == want, name, buf);
}

int main(void) {
  const double SQRT2 = sqrt(2.0);

  /* ---- analytic cases: every pair family, sep/touch/deep-pen ---- */
  case_ok("sphere-sphere-sep",
          mk_sphere(0, 0, 0, 1), mk_sphere(3, 0, 0, 1), 1.5, 1.0);
  case_ok("sphere-sphere-touch",
          mk_sphere(0, 0, 0, 1), mk_sphere(2, 0, 0, 1), 1.0, 0.0);
  case_ok("sphere-sphere-deep",
          mk_sphere(0, 0, 0, 1), mk_sphere(0.5, 0, 0, 1), 0.25, -1.5);
  case_ok("box-box-sep",
          mk_box(0, 0, 0, 1, 1, 1), mk_box(4, 0, 0, 1, 1, 1), 2.0, 2.0);
  case_ok("box-box-touch",
          mk_box(0, 0, 0, 1, 1, 1), mk_box(2, 0, 0, 1, 1, 1), 1.0, 0.0);
  case_ok("box-box-deep",
          mk_box(0, 0, 0, 1, 1, 1), mk_box(0.2, 0, 0, 1, 1, 1), 0.1, -1.8);
  case_ok("sphere-box-sep",
          mk_box(0, 0, 0, 1, 1, 1), mk_sphere(3, 0, 0, 0.5),
          2.0, 1.5);
  case_ok("sphere-box-touch",
          mk_box(0, 0, 0, 1, 1, 1), mk_sphere(1.5, 0, 0, 0.5), 1.0, 0.0);
  case_ok("sphere-box-pen",
          mk_box(0, 0, 0, 1, 1, 1), mk_sphere(1.2, 0, 0, 0.5),
          0.8, 1.2 * (1.0 - 1.0 / 0.8));
  case_ok("capsule-capsule-sep",
          mk_capsule(0, 0, 0, 0.5, 2), mk_capsule(0, 3, 0, 0.5, 2),
          3.0, 2.0);
  case_ok("capsule-capsule-touch",
          mk_capsule(0, 0, 0, 0.5, 2), mk_capsule(0, 1, 0, 0.5, 2),
          1.0, 0.0);
  case_ok("capsule-capsule-deep",
          mk_capsule(0, 0, 0, 0.5, 2), mk_capsule(0, 0.5, 0, 0.5, 2),
          0.5, -0.5);
  case_ok("sphere-capsule-sep",
          mk_capsule(0, 0, 0, 0.5, 2), mk_sphere(0, 3, 0, 1), 2.0, 1.5);
  case_ok("box-capsule-sep",
          mk_box(0, 0, 0, 1, 1, 1), mk_capsule(0, 4, 0, 0.5, 2),
          8.0 / 3.0, 2.5);
  case_ok("poly-poly-sep",
          mk_cube_poly(0, 0, 0, 1), mk_cube_poly(4, 0, 0, 1), 2.0, 2.0);
  case_ok("poly-poly-deep",
          mk_cube_poly(0, 0, 0, 1), mk_cube_poly(1, 0, 0, 1), 0.5, -1.0);
  case_ok("sphere-poly-sep",
          mk_cube_poly(0, 0, 0, 1), mk_sphere(4, 0, 0, 1), 2.0, 2.0);
  case_ok("box-poly-sep",
          mk_cube_poly(0, 0, 0, 1), mk_box(3, 0, 0, 1, 1, 1), 1.5, 1.0);
  case_ok("capsule-poly-sep",
          mk_cube_poly(0, 0, 0, 1), mk_capsule(0, 3, 0, 0.5, 2), 2.0, 1.5);

  /* rotated cases */
  {
    cp_prim box = mk_box(0, 0, 0, 1, 1, 1);
    rot_z(box.rot, PI / 4.0);
    case_ok("rot45box-sphere",
            box, mk_sphere(4, 0, 0, 1),
            4.0 / (1.0 + SQRT2), 3.0 - SQRT2);
  }
  {
    cp_prim cap = mk_capsule(3, 0, 0, 0.5, 2);
    rot_z(cap.rot, PI / 2.0); /* axis becomes world y */
    case_ok("rotcapsule-sphere",
            mk_sphere(0, 0, 0, 1), cap, 2.0, 1.5);
  }

  /* validator-only case (no simple analytic value) */
  case_ok("tetra-sphere", mk_tetra(0, 0, 0, 1), mk_sphere(4, 0.3, 0.2, 1),
          NAN, NAN);

  /* coincident scaling centers: explicit documented policy */
  case_ok("coincident-centers",
          mk_sphere(0, 0, 0, 1), mk_sphere(0, 0, 0, 0.5), NAN, 0.0);

  /* ---- domain edge (+/- 8 km), 1 mm correctness ---- */
  case_ok("edge+x-spheres",
          mk_sphere(8190, 0, 0, 1), mk_sphere(8187, 0, 0, 1), 1.5, 1.0);
  case_ok("edge-x-spheres",
          mk_sphere(-8190, 0, 0, 1), mk_sphere(-8187, 0, 0, 1), 1.5, 1.0);
  case_ok("edge-corner-spheres",
          mk_sphere(8190, 8190, 8190, 2), mk_sphere(8186, 8190, 8190, 1),
          4.0 / 3.0, 1.0);
  case_ok("edge-boxes",
          mk_box(8191, 0, 0, 1, 1, 1), mk_box(8188, 0, 0, 1, 1, 1),
          1.5, 1.0);

  /* ---- size-range boundaries (inclusive) ---- */
  case_ok("min-extent-spheres",
          mk_sphere(0, 0, 0, 0.05), mk_sphere(0.3, 0, 0, 0.05), 3.0, 0.2);
  case_ok("max-extent-box-sphere",
          mk_box(0, 0, 0, 125, 125, 125), mk_sphere(130, 0, 0, 1),
          130.0 / 126.0, 4.0);

  /* ---- explicit rejection ---- */
  case_reject("coord-over+x",
              mk_sphere(8191.5, 0, 0, 1), mk_sphere(0, 0, 0, 1),
              CP_ERR_COORD_RANGE);
  case_reject("coord-over-x",
              mk_sphere(-8191.5, 0, 0, 1), mk_sphere(0, 0, 0, 1),
              CP_ERR_COORD_RANGE);
  case_reject("coord-over-y-box",
              mk_box(0, 8191.5, 0, 1, 1, 1), mk_sphere(0, 0, 0, 1),
              CP_ERR_COORD_RANGE);
  case_reject("size-under-sphere",
              mk_sphere(0, 0, 0, 0.04), mk_sphere(3, 0, 0, 1),
              CP_ERR_SIZE_RANGE);
  case_reject("size-over-box",
              mk_box(0, 0, 0, 130, 1, 1), mk_sphere(300, 0, 0, 1),
              CP_ERR_SIZE_RANGE);
  case_reject("size-under-capsule",
              mk_capsule(0, 0, 0, 0.04, 0.5), mk_sphere(3, 0, 0, 1),
              CP_ERR_SIZE_RANGE);
  {
    cp_prim bad = mk_sphere(0, 0, 0, 1);
    memset(bad.rot, 0, sizeof bad.rot);
    case_reject("bad-rotation", bad, mk_sphere(3, 0, 0, 1),
                CP_ERR_BAD_PRIM);
  }
  {
    cp_prim bad = mk_tetra(0, 0, 0, 1);
    bad.vert_count = 3;
    case_reject("bad-polytope-nverts", bad, mk_sphere(3, 0, 0, 1),
                CP_ERR_BAD_PRIM);
  }
  { /* bad pair index */
    cp_prim prims[2];
    cp_pair pair = { 0, 5 };
    cp_result r;
    prims[0] = mk_sphere(0, 0, 0, 1);
    prims[1] = mk_sphere(3, 0, 0, 1);
    tcollide(prims, 2, &pair, 1, &r);
    check(r.status == CP_ERR_BAD_INDEX, "bad-pair-index", "status");
  }
  { /* per-pair status isolation in a mixed batch */
    cp_prim prims[3];
    cp_pair pairs[2];
    cp_result rr[2];
    prims[0] = mk_sphere(0, 0, 0, 1);
    prims[1] = mk_sphere(3, 0, 0, 1);
    prims[2] = mk_sphere(9000, 0, 0, 1);
    pairs[0].a = 0; pairs[0].b = 1;
    pairs[1].a = 0; pairs[1].b = 2;
    tcollide(prims, 3, pairs, 2, rr);
    check(rr[0].status == CP_OK && rr[1].status == CP_ERR_COORD_RANGE,
          "mixed-batch", "good pair unaffected by bad neighbor");
  }

  /* ---- batch == singular, and determinism ---- */
  {
    cp_result batch[MAX_CASES], batch2[MAX_CASES];
    int i, same = 1, same2 = 1;
    tcollide(g_prims, (uint32_t)(2 * g_ncase),
                     g_pairs, (uint32_t)g_ncase, batch);
    for (i = 0; i < g_ncase; ++i)
      if (memcmp(&batch[i], &g_single[i], sizeof(cp_result)) != 0)
        same = 0;
    check(same, "batch-vs-single", "bit-identical results");
    tcollide(g_prims, (uint32_t)(2 * g_ncase),
                     g_pairs, (uint32_t)g_ncase, batch2);
    for (i = 0; i < g_ncase; ++i)
      if (memcmp(&batch[i], &batch2[i], sizeof(cp_result)) != 0)
        same2 = 0;
    check(same2, "determinism", "two runs bit-identical");
  }

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
