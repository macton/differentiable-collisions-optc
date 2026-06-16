/* performance-test-optimized/gen_pairs_seed.c — alternate-seed generator for
 * the generalization check. A copy of performance-test/gen_pairs.c with the
 * PRNG seed and pair count taken from the command line and the output path
 * named by the caller, so alternate in-domain input sets can be produced
 * WITHOUT touching the committed performance-test/pairs.txt.
 *
 * With the reference seed (0x123456789ABCDEF7) and count 1000 it reproduces
 * the committed pairs.txt byte-for-byte. Generation logic is identical to
 * the reference: every draw is checked through cp_collide_pairs and rejects
 * are re-rolled, keeping the set 100% in-domain.
 *
 * Usage: perf_gen_seed <out-pairs.txt> <seed-u64> [npairs]
 */
#include "pairs_io.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

static const double PI = 3.14159265358979323846;

static uint64_t g_rng = 0x123456789ABCDEF7ULL;

static uint64_t rnd_u64(void) {
  uint64_t x = g_rng;
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  g_rng = x;
  return x * 0x2545F4914F6CDD1DULL;
}

static double rnd01(void) {
  return (double)(rnd_u64() >> 11) * (1.0 / 9007199254740992.0);
}

static double rnd_range(double lo, double hi) {
  return lo + (hi - lo) * rnd01();
}

static void rnd_rot(float r[9]) {
  double ax[3], n2, n, c, s, t, ang;
  int i, j;
  do {
    ax[0] = rnd_range(-1.0, 1.0);
    ax[1] = rnd_range(-1.0, 1.0);
    ax[2] = rnd_range(-1.0, 1.0);
    n2 = ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2];
  } while (n2 < 1e-4 || n2 > 1.0);
  n = sqrt(n2);
  for (i = 0; i < 3; ++i)
    ax[i] /= n;
  ang = rnd_range(0.0, 2.0 * PI);
  c = cos(ang);
  s = sin(ang);
  t = 1.0 - c;
  {
    double m[3][3];
    m[0][0] = c + t * ax[0] * ax[0];
    m[0][1] = t * ax[0] * ax[1] - s * ax[2];
    m[0][2] = t * ax[0] * ax[2] + s * ax[1];
    m[1][0] = t * ax[1] * ax[0] + s * ax[2];
    m[1][1] = c + t * ax[1] * ax[1];
    m[1][2] = t * ax[1] * ax[2] - s * ax[0];
    m[2][0] = t * ax[2] * ax[0] - s * ax[1];
    m[2][1] = t * ax[2] * ax[1] + s * ax[0];
    m[2][2] = c + t * ax[2] * ax[2];
    for (i = 0; i < 3; ++i)
      for (j = 0; j < 3; ++j)
        r[3 * i + j] = (float)m[i][j];
  }
}

static cp_prim rnd_prim(const double pos[3]) {
  cp_prim p;
  int i, j;
  memset(&p, 0, sizeof p);
  p.type = (uint32_t)(rnd_u64() & 3u);
  for (i = 0; i < 3; ++i)
    p.pos[i] = (float)pos[i];
  rnd_rot(p.rot);
  switch (p.type) {
  case CP_SPHERE:
    p.radius = (float)rnd_range(0.5, 4.0);
    break;
  case CP_BOX:
    for (i = 0; i < 3; ++i)
      p.half_extent[i] = (float)rnd_range(0.4, 4.0);
    break;
  case CP_CAPSULE:
    p.radius = (float)rnd_range(0.3, 2.0);
    p.length = (float)rnd_range(0.5, 6.0);
    break;
  default: { /* CP_POLYTOPE */
    int nv = 8 + (int)(rnd_u64() % 9u);
    double s = rnd_range(1.0, 4.0);
    p.vert_count = (uint32_t)nv;
    for (j = 0; j < nv; ++j)
      for (i = 0; i < 3; ++i)
        p.verts[j][i] = (float)rnd_range(-s, s);
  } break;
  }
  return p;
}

/* World axis-aligned bounding box of a primitive, [lo, hi] per axis. */
static void prim_aabb(const cp_prim *p, double lo[3], double hi[3]) {
  double R[3][3], pos[3];
  int i, j, k;
  for (i = 0; i < 3; ++i) {
    pos[i] = p->pos[i];
    for (j = 0; j < 3; ++j) R[i][j] = p->rot[3 * i + j];
  }
  if (p->type == CP_POLYTOPE) {
    for (i = 0; i < 3; ++i) { lo[i] = 1e300; hi[i] = -1e300; }
    for (k = 0; k < (int)p->vert_count; ++k)
      for (i = 0; i < 3; ++i) {
        double w = pos[i];
        for (j = 0; j < 3; ++j) w += R[i][j] * p->verts[k][j];
        if (w < lo[i]) lo[i] = w;
        if (w > hi[i]) hi[i] = w;
      }
  } else if (p->type == CP_BOX) {
    for (i = 0; i < 3; ++i) {
      double e = 0.0;
      for (j = 0; j < 3; ++j) e += fabs(R[i][j]) * p->half_extent[j];
      lo[i] = pos[i] - e; hi[i] = pos[i] + e;
    }
  } else if (p->type == CP_CAPSULE) {
    double hl = 0.5 * p->length;     /* axis = R * [1,0,0] = column 0 */
    for (i = 0; i < 3; ++i) {
      double e = fabs(R[i][0]) * hl + p->radius;
      lo[i] = pos[i] - e; hi[i] = pos[i] + e;
    }
  } else { /* CP_SPHERE */
    for (i = 0; i < 3; ++i) { lo[i] = pos[i] - p->radius; hi[i] = pos[i] + p->radius; }
  }
}

/* Do two world AABBs overlap on all three axes? Models the broadphase a caller
 * is assumed to run before invoking the library (see README "Assumptions"). */
static int aabb_overlap(const double la[3], const double ha[3],
                        const double lb[3], const double hb[3]) {
  int i;
  for (i = 0; i < 3; ++i)
    if (la[i] > hb[i] || lb[i] > ha[i]) return 0;
  return 1;
}

int main(int argc, char **argv) {
  uint32_t i, npairs, ncoll = 0, nrej = 0, nrej_aabb = 0;
  cp_prim *prims;
  cp_pair *pairs;
  uint64_t seed_val;
  char *endp;
  if (argc != 3 && argc != 4) {
    fprintf(stderr, "usage: perf_gen_seed <out-pairs.txt> <seed-u64> "
                    "[npairs]\n");
    return 2;
  }
  /* seed: full uint64 range; base 0 accepts decimal or 0x-prefixed hex.
   * Use strtoull (UNSIGNED) so values above LLONG_MAX are not clamped. */
  errno = 0;
  seed_val = (uint64_t)strtoull(argv[2], &endp, 0);
  if (endp == argv[2] || *endp != '\0' || errno == ERANGE) {
    fprintf(stderr, "error: bad seed '%s' (need a uint64, dec or 0xHEX)\n",
            argv[2]);
    return 2;
  }
  if (seed_val == 0ULL) {
    fprintf(stderr, "error: seed must be nonzero (xorshift requires it)\n");
    return 2;
  }
  g_rng = seed_val;
  npairs = 1000u;
  if (argc == 4) {
    long v = strtol(argv[3], NULL, 10);
    if (v <= 0 || v > 1000000L) {
      fprintf(stderr, "error: npairs out of range (1..1000000)\n");
      return 2;
    }
    npairs = (uint32_t)v;
  }
  prims = (cp_prim *)malloc(sizeof(cp_prim) * 2u * npairs);
  pairs = (cp_pair *)malloc(sizeof(cp_pair) * npairs);
  if (!prims || !pairs) {
    fprintf(stderr, "error: out of memory\n");
    free(prims);
    free(pairs);
    return 2;
  }
  for (i = 0; i < npairs; ++i) {
    for (;;) {
      double base[3], b2[3], dir[3], n2, mag, la[3], ha[3], lb[3], hb[3];
      cp_prim two[2];
      cp_pair pr = { 0, 1 };
      cp_result r;
      int j;
      for (j = 0; j < 3; ++j)
        base[j] = rnd_range(-7900.0, 7900.0);
      do {
        dir[0] = rnd_range(-1.0, 1.0);
        dir[1] = rnd_range(-1.0, 1.0);
        dir[2] = rnd_range(-1.0, 1.0);
        n2 = dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2];
      } while (n2 < 1e-4 || n2 > 1.0);
      n2 = sqrt(n2);
      /* Place close enough that the AABBs can overlap; the test below enforces it. */
      mag = rnd_range(0.0, 10.0);
      for (j = 0; j < 3; ++j)
        b2[j] = base[j] + mag * dir[j] / n2;
      two[0] = rnd_prim(base);
      two[1] = rnd_prim(b2);
      {
        size_t sb = cp_collide_scratch_bytes(2);
        void *ws = malloc(sb);
        cp_collide_pairs(two, 2, &pr, 1, &r, ws, sb);
        free(ws);
      }
      if (r.status != CP_OK) {
        ++nrej;
        continue; /* re-roll: keep the data set 100% in-domain */
      }
      /* Broadphase assumption (README): only overlapping-AABB pairs reach the
       * solver; reject trivially-separated pairs. */
      prim_aabb(&two[0], la, ha);
      prim_aabb(&two[1], lb, hb);
      if (!aabb_overlap(la, ha, lb, hb)) {
        ++nrej_aabb;
        continue;
      }
      prims[2 * i] = two[0];
      prims[2 * i + 1] = two[1];
      pairs[i].a = 2 * i;
      pairs[i].b = 2 * i + 1;
      if (r.colliding)
        ++ncoll;
      break;
    }
  }
  if (pairs_write_text(argv[1], prims, pairs, npairs)) {
    fprintf(stderr, "error: cannot write %s\n", argv[1]);
    free(prims);
    free(pairs);
    return 1;
  }
  printf("wrote %u pairs to %s (seed 0x%llx, %u colliding, %u out-of-domain "
         "re-rolled, %u non-overlapping-AABB re-rolled)\n",
         npairs, argv[1], (unsigned long long)seed_val, ncoll, nrej, nrej_aabb);
  free(prims);
  free(pairs);
  return 0;
}
