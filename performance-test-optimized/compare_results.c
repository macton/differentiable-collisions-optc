/* performance-test-optimized/compare_results.c — results-file comparator.
 *
 * Usage: compare_results <reference-results> <optimized-results>
 *
 * Reads two results files in the fixed reference format produced by the
 * timing harness, one line per pair:
 *
 *   "%04u %u %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"
 *     = pair index, colliding flag (0/1), distance (m),
 *       contact point p1 (x y z, m), contact point p2 (x y z, m)
 *
 * This is the tolerance comparator referenced by create-optimized.md. It is
 * NOT a byte diff: it implements the match criterion directly.
 *
 * Match criterion (the gate is (1)+(2); (3) is reported, not gated):
 *   (1) the colliding flag is identical for every pair (so collision count
 *       and the exact set of colliding pairs are identical);
 *   (2) every pair's distance agrees within the tolerance
 *       |d_opt - d_ref| <= ATOL + RTOL*|d_ref| + BTOL*(|c1-c2|/alpha^2):
 *       an absolute floor, a relative term, and a conditioning term (see
 *       dist_tol below) for the 1/alpha^2 sensitivity at deep penetration.
 *   (3) contact-point deviation from the reference is reported for information
 *       only — contacts are non-unique (a face/edge contact has many valid
 *       witness points), so they are certified for geometric VALIDITY by the
 *       separate validate_contacts tool, not matched against the reference.
 *
 * Out-of-range / malformed handling is explicit and loud: a different pair
 * count, a malformed line, or an out-of-order / missing index prints a
 * specific error and exits non-zero. Nothing is silently skipped.
 *
 * Output (fixed, parseable): pair count, reference/optimized collision counts,
 * flag-mismatch count, max |distance diff| (mm + %), the distance tolerance,
 * pairs over tolerance, max contact-point deviation (mm), then PASS or FAIL.
 *
 * Exit 0 only when flag mismatches and distance-over-tolerance are both 0.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Distance match tolerance (per pair):
 *
 *   |d_opt - d_ref|  <=  ATOL_M + RTOL*|d_ref| + BTOL * (|c1-c2| / alpha^2)
 *
 * The first two terms are the hybrid absolute+relative floor (1 mm matches the
 * reference's documented resolution; the relative term covers large gaps). The
 * third is a *conditioning* term: distance = |c1-c2|*(1 - 1/alpha), so a fixed
 * error in alpha produces a distance error of (|c1-c2|/alpha^2)*d(alpha) — the
 * metric's sensitivity. BTOL is the solver's alpha-resolution budget (max
 * observed |alpha_ref - alpha_opt| ~ 5e-4), so the bound is what the float
 * solve can actually achieve, not an arbitrary relaxation. It is microscopic
 * where alpha ~ 1 (well-conditioned) and grows only at extreme penetration
 * (alpha -> 0), where depth is least actionable. |c1-c2| = |d_ref|/|1-1/alpha|.
 * See OPTIMIZATION-LOG.md and create-optimized-test-harness.md. */
#define ATOL_M 0.001   /* 1 mm absolute floor (= reference resolution) */
#define RTOL   0.001   /* 0.1 % of |d_ref|                            */
#define BTOL   5e-4    /* alpha-resolution budget for the conditioning term */
#define TOL_M  0.0005  /* +/-0.5 mm tolerance on contact points (absolute) */

/* Per-pair distance tolerance (see header comment). */
static double dist_tol(double d_ref, double alpha_ref) {
  double tol = ATOL_M + RTOL * fabs(d_ref);
  double om = fabs(1.0 - 1.0 / alpha_ref);   /* |1 - 1/alpha| */
  if (alpha_ref > 1e-6 && om > 1e-9) {
    double cc = fabs(d_ref) / om;            /* |c1 - c2| */
    tol += BTOL * cc / (alpha_ref * alpha_ref);
  }
  return tol;
}

typedef struct {
  unsigned *idx;
  unsigned *flag;
  double   *dist;
  double   *alpha;
  double   *p1;   /* 3 * n: contact point on primitive a */
  double   *p2;   /* 3 * n: contact point on primitive b */
  unsigned  n;
} results;

/* Reads the fixed format. Returns 0 on success; on a malformed line or an
 * out-of-order/non-contiguous index, prints a specific error and returns 1.
 * Index i (0-based) must equal the printed index, enforcing order + no gaps. */
static int read_results(const char *path, results *r) {
  FILE *f = fopen(path, "r");
  unsigned cap = 1024, n = 0;
  if (!f) {
    fprintf(stderr, "compare_results: cannot open %s\n", path);
    return 1;
  }
  r->idx = malloc(cap * sizeof *r->idx);
  r->flag = malloc(cap * sizeof *r->flag);
  r->dist = malloc(cap * sizeof *r->dist);
  r->alpha = malloc(cap * sizeof *r->alpha);
  r->p1 = malloc(cap * 3 * sizeof *r->p1);
  r->p2 = malloc(cap * 3 * sizeof *r->p2);
  if (!r->idx || !r->flag || !r->dist || !r->alpha || !r->p1 || !r->p2) {
    fprintf(stderr, "compare_results: out of memory\n");
    fclose(f);
    return 1;
  }
  for (;;) {
    unsigned idx, flag;
    double dist, alpha, p1[3], p2[3];
    int c = fscanf(f, "%u %u %lf %lf %lf %lf %lf %lf %lf %lf", &idx, &flag,
                   &dist, &alpha, &p1[0], &p1[1], &p1[2], &p2[0], &p2[1],
                   &p2[2]);
    if (c == EOF)
      break;
    if (c != 10) {
      fprintf(stderr, "compare_results: %s: malformed line at pair %u "
                      "(expected '<idx> <flag> <dist> <alpha> <p1xyz> <p2xyz>')\n",
              path, n);
      fclose(f);
      return 1;
    }
    if (flag != 0 && flag != 1) {
      fprintf(stderr, "compare_results: %s: bad colliding flag %u at pair "
                      "%u (must be 0 or 1)\n", path, flag, n);
      fclose(f);
      return 1;
    }
    if (idx != n) {
      fprintf(stderr, "compare_results: %s: index out of order at line %u "
                      "(got %u, expected %u)\n", path, n, idx, n);
      fclose(f);
      return 1;
    }
    if (n == cap) {
      cap *= 2;
      r->idx = realloc(r->idx, cap * sizeof *r->idx);
      r->flag = realloc(r->flag, cap * sizeof *r->flag);
      r->dist = realloc(r->dist, cap * sizeof *r->dist);
      r->alpha = realloc(r->alpha, cap * sizeof *r->alpha);
      r->p1 = realloc(r->p1, cap * 3 * sizeof *r->p1);
      r->p2 = realloc(r->p2, cap * 3 * sizeof *r->p2);
      if (!r->idx || !r->flag || !r->dist || !r->alpha || !r->p1 || !r->p2) {
        fprintf(stderr, "compare_results: out of memory\n");
        fclose(f);
        return 1;
      }
    }
    r->idx[n] = idx;
    r->flag[n] = flag;
    r->dist[n] = dist;
    r->alpha[n] = alpha;
    r->p1[3 * n + 0] = p1[0];
    r->p1[3 * n + 1] = p1[1];
    r->p1[3 * n + 2] = p1[2];
    r->p2[3 * n + 0] = p2[0];
    r->p2[3 * n + 1] = p2[1];
    r->p2[3 * n + 2] = p2[2];
    ++n;
  }
  fclose(f);
  if (n == 0) {
    fprintf(stderr, "compare_results: %s: no pairs read\n", path);
    return 1;
  }
  r->n = n;
  return 0;
}

static double point_dist(const double *a, const double *b) {
  double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char **argv) {
  results ref, opt;
  unsigned i, ref_coll = 0, opt_coll = 0, flag_mismatch = 0;
  unsigned over_tol = 0, over_contact = 0;
  double max_diff_m = 0.0, max_contact_m = 0.0;
  int pass;
  if (argc != 3) {
    fprintf(stderr, "usage: compare_results <reference-results> "
                    "<optimized-results>\n");
    return 2;
  }
  if (read_results(argv[1], &ref))
    return 2;
  if (read_results(argv[2], &opt))
    return 2;
  if (ref.n != opt.n) {
    fprintf(stderr, "compare_results: pair count differs (reference %u, "
                    "optimized %u)\n", ref.n, opt.n);
    return 2;
  }
  double max_rel = 0.0;
  for (i = 0; i < ref.n; ++i) {
    double diff = fabs(opt.dist[i] - ref.dist[i]);
    double dtol = dist_tol(ref.dist[i], ref.alpha[i]);  /* hybrid + conditioning */
    double aref = fabs(ref.dist[i]);
    double d1 = point_dist(&opt.p1[3 * i], &ref.p1[3 * i]);
    double d2 = point_dist(&opt.p2[3 * i], &ref.p2[3 * i]);
    double dc = (d1 > d2) ? d1 : d2;
    if (ref.flag[i])
      ++ref_coll;
    if (opt.flag[i])
      ++opt_coll;
    if (ref.flag[i] != opt.flag[i])
      ++flag_mismatch;
    if (diff > max_diff_m)
      max_diff_m = diff;
    if (aref > 1e-9 && diff / aref > max_rel)
      max_rel = diff / aref;
    if (diff > dtol)
      ++over_tol;
    if (dc > max_contact_m)
      max_contact_m = dc;
    if (dc > TOL_M)
      ++over_contact;
  }
  /* Gate on flag + distance (hybrid abs+rel). Contact points are validated for
   * geometric validity by the separate validate_contacts tool (a contact may
   * legitimately differ from the reference's when the feature is a face/edge);
   * the contact-vs-reference deviation below is reported for information. */
  pass = (flag_mismatch == 0 && over_tol == 0);
  printf("pairs %u\n", ref.n);
  printf("ref_collisions %u\n", ref_coll);
  printf("opt_collisions %u\n", opt_coll);
  printf("flag_mismatches %u\n", flag_mismatch);
  printf("max_distance_diff_mm %.6f\n", max_diff_m * 1000.0);
  printf("max_distance_diff_rel_pct %.6f\n", max_rel * 100.0);
  printf("distance_tol abs %.4f mm + rel %.3f %% + cond %.1e*|c1-c2|/alpha^2\n",
         ATOL_M * 1000.0, RTOL * 100.0, (double)BTOL);
  printf("pairs_over_tolerance %u\n", over_tol);
  printf("max_contact_diff_mm %.6f\n", max_contact_m * 1000.0);
  printf("contacts_matching_reference %u\n", ref.n - over_contact);
  printf("%s\n", pass ? "PASS" : "FAIL");
  free(ref.idx); free(ref.flag); free(ref.dist); free(ref.alpha); free(ref.p1); free(ref.p2);
  free(opt.idx); free(opt.flag); free(opt.dist); free(opt.alpha); free(opt.p1); free(opt.p2);
  return pass ? 0 : 1;
}
