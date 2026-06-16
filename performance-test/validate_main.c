/* performance-test/validate_main.c — independent-validator agreement over
 * the full performance data set. Runs OUTSIDE the timed path. Fails (exit
 * 1) if any pair disagrees with the primary solver by more than 1 mm.
 */
#include "pairs_io.h"
#include "../bin_format.h"
#include "../test/validator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  cp_prim *prims = NULL;
  cp_pair *pairs = NULL;
  cp_result *res;
  void *scratch = NULL;
  size_t scratch_bytes;
  uint32_t prc = 0, pc = 0, i, nbad = 0, nflag = 0, ncoll = 0;
  double maxdiff = 0.0;
  if (argc != 3) {
    fprintf(stderr, "usage: perf_validate <shapes.bin> <pairs.bin>\n");
    return 2;
  }
  if (bin_read(argv[1], BIN_MAGIC_SHAPES, sizeof(cp_prim), (void **)&prims, &prc)) {
    fprintf(stderr, "error: cannot read %s\n", argv[1]);
    return 2;
  }
  if (bin_read(argv[2], BIN_MAGIC_PAIRS, sizeof(cp_pair), (void **)&pairs, &pc)) {
    fprintf(stderr, "error: cannot read %s\n", argv[2]);
    free(prims);
    return 2;
  }
  res = (cp_result *)malloc(sizeof(cp_result) * pc);
  scratch_bytes = cp_collide_scratch_bytes(prc);
  scratch = scratch_bytes ? malloc(scratch_bytes) : NULL;
  if (!res || (scratch_bytes && !scratch)) {
    fprintf(stderr, "error: out of memory\n");
    return 2;
  }
  cp_collide_pairs(prims, prc, pairs, pc, res, scratch, scratch_bytes);
  for (i = 0; i < pc; ++i) {
    double va = 0.0, vd = 0.0, diff;
    if (res[i].status != CP_OK) {
      printf("pair %u: primary status %u\n", i, res[i].status);
      ++nbad;
      continue;
    }
    if (val_collide(&prims[pairs[i].a], &prims[pairs[i].b], &va, &vd)) {
      printf("pair %u: validator failed\n", i);
      ++nbad;
      continue;
    }
    diff = fabs(vd - (double)res[i].distance);
    if (diff > maxdiff)
      maxdiff = diff;
    if (diff > 1e-3) {
      printf("pair %u: primary %.6f validator %.6f diff %.6f\n",
             i, (double)res[i].distance, vd, diff);
      ++nbad;
    }
    /* flag check only away from the alpha==1 boundary */
    if (fabs(vd) > 2e-3 && res[i].colliding != (vd < 0.0 ? 1u : 0u)) {
      printf("pair %u: colliding flag mismatch\n", i);
      ++nflag;
    }
    if (res[i].colliding)
      ++ncoll;
  }
  printf("validated %u pairs: %u colliding, max |dist diff| = %.9f m, "
         "distance failures = %u, flag mismatches = %u\n",
         pc, ncoll, maxdiff, nbad, nflag);
  free(res);
  free(prims);
  free(pairs);
  return (nbad || nflag) ? 1 : 0;
}
