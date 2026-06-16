/* perf_main.c — timing harness.
 *
 * Usage: perf_harness <shapes.bin> <pairs.bin> <results.txt>
 *
 * - Loads the two binary inputs (primitives + pair indices) produced by the
 *   build steps. There is NO text parsing or pre-conversion here; building the
 *   .bin inputs is a separate build stage, excluded from this measurement.
 * - Times ONLY the single cp_collide_pairs batch call over all pairs
 *   (CLOCK_MONOTONIC), single-threaded, and prints that measured total.
 * - Writes one fixed-format line per pair: "<index> <colliding> <distance>"
 *   with 6 decimals, deterministic and diff-stable across runs.
 */
#define _POSIX_C_SOURCE 199309L
#include "pairs_io.h"        /* pulls in the matching collide.h for this dir */
#include "../bin_format.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv) {
  cp_prim *prims = NULL;
  cp_pair *pairs = NULL;
  cp_result *results;
  void *scratch = NULL;
  size_t scratch_bytes;
  uint32_t prc = 0, pc = 0, i;
  struct timespec t0, t1;
  double dt;
  FILE *out;
  if (argc != 4) {
    fprintf(stderr,
            "usage: perf_harness <shapes.bin> <pairs.bin> <results.txt>\n");
    return 2;
  }
  if (bin_read(argv[1], BIN_MAGIC_SHAPES, sizeof(cp_prim), (void **)&prims, &prc)) {
    fprintf(stderr, "error: cannot load %s\n", argv[1]);
    return 2;
  }
  if (bin_read(argv[2], BIN_MAGIC_PAIRS, sizeof(cp_pair), (void **)&pairs, &pc)) {
    fprintf(stderr, "error: cannot load %s\n", argv[2]);
    free(prims);
    return 2;
  }
  results = (cp_result *)malloc(sizeof(cp_result) * pc);
  /* Pre-allocate the solver's working buffer here, before the timed region, so
   * the timed collision call allocates nothing (the library never mallocs). */
  scratch_bytes = cp_collide_scratch_bytes(prc);
  scratch = scratch_bytes ? malloc(scratch_bytes) : NULL;
  if (!results || (scratch_bytes && !scratch)) {
    fprintf(stderr, "error: out of memory\n");
    free(prims);
    free(pairs);
    free(results);
    free(scratch);
    return 2;
  }

  clock_gettime(CLOCK_MONOTONIC, &t0);
  cp_collide_pairs(prims, prc, pairs, pc, results, scratch, scratch_bytes);
  clock_gettime(CLOCK_MONOTONIC, &t1);
  dt = (double)(t1.tv_sec - t0.tv_sec) +
       1e-9 * (double)(t1.tv_nsec - t0.tv_nsec);

  for (i = 0; i < pc; ++i) {
    if (results[i].status != CP_OK) {
      fprintf(stderr, "error: pair %u status %u (input data invalid)\n",
              i, results[i].status);
      return 2;
    }
  }
  out = fopen(argv[3], "w");
  if (!out) {
    fprintf(stderr, "error: cannot write %s\n", argv[3]);
    return 2;
  }
  for (i = 0; i < pc; ++i)
    fprintf(out, "%04u %u %.6f %.9f %.6f %.6f %.6f %.6f %.6f %.6f\n", i,
            results[i].colliding, (double)results[i].distance,
            (double)results[i].alpha,
            (double)results[i].p1[0], (double)results[i].p1[1],
            (double)results[i].p1[2], (double)results[i].p2[0],
            (double)results[i].p2[1], (double)results[i].p2[2]);
  if (fclose(out) != 0) {
    fprintf(stderr, "error: write failed for %s\n", argv[3]);
    return 2;
  }
  printf("total collision time: %.6f s (%u pairs, single thread)\n", dt, pc);
  free(results);
  free(scratch);
  free(prims);
  free(pairs);
  return 0;
}
