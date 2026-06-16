/* perf_main.c (optimized) — timing harness for the precomputed-shape runtime.
 *
 * Usage: perf_harness_opt <shapes_optimized.bin> <pairs_optimized.bin> <results.txt>
 *
 * - shapes_optimized.bin is the serialized cp_vshapes table produced by the
 *   build-stage transform build_optimized_shapes (per-shape build already done).
 * - Times ONLY cp_collide_pairs_vshapes — the solve over the precomputed table.
 *   The per-shape precompute is a build step, excluded from this measurement.
 * - Writes one fixed-format line per pair: "<index> <colliding> <distance>".
 */
#define _POSIX_C_SOURCE 199309L
#include "pairs_io.h"        /* pulls in the matching collide.h for this dir */
#include "../bin_format.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv) {
  void *blob = NULL;
  uint32_t nbytes = 0;
  const cp_vshapes *shapes;
  cp_pair *pairs = NULL;
  cp_result *results;
  uint32_t pc = 0, i;
  struct timespec t0, t1;
  double dt;
  FILE *out;
  if (argc != 4) {
    fprintf(stderr,
            "usage: perf_harness_opt <shapes_optimized.bin> "
            "<pairs_optimized.bin> <results.txt>\n");
    return 2;
  }
  if (bin_read(argv[1], BIN_MAGIC_VSHAPES, 1, &blob, &nbytes)) {
    fprintf(stderr, "error: cannot load %s\n", argv[1]);
    return 2;
  }
  /* View the blob as the table in place — no copy. `blob` stays alive (and owns
   * the memory) for as long as `shapes` is used; freed at the end. */
  shapes = cp_vshapes_from_blob(blob, nbytes);
  if (!shapes) {
    fprintf(stderr, "error: malformed precomputed shapes in %s\n", argv[1]);
    free(blob);
    return 2;
  }
  if (bin_read(argv[2], BIN_MAGIC_PAIRS, sizeof(cp_pair), (void **)&pairs, &pc)) {
    fprintf(stderr, "error: cannot load %s\n", argv[2]);
    free(blob);
    return 2;
  }
  results = (cp_result *)malloc(sizeof(cp_result) * pc);
  if (!results) {
    fprintf(stderr, "error: out of memory\n");
    free(blob);
    free(pairs);
    return 2;
  }

  clock_gettime(CLOCK_MONOTONIC, &t0);
  cp_collide_pairs_vshapes(shapes, pairs, pc, results);
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
  free(blob);
  free(pairs);
  return 0;
}
