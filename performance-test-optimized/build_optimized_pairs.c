/* build_optimized_pairs.c — optimized build-stage transform for the pairs.
 *
 *   build_optimized_pairs <pairs.bin> <pairs_optimized.bin>
 *
 * Reads ONLY the pair indices (never the shapes), so it cannot compute any
 * collision result — it can only re-lay-out / reorder the pair list. IDENTITY
 * transform for now; a later step replaces the body with real pair-layout
 * optimization while keeping the same isolation guarantee.
 */
#include "../src-optimized/collide.h"
#include "../bin_format.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  cp_pair *pairs = NULL;
  uint32_t pc = 0;
  if (argc != 3) {
    fprintf(stderr,
            "usage: build_optimized_pairs <pairs.bin> <pairs_optimized.bin>\n");
    return 2;
  }
  if (bin_read(argv[1], BIN_MAGIC_PAIRS, sizeof(cp_pair), (void **)&pairs, &pc)) {
    fprintf(stderr, "error: cannot read %s\n", argv[1]);
    return 2;
  }

  /* identity transform (placeholder for pair-layout optimization) */

  if (bin_write(argv[2], BIN_MAGIC_PAIRS, sizeof(cp_pair), pairs, pc)) {
    fprintf(stderr, "error: cannot write %s\n", argv[2]);
    return 2;
  }
  free(pairs);
  return 0;
}
