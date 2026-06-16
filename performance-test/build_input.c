/* build_input.c — build step: split the committed text pair set into two
 * separate binary files.
 *
 *   build_input <pairs.txt> <shapes.bin> <pairs.bin>
 *
 * shapes.bin holds the primitives, pairs.bin holds the pair indices. They are
 * deliberately separate so the downstream optimized transforms each see only
 * one half and cannot precompute collision results. No solver code here.
 */
#include "pairs_io.h"
#include "../bin_format.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  cp_prim *prims = NULL;
  cp_pair *pairs = NULL;
  uint32_t prc = 0, pc = 0;
  if (argc != 4) {
    fprintf(stderr, "usage: build_input <pairs.txt> <shapes.bin> <pairs.bin>\n");
    return 2;
  }
  if (pairs_read_text(argv[1], &prims, &pairs, &prc, &pc)) {
    fprintf(stderr, "error: cannot read %s\n", argv[1]);
    return 2;
  }
  if (bin_write(argv[2], BIN_MAGIC_SHAPES, sizeof(cp_prim), prims, prc)) {
    fprintf(stderr, "error: cannot write %s\n", argv[2]);
    return 2;
  }
  if (bin_write(argv[3], BIN_MAGIC_PAIRS, sizeof(cp_pair), pairs, pc)) {
    fprintf(stderr, "error: cannot write %s\n", argv[3]);
    return 2;
  }
  free(prims);
  free(pairs);
  return 0;
}
