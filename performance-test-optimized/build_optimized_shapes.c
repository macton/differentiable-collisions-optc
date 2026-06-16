/* build_optimized_shapes.c — optimized build-stage transform for the shapes.
 *
 *   build_optimized_shapes <shapes.bin> <shapes_optimized.bin>
 *
 * Reads ONLY the shapes (never the pairs), so it cannot compute any collision
 * result. It performs the whole per-shape precompute up front: validation,
 * face construction, and the world-space solver geometry (cp_vshapes_create).
 * The optimized runtime then solves directly from this table, so the per-shape
 * build is excluded from the timed collision cost. Output is the serialized
 * cp_vshapes blob.
 */
#include "../src-optimized/collide.h"
#include "../bin_format.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  cp_prim *prims = NULL;
  uint32_t prc = 0;
  cp_vshapes *vs;
  void *vsbuf;
  size_t vsbytes;
  const void *blob;
  size_t nbytes;
  if (argc != 3) {
    fprintf(stderr,
            "usage: build_optimized_shapes <shapes.bin> <shapes_optimized.bin>\n");
    return 2;
  }
  if (bin_read(argv[1], BIN_MAGIC_SHAPES, sizeof(cp_prim), (void **)&prims, &prc)) {
    fprintf(stderr, "error: cannot read %s\n", argv[1]);
    return 2;
  }

  /* precompute the full per-shape table (validation + faces + world geometry).
   * The library allocates nothing; this build stage owns the table's memory. */
  vsbytes = cp_vshapes_bytes(prc);
  vsbuf = vsbytes ? malloc(vsbytes) : NULL;
  vs = cp_vshapes_create(prims, prc, vsbuf, vsbytes);
  if (!vs) {
    fprintf(stderr, "error: cp_vshapes_create failed\n");
    free(vsbuf);
    free(prims);
    return 2;
  }
  blob = cp_vshapes_blob(vs, &nbytes);
  if (nbytes > 0xFFFFFFFFu) {
    fprintf(stderr, "error: precomputed table too large\n");
    return 2;
  }
  if (bin_write(argv[2], BIN_MAGIC_VSHAPES, 1, blob, (uint32_t)nbytes)) {
    fprintf(stderr, "error: cannot write %s\n", argv[2]);
    return 2;
  }
  free(vsbuf);
  free(prims);
  return 0;
}
