/* bin_format.h — count-prefixed binary container for the split runtime input.
 *
 * The runtime input is two SEPARATE files so that no build-stage transform can
 * see both halves at once, and therefore cannot precompute collision answers
 * (an answer needs shapes AND pairs together — they only meet at runtime):
 *
 *   shapes.bin : [u32 magic 'SHP1'][u32 count][ count * cp_prim records ]
 *   pairs.bin  : [u32 magic 'PRS1'][u32 count][ count * cp_pair records ]
 *
 * Records are raw fixed-layout host-native structs (the data never leaves this
 * machine). The container itself is type-agnostic: callers pass the record
 * size, so this header has no dependency on collide.h.
 */
#ifndef BIN_FORMAT_H
#define BIN_FORMAT_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define BIN_MAGIC_SHAPES 0x53485031u /* 'S','H','P','1' — raw cp_prim records   */
#define BIN_MAGIC_PAIRS  0x50525331u /* 'P','R','S','1' — raw cp_pair records   */
#define BIN_MAGIC_VSHAPES 0x53485056u /* 'S','H','P','V' — precomputed cp_vshapes blob */

/* Read a count-prefixed record array; mallocs *out_data (caller frees).
 * Returns 0 on success, nonzero on any error. */
static inline int bin_read(const char *path, uint32_t magic, size_t elem_size,
                           void **out_data, uint32_t *out_count) {
  uint32_t hdr[2];
  void *data;
  FILE *f = fopen(path, "rb");
  if (!f)
    return 1;
  if (fread(hdr, sizeof hdr, 1, f) != 1 || hdr[0] != magic || hdr[1] == 0) {
    fclose(f);
    return 1;
  }
  data = malloc(elem_size * (size_t)hdr[1]);
  if (!data) {
    fclose(f);
    return 1;
  }
  if (fread(data, elem_size, hdr[1], f) != hdr[1]) {
    fclose(f);
    free(data);
    return 1;
  }
  fclose(f);
  *out_data = data;
  *out_count = hdr[1];
  return 0;
}

/* Write a count-prefixed record array. Returns 0 on success. */
static inline int bin_write(const char *path, uint32_t magic, size_t elem_size,
                            const void *data, uint32_t count) {
  uint32_t hdr[2];
  int ok;
  FILE *f = fopen(path, "wb");
  if (!f)
    return 1;
  hdr[0] = magic;
  hdr[1] = count;
  ok = fwrite(hdr, sizeof hdr, 1, f) == 1 &&
       fwrite(data, elem_size, count, f) == count;
  if (fclose(f) != 0)
    ok = 0;
  return ok ? 0 : 1;
}

#endif /* BIN_FORMAT_H */
