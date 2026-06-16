/* performance-test/pairs_io.c — see pairs_io.h.
 * Floats are written with %.9g (exact float32 round trip) and read with
 * strtof-compatible %f scanning, so text -> struct is deterministic.
 */
#include "pairs_io.h"

#include <stdio.h>
#include <stdlib.h>

int pairs_write_text(const char *path, const cp_prim *prims,
                     const cp_pair *pairs, uint32_t pair_count) {
  FILE *f = fopen(path, "w");
  uint32_t i, j, prim_count = 2 * pair_count;
  if (!f)
    return 1;
  fprintf(f, "collide-pairs-v1 %u %u\n", pair_count, prim_count);
  for (i = 0; i < prim_count; ++i) {
    const cp_prim *p = &prims[i];
    fprintf(f, "PRIM %u", p->type);
    for (j = 0; j < 3; ++j)
      fprintf(f, " %.9g", (double)p->pos[j]);
    for (j = 0; j < 9; ++j)
      fprintf(f, " %.9g", (double)p->rot[j]);
    fprintf(f, " %.9g %.9g", (double)p->radius, (double)p->length);
    for (j = 0; j < 3; ++j)
      fprintf(f, " %.9g", (double)p->half_extent[j]);
    fprintf(f, " %u", p->vert_count);
    for (j = 0; j < p->vert_count && j < CP_MAX_POLY_VERTS; ++j)
      fprintf(f, " %.9g %.9g %.9g", (double)p->verts[j][0],
              (double)p->verts[j][1], (double)p->verts[j][2]);
    fputc('\n', f);
  }
  for (i = 0; i < pair_count; ++i)
    fprintf(f, "PAIR %u %u\n", pairs[i].a, pairs[i].b);
  return fclose(f) ? 1 : 0;
}

int pairs_read_text(const char *path, cp_prim **prims_out,
                    cp_pair **pairs_out, uint32_t *prim_count_out,
                    uint32_t *pair_count_out) {
  FILE *f = fopen(path, "r");
  cp_prim *prims = NULL;
  cp_pair *pairs = NULL;
  uint32_t pc = 0, prc = 0, i, j;
  if (!f)
    return 1;
  if (fscanf(f, "collide-pairs-v1 %u %u", &pc, &prc) != 2 ||
      pc == 0 || prc == 0 || pc > 1000000u || prc > 2000000u)
    goto fail;
  prims = (cp_prim *)calloc(prc, sizeof *prims);
  pairs = (cp_pair *)calloc(pc, sizeof *pairs);
  if (!prims || !pairs)
    goto fail;
  for (i = 0; i < prc; ++i) {
    cp_prim *p = &prims[i];
    if (fscanf(f, " PRIM %u", &p->type) != 1)
      goto fail;
    for (j = 0; j < 3; ++j)
      if (fscanf(f, " %f", &p->pos[j]) != 1)
        goto fail;
    for (j = 0; j < 9; ++j)
      if (fscanf(f, " %f", &p->rot[j]) != 1)
        goto fail;
    if (fscanf(f, " %f %f", &p->radius, &p->length) != 2)
      goto fail;
    for (j = 0; j < 3; ++j)
      if (fscanf(f, " %f", &p->half_extent[j]) != 1)
        goto fail;
    if (fscanf(f, " %u", &p->vert_count) != 1 ||
        p->vert_count > CP_MAX_POLY_VERTS)
      goto fail;
    for (j = 0; j < p->vert_count; ++j)
      if (fscanf(f, " %f %f %f", &p->verts[j][0], &p->verts[j][1],
                 &p->verts[j][2]) != 3)
        goto fail;
  }
  for (i = 0; i < pc; ++i)
    if (fscanf(f, " PAIR %u %u", &pairs[i].a, &pairs[i].b) != 2)
      goto fail;
  fclose(f);
  *prims_out = prims;
  *pairs_out = pairs;
  *prim_count_out = prc;
  *pair_count_out = pc;
  return 0;
fail:
  fclose(f);
  free(prims);
  free(pairs);
  return 1;
}
