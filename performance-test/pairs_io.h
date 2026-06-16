/* performance-test/pairs_io.h — text I/O for the static pair data set.
 * Shared by the generator, the timing harness (pre-conversion only), and
 * the validation tool. Contains no solver code.
 */
#ifndef PAIRS_IO_H
#define PAIRS_IO_H

#include "../src/collide.h"

/* Writes "collide-pairs-v1" text format; prims has 2*pair_count entries. */
int pairs_write_text(const char *path, const cp_prim *prims,
                     const cp_pair *pairs, uint32_t pair_count);

/* Reads the text format; mallocs *prims_out / *pairs_out (caller frees).
 * Returns 0 on success. */
int pairs_read_text(const char *path, cp_prim **prims_out,
                    cp_pair **pairs_out, uint32_t *prim_count_out,
                    uint32_t *pair_count_out);

#endif /* PAIRS_IO_H */
