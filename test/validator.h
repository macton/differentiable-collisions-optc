/* test/validator.h — secondary, independent validator.
 *
 * Independent method: GJK distance queries (support-function sampling, raw
 * polytope vertices, double precision) + bisection on the uniform scaling
 * alpha. Shares only the cp_prim data definitions with src/ — no solver,
 * hull, assembly, or barrier code.
 */
#ifndef VALIDATOR_H
#define VALIDATOR_H

#include "../src/collide.h"

/* Returns 0 on success. Outputs the minimum uniform scaling alpha and the
 * signed distance |c1-c2|*(1 - 1/alpha) (definitionally identical to the
 * signed gap between the paper's eq.-24 contact points). */
int val_collide(const cp_prim *a, const cp_prim *b,
                double *alpha_out, double *dist_out);

#endif /* VALIDATOR_H */
