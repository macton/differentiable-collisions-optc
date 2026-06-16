/* src/collide.h — convex primitive collision detection (reference).
 *
 * Method: Tracy/Howell/Manchester, "Differentiable Collision Detection for
 * a Set of Convex Primitives" (arXiv:2207.00669), restricted to the pinned
 * primitive set {sphere, box, capsule, polytope}. See src/README.md.
 *
 * Units: meters. Domain: every world AABB corner within +/-8192 m; world
 * AABB extent in [0.1, 250] m on every axis. Out-of-range input is rejected
 * per pair via cp_result.status — never clamped.
 *
 * Batch contract: arrays in, arrays out; pairs reference primitives by
 * index. A single query is pair_count = 1. Deterministic: same input bytes
 * produce same output bytes.
 */
#ifndef COLLIDE_H
#define COLLIDE_H

#include <stddef.h>
#include <stdint.h>

#define CP_MAX_POLY_VERTS 32

typedef enum {
  CP_SPHERE   = 0,  /* radius                                       */
  CP_BOX      = 1,  /* half_extent[3]                               */
  CP_CAPSULE  = 2,  /* radius + segment length, segment on body x   */
  CP_POLYTOPE = 3   /* vert_count body-frame points (convex cloud)  */
} cp_type;

typedef struct {                /* one convex primitive               */
  float    pos[3];              /* r: body origin in world, meters    */
  float    rot[9];              /* WQB, row-major, world = rot * body */
  uint32_t type;                /* cp_type                            */
  uint32_t vert_count;          /* CP_POLYTOPE only: 4..32, else 0    */
  float    radius;              /* sphere, capsule                    */
  float    length;              /* capsule: segment length L          */
  float    half_extent[3];      /* box                                */
  float    verts[CP_MAX_POLY_VERTS][3]; /* polytope, body frame       */
} cp_prim;

typedef struct { uint32_t a, b; } cp_pair;  /* indices into prim array */

typedef enum {
  CP_OK              = 0,
  CP_ERR_COORD_RANGE = 1,  /* world AABB corner outside +/-8192 m        */
  CP_ERR_SIZE_RANGE  = 2,  /* world AABB extent < 0.1 m or > 250 m       */
  CP_ERR_BAD_INDEX   = 3,  /* pair index >= prim_count                   */
  CP_ERR_BAD_PRIM    = 4,  /* bad type/params/vert_count/rotation        */
  CP_ERR_NO_CONVERGE = 5   /* solver failed (explicit, never silent)     */
} cp_status;

typedef struct {
  uint32_t status;     /* cp_status; fields below valid only if CP_OK */
  uint32_t colliding;  /* 1 iff alpha < 1                             */
  float    alpha;      /* minimum uniform scaling (paper's output)    */
  float    distance;   /* |c1-c2|*(1 - 1/alpha): + separated, - pen.  */
  float    p1[3];      /* contact point on primitive a (eq. 24)       */
  float    p2[3];      /* contact point on primitive b (eq. 24)       */
} cp_result;

/* The library allocates nothing: the caller provides a scratch buffer for the
 * per-batch working set. cp_collide_scratch_bytes() returns the number of bytes
 * cp_collide_pairs needs for prim_count primitives (0 for prim_count 0); pass a
 * buffer of at least that size, suitably aligned (malloc's alignment suffices).
 * If the buffer is NULL or too small for prim_count > 0, every result is set to
 * CP_ERR_NO_CONVERGE (explicit, never a silent or hidden allocation). */
size_t cp_collide_scratch_bytes(uint32_t prim_count);

void cp_collide_pairs(const cp_prim *prims, uint32_t prim_count,
                      const cp_pair *pairs, uint32_t pair_count,
                      cp_result *results, void *scratch, size_t scratch_bytes);

#endif /* COLLIDE_H */
