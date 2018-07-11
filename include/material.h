#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "vec.h"

typedef enum {
    LAMBERTIAN = 0,
    CONDUCTOR,
    DIELECTRIC,
    MATTE,
    PLASTIC,
    METAL
} MaterialType;

typedef struct {
    short type;

    // matte, labertian
    Vec kd;
    // plastic
    Vec ks;

    // matte (oren-nayar facet distribution - std deviation of the facet orientations)
    float sigma;
    // plastic
    float roughness;

    // specific pre calc'ed oren nayar values
    float A, B;

} Material;
#endif // _MATERIAL_H_
