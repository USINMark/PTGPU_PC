
#include "vec.h"

#include "camera.h"
#include "geom.h"
#include "BVHNodeGPU.h"
#include "KDNodeGPU.h"

#define MTEPSILON 0.000001

#define INTERSECT_STACK_SIZE (18)
#define RESTRUCT_STACK_SIZE (4)

#define Ci 1.2
#define Ct 1.0
