
#include "clheader.h"

#define ACCELSTR 2 //0 is no accel, 1 is BVH and 2 is KDTREE
#define CPU_PARTRENDERING
//#define DEBUG_INTERSECTIONS

# 1 "<stdin>"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "<stdin>"
# 26 "<stdin>"
# 1 "camera.h" 1
# 27 "camera.h"
# 1 "vec.h" 1
# 27 "vec.h"

# 28 "camera.h" 2

# 27 "<stdin>" 2
# 1 "geomfunc.h" 1
# 27 "geomfunc.h"
# 1 "geom.h" 1
# 32 "geom.h"

# 28 "geomfunc.h" 2
# 1 "simplernd.h" 1
# 34 "simplernd.h"

inline float min2(float a, float b) {
    //return (a < b) ? a : b;
	return fmin(a, b);
}

inline float max2(float a, float b) {
    //return (a > b) ? a : b;
	return fmax(a, b);
}

inline float GetRandom(unsigned int *seed0, unsigned int *seed1) {
 *seed0 = 36969 * ((*seed0) & 65535) + ((*seed0) >> 16);
 *seed1 = 18000 * ((*seed1) & 65535) + ((*seed1) >> 16);

 unsigned int ires = ((*seed0) << 16) + (*seed1);

 union {
  float f;
  unsigned int ui;
 } res;
 res.ui = (ires & 0x007fffff) | 0x40000000;

 return (res.f - 2.f) / 2.f;
}
# 29 "geomfunc.h" 2

inline void swap(float *a, float *b)
{
	float temp = *a;
	*a = *b;
	*b = temp;
}

float SphereIntersect(

__constant

 const Sphere *s,
 const Ray *r) {
 Vec op;
 { (op).x = (s->p).x - (r->o).x; (op).y = (s->p).y - (r->o).y; (op).z = (s->p).z - (r->o).z; };
 
 float b = ((op).x * (r->d).x + (op).y * (r->d).y + (op).z * (r->d).z);
 float det = b * b - ((op).x * (op).x + (op).y * (op).y + (op).z * (op).z) + s->rad * s->rad;
 if (det < 0.f)
  return 0.f;
 else
  det = sqrt(det);
 
 float t = b - det;
 if (t > 0.01f)
  return t;
 else { 
  t = b + det;
  
  if (t > 0.01f)
   return t;
  else
   return 0.f;
 } 
}

//Written in the paper "Fast, Minimum Storage Ray/ Triangle Intersection".
float TriangleIntersect(

__constant

	const Triangle *tr,

__constant

	Poi *pois,
	const unsigned int poiCnt,
	const Ray *r) { 
	/* returns distance, 0 if nohit */
	Vec v0 = pois[tr->p1].p, v1 = pois[tr->p2].p, v2 = pois[tr->p3].p, e1, e2, tvec, pvec, qvec;
	float t = 0.0f, u, v;
	float det, inv_det;

	{ e1.x = (v1).x - (v0).x; e1.y = (v1).y - (v0).y; e1.z = (v1).z - (v0).z; }
	{ e2.x = (v2).x - (v0).x; e2.y = (v2).y - (v0).y; e2.z = (v2).z - (v0).z; }

	//vxcross(pvec, r->d, e2);
	{ pvec.x = (r->d).y * (e2).z - (r->d).z * (e2).y; pvec.y = (r->d).z * (e2).x - (r->d).x * (e2).z; pvec.z = (r->d).x * (e2).y - (r->d).y * (e2).x; }
	//det = vdot(e1, pvec);
	det = ((e1).x * (pvec).x + (e1).y * (pvec).y + (e1).z * (pvec).z);
#ifdef TEST_CULL
	if (det < MTEPSILON) return 0.0f;

	{ tvec.x = (r->o).x - (v0).x; tvec.y = (r->o).y - (v0).y; tvec.z = (r->o).z - (v0).z; }
	//u = vdot(tvec, pvec);
	u = ((tvec).x * (pvec).x + (tvec).y * (pvec).y + (tvec).z * (pvec).z);
	if (u < 0.0 || u > det) return 0.0f;

	//vxcross(qvec, tvec, e1);
	{ qvec.x = (tvec).y * (e1).z - (tvec).z * (e1).y; qvec.y = (tvec).z * (e1).x - (tvec).x * (e1).z; qvec.z = (tvec).x * (e1).y - (tvec).y * (e1).x; }
	//v = vdot(r->d, qvec);
	v = ((r->d).x * (qvec).x + (r->d).y * (qvec).y + (r->d).z * (qvec).z);
	if (v < 0.0 || u + v > det) return 0.0f;

	//t = vdot(e2, qvec);
	t = ((e2).x * (qvec).x + (e2).y * (qvec).y + (e2).z * (qvec).z);
	inv_det = 1.0 / det;

	t *= inv_det;
	u *= inv_det;
	v *= inv_det;
#else
	if (det > -MTEPSILON && det < MTEPSILON) return 0.0f;

	inv_det = 1.0 / det;

	//vsub(tvec, r->o, v0);
	{ tvec.x = (r->o).x - (v0).x; tvec.y = (r->o).y - (v0).y; tvec.z = (r->o).z - (v0).z; }
	//u = vdot(tvec, pvec) * inv_det;
	u = ((tvec).x * (pvec).x + (tvec).y * (pvec).y + (tvec).z * (pvec).z) * inv_det;
	if (u < 0.0 || u > 1.0) return 0.0f;

	//vxcross(qvec, tvec, e1);
	{ qvec.x = (tvec).y * (e1).z - (tvec).z * (e1).y; qvec.y = (tvec).z * (e1).x - (tvec).x * (e1).z; qvec.z = (tvec).x * (e1).y - (tvec).y * (e1).x; }
	//v = vdot(r->d, qvec) * inv_det;
	v = ((r->d).x * (qvec).x + (r->d).y * (qvec).y + (r->d).z * (qvec).z) * inv_det;
	if (v < 0.0 || u + v > 1.0) return 0.0f;

	//t = vdot(e2, qvec) * inv_det;
	t = ((e2).x * (qvec).x + (e2).y * (qvec).y + (e2).z * (qvec).z) * inv_det;
#endif
	return t;
}

void UniformSampleSphere(const float u1, const float u2, Vec *v) {
 const float zz = 1.f - 2.f * u1;
 const float r = sqrt(max2(0.f, 1.f - zz * zz));
 const float phi = 2.f * 3.14159265358979323846f * u2;
 const float xx = r * cos(phi);
 const float yy = r * sin(phi);

 { (*v).x = xx; (*v).y = yy; (*v).z = zz; };
}

/**
 * Test if a ray intersect a bound
 */
bool intersection_bound_test(const Ray r, Bound bound
#ifdef DEBUG_INTERSECTIONS
	, __global float *debug2
#endif
	) {
    float t_min, t_max, t_xmin, t_xmax, t_ymin, t_ymax, t_zmin, t_zmax;
    float x_a = 1.0/r.d.x, y_a = 1.0/r.d.y, z_a = 1.0/r.d.z;
    float  x_e = r.o.x, y_e = r.o.y, z_e = r.o.z;

    // calculate t interval in x-axis
    if (x_a >= 0) {
        t_xmin = (bound.min_x - x_e) * x_a;
        t_xmax = (bound.max_x - x_e) * x_a;
    }
    else {
        t_xmin = (bound.max_x - x_e) * x_a;
        t_xmax = (bound.min_x - x_e) * x_a;
    }

    // calculate t interval in y-axis
    if (y_a >= 0) {
        t_ymin = (bound.min_y - y_e) * y_a;
        t_ymax = (bound.max_y - y_e) * y_a;
    }
    else {
        t_ymin = (bound.max_y - y_e) * y_a;
        t_ymax = (bound.min_y - y_e) * y_a;
    }

    // calculate t interval in z-axis
    if (z_a >= 0) {
        t_zmin = (bound.min_z - z_e) * z_a;
        t_zmax = (bound.max_z - z_e) * z_a;
    }
    else {
        t_zmin = (bound.max_z - z_e) * z_a;
        t_zmax = (bound.min_z - z_e) * z_a;
    }

    // find if there an intersection among three t intervals
    t_min = max2(t_xmin, max2(t_ymin, t_zmin));
    t_max = min2(t_xmax, min2(t_ymax, t_zmax));
	
#ifdef DEBUG_INTERSECTIONS
    if (get_global_id(0) == 0) {
        debug2[0] = t_min, debug2[1] = t_max, debug2[2] = t_xmin, debug2[3] = t_xmax, debug2[4] = t_ymin, debug2[5] = t_ymax, debug2[6] = t_zmin, debug2[7] = t_zmax;
        debug2[8] = x_a, debug2[9] = y_a, debug2[10] = z_a;
        debug2[11] = bound.min_x, debug2[12] = bound.max_x, debug2[13] = bound.min_y, debug2[14] = bound.max_y, debug2[15] = bound.min_z, debug2[16] = bound.max_z;
        debug2[17] = r.o.x, debug2[18] = r.o.y, debug2[19] = r.o.z, debug2[20] = r.d.x, debug2[21] = r.d.y, debug2[22] = r.d.z;
    }
#endif
    return (t_min <= t_max);
}

int Intersect(

__constant

 const Shape *shapes,
 const unsigned int shapeCnt,

__constant

 Poi *pois,
 const unsigned int poiCnt,
#if (ACCELSTR == 1)
__constant

 BVHTreeNode *btn, 
__constant

 BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
 __constant
 
 KDNodeGPU *kng, 
__constant

 int *kn, 
 int szkng, int szkn, 
#endif
 const Ray *r,
 float *t,
 unsigned int *id
#ifdef DEBUG_INTERSECTIONS
 , __global int *debug1,
 __global float *debug2
#endif
 ) {
#if (ACCELSTR == 0)
	float inf = (*t) = 1e20f;
	unsigned int i = shapeCnt;
	
	for (; i--;) {
		float d = 0.0f;
		if (shapes[i].type == SPHERE ) d = SphereIntersect(&shapes[i].s, r);
		if (shapes[i].type == TRIANGLE ) d = TriangleIntersect(&shapes[i].t, pois, poiCnt, r);
		if ((d != 0.f) && (d < *t)) {
			*t = d;
			*id = i;
		}
	}
#ifdef DEBUG_INTERSECTIONS
	debug1[get_global_id(0)] = *id;

	debug2[get_global_id(0)] = *t;
#endif
	return (*t < inf);
#elif (ACCELSTR == 1)
	(*t) = 1e20f;
	
    // Use static allocation because malloc() can't be called in parallel
    // Use stack to traverse BVH to save space (cost is O(height))
    int stack[INTERSECT_STACK_SIZE];
    int topIndex = INTERSECT_STACK_SIZE;
    stack[--topIndex] = 0; //btn;
    int intersected = 0, status = 0;

    // Do while stack is not empty
    while (topIndex != INTERSECT_STACK_SIZE) {
        int n = stack[topIndex++];
#ifdef DEBUG_INTERSECTIONS
        atomic_add(&debug1[n], 1);
#endif

        if (intersection_bound_test(*r, btn[n].bound
#ifdef DEBUG_INTERSECTIONS
			, debug2
#endif
			)) {
            if (btn[n].leaf == 0) {
                stack[--topIndex] = btn[n].nRight;
                stack[--topIndex] = btn[n].nLeft;

                if (topIndex < 0) {
                    //printf("Intersect stack not big enough. Increase INTERSECT_STACK_SIZE!\n");
                    return 0;
                }
			}
			else if (btn[n].leaf == 2) {
                float d = 0.0f;

			    if (shapes[btl[btn[n].nLeft].nShape].type == SPHERE ) d = SphereIntersect(&shapes[btl[btn[n].nLeft].nShape].s, r);
				if (shapes[btl[btn[n].nLeft].nShape].type == TRIANGLE ) d = TriangleIntersect(&shapes[btl[btn[n].nLeft].nShape].t, pois, poiCnt, r);

                if (d != 0.0) {
                    if (d < *t) {
                        *t = d;
                        *id = btl[btn[n].nLeft].nShape;
                    }
                    intersected = 1;
                }
				
			    if (shapes[btl[btn[n].nRight].nShape].type == SPHERE ) d = SphereIntersect(&shapes[btl[btn[n].nRight].nShape].s, r);
				if (shapes[btl[btn[n].nRight].nShape].type == TRIANGLE ) d = TriangleIntersect(&shapes[btl[btn[n].nRight].nShape].t, pois, poiCnt, r);

                if (d != 0.0) {
                    if (d < *t) {
                        *t = d;
                        *id = btl[btn[n].nRight].nShape;
                    }
                    intersected = 1;
                }				
			}
			else {
				//printf("Unknown node, %d\n", btn[n].leaf);
            }
        }
    }
	
    return intersected;
#elif (ACCELSTR == 2)
	(*t) = 1e20f;

	// Use static allocation because malloc() can't be called in parallel
	// Use stack to traverse BVH to save space (cost is O(height))
	int stack[INTERSECT_STACK_SIZE];
	int topIndex = INTERSECT_STACK_SIZE;
	stack[--topIndex] = 1; //tn;
	int intersected = 0, status = 0;

	// Do while stack is not empty
	while (topIndex != INTERSECT_STACK_SIZE) {
		int n = stack[topIndex++];
#ifdef DEBUG_INTERSECTIONS
        atomic_add(&debug1[n], 1);
#endif

		if (intersection_bound_test(*r, kng[n].bound
#ifdef DEBUG_INTERSECTIONS
			, debug2
#endif
		)) {
			if (kng[n].leaf == 0) {
				stack[--topIndex] = kng[n].nRight;
				stack[--topIndex] = kng[n].nLeft;

				if (topIndex < 0) {
					//printf("Intersect stack not big enough. Increase INTERSECT_STACK_SIZE!\n");
					return 0;
				}
			}
			else if (kng[n].leaf == 1) {
				float d = 0.0f;

				for (int i = kng[n].min; i < kng[n].max; i++) {
					if (shapes[kn[i]].type == SPHERE) d = SphereIntersect(&shapes[kn[i]].s, r);
					if (shapes[kn[i]].type == TRIANGLE) d = TriangleIntersect(&shapes[kn[i]].t, pois, poiCnt, r);

					if (d != 0.0) {
						if (d < *t) {
							*t = d;
							*id = kn[i];
						}
						intersected = 1;
					}
				}				
			}
			else {
				//printf("Unknown node, %d\n", btn[n].leaf);
			}
		}
	}

	return intersected;
#endif
}

int IntersectP(

__constant

 const Shape *shapes,
 const unsigned int shapeCnt,
 const Ray *r,
 const float maxt) {
 unsigned int i = shapeCnt;
 for (; i--;) {
  float d = 0.0f;
  if (shapes[i].type == SPHERE ) d = SphereIntersect(&shapes[i].s, r);
  if ((d != 0.f) && (d < maxt))
   return 1;
 }

 return 0;
}

void SampleLights(

__constant

 const Shape *shapes,
 const unsigned int shapeCnt,
 unsigned int *seed0, unsigned int *seed1,
 const Vec *hitPoint,
 const Vec *normal,
 Vec *result) {
 { (*result).x = 0.f; (*result).y = 0.f; (*result).z = 0.f; };

 unsigned int i;
 for (i = 0; i < shapeCnt; i++) {
	if (shapes[i].type == TRIANGLE) continue;
	
__constant

  const Sphere *light = &shapes[i].s;

  if (!(((light->e).x == 0.f) && ((light->e).x == 0.f) && ((light->e).z == 0.f))) {

   Ray shadowRay;
   shadowRay.o = *hitPoint;

   Vec unitSpherePoint;
   UniformSampleSphere(GetRandom(seed0, seed1), GetRandom(seed0, seed1), &unitSpherePoint);
   
   Vec spherePoint;
   { float k = (light->rad); { (spherePoint).x = k * (unitSpherePoint).x; (spherePoint).y = k * (unitSpherePoint).y; (spherePoint).z = k * (unitSpherePoint).z; } };
   { (spherePoint).x = (spherePoint).x + (light->p).x; (spherePoint).y = (spherePoint).y + (light->p).y; (spherePoint).z = (spherePoint).z + (light->p).z; };

   { (shadowRay.d).x = (spherePoint).x - (*hitPoint).x; (shadowRay.d).y = (spherePoint).y - (*hitPoint).y; (shadowRay.d).z = (spherePoint).z - (*hitPoint).z; };
   const float len = sqrt(((shadowRay.d).x * (shadowRay.d).x + (shadowRay.d).y * (shadowRay.d).y + (shadowRay.d).z * (shadowRay.d).z));
   { float k = (1.f / len); { (shadowRay.d).x = k * (shadowRay.d).x; (shadowRay.d).y = k * (shadowRay.d).y; (shadowRay.d).z = k * (shadowRay.d).z; } };

   float wo = ((shadowRay.d).x * (unitSpherePoint).x + (shadowRay.d).y * (unitSpherePoint).y + (shadowRay.d).z * (unitSpherePoint).z);
   
   if (wo > 0.f) {
    continue;
   } else
    wo = -wo;
	
   const float wi = ((shadowRay.d).x * (*normal).x + (shadowRay.d).y * (*normal).y + (shadowRay.d).z * (*normal).z);
   if ((wi > 0.f) && (!IntersectP(shapes, shapeCnt, &shadowRay, len - 0.01f))) {
    Vec c; { (c).x = (light->e).x; (c).y = (light->e).y; (c).z = (light->e).z; };
    const float s = (4.f * 3.14159265358979323846f * light->rad * light->rad) * wi * wo / (len *len);
    { float k = (s); { (c).x = k * (c).x; (c).y = k * (c).y; (c).z = k * (c).z; } };
    { (*result).x = (*result).x + (c).x; (*result).y = (*result).y + (c).y; (*result).z = (*result).z + (c).z; };
   }
  }
 }
}

void RadianceOnePathTracing(

__constant

 const Shape *shapes,
 const unsigned int shapeCnt,
 
__constant

 Poi *pois,
 const unsigned int poiCnt,
#if (ACCELSTR == 1)
__constant

 BVHTreeNode *btn, 
__constant

 BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
__constant 
 
 KDNodeGPU *kng,
__constant 

 int *kn, 
 int szkng, int szkn, 
#endif
 Ray *currentRay,
 unsigned int *seed0, unsigned int *seed1, 
 int depth, Vec *rad, Vec *throughput, int *specularBounce, int *terminated, 
 Vec *result
#ifdef DEBUG_INTERSECTIONS
 , __global int *debug1,
 __global float *debug2
#endif
 ) {
  float t;
  unsigned int id = 0;

  if (!Intersect(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
  btn, btl,   
#elif (ACCELSTR == 2)
  kng, kn, szkng, szkn, 
#endif
  currentRay, &t, &id
#ifdef DEBUG_INTERSECTIONS
  , debug1, debug2
#endif
  )) {
   *result = *rad;
   *terminated = 1;
   
   return;
  }

  Vec hitPoint;
  //vsmul(hitPoint, t, currentRay.d);
  { float k = (t); hitPoint.x = t * (currentRay->d).x; hitPoint.y = t * (currentRay->d).y; hitPoint.z = t * (currentRay->d).z; }
  //vadd(hitPoint, currentRay.o, hitPoint);
  { hitPoint.x = (currentRay->o).x + (hitPoint).x; hitPoint.y = (currentRay->o).y + (hitPoint).y; hitPoint.z = (currentRay->o).z + (hitPoint).z; } 

  Vec normal, col;
  enum Refl refl;
  
  Shape s = shapes[id];

  if (s.type == SPHERE)
  {
	//vsub(normal, hitPoint, s.s.p);
	{ normal.x = (hitPoint).x - (s.s.p).x; normal.y = (hitPoint).y - (s.s.p).y; normal.z = (hitPoint).z - (s.s.p).z; }

	refl = s.s.refl;
	col = s.s.c;
  }
  else if (s.type == TRIANGLE)
  {
	Vec v0 = pois[s.t.p1].p, v1 = pois[s.t.p2].p, v2 = pois[s.t.p3].p, e1, e2;
		
	//vsub(e1, v1, v0);
	{ e1.x = (v1).x - (v0).x; e1.y = (v1).y - (v0).y; e1.z = (v1).z - (v0).z; }
	//vsub(e2, v2, v0);
	{ e2.x = (v2).x - (v0).x; e2.y = (v2).y - (v0).y; e2.z = (v2).z - (v0).z; }

	//vxcross(normal, e1, e2);
	{ normal.x = (e1).y * (e2).z - (e1).z * (e2).y; normal.y = (e1).z * (e2).x - (e1).x * (e2).z; normal.z = (e1).x * (e2).y - (e1).y * (e2).x; }

	refl = s.t.refl;
	col.x = col.y = 0.0f;
	col.z = 0.6f;
  }

  //vnorm(normal);
  { float l = 1.f / sqrt(((normal).x * (normal).x + (normal).y * (normal).y + (normal).z * (normal).z)); float k = (l); normal.x = k * (normal).x; normal.y = k * (normal).y; normal.z = k * (normal).z; }

  const float dp = ((normal).x * (currentRay->d).x + (normal).y * (currentRay->d).y + (normal).z * (currentRay->d).z);

  Vec nl;
  const float invSignDP = -1.f * sign(dp);
  
  { float k = (invSignDP); { (nl).x = k * (normal).x; (nl).y = k * (normal).y; (nl).z = k * (normal).z; } };

  if (s.type == SPHERE)
  {
   Vec eCol; { (eCol).x = (s.s.e).x; (eCol).y = (s.s.e).y; (eCol).z = (s.s.e).z; };
   if (!(((eCol).x == 0.f) && ((eCol).x == 0.f) && ((eCol).z == 0.f))) {
   if (*specularBounce) {
    { float k = (fabs(dp)); { (eCol).x = k * (eCol).x; (eCol).y = k * (eCol).y; (eCol).z = k * (eCol).z; } };
    { (eCol).x = (*throughput).x * (eCol).x; (eCol).y = (*throughput).y * (eCol).y; (eCol).z = (*throughput).z * (eCol).z; };
    { (rad)->x = (rad)->x + (eCol).x; (rad)->y = (rad)->y + (eCol).y; (rad)->z = (rad)->z + (eCol).z; };
   }

   *result = *rad;
   *terminated = 1;
   
   return;
   }
  }
  if (refl == DIFF) {
   *specularBounce = 0;
   { (throughput)->x = (throughput)->x * (col).x; (throughput)->y = (throughput)->y * (col).y; (throughput)->z = (throughput)->z * (col).z; };

   Vec Ld;
   SampleLights(shapes, shapeCnt, seed0, seed1, &hitPoint, &nl, &Ld);
   { (Ld).x = (throughput)->x * (Ld).x; (Ld).y = (throughput)->y * (Ld).y; (Ld).z = (throughput)->z * (Ld).z; };
   { (rad)->x = (rad)->x + (Ld).x; (rad)->y = (rad)->y + (Ld).y; (rad)->z = (rad)->z + (Ld).z; };

   float r1 = 2.f * 3.14159265358979323846f * GetRandom(seed0, seed1);
   float r2 = GetRandom(seed0, seed1);
   float r2s = sqrt(r2);

   Vec w; { (w).x = (nl).x; (w).y = (nl).y; (w).z = (nl).z; };

   Vec u, a;
   if (fabs(w.x) > .1f) {
    { (a).x = 0.f; (a).y = 1.f; (a).z = 0.f; };
   } else {
    { (a).x = 1.f; (a).y = 0.f; (a).z = 0.f; };
   }
   { (u).x = (a).y * (w).z - (a).z * (w).y; (u).y = (a).z * (w).x - (a).x * (w).z; (u).z = (a).x * (w).y - (a).y * (w).x; };
   { float l = 1.f / sqrt(((u).x * (u).x + (u).y * (u).y + (u).z * (u).z)); { float k = (l); { (u).x = k * (u).x; (u).y = k * (u).y; (u).z = k * (u).z; } }; };

   Vec v;
   { (v).x = (w).y * (u).z - (w).z * (u).y; (v).y = (w).z * (u).x - (w).x * (u).z; (v).z = (w).x * (u).y - (w).y * (u).x; };

   Vec newDir;
   { float k = (cos(r1) * r2s); { (u).x = k * (u).x; (u).y = k * (u).y; (u).z = k * (u).z; } };
   { float k = (sin(r1) * r2s); { (v).x = k * (v).x; (v).y = k * (v).y; (v).z = k * (v).z; } };
   { (newDir).x = (u).x + (v).x; (newDir).y = (u).y + (v).y; (newDir).z = (u).z + (v).z; };
   { float k = (sqrt(1 - r2)); { (w).x = k * (w).x; (w).y = k * (w).y; (w).z = k * (w).z; } };
   { (newDir).x = (newDir).x + (w).x; (newDir).y = (newDir).y + (w).y; (newDir).z = (newDir).z + (w).z; };

   currentRay->o = hitPoint;
   currentRay->d = newDir;
   
   return;
  } else if (refl == SPEC) {
   *specularBounce = 1;

   Vec newDir;
   { float k = (2.f * ((normal).x * (currentRay->d).x + (normal).y * (currentRay->d).y + (normal).z * (currentRay->d).z)); { (newDir).x = k * (normal).x; (newDir).y = k * (normal).y; (newDir).z = k * (normal).z; } };
   { (newDir).x = (currentRay->d).x - (newDir).x; (newDir).y = (currentRay->d).y - (newDir).y; (newDir).z = (currentRay->d).z - (newDir).z; };

   { (throughput)->x = (throughput)->x * (col).x; (throughput)->y = (throughput)->y * (col).y; (throughput)->z = (throughput)->z * (col).z; };

   { { ((currentRay)->o).x = (hitPoint).x; ((currentRay)->o).y = (hitPoint).y; ((currentRay)->o).z = (hitPoint).z; }; { ((currentRay)->d).x = (newDir).x; ((currentRay)->d).y = (newDir).y; ((currentRay)->d).z = (newDir).z; }; };
   
   return;
  } else {
   *specularBounce = 1;

   Vec newDir;
   { float k = (2.f * ((normal).x * (currentRay->d).x + (normal).y * (currentRay->d).y + (normal).z * (currentRay->d).z)); { (newDir).x = k * (normal).x; (newDir).y = k * (normal).y; (newDir).z = k * (normal).z; } };
   { (newDir).x = (currentRay->d).x - (newDir).x; (newDir).y = (currentRay->d).y - (newDir).y; (newDir).z = (currentRay->d).z - (newDir).z; };

   Ray reflRay; { { ((reflRay).o).x = (hitPoint).x; ((reflRay).o).y = (hitPoint).y; ((reflRay).o).z = (hitPoint).z; }; { ((reflRay).d).x = (newDir).x; ((reflRay).d).y = (newDir).y; ((reflRay).d).z = (newDir).z; }; };
   int into = (((normal).x * (nl).x + (normal).y * (nl).y + (normal).z * (nl).z) > 0);

   float nc = 1.f;
   float nt = 1.5f;
   float nnt = into ? nc / nt : nt / nc;
   float ddn = ((currentRay->d).x * (nl).x + (currentRay->d).y * (nl).y + (currentRay->d).z * (nl).z);
   float cos2t = 1.f - nnt * nnt * (1.f - ddn * ddn);

   if (cos2t < 0.f) {
    { (throughput)->x = (throughput)->x * (col).x; (throughput)->y = (throughput)->y * (col).y; (throughput)->z = (throughput)->z * (col).z; };

    { { ((currentRay)->o).x = ((reflRay).o).x; ((currentRay)->o).y = ((reflRay).o).y; ((currentRay)->o).z = ((reflRay).o).z; }; { ((currentRay)->d).x = ((reflRay).d).x; ((currentRay)->d).y = ((reflRay).d).y; ((currentRay)->d).z = ((reflRay).d).z; }; };
    
	return;
   }

   float kk = (into ? 1 : -1) * (ddn * nnt + sqrt(cos2t));
   Vec nkk;
   { float k = (kk); { (nkk).x = k * (normal).x; (nkk).y = k * (normal).y; (nkk).z = k * (normal).z; } };
   Vec transDir;
   { float k = (nnt); { (transDir).x = k * (currentRay->d).x; (transDir).y = k * (currentRay->d).y; (transDir).z = k * (currentRay->d).z; } };
   { (transDir).x = (transDir).x - (nkk).x; (transDir).y = (transDir).y - (nkk).y; (transDir).z = (transDir).z - (nkk).z; };
   { float l = 1.f / sqrt(((transDir).x * (transDir).x + (transDir).y * (transDir).y + (transDir).z * (transDir).z)); { float k = (l); { (transDir).x = k * (transDir).x; (transDir).y = k * (transDir).y; (transDir).z = k * (transDir).z; } }; };

   float a = nt - nc;
   float b = nt + nc;
   float R0 = a * a / (b * b);
   float c = 1 - (into ? -ddn : ((transDir).x * (normal).x + (transDir).y * (normal).y + (transDir).z * (normal).z));

   float Re = R0 + (1 - R0) * c * c * c * c*c;
   float Tr = 1.f - Re;
   float P = .25f + .5f * Re;
   float RP = Re / P;
   float TP = Tr / (1.f - P);

   if (GetRandom(seed0, seed1) < P) {
    { float k = (RP); { (throughput)->x = k * (throughput)->x; (throughput)->y = k * (throughput)->y; (throughput)->z = k * (throughput)->z; } };
    { (throughput)->x = (throughput)->x * (col).x; (throughput)->y = (throughput)->y * (col).y; (throughput)->z = (throughput)->z * (col).z; };

    { { ((currentRay)->o).x = ((reflRay).o).x; ((currentRay)->o).y = ((reflRay).o).y; ((currentRay)->o).z = ((reflRay).o).z; }; { ((currentRay)->d).x = ((reflRay).d).x; ((currentRay)->d).y = ((reflRay).d).y; ((currentRay)->d).z = ((reflRay).d).z; }; };
    
	return;
   } else {
    { float k = (TP); { (throughput)->x = k * (throughput)->x; (throughput)->y = k * (throughput)->y; (throughput)->z = k * (throughput)->z; } };
    { (throughput)->x = (throughput)->x * (col).x; (throughput)->y = (throughput)->y * (col).y; (throughput)->z = (throughput)->z * (col).z; };

    { { ((currentRay)->o).x = (hitPoint).x; ((currentRay)->o).y = (hitPoint).y; ((currentRay)->o).z = (hitPoint).z; }; { ((currentRay)->d).x = (transDir).x; ((currentRay)->d).y = (transDir).y; ((currentRay)->d).z = (transDir).z; }; };
    
	return;
   }
 }
}

void RadiancePathTracing(

__constant

 const Shape *shapes,
 const unsigned int shapeCnt,
 
__constant

 Poi *pois,
 const unsigned int poiCnt,
#if (ACCELSTR == 1)
__constant

 BVHTreeNode *btn, 
__constant

 BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
__constant 
 
 KDNodeGPU *kng,
__constant 

 int *kn, 
 int szkng, int szkn, 
#endif
 const Ray *startRay,
 unsigned int *seed0, unsigned int *seed1,
 Vec *result
#ifdef DEBUG_INTERSECTIONS
 , __global int *debug1,
 __global float *debug2
#endif
 ) {
 Ray currentRay; { { ((currentRay).o).x = ((*startRay).o).x; ((currentRay).o).y = ((*startRay).o).y; ((currentRay).o).z = ((*startRay).o).z; }; { ((currentRay).d).x = ((*startRay).d).x; ((currentRay).d).y = ((*startRay).d).y; ((currentRay).d).z = ((*startRay).d).z; }; };
 Vec rad; { (rad).x = 0.f; (rad).y = 0.f; (rad).z = 0.f; };
 Vec throughput; { (throughput).x = 1.f; (throughput).y = 1.f; (throughput).z = 1.f; };

 unsigned int depth = 0;
 int specularBounce = 1;
 int terminated = 0;
 
 for (;; ++depth) {  
  if (depth > 6) {
   *result = rad;
   return;
  }

	RadianceOnePathTracing(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
		btn, btl, 
#elif (ACCELSTR == 2)
		kng, kn, szkng, szkn, 
#endif
		&currentRay, seed0, seed1, depth, &rad, &throughput, &specularBounce, &terminated, result
#ifdef DEBUG_INTERSECTIONS
		, debug1, debug2
#endif
);

	if (terminated == 1) {
		*result = rad;
		return;
	}
 }
}

void RadianceDirectLighting(

__constant

 const Shape *shapes,
 const unsigned int shapeCnt,
 
__constant

 Poi *pois,
 const unsigned int poiCnt,
#if (ACCELSTR == 1)
__constant

 BVHTreeNode *btn, 
__constant

 BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
__constant 
 
 KDNodeGPU *kng,
__constant 

 int *kn, 
 int szkng, int szkn, 
#endif
 const Ray *startRay,
 unsigned int *seed0, unsigned int *seed1,
 Vec *result
#ifdef DEBUG_INTERSECTIONS
 , __global int *debug1,
 __global float *debug2
#endif
 ) {
 Ray currentRay; { { ((currentRay).o).x = ((*startRay).o).x; ((currentRay).o).y = ((*startRay).o).y; ((currentRay).o).z = ((*startRay).o).z; }; { ((currentRay).d).x = ((*startRay).d).x; ((currentRay).d).y = ((*startRay).d).y; ((currentRay).d).z = ((*startRay).d).z; }; };
 Vec rad; { (rad).x = 0.f; (rad).y = 0.f; (rad).z = 0.f; };
 Vec throughput; { (throughput).x = 1.f; (throughput).y = 1.f; (throughput).z = 1.f; };

 unsigned int depth = 0;
 int specularBounce = 1;
 for (;; ++depth) {

  if (depth > MAX_DEPTH) {
   *result = rad;
   return;
  }

  float t;
  unsigned int id = 0;
  if (!Intersect(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
  btn, btl, 
#elif (ACCELSTR == 2)
  kng, kn, szkng, szkn, 
#endif
  &currentRay, &t, &id
#ifdef DEBUG_INTERSECTIONS
  , debug1, debug2
#endif
  )) {
   *result = rad;

   return;
  }

__constant

  const Sphere *obj = &shapes[id].s;

  Vec hitPoint;
  { float k = (t); { (hitPoint).x = k * (currentRay.d).x; (hitPoint).y = k * (currentRay.d).y; (hitPoint).z = k * (currentRay.d).z; } };
  { (hitPoint).x = (currentRay.o).x + (hitPoint).x; (hitPoint).y = (currentRay.o).y + (hitPoint).y; (hitPoint).z = (currentRay.o).z + (hitPoint).z; };

  Vec normal;
  { (normal).x = (hitPoint).x - (obj->p).x; (normal).y = (hitPoint).y - (obj->p).y; (normal).z = (hitPoint).z - (obj->p).z; };
  { float l = 1.f / sqrt(((normal).x * (normal).x + (normal).y * (normal).y + (normal).z * (normal).z)); { float k = (l); { (normal).x = k * (normal).x; (normal).y = k * (normal).y; (normal).z = k * (normal).z; } }; };

  const float dp = ((normal).x * (currentRay.d).x + (normal).y * (currentRay.d).y + (normal).z * (currentRay.d).z);

  Vec nl;

  const float invSignDP = -1.f * sign(dp);
  { float k = (invSignDP); { (nl).x = k * (normal).x; (nl).y = k * (normal).y; (nl).z = k * (normal).z; } };

  Vec eCol; { (eCol).x = (obj->e).x; (eCol).y = (obj->e).y; (eCol).z = (obj->e).z; };
  if (!(((eCol).x == 0.f) && ((eCol).x == 0.f) && ((eCol).z == 0.f))) {
   if (specularBounce) {
    { float k = (fabs(dp)); { (eCol).x = k * (eCol).x; (eCol).y = k * (eCol).y; (eCol).z = k * (eCol).z; } };
    { (eCol).x = (throughput).x * (eCol).x; (eCol).y = (throughput).y * (eCol).y; (eCol).z = (throughput).z * (eCol).z; };
    { (rad).x = (rad).x + (eCol).x; (rad).y = (rad).y + (eCol).y; (rad).z = (rad).z + (eCol).z; };
   }

   *result = rad;
   return;
  }

  if (obj->refl == DIFF) {
   specularBounce = 0;
   { (throughput).x = (throughput).x * (obj->c).x; (throughput).y = (throughput).y * (obj->c).y; (throughput).z = (throughput).z * (obj->c).z; };

   Vec Ld;
   SampleLights(shapes, shapeCnt, seed0, seed1, &hitPoint, &nl, &Ld);
   { (Ld).x = (throughput).x * (Ld).x; (Ld).y = (throughput).y * (Ld).y; (Ld).z = (throughput).z * (Ld).z; };
   { (rad).x = (rad).x + (Ld).x; (rad).y = (rad).y + (Ld).y; (rad).z = (rad).z + (Ld).z; };

   *result = rad;
   return;
  } else if (obj->refl == SPEC) {
   specularBounce = 1;

   Vec newDir;
   { float k = (2.f * ((normal).x * (currentRay.d).x + (normal).y * (currentRay.d).y + (normal).z * (currentRay.d).z)); { (newDir).x = k * (normal).x; (newDir).y = k * (normal).y; (newDir).z = k * (normal).z; } };
   { (newDir).x = (currentRay.d).x - (newDir).x; (newDir).y = (currentRay.d).y - (newDir).y; (newDir).z = (currentRay.d).z - (newDir).z; };

   { (throughput).x = (throughput).x * (obj->c).x; (throughput).y = (throughput).y * (obj->c).y; (throughput).z = (throughput).z * (obj->c).z; };

   { { ((currentRay).o).x = (hitPoint).x; ((currentRay).o).y = (hitPoint).y; ((currentRay).o).z = (hitPoint).z; }; { ((currentRay).d).x = (newDir).x; ((currentRay).d).y = (newDir).y; ((currentRay).d).z = (newDir).z; }; };
   continue;
  } else {
   specularBounce = 1;

   Vec newDir;
   { float k = (2.f * ((normal).x * (currentRay.d).x + (normal).y * (currentRay.d).y + (normal).z * (currentRay.d).z)); { (newDir).x = k * (normal).x; (newDir).y = k * (normal).y; (newDir).z = k * (normal).z; } };
   { (newDir).x = (currentRay.d).x - (newDir).x; (newDir).y = (currentRay.d).y - (newDir).y; (newDir).z = (currentRay.d).z - (newDir).z; };

   Ray reflRay; { { ((reflRay).o).x = (hitPoint).x; ((reflRay).o).y = (hitPoint).y; ((reflRay).o).z = (hitPoint).z; }; { ((reflRay).d).x = (newDir).x; ((reflRay).d).y = (newDir).y; ((reflRay).d).z = (newDir).z; }; };
   int into = (((normal).x * (nl).x + (normal).y * (nl).y + (normal).z * (nl).z) > 0);

   float nc = 1.f;
   float nt = 1.5f;
   float nnt = into ? nc / nt : nt / nc;
   float ddn = ((currentRay.d).x * (nl).x + (currentRay.d).y * (nl).y + (currentRay.d).z * (nl).z);
   float cos2t = 1.f - nnt * nnt * (1.f - ddn * ddn);

   if (cos2t < 0.f) {
    { (throughput).x = (throughput).x * (obj->c).x; (throughput).y = (throughput).y * (obj->c).y; (throughput).z = (throughput).z * (obj->c).z; };

    { { ((currentRay).o).x = ((reflRay).o).x; ((currentRay).o).y = ((reflRay).o).y; ((currentRay).o).z = ((reflRay).o).z; }; { ((currentRay).d).x = ((reflRay).d).x; ((currentRay).d).y = ((reflRay).d).y; ((currentRay).d).z = ((reflRay).d).z; }; };
    continue;
   }

   float kk = (into ? 1 : -1) * (ddn * nnt + sqrt(cos2t));
   Vec nkk;
   { float k = (kk); { (nkk).x = k * (normal).x; (nkk).y = k * (normal).y; (nkk).z = k * (normal).z; } };
   Vec transDir;
   { float k = (nnt); { (transDir).x = k * (currentRay.d).x; (transDir).y = k * (currentRay.d).y; (transDir).z = k * (currentRay.d).z; } };
   { (transDir).x = (transDir).x - (nkk).x; (transDir).y = (transDir).y - (nkk).y; (transDir).z = (transDir).z - (nkk).z; };
   { float l = 1.f / sqrt(((transDir).x * (transDir).x + (transDir).y * (transDir).y + (transDir).z * (transDir).z)); { float k = (l); { (transDir).x = k * (transDir).x; (transDir).y = k * (transDir).y; (transDir).z = k * (transDir).z; } }; };

   float a = nt - nc;
   float b = nt + nc;
   float R0 = a * a / (b * b);
   float c = 1 - (into ? -ddn : ((transDir).x * (normal).x + (transDir).y * (normal).y + (transDir).z * (normal).z));

   float Re = R0 + (1 - R0) * c * c * c * c*c;
   float Tr = 1.f - Re;
   float P = .25f + .5f * Re;
   float RP = Re / P;
   float TP = Tr / (1.f - P);

   if (GetRandom(seed0, seed1) < P) {
    { float k = (RP); { (throughput).x = k * (throughput).x; (throughput).y = k * (throughput).y; (throughput).z = k * (throughput).z; } };
    { (throughput).x = (throughput).x * (obj->c).x; (throughput).y = (throughput).y * (obj->c).y; (throughput).z = (throughput).z * (obj->c).z; };

    { { ((currentRay).o).x = ((reflRay).o).x; ((currentRay).o).y = ((reflRay).o).y; ((currentRay).o).z = ((reflRay).o).z; }; { ((currentRay).d).x = ((reflRay).d).x; ((currentRay).d).y = ((reflRay).d).y; ((currentRay).d).z = ((reflRay).d).z; }; };
    continue;
   } else {
    { float k = (TP); { (throughput).x = k * (throughput).x; (throughput).y = k * (throughput).y; (throughput).z = k * (throughput).z; } };
    { (throughput).x = (throughput).x * (obj->c).x; (throughput).y = (throughput).y * (obj->c).y; (throughput).z = (throughput).z * (obj->c).z; };

    { { ((currentRay).o).x = (hitPoint).x; ((currentRay).o).y = (hitPoint).y; ((currentRay).o).z = (hitPoint).z; }; { ((currentRay).d).x = (transDir).x; ((currentRay).d).y = (transDir).y; ((currentRay).d).z = (transDir).z; }; };
    continue;
   }
  }
 }
}
# 28 "<stdin>" 2

void GenerateCameraRay(
 __constant  Camera *camera,
  unsigned int *seed0, unsigned int *seed1,
  const int width, const int height, const int x, const int y, Ray *ray) {
 const float invWidth = 1.f / width;
 const float invHeight = 1.f / height;
 const float r1 = GetRandom(seed0, seed1) - .5f;
 const float r2 = GetRandom(seed0, seed1) - .5f;
 const float kcx = (x + r1) * invWidth - .5f;
 const float kcy = (y + r2) * invHeight - .5f;

 Vec rdir;
 { (rdir).x = camera->x.x * kcx + camera->y.x * kcy + camera->dir.x; (rdir).y = camera->x.y * kcx + camera->y.y * kcy + camera->dir.y; (rdir).z = camera->x.z * kcx + camera->y.z * kcy + camera->dir.z; };

 Vec rorig;
 { float k = (0.1f); { (rorig).x = k * (rdir).x; (rorig).y = k * (rdir).y; (rorig).z = k * (rdir).z; } };
 { (rorig).x = (rorig).x + (camera->orig).x; (rorig).y = (rorig).y + (camera->orig).y; (rorig).z = (rorig).z + (camera->orig).z; }

 { float l = 1.f / sqrt(((rdir).x * (rdir).x + (rdir).y * (rdir).y + (rdir).z * (rdir).z)); { float k = (l); { (rdir).x = k * (rdir).x; (rdir).y = k * (rdir).y; (rdir).z = k * (rdir).z; } }; };
 { { ((*ray).o).x = (rorig).x; ((*ray).o).y = (rorig).y; ((*ray).o).z = (rorig).z; }; { ((*ray).d).x = (rdir).x; ((*ray).d).y = (rdir).y; ((*ray).d).z = (rdir).z; }; };
}

__kernel void RadianceGPU(
    __global Vec *colors, __global unsigned int *seedsInput,
 __constant Shape *shapes, 
 __constant Camera *camera,
 const unsigned int shapeCnt,
 __constant Poi *pois,
 const unsigned int poiCnt,
#if (ACCELSTR == 1)
 __constant BVHTreeNode *btn, 
 __constant BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
__constant
 
 KDNodeGPU *kng, 
__constant

 int *kn, 
#endif
 const int width, const int height,
 const int currentSample,
 __global int *pixels
#ifdef DEBUG_INTERSECTIONS
 , __global int *debug1,
 __global float *debug2
#endif 
#if (ACCELSTR == 2)
 , int szkng, int szkn
#endif
 ) {
    const int gid = get_global_id(0);
 const int gid2 = gid << 1;
 const int x = gid % width;
 const int y = gid / width;

 if (y >= height)
  return;

 unsigned int seed0 = seedsInput[gid2];
 unsigned int seed1 = seedsInput[gid2 + 1];

 Ray ray;
 GenerateCameraRay(camera, &seed0, &seed1, width, height, x, y, &ray);

 Vec r;
 RadiancePathTracing(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
 btn, btl, 
#elif (ACCELSTR == 2)
 kng, kn, szkng, szkn, 
#endif
 &ray, &seed0, &seed1, &r
#ifdef DEBUG_INTERSECTIONS
 , debug1, debug2
#endif
 );

 const int i = (height - y - 1) * width + x;
 if (currentSample == 0) {
  { (colors[i]).x = (r).x; (colors[i]).y = (r).y; (colors[i]).z = (r).z; };
 } else {
  const float k1 = currentSample;
  const float k2 = 1.f / (currentSample + 1.f);
  colors[i].x = (colors[i].x * k1 + r.x) * k2;
  colors[i].y = (colors[i].y * k1 + r.y) * k2;
  colors[i].z = (colors[i].z * k1 + r.z) * k2;
 }

 pixels[y * width + x] = ((int)(pow(clamp(colors[i].x, 0.f, 1.f), 1.f / 2.2f) * 255.f + .5f)) |
   (((int)(pow(clamp(colors[i].y, 0.f, 1.f), 1.f / 2.2f) * 255.f + .5f)) << 8) |
   (((int)(pow(clamp(colors[i].z, 0.f, 1.f), 1.f / 2.2f) * 255.f + .5f)) << 16) | 0xff000000;

 seedsInput[gid2] = seed0;
 seedsInput[gid2 + 1] = seed1;
}

__kernel void RadianceBoxGPU(
    __global Vec *colors, __global unsigned int *seedsInput,
 __constant Shape *shapes, 
 __constant Camera *camera,
 const unsigned int shapeCnt,
 __constant Poi *pois,
 const unsigned int poiCnt,
#if (ACCELSTR == 1)
 __constant BVHTreeNode *btn, 
 __constant BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
 __constant KDNodeGPU *kng,
 __constant int *kn, 
#endif
 const int x, const int y,
 const int bwidth, const int bheight,
 const int twidth, const int theight,
 const int currentSample,
 __global int *pixels
#ifdef DEBUG_INTERSECTIONS
 , __global int *debug1,
 __global float *debug2
#endif 
#if (ACCELSTR == 2)
 , int szkngbuf, int szknbuf
#endif
 ) {
    const int gid = get_global_id(0);
	
 const int xwithinbox = gid % bwidth;
 const int ywithinbox = gid / bwidth;
 
 const int sgid = (y + ywithinbox - 1) * twidth + (x + xwithinbox);
 const int sgid2 = sgid << 1;
 
 if (y + ywithinbox >= theight)
  return;
  
 unsigned int seed0 = seedsInput[sgid2];
 unsigned int seed1 = seedsInput[sgid2 + 1];

 Ray ray;
 GenerateCameraRay(camera, &seed0, &seed1, twidth, theight, x + xwithinbox, y + ywithinbox, &ray);

 Vec r;
 RadiancePathTracing(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
 btn, btl, 
#elif (ACCELSTR == 2)
 kng, kn, szkngbuf, szknbuf, 
#endif
 &ray, &seed0, &seed1, &r
#ifdef DEBUG_INTERSECTIONS
 , debug1, debug2
#endif
 );

 const int i = (theight - y - ywithinbox - 1) * twidth + x + xwithinbox;
 if (currentSample == 0) {

  { (colors[i]).x = (r).x; (colors[i]).y = (r).y; (colors[i]).z = (r).z; };
 } else {
  const float k1 = currentSample;
  const float k2 = 1.f / (currentSample + 1.f);
  colors[i].x = (colors[i].x * k1 + r.x) * k2;
  colors[i].y = (colors[i].y * k1 + r.y) * k2;
  colors[i].z = (colors[i].z * k1 + r.z) * k2;
 }

 pixels[(y + ywithinbox) * twidth + x + xwithinbox] = ((int)(pow(clamp(colors[i].x, 0.f, 1.f), 1.f / 2.2f) * 255.f + .5f)) |
   (((int)(pow(clamp(colors[i].y, 0.f, 1.f), 1.f / 2.2f) * 255.f + .5f)) << 8) |
   (((int)(pow(clamp(colors[i].z, 0.f, 1.f), 1.f / 2.2f) * 255.f + .5f)) << 16) | 0xff000000;

 seedsInput[sgid2] = seed0;
 seedsInput[sgid2 + 1] = seed1;
}