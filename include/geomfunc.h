/*
Copyright (c) 2009 David Bucciarelli (davibu@interfree.it)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _GEOMFUNC_H
#define	_GEOMFUNC_H

#include "geom.h"
#include "simplernd.h"

#ifndef SMALLPT_GPU
#define INTERSECT_STACK_SIZE (18)
#define MTEPSILON 0.000001

inline void swap(float *a, float *b)
{
	float temp = *a;
	*a = *b;
	*b = temp;
}

float SphereIntersect(
#ifdef GPU_KERNEL
__constant
#endif
	const Sphere *s,
	const Ray *r) { /* returns distance, 0 if nohit */
	Vec op; /* Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0 */
	vsub(op, s->p, r->o);

	float b = vdot(op, r->d);
	float det = b * b - vdot(op, op) + s->rad * s->rad;
	if (det < 0.f)
		return 0.f;
	else
		det = sqrt(det);

	float t = b - det;
	if (t >  EPSILON)
		return t;
	else {
		t = b + det;

		if (t >  EPSILON)
			return t;
		else
			return 0.f;
	}
}


//Written in the paper "Fast, Minimum Storage Ray/ Triangle Intersection".
float TriangleIntersect(
#ifdef GPU_KERNEL
	__constant
#endif
	const Triangle *tr,
#ifdef GPU_KERNEL
	__constant
#endif
	const Poi *pois,
	const unsigned int poiCnt,
	const Ray *r) { /* returns distance, 0 if nohit */
	Vec v0 = pois[tr->p1].p, v1 = pois[tr->p2].p, v2 = pois[tr->p3].p, e1, e2, tvec, pvec, qvec;
	float t = 0.0f, u, v;
	double det, inv_det;

	vsub(e1, v1, v0);
	vsub(e2, v2, v0);

	vxcross(pvec, r->d, e2);
	det = vdot(e1, pvec);
#ifdef TEST_CULL
	if (det < MTEPSILON) return 0.0f;

	vsub(tvec, r->o, v0);
	u = vdot(tvec, pvec);
	if (u < 0.0 || u > det) return 0.0f;

	vxcross(qvec, tvec, e1);
	v = vdot(r->d, qvec);
	if (v < 0.0 || u + v > det) return 0.0f;

	t = vdot(e2, qvec);
	inv_det = 1.0 / det;

	t *= inv_det;
	u *= inv_det;
	v *= inv_det;
#else
	if (det > -MTEPSILON && det < MTEPSILON) return 0.0f;

	inv_det = 1.0 / det;

	vsub(tvec, r->o, v0);
	u = vdot(tvec, pvec) * inv_det;
	if (u < 0.0 || u > 1.0) return 0.0f;

	vxcross(qvec, tvec, e1);
	v = vdot(r->d, qvec) * inv_det;
	if (v < 0.0 || u + v > 1.0) return 0.0f;

	t = vdot(e2, qvec) * inv_det;
#endif
	return t;
}

void UniformSampleSphere(const float u1, const float u2, Vec *v) {
	const float zz = 1.f - 2.f * u1;
	const float r = sqrt(max(0.f, 1.f - zz * zz));
	const float phi = 2.f * FLOAT_PI * u2;
	const float xx = r * cos(phi);
	const float yy = r * sin(phi);

	vinit(*v, xx, yy, zz);
}

inline float min2(float a, float b)
{
	return (a < b) ? a : b;
}

inline float max2(float a, float b)
{
	return (a > b) ? a : b;
}

bool intersection_bound_test(const Ray r, Bound bound) {
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

	return (t_min <= t_max);
}

int Intersect(
#ifdef GPU_KERNEL
__constant
#endif
	const Shape *shapes,
	const unsigned int shapeCnt,
#ifdef GPU_KERNEL
	__constant
#endif
	const Poi *pois,
	const unsigned int poiCnt,
#if (ACCELSTR == 1)
#ifdef GPU_KERNEL
	__constant
#endif
	BVHTreeNode *tn,
#ifdef GPU_KERNEL
	__constant
#endif
	BVHTreeNode *tl, 
#elif (ACCELSTR == 2)
#ifdef GPU_KERNEL
	__constant
#endif
	KDNodeGPU *kng,
#ifdef GPU_KERNEL
	__constant
#endif
	int *kn,
	int szkng, int szkn,
#endif
#ifdef GPU_KERNEL
	__constant
#endif
	const Ray *r,
	float *t,
	unsigned int *id) {
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

	return (*t < inf);
#elif (ACCELSTR == 1)
	(*t) = 1e20f;
	
    // Use static allocation because malloc() can't be called in parallel
    // Use stack to traverse BVH to save space (cost is O(height))
    int stack[INTERSECT_STACK_SIZE];
    int topIndex = INTERSECT_STACK_SIZE;
    stack[--topIndex] = 0; //tn;
    int intersected = 0, status = 0;

    // Do while stack is not empty
    while (topIndex != INTERSECT_STACK_SIZE) {
        int n = stack[topIndex++];
		
        if (intersection_bound_test(*r, tn[n].bound)) {
			if (tn[n].leaf == 0) {				
                stack[--topIndex] = tn[n].nRight;
                stack[--topIndex] = tn[n].nLeft;

                if (topIndex < 0) {
                    //printf("Intersect stack not big enough. Increase INTERSECT_STACK_SIZE!\n");
                    return 0;
                }
			}
			else if (tn[n].leaf == 2) {
                double d = 0.0f;

			    if (shapes[tl[tn[n].nLeft].nShape].type == SPHERE ) d = SphereIntersect(&shapes[tl[tn[n].nLeft].nShape].s, r);
				if (shapes[tl[tn[n].nLeft].nShape].type == TRIANGLE ) d = TriangleIntersect(&shapes[tl[tn[n].nLeft].nShape].t, pois, poiCnt, r);

                if (d != 0.0) {
                    if (d < *t) {
                        *t = d;
                        *id = tl[tn[n].nLeft].nShape;
                    }
                    intersected = 1;
                }
				
			    if (shapes[tl[tn[n].nRight].nShape].type == SPHERE ) d = SphereIntersect(&shapes[tl[tn[n].nRight].nShape].s, r);
				if (shapes[tl[tn[n].nRight].nShape].type == TRIANGLE ) d = TriangleIntersect(&shapes[tl[tn[n].nRight].nShape].t, pois, poiCnt, r);

                if (d != 0.0) {
                    if (d < *t) {
                        *t = d;
                        *id = tl[tn[n].nRight].nShape;
                    }
                    intersected = 1;
                }				
			}
			else {
				//printf("Unknown node, %d\n", tn[n].leaf);
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

		if (intersection_bound_test(*r, kng[n].bound)) {
			if (kng[n].leaf == 0) {
				stack[--topIndex] = kng[n].nRight;
				stack[--topIndex] = kng[n].nLeft;

				if (topIndex < 0) {
					//printf("Intersect stack not big enough. Increase INTERSECT_STACK_SIZE!\n");
					return 0;
				}
			}
			else if (kng[n].leaf == 1) {
				double d = 0.0f;

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
				//printf("Unknown node, %d\n", tn[n].leaf);
			}
		}
	}

	return intersected;
#endif
}

int IntersectP(
#ifdef GPU_KERNEL
__constant
#endif
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
#ifdef GPU_KERNEL
__constant
#endif
	const Shape *shapes,
	const unsigned int shapeCnt,
	unsigned int *seed0, unsigned int *seed1,
	const Vec *hitPoint,
	const Vec *normal,
	Vec *result) {
	vclr(*result);

	/* For each light */
	unsigned int i;
	for (i = 0; i < shapeCnt; i++) {
		if (shapes[i].type == TRIANGLE) continue;
#ifdef GPU_KERNEL
__constant
#endif
		const Sphere *light = &shapes[i].s;
		if (!viszero(light->e)) {
			/* It is a light source */
			Ray shadowRay;
			shadowRay.o = *hitPoint;

			/* Choose a point over the light source */
			Vec unitSpherePoint;
			UniformSampleSphere(GetRandom(seed0, seed1), GetRandom(seed0, seed1), &unitSpherePoint);
			Vec spherePoint;
			vsmul(spherePoint, light->rad, unitSpherePoint);
			vadd(spherePoint, spherePoint, light->p);

			/* Build the shadow ray direction */
			vsub(shadowRay.d, spherePoint, *hitPoint);
			const float len = sqrt(vdot(shadowRay.d, shadowRay.d));
			vsmul(shadowRay.d, 1.f / len, shadowRay.d);

			float wo = vdot(shadowRay.d, unitSpherePoint);
			if (wo > 0.f) {
				/* It is on the other half of the sphere */
				continue;
			} else
				wo = -wo;

			/* Check if the light is visible */
			const float wi = vdot(shadowRay.d, *normal);
			if ((wi > 0.f) && (!IntersectP(shapes, shapeCnt, &shadowRay, len - EPSILON))) {
				Vec c; vassign(c, light->e);
				const float s = (4.f * FLOAT_PI * light->rad * light->rad) * wi * wo / (len *len);
				vsmul(c, s, c);
				vadd(*result, *result, c);
			}
		}
	}
}

void RadianceOnePathTracing(
#ifdef GPU_KERNEL
	__constant
#endif
	const Shape *shapes,
	const unsigned int shapeCnt,
#ifdef GPU_KERNEL
	__constant
#endif
	const Poi *pois,
	const unsigned int poiCnt,
#if (ACCELSTR == 1)
#ifdef GPU_KERNEL
	__constant
#endif
	BVHTreeNode *btn,
#ifdef GPU_KERNEL
	__constant
#endif	
	BVHTreeNode *btl,
#elif (ACCELSTR ==2)
#ifdef GPU_KERNEL
	__constant
#endif
	KDNodeGPU *kng,
#ifdef GPU_KERNEL
	__constant
#endif
	int *kn,
	int szkng, int szkn,
#endif
	Ray *currentRay,
	unsigned int *seed0, unsigned int *seed1, int depth, Vec *rad, Vec *throughput, int *specularBounce, int *terminated, 
	Vec *result) {
	float t; /* distance to intersection */
	unsigned int id = 0; /* id of intersected object */

	if (!Intersect(shapes, shapeCnt, pois, poiCnt,
#if (ACCELSTR == 1)
		btn, btl,
#elif (ACCELSTR == 2)
		kng, kn, szkng, szkn, 
#endif
		currentRay, &t, &id)) {
		*result = *rad;
		*terminated = 1;
		return;
	}

	Vec hitPoint;
	vsmul(hitPoint, t, currentRay->d);
	vadd(hitPoint, currentRay->o, hitPoint);

	Vec normal, col;
	enum Refl refl;
	Shape s = shapes[id];

	if (s.type == SPHERE)
	{
		vsub(normal, hitPoint, s.s.p);

		refl = s.s.refl;
		col = s.s.c;
	}
	else if (s.type == TRIANGLE)
	{
		Vec v0 = pois[s.t.p1].p, v1 = pois[s.t.p2].p, v2 = pois[s.t.p3].p, e1, e2;

		vsub(e1, v1, v0);
		vsub(e2, v2, v1);

		vxcross(normal, e1, e2);

		refl = s.t.refl;
		col.x = col.y = 0.0f;
		col.z = 1.0f;
	}
	vnorm(normal);

	const float dp = vdot(normal, currentRay->d);

	Vec nl;
	// SIMT optimization
	const float invSignDP = -1.f * sign(dp);

	vsmul(nl, invSignDP, normal);

	if (s.type == SPHERE)
	{
		/* Add emitted light */
		Vec eCol;

		vassign(eCol, s.s.e);
		if (!viszero(eCol)) {
			if (*specularBounce) {
				vsmul(eCol, fabs(dp), eCol);
				vmul(eCol, *throughput, eCol);
				vadd(*rad, *rad, eCol);
			}

			*result = *rad;
			*terminated = 1;

			return;
		}
	}

	if (refl == DIFF) { /* Ideal DIFFUSE reflection */
		*specularBounce = 0;

		vmul(*throughput, *throughput, col);

		/* Direct lighting component */
		Vec Ld;

		SampleLights(shapes, shapeCnt, seed0, seed1, &hitPoint, &nl, &Ld);
		vmul(Ld, *throughput, Ld);
		vadd(*rad, *rad, Ld);

		/* Diffuse component */
		float r1 = 2.f * FLOAT_PI * GetRandom(seed0, seed1);
		float r2 = GetRandom(seed0, seed1);
		float r2s = sqrt(r2);

		Vec w; 
		vassign(w, nl);

		Vec u, a;
		if (fabs(w.x) > .1f) {
			vinit(a, 0.f, 1.f, 0.f);
		}
		else {
			vinit(a, 1.f, 0.f, 0.f);
		}
		vxcross(u, a, w);
		vnorm(u);

		Vec v, newDir;
		vxcross(v, w, u);

		vsmul(u, cos(r1) * r2s, u);
		vsmul(v, sin(r1) * r2s, v);
		vadd(newDir, u, v);
		vsmul(w, sqrt(1 - r2), w);
		vadd(newDir, newDir, w);

		currentRay->o = hitPoint;
		currentRay->d = newDir;

		return;
	}
	else if (refl == SPEC) { /* Ideal SPECULAR reflection */
		*specularBounce = 1;

		Vec newDir;

		vsmul(newDir, 2.f * vdot(normal, currentRay->d), normal);
		vsub(newDir, currentRay->d, newDir);

		vmul(*throughput, *throughput, col);
		rinit(*currentRay, hitPoint, newDir);

		return;
	}
	else {
		*specularBounce = 1;

		Vec newDir;
		vsmul(newDir, 2.f * vdot(normal, currentRay->d), normal);
		vsub(newDir, currentRay->d, newDir);

		Ray reflRay; rinit(reflRay, hitPoint, newDir); /* Ideal dielectric REFRACTION */
		int into = (vdot(normal, nl) > 0); /* Ray from outside going in? */

		float nc = 1.f;
		float nt = 1.5f;
		float nnt = into ? nc / nt : nt / nc;
		float ddn = vdot(currentRay->d, nl);
		float cos2t = 1.f - nnt * nnt * (1.f - ddn * ddn);

		if (cos2t < 0.f) { /* Total internal reflection */
			vmul(*throughput, *throughput, col);
			rassign(*currentRay, reflRay);

			return;
		}

		float kk = (into ? 1 : -1) * (ddn * nnt + sqrt(cos2t));

		Vec nkk, transDir;
		vsmul(nkk, kk, normal);

		vsmul(transDir, nnt, currentRay->d);
		vsub(transDir, transDir, nkk);
		vnorm(transDir);

		float a = nt - nc;
		float b = nt + nc;
		float R0 = a * a / (b * b);
		float c = 1 - (into ? -ddn : vdot(transDir, normal));

		float Re = R0 + (1 - R0) * c * c * c * c*c;
		float Tr = 1.f - Re;
		float P = .25f + .5f * Re;
		float RP = Re / P;
		float TP = Tr / (1.f - P);

		if (GetRandom(seed0, seed1) < P) { /* R.R. */
			vsmul(*throughput, RP, *throughput);
			vmul(*throughput, *throughput, col);
			rassign(*currentRay, reflRay);

			return;
		}
		else {
			vsmul(*throughput, TP, *throughput);
			vmul(*throughput, *throughput, col);
			rinit(*currentRay, hitPoint, transDir);

			return;
		}
	}
}

void RadiancePathTracing(
#ifdef GPU_KERNEL
	__constant
#endif
	const Shape *shapes,
	const unsigned int shapeCnt,
#ifdef GPU_KERNEL
	__constant
#endif
	const Poi *pois,
	const unsigned int poiCnt,
#if (ACCELSTR == 1)
#ifdef GPU_KERNEL
	__constant
#endif
	BVHTreeNode *btn,
#ifdef GPU_KERNEL
	__constant
#endif	
	BVHTreeNode *btl, 
#elif (ACCELSTR == 2)
#ifdef GPU_KERNEL
	__constant
#endif
	KDNodeGPU *kng,
#ifdef GPU_KERNEL
	__constant
#endif
	int *kn,
	int szkng, int szkn,
#endif
	const Ray *startRay,
	unsigned int *seed0, unsigned int *seed1,
	Vec *result) {
	Ray currentRay; rassign(currentRay, *startRay);
	Vec rad; vinit(rad, 0.f, 0.f, 0.f);
	Vec throughput; vinit(throughput, 1.f, 1.f, 1.f);

	unsigned int depth = 0;
	int specularBounce = 1;
	int terminated = 0;

	for (;; ++depth) {
		if (depth > MAX_DEPTH) {
			*result = rad;
			return;
		}

		RadianceOnePathTracing(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
			btn, btl, 
#elif (ACCELSTR == 2)
			kng, kn, szkng, szkn, 
#endif
			&currentRay, seed0, seed1, depth, &rad, &throughput, &specularBounce, &terminated, result);

		if (terminated == 1) {
			*result = rad;
			return;
		}
	}	
}

void RadianceDirectLighting(
#ifdef GPU_KERNEL
	__constant
#endif
	const Shape *shapes,
	const unsigned int shapeCnt,
#ifdef GPU_KERNEL
	__constant
#endif
	const Poi *pois,
	const unsigned int poiCnt,
#if (ACCELSTR == 1)
#ifdef GPU_KERNEL
	__constant
#endif
	BVHTreeNode *btn,
#ifdef GPU_KERNEL
	__constant
#endif
	BVHTreeNode *btl,
#elif (ACCELSTR == 2)
#ifdef GPU_KERNEL
	__constant
#endif
	KDNodeGPU *kng,
#ifdef GPU_KERNEL
	__constant
#endif
	int *kn,
	int szkng, int szkn,
#endif
#ifdef GPU_KERNEL
	__constant
#endif
	const Ray *startRay,
	unsigned int *seed0, unsigned int *seed1,
	Vec *result, KDTreeNode *node) {
	Ray currentRay; rassign(currentRay, *startRay);
	Vec rad; vinit(rad, 0.f, 0.f, 0.f);
	Vec throughput; vinit(throughput, 1.f, 1.f, 1.f);

	unsigned int depth = 0;
	int specularBounce = 1;
	for (;; ++depth) {
		// Removed Russian Roulette in order to improve execution on SIMT
		if (depth > 6) {
			*result = rad;
			return;
		}

		float t; /* distance to intersection */
		unsigned int id = 0; /* id of intersected object */
		if (!Intersect(shapes, shapeCnt, pois, poiCnt, 
#if (ACCELSTR == 1)
			btn, btl,
#elif (ACCELSTR == 2)
			kng, kn, szkng, szkn, 
#endif
			&currentRay, &t, &id)) {
			*result = rad; /* if miss, return */
			return;
		}

#ifdef GPU_KERNEL
OCL_CONSTANT_BUFFER
#endif
		const Sphere *obj = &shapes[id].s; /* the hit object */

		Vec hitPoint;
		vsmul(hitPoint, t, currentRay.d);
		vadd(hitPoint, currentRay.o, hitPoint);

		Vec normal;
		vsub(normal, hitPoint, obj->p);
		vnorm(normal);

		const float dp = vdot(normal, currentRay.d);

		Vec nl;
		// SIMT optimization
		const float invSignDP = -1.f * sign(dp);
		vsmul(nl, invSignDP, normal);

		/* Add emitted light */
		Vec eCol; vassign(eCol, obj->e);
		if (!viszero(eCol)) {
			if (specularBounce) {
				vsmul(eCol, fabs(dp), eCol);
				vmul(eCol, throughput, eCol);
				vadd(rad, rad, eCol);
			}

			*result = rad;
			return;
		}

		if (obj->refl == DIFF) { /* Ideal DIFFUSE reflection */
			specularBounce = 0;
			vmul(throughput, throughput, obj->c);

			/* Direct lighting component */

			Vec Ld;
			SampleLights(shapes, shapeCnt, seed0, seed1, &hitPoint, &nl, &Ld);
			vmul(Ld, throughput, Ld);
			vadd(rad, rad, Ld);

			*result = rad;
			return;
		} else if (obj->refl == SPEC) { /* Ideal SPECULAR reflection */
			specularBounce = 1;

			Vec newDir;
			vsmul(newDir,  2.f * vdot(normal, currentRay.d), normal);
			vsub(newDir, currentRay.d, newDir);

			vmul(throughput, throughput, obj->c);

			rinit(currentRay, hitPoint, newDir);
			continue;
		} else {
			specularBounce = 1;

			Vec newDir;
			vsmul(newDir,  2.f * vdot(normal, currentRay.d), normal);
			vsub(newDir, currentRay.d, newDir);

			Ray reflRay; rinit(reflRay, hitPoint, newDir); /* Ideal dielectric REFRACTION */
			int into = (vdot(normal, nl) > 0); /* Ray from outside going in? */

			float nc = 1.f;
			float nt = 1.5f;
			float nnt = into ? nc / nt : nt / nc;
			float ddn = vdot(currentRay.d, nl);
			float cos2t = 1.f - nnt * nnt * (1.f - ddn * ddn);

			if (cos2t < 0.f)  { /* Total internal reflection */
				vmul(throughput, throughput, obj->c);

				rassign(currentRay, reflRay);
				continue;
			}

			float kk = (into ? 1 : -1) * (ddn * nnt + sqrt(cos2t));
			Vec nkk;
			vsmul(nkk, kk, normal);
			Vec transDir;
			vsmul(transDir, nnt, currentRay.d);
			vsub(transDir, transDir, nkk);
			vnorm(transDir);

			float a = nt - nc;
			float b = nt + nc;
			float R0 = a * a / (b * b);
			float c = 1 - (into ? -ddn : vdot(transDir, normal));

			float Re = R0 + (1 - R0) * c * c * c * c*c;
			float Tr = 1.f - Re;
			float P = .25f + .5f * Re;
			float RP = Re / P;
			float TP = Tr / (1.f - P);

			if (GetRandom(seed0, seed1) < P) { /* R.R. */
				vsmul(throughput, RP, throughput);
				vmul(throughput, throughput, obj->c);

				rassign(currentRay, reflRay);
				continue;
			} else {
				vsmul(throughput, TP, throughput);
				vmul(throughput, throughput, obj->c);

				rinit(currentRay, hitPoint, transDir);
				continue;
			}
		}
	}
}

#endif

#endif	/* _GEOMFUNC_H */

