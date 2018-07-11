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

#ifndef _GEOM_H
#define	_GEOM_H

#include "vec.h"

#define EPSILON 0.01f
#define FLOAT_PI 3.14159265358979323846f

#define MAX_DEPTH 6
#define MAX_SPP 6

#define ACCELSTR 2 //2 //0 is no accel, 1 is BVH and 2 is KDTREE
//#define EXP_KERNEL
//#define CPU_PARTRENDERING
//#define DEBUG_INTERSECTIONS

typedef struct {
	Vec o, d;
} Ray;

#define rinit(r, a, b) { vassign((r).o, a); vassign((r).d, b); }
#define rassign(a, b) { vassign((a).o, (b).o); vassign((a).d, (b).d); }

enum Refl {
	DIFF, SPEC, REFR
};

typedef struct {
	float rad;
	Vec p;
} Sphere;

typedef struct {
	Vec p, n, c;	/* position, normal, emission, color */
} Poi;

typedef struct {
	int p1, p2, p3;
} Triangle;

typedef enum tagType { SPHERE, TRIANGLE } Type;
typedef struct { float min_x, max_x, min_y, max_y, min_z, max_z; } Bound;

typedef struct {
	Type type;

	union
	{
		Triangle t;
		Sphere s;
	};
	Vec e, c; /* emission, color */
	enum Refl refl;

	int index;
	unsigned int morton_code;
	Bound b;
	float area;
} Shape;

#endif	/* _GEOM_H */
