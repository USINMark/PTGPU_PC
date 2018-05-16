#pragma once

#include <float.h>
#ifdef WIN32
#include <stdint.h>
#else
#include <string.h>
#endif
#include <stdlib.h>

#include "include\\geom.h"
#include "include\\BVHTree.h"
#include "CL\\cl.h"
#include "include\\native-lib.h"

typedef struct
{
	unsigned int key_mc;
	int value;
} Geometry;

class CLBVH
{
private:
	cl_context m_ctx;
	cl_kernel m_kRad, m_kBvh, m_kOpt;
	cl_mem m_nBuf, m_lBuf, m_shBuf;
	cl_command_queue m_cq;

	BVHTreeNode *btn, *btl;
	
	unsigned int m_poiCnt, m_shapeCnt;
	Poi *m_pois;
	Shape *m_shapes;

	Bound getBound(Sphere s);
	Bound getBound(Triangle t);

public:
	CLBVH(Shape *shapes, int shapeCnt, Poi *pois, int poiCnt, cl_command_queue cq, cl_context ctx, cl_kernel kRad, cl_kernel kBvh, cl_kernel kOpt);
	~CLBVH();

	void buildRadixTree();
	void buildBVHTree();
	void optimize();

	void getTrees(BVHTreeNode **ppbtn, BVHTreeNode **ppbtl);
	int *bubblesort_Geometry(Geometry *geo);
};