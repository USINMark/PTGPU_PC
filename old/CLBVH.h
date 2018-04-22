#pragma once

#include <float.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "include\\geom.h"
#include "include\\CL\\cl.h"

const float neg_infinity = FLT_MIN;
const float pos_infinity = FLT_MAX;

typedef struct TreeNode {
	int min;
	int max;

	Sphere *sphere;
	bool leaf;

	TreeNode *left;
	TreeNode *right;
	TreeNode *parent;

	Bound bound;
	float cost;
	float area;

	Shape *shape;
} TreeNode;

struct PartitionEntry {
	unsigned char partition;
	bool left;
	TreeNode *parent;
};

typedef enum {X_AXIS, Y_AXIS, Z_AXIS} Axis;
typedef enum {INVALID, VALID, LEAF} TreeNodeType;
typedef struct NaiveBVHTreeNode
{
	TreeNodeType tnt;
	int lFrom, lTo;

	int left;
	int right;
	int parent;

	Bound bound;
} NaiveBVHTreeNode;

typedef struct NaiveBVHTreeLeafNode
{
	int shape;
	Vec centroid;
	Bound bound;
} NaiveBVHTreeLeafNode;

struct CompareNodes
{
private:
	Axis sort_axis;

public:
	void setAxis(Axis a)
	{
		sort_axis = a;
	}

	Axis getAxis()
	{
		return sort_axis;
	}

	bool operator() (NaiveBVHTreeLeafNode a, NaiveBVHTreeLeafNode b) 
	{ 
		if (sort_axis == X_AXIS) return (a.centroid.x < b.centroid.x);
		else if (sort_axis == Y_AXIS) return (a.centroid.y < b.centroid.y);
		else if (sort_axis == Z_AXIS) return (a.centroid.z < b.centroid.z);

		return false;
	}
};

class CLBVH
{
private:
	TreeNode *dtn, *dtl;

	float getArea(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);
	int longestCommonPrefix(int i, int j, int len);
	float get_total_area(int n, TreeNode *leaves[], unsigned s);
	void merge_bounds(Bound& b1, Bound& b2, Bound* b3);
	void calculateOptimalTreelet(int n, TreeNode **leaves, unsigned char *p_opt);
	void propagateAreaCost(TreeNode *root, TreeNode **leaves, int num_leaves);
	void restructTree(TreeNode *parent, TreeNode **leaves, TreeNode **nodes, unsigned char partition, unsigned char *optimal, int &index, bool left, int num_leaves);
	void treeletOptimize(TreeNode *root);

	// NaiveBVHTree
	unsigned int m_poiCnt, m_shapeCnt;
	Poi *m_pois;
	Shape *m_shapes;
	unsigned int nbtnCnt;
	NaiveBVHTreeNode *vnbtn;
	NaiveBVHTreeLeafNode *vnbtl;

	Bound getBound(Sphere s);
	Bound getBound(Triangle t);

	void buildNaiveBVHTreeNodes(unsigned int node, Axis axis, unsigned int from, unsigned int to);

public:
	CLBVH(Shape *shapes, int shapeCnt, Poi *pois, int poiCnt);
	~CLBVH();

	void buildRadixTree();
	void buildBVHTree();
	void optimize();
	
	inline void getTreeNodes(TreeNode **ppdtn, TreeNode **ppdtl)
	{
		*ppdtn = dtn;
		*ppdtl = dtl;
	}

	// NaiveBVHTree
	void buildNaiveBVHTree();
	void getTree(NaiveBVHTreeNode **ppnbtn, NaiveBVHTreeLeafNode **ppnbtl, int *pnbtnCnt);
};