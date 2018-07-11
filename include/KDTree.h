#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <queue>
#ifdef __ANDROID__
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#endif

#include "include\\geom.h"
#include "include\\KDNodeGPU.h"

#define MAX_KDTREEDEPTH 25

class KDTreeNode {
private:
	Poi *m_pois;
	unsigned int m_poiCnt;
	
public:
	bool leaf;
	KDTreeNode *left;
	KDTreeNode *right;

	Bound box;

	std::vector<Shape *> shapes;

	KDTreeNode(Poi *pois, unsigned int poiCnt) : m_pois(pois), m_poiCnt(poiCnt) {};
	~KDTreeNode() {};

	//KDNode* build(std::vector<Shape *> s, int depth);
	Bound getBoundingBox() { return box; };
	KDTreeNode* getLeft() { return left; };
	KDTreeNode* getRight() { return right; };
	
	void expandBound(Bound b)
	{
		if (b.min_x < box.min_x) box.min_x = b.min_x;
		if (b.min_y < box.min_y) box.min_y = b.min_y;
		if (b.min_z < box.min_z) box.min_z = b.min_z;

		if (b.max_x > box.max_x) box.max_x = b.max_x;
		if (b.max_y > box.max_y) box.max_y = b.max_y;
		if (b.max_z > box.max_z) box.max_z = b.max_z;
	}
};

class KDTree {
private:
	Poi * m_pois;
	unsigned int m_poiCnt;

	int m_maxdepth, m_szbuf;

public:
	KDTree(Poi *pois, unsigned int poiCnt) : m_pois(pois), m_poiCnt(poiCnt), m_szbuf(0), m_maxdepth(0)
	{
	};

#ifdef WIN32
	void printNode(KDTreeNode *node, int depth)
	{
		KDTreeNode *left = node->getLeft(), *right = node->getRight();
		
		for (int i = 0; i < depth; i++)
			printf("-");

		printf("Shapes (%d, %d, %d, %f, %f, %f, %f, %f, %f): ", depth, node->shapes.size(), node->leaf, node->box.min_x, node->box.min_y, node->box.min_z, node->box.max_x, node->box.max_y, node->box.max_z);
		for (int i = 0; i < node->shapes.size(); i++)
			printf("%d, ", node->shapes[i]->index);

		printf("\n");

		if (left) printNode(left, depth + 1);
		if (right) printNode(right, depth + 1);
	}
#endif
	Vec getMidpoint(Shape s);
	Vec getMidpoint(Triangle t);
	Vec getMidpoint(Sphere s);

	Bound getBound(Triangle t);
	Bound getBound(Sphere s);
	Bound getBound(Shape s);
	
	int getLongestAxis(Bound b);
	KDTreeNode* build(std::vector<Shape *> s, int depth);
	void getTrees(KDTreeNode *rootNode, KDNodeGPU **ppktn, int **ppktnbuffer, int *pszkngbuf, int *pszknbuf);
	void fillNode(KDTreeNode *tnode, int &locnode, int &locshape, KDNodeGPU *knode, int *pnbuf);
	void traverseTree(KDTreeNode *tnode, int locNode, int locShape, KDNodeGPU *pkngbuf, int *pknbuf);
	void traverseTreeDFS(KDTreeNode *tnode, int &locNode, int &locShape, KDNodeGPU *pkngbuf, int *pknbuf);
};
#endif // KDTREE_H