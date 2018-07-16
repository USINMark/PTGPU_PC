//VS2017community에서 작성,rendering_kernel.cl에 변경하기 위한 공부중
//kdtree이해 참조 -https://www.youtube.com/watch?v=Z4dNLvno-EY

#include "stdafx.h"
#include "include\\KDTree.h"
#include "include\\AccelCommon.h"

Bound KDTree::getBound(Sphere s)
{
	Bound b;

	b.min_x = s.p.x - s.rad;
	b.max_x = s.p.x + s.rad;
	b.min_y = s.p.y - s.rad;
	b.max_y = s.p.y + s.rad;
	b.min_z = s.p.z - s.rad;
	b.max_z = s.p.z + s.rad;

	return b;
}

Bound KDTree::getBound(Triangle t)
{
	Bound b;

	b.min_x = min3(m_pois[t.p1].p.x, m_pois[t.p2].p.x, m_pois[t.p3].p.x);
	b.max_x = max3(m_pois[t.p1].p.x, m_pois[t.p2].p.x, m_pois[t.p3].p.x);
	b.min_y = min3(m_pois[t.p1].p.y, m_pois[t.p2].p.y, m_pois[t.p3].p.y);
	b.max_y = max3(m_pois[t.p1].p.y, m_pois[t.p2].p.y, m_pois[t.p3].p.y);
	b.min_z = min3(m_pois[t.p1].p.z, m_pois[t.p2].p.z, m_pois[t.p3].p.z);
	b.max_z = max3(m_pois[t.p1].p.z, m_pois[t.p2].p.z, m_pois[t.p3].p.z);

	return b;
}

Bound KDTree::getBound(Shape s)
{
	Bound b;

	if (s.type == TRIANGLE) b = getBound(s.t);
	else if (s.type == SPHERE) b = getBound(s.s);

	return b;
}

Vec KDTree::getMidpoint(Sphere s)
{
	return s.p;
}

Vec KDTree::getMidpoint(Triangle t)
{
	Vec v;

	v.x = (m_pois[t.p1].p.x + m_pois[t.p2].p.x + m_pois[t.p3].p.x) / 3.0f;
	v.y = (m_pois[t.p1].p.y + m_pois[t.p2].p.y + m_pois[t.p3].p.y) / 3.0f;
	v.z = (m_pois[t.p1].p.z + m_pois[t.p2].p.z + m_pois[t.p3].p.z) / 3.0f;

	return v;
}

Vec KDTree::getMidpoint(Shape s)
{
	Vec v;

	if (s.type == TRIANGLE) v = getMidpoint(s.t);
	else if (s.type == SPHERE) v = getMidpoint(s.s);

	return v;
}

int KDTree::getLongestAxis(Bound b)
{
	float xdist = b.max_x - b.min_x, ydist = b.max_y - b.min_y, zdist = b.max_z - b.min_z;

	if (xdist >= ydist) {
		if (xdist >= zdist) return 0;
		else return 2;
	}
	else {
		if (ydist >= zdist) return 1;
		else return 2;
	}

	return 2;
}

// 삼각형들을 위한 KDTREE 생성 
KDTreeNode* KDTree::build(std::vector<Shape *> s, int depth)
{
	KDTreeNode* node = new KDTreeNode(m_pois, m_poiCnt);
	if (depth > m_maxdepth) m_maxdepth = depth;

	node->leaf = false;
    node->shapes = std::vector<Shape *>();
	node->left = NULL;//"
    node->right = NULL;//초기화
	node->box = Bound();

	if (s.size() == 0) return node;
	if (s.size() <= 1) //depth > MAX_KDTREEDEPTH || )
	{
		node->shapes = s;
		node->leaf = true;
		node->box = getBound(*s[0]);

		for (long i = 1; i<s.size(); i++) {
			node->expandBound(getBound(*s[i]));
		}

		node->left = NULL; // new KDNode(m_pois, m_poiCnt);
		node->right = NULL; // new KDNode(m_pois, m_poiCnt);

		//node->left->shapes = std::vector<Shape *>();
		//node->right->shapes = std::vector<Shape*>();

		m_szbuf += s.size();
		return node;
	}

	Vec midpoint;		
	float shapesRecp = 1.0 / s.size();

	midpoint.x = midpoint.y = midpoint.z = 0.0;
	node->box = getBound(*s[0]);

	for (long i = 1; i < s.size(); i++)
	{
		node->expandBound(getBound(*s[i]));

		Vec m = getMidpoint(*s[i]);

		m.x *= shapesRecp, m.y *= shapesRecp, m.z *= shapesRecp;
		midpoint.x += m.x, midpoint.y += m.y, midpoint.z += m.z;
	}
	
	std::vector<Shape *> left_shapes;
	std::vector<Shape *> right_shapes;

	int axis = getLongestAxis(node->box); //xyz 중 가장 긴축 (x:0,y:1,z:2)

	for (long i = 0; i < s.size(); i++)
	{
		switch (axis) {
		case 0://중앙 값보다 크면 오른쪽 트리에, 아니면 왼쪽 트리에 넣는다
			midpoint.x >= getMidpoint(*s[i]).x ? right_shapes.push_back(s[i]) : left_shapes.push_back(s[i]);
			break;
		case 1:
			midpoint.y >= getMidpoint(*s[i]).y ? right_shapes.push_back(s[i]) : left_shapes.push_back(s[i]);
			break;
		case 2:
			midpoint.z >= getMidpoint(*s[i]).z ? right_shapes.push_back(s[i]) : left_shapes.push_back(s[i]);
			break;
		}
	}

	if (s.size() == left_shapes.size() || s.size() == right_shapes.size())
	{
		node->shapes = s;
		node->leaf = true;
		node->box = getBound(*s[0]);

		for (long i = 1; i < s.size(); i++) {
			node->expandBound(getBound(*s[i]));
		}

		node->left = NULL; // new KDNode(m_pois, m_poiCnt);
		node->right = NULL; // new KDNode(m_pois, m_poiCnt);

		//node->left->shapes = std::vector<Shape *>();
		//node->right->shapes = std::vector<Shape *>();

		m_szbuf += (left_shapes.size() + right_shapes.size());
		return node;
	}

	//깊이를 더하여 재귀
	node->left = build(left_shapes, depth + 1);
	node->right = build(right_shapes, depth + 1);
	
    return node;
}

void KDTree::fillNode(KDTreeNode *tnode, int &locnode, int &locshape, KDNodeGPU *kngnode, int *pnbuf)
{
	kngnode[locnode].bound = tnode->box;
	kngnode[locnode].leaf = tnode->leaf;

	if (tnode->leaf)
	{
		kngnode[locnode].min = locshape;

		for (int i = 0; i < tnode->shapes.size(); i++)
			pnbuf[locshape++] = tnode->shapes[i]->index;

		kngnode[locnode].max = locshape;
	}

	kngnode[locnode].nShape = tnode->shapes.size();

	kngnode[locnode].nLeft = 2 * locnode;
	kngnode[locnode].nRight = 2 * locnode + 1;
	kngnode[locnode].nParent = locnode / 2;
}

void KDTree::traverseTree(KDTreeNode *tnode, int locNode, int locShape, KDNodeGPU *pkngbuf, int *pknbuf)
{
	int curnodeloc = locNode, curshapeloc = locShape, size = pow(2, m_maxdepth + 1);

	std::queue<KDTreeNode *> q;
	q.push(tnode);

	while (!q.empty())
	{
		KDTreeNode *node = q.front();
		q.pop();
				
		if (node)
		{
			fillNode(node, curnodeloc, curshapeloc, pkngbuf, pknbuf);
			curnodeloc++;

			//if (node->leaf) continue;

			if (node->left) q.push(node->left);
			else q.push(NULL);

			if (node->right) q.push(node->right);
			else q.push(NULL);
		}
		else
		{
			q.push(NULL);
			q.push(NULL);

			curnodeloc++;
		}

		if (curnodeloc >= size) break;
	}
}

void KDTree::traverseTreeDFS(KDTreeNode *tnode, int &locNode, int &locShape, KDNodeGPU *pkngbuf, int *pknbuf)
{
	if (tnode)
	{
		int curnode = locNode;
		fillNode(tnode, curnode, locShape, pkngbuf, pknbuf);
				
		traverseTreeDFS(tnode->left, ++curnode, locShape, pkngbuf, pknbuf);
		traverseTreeDFS(tnode->right, ++curnode, locShape, pkngbuf, pknbuf);

		locNode = curnode;
	}
}

void KDTree::getTrees(KDTreeNode *rootNode, KDNodeGPU **ppkngbuf, int **ppknbuf, int *pszkngbuf, int *pszknbuf)
{
	int *pknbuf = (int *)malloc(sizeof(int) * m_szbuf), size = pow(2, m_maxdepth + 1);
	KDNodeGPU *pkngbuf = (KDNodeGPU *)malloc(sizeof(KDNodeGPU) * size);

	//int locNode = 0, locShape = 0;
	traverseTree(rootNode, 1, 0, pkngbuf, pknbuf);
	//traverseTreeDFS(rootNode, locNode, locShape, pkngbuf, pknbuf);

	*ppkngbuf = pkngbuf;
	*ppknbuf = pknbuf;
	*pszkngbuf = size;
	*pszknbuf = m_szbuf;
}