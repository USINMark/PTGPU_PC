
#include "stdafx.h"
#include "CLBVH.h"

//#define DEBUG_PRINT
#define RESTRUCT_STACK_SIZE (4)

#define Ci 1.2
#define Ct 1.0

#define BLOCK_SIZE 256

#define CODE_OFFSET (1<<10)
#define CODE_LENGTH (10)

bool compFunc(int i, int j) { return (i<j); }

#ifdef _MSC_VER
#include <intrin.h>

inline uint32_t __clz(uint32_t x)
{
	unsigned long r = 0;
	
	_BitScanForward(&r, x);
	
	return r;
}

inline int __ffs(unsigned char value)
{
	if (value & 1) return 1;
	if (value & 2) return 2;
	if (value & 4) return 3;
	if (value & 8) return 4;
	if (value & 16) return 5;
	if (value & 32) return 6;
	if (value & 64) return 7;
	if (value & 128) return 8;

	return 0;
}

inline int __popc(unsigned char value)
{
	return __popcnt16(value);
}
#endif

#define clErrchk(ans) { clAssert((ans), __FILE__, __LINE__); }

inline void clAssert(cl_int code, const char *file, int line)
{
	if (code != CL_SUCCESS)
	{
		fprintf(stderr, "clAssert: %d %s %d\n", code, file, line);
	}
}

inline int min2(int a, int b)
{
	return (a < b) ? a : b;
}

inline int max2(int a, int b)
{
	return (a > b) ? a : b;
}

inline float min2(float a, float b)
{
	return (a < b) ? a : b;
}

inline float max2(float a, float b)
{
	return (a > b) ? a : b;
}

inline float min3(float a, float b, float c)
{
	return min2(min2(a, b), c);
}

inline float max3(float a, float b, float c)
{
	return max2(max2(a, b), c);
}

inline int pwr(int base, unsigned exp)
{
	int acc = 1;

	for (unsigned c = 0; c < exp; c++)
	{
		acc *= base;
	}

	return acc;
}

#ifdef DEBUG_PRINT
class Queue // as a doubly-linked list
{
private:
	typedef struct Node {
		struct Node *prev;
		struct Node *next;
		void *v;
	} Node;

	Node *head;
	Node *tail;
public:

	Queue() {
		head = NULL;
		tail = NULL;
	}

	void push(void *v) {
		Node *n = new Node();
		n->v = v;
		if (head == NULL) {
			n->prev = NULL;
			n->next = NULL;
			head = n;
			tail = n;
		}
		else {
			n->prev = NULL;
			n->next = head;
			if (head != NULL) {
				head->prev = n;
			}
			head = n;
		}
	}

	void pop() {
		if (head != NULL) {
			if (head == tail) {
				delete head;
				head = NULL;
				tail = NULL;
			}
			else {
				Node *subtail = tail->prev;
				delete tail;
				subtail->next = NULL;
				tail = subtail;
			}
		}
	}

	void *last() {
		if (tail != NULL) {
			return tail->v;
		}
		else {
			return NULL;
		}
	}

	bool empty() {
		return head == NULL;
	}
};

bool check_bound(TreeNode *p, TreeNode *l, TreeNode *r) {
	return (
		p->bound.min_x == min2(l->bound.min_x, r->bound.min_x) &&
		p->bound.max_x == max2(l->bound.max_x, r->bound.max_x) &&
		p->bound.min_y == min2(l->bound.min_y, r->bound.min_y) &&
		p->bound.max_y == max2(l->bound.max_y, r->bound.max_y) &&
		p->bound.min_z == min2(l->bound.min_z, r->bound.min_z) &&
		p->bound.max_z == max2(l->bound.max_z, r->bound.max_z)
		);
}

bool check_sanity(TreeNode *n) {
	if (n->leaf) {
		return true;
	}
	else {
		return (
			n->left->parent == n &&
			n->right->parent == n
			);
	}
}

void printPartition(TreeNode *root, unsigned char *optimal,
	unsigned char start, unsigned char mask) {
	int level = 1;
	Queue *q = new Queue();

	q->push((void *)start);
	q->push((void *)((~start) & mask));

	Queue *qt = new Queue();

	while (!q->empty()) {
		while (!q->empty()) {
			unsigned char n = (unsigned char)(unsigned long)(q->last());
			q->pop();

			if (__popc(n) != 1) {
				printf("[%d %p] %x\n", level, root, n & 0xff);
				qt->push((void *)optimal[n]);
				qt->push((void *)((~optimal[n]) & n));
			}
			else {
				printf("[%d %p] (%d)\n", level, root, __ffs(n));
			}
		}
		level++;
		printf("\n");

		Queue *t = q;
		q = qt;
		qt = t;
	}

	printf("\n");

	delete q;
	delete qt;
}

void printBVH(TreeNode *root) {
	int level = 1;
	Queue *q = new Queue();

	q->push((void *)root);

	Queue *qt = new Queue();

	while (!q->empty()) {
		printf("\n######### Level %d ##########\n", level++);
		while (!q->empty()) {
			TreeNode *n = (TreeNode *)(q->last());
			q->pop();
			printf("(%d %d) %p", n->min, n->max, n);

			if (!check_sanity(n)) {
				printf(" !SanityError! ");
			}

			if (!n->leaf) {
				if (!check_bound(n, n->left, n->right)) {
					printf(" !BoundError!");
				}
				printf("\n");
				qt->push((void *)n->left);
				qt->push((void *)n->right);
			}
			else {
				printf(" ((A:%.0lf C:%.0lf) Sphere: %d)\n", n->area, n->cost, n->shape->index);
			}
		}
		printf("\n");

		Queue *t = q;
		q = qt;
		qt = t;
	}

	printf("\n");

	delete q;
	delete qt;
}
#endif

inline float CLBVH::getArea(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
{
	float dx = max_x - min_x;
	float dy = max_y - min_y;
	float dz = max_z - min_z;
	
	return 2 * (dx * dy + dx * dz + dy * dz);
}

/**
* Longest common prefix for morton code
*/
inline int CLBVH::longestCommonPrefix(int i, int j, int len)
{
	if (0 <= j && j < len)
	{
		return __clz(i ^ j);
	}
	else
	{
		return -1;
	}
}

float CLBVH::get_total_area(int n, TreeNode *leaves[], unsigned s) 
{
	float lmin_x, lmin_y, lmin_z, lmax_x, lmax_y, lmax_z;
	
	float min_x = pos_infinity;
	float max_x = neg_infinity;
	float min_y = pos_infinity;
	float max_y = neg_infinity;
	float min_z = pos_infinity;
	float max_z = neg_infinity;

	for (int i = 0; i < n; i++) 
	{
		if (((s >> i) & 1) == 1) 
		{
			lmin_x = leaves[i]->bound.min_x;
			lmin_y = leaves[i]->bound.min_y;
			lmin_z = leaves[i]->bound.min_z;
			lmax_x = leaves[i]->bound.max_x;
			lmax_y = leaves[i]->bound.max_y;
			lmax_z = leaves[i]->bound.max_z;
	
			if (lmin_x < min_x) min_x = lmin_x;
			if (lmin_y < min_y) min_y = lmin_y;
			if (lmin_z < min_z) min_z = lmin_z;
			if (lmax_x > max_x) max_x = lmax_x;
			if (lmax_y > max_y) max_y = lmax_y;
			if (lmax_z > max_z) max_z = lmax_z;
		}
	}
	
	return getArea(min_x, max_x, min_y, max_y, min_z, max_z);
}

/**
* CLBVH constructor
*/
CLBVH::CLBVH(Shape *shapes, int shapeCnt, Poi *pois, int poiCnt): m_shapes(shapes), m_shapeCnt(shapeCnt), m_pois(pois), m_poiCnt(poiCnt)
{
	// For internal nodes, leaf = false
	dtn = (TreeNode *)malloc(sizeof(TreeNode) * (shapeCnt - 1));
	memset(dtn, 0, sizeof(TreeNode) * (shapeCnt - 1));

	// For leaves, leaf = true
	dtl = (TreeNode *)malloc(sizeof(TreeNode) * shapeCnt);
	memset(dtl, ~0, sizeof(TreeNode) * shapeCnt);

	// Initialize morton codes
	float min_x = pos_infinity;
	float max_x = neg_infinity;
	float min_y = pos_infinity;
	float max_y = neg_infinity;
	float min_z = pos_infinity;
	float max_z = neg_infinity;

	for (unsigned int i = 0; i < m_shapeCnt; i++)
	{
		Vec p;
		m_shapes[i].index = i;
		if (m_shapes[i].type == SPHERE)
		{
			m_shapes[i].b = getBound(m_shapes[i].s);
			p = m_shapes[i].s.p;

		}
		else if (m_shapes[i].type == TRIANGLE)
		{
			m_shapes[i].b = getBound(m_shapes[i].t);
			p.x = (m_shapes[i].b.min_x + m_shapes[i].b.max_x) / 2.0f, //(m_pois[m_shapes[i].t.p1].p.x + m_pois[m_shapes[i].t.p2].p.x + m_pois[m_shapes[i].t.p3].p.x) / 3.0f,
			p.y = (m_shapes[i].b.min_y + m_shapes[i].b.max_y) / 2.0f, //(m_pois[m_shapes[i].t.p1].p.y + m_pois[m_shapes[i].t.p2].p.y + m_pois[m_shapes[i].t.p3].p.y) / 3.0f,
			p.z = (m_shapes[i].b.min_z + m_shapes[i].b.max_z) / 2.0f; // (m_pois[m_shapes[i].t.p1].p.z + m_pois[m_shapes[i].t.p2].p.z + m_pois[m_shapes[i].t.p3].p.z) / 3.0f;
		}

		// find min and max coordinates
		if (p.x < min_x) min_x = p.x;
		if (p.x > max_x) max_x = p.x;
		if (p.y < min_y) min_y = p.y;
		if (p.y > max_y) max_y = p.y;
		if (p.z < min_z) min_z = p.z;
		if (p.z > max_z) max_z = p.z;
	}

	for (unsigned int i = 0; i < m_shapeCnt; i++)
	{
		// calculate morton code
		m_shapes[i].index = i;
		m_shapes[i].morton_code = 0;
		
		Vec p;

		if (m_shapes[i].type == SPHERE)
		{
			p = m_shapes[i].s.p;
		}
		else if (m_shapes[i].type == TRIANGLE)
		{
			float min_x = min3(m_pois[m_shapes[i].t.p1].p.x, m_pois[m_shapes[i].t.p2].p.x, m_pois[m_shapes[i].t.p3].p.x);
			float min_y = min3(m_pois[m_shapes[i].t.p1].p.y, m_pois[m_shapes[i].t.p2].p.y, m_pois[m_shapes[i].t.p3].p.y);
			float min_z = min3(m_pois[m_shapes[i].t.p1].p.z, m_pois[m_shapes[i].t.p2].p.z, m_pois[m_shapes[i].t.p3].p.z);

			float max_x = max3(m_pois[m_shapes[i].t.p1].p.x, m_pois[m_shapes[i].t.p2].p.x, m_pois[m_shapes[i].t.p3].p.x);
			float max_y = max3(m_pois[m_shapes[i].t.p1].p.y, m_pois[m_shapes[i].t.p2].p.y, m_pois[m_shapes[i].t.p3].p.y);
			float max_z = max3(m_pois[m_shapes[i].t.p1].p.z, m_pois[m_shapes[i].t.p2].p.z, m_pois[m_shapes[i].t.p3].p.z);

			p.x = (min_x + max_x) / 2.0f;// (m_pois[m_shapes[i].t.p1].p.x + m_pois[m_shapes[i].t.p2].p.x + m_pois[m_shapes[i].t.p3].p.x) / 3.0f;
			p.y = (min_y + max_y) / 2.0f;//(m_pois[m_shapes[i].t.p1].p.y + m_pois[m_shapes[i].t.p2].p.y + m_pois[m_shapes[i].t.p3].p.y) / 3.0f;
			p.z = (min_z + max_z) / 2.0f;//(m_pois[m_shapes[i].t.p1].p.z + m_pois[m_shapes[i].t.p2].p.z + m_pois[m_shapes[i].t.p3].p.z) / 3.0f;
		}

		// get the first 10 bits of each coordinate 
		unsigned int a = (unsigned int)(((p.x - min_x) / (max_x - min_x)) * CODE_OFFSET);
		unsigned int b = (unsigned int)(((p.y - min_y) / (max_y - min_y)) * CODE_OFFSET);
		unsigned int c = (unsigned int)(((p.z - min_z) / (max_z - min_z)) * CODE_OFFSET);

		// combine into 30 bits morton code
		for (unsigned int j = 0; j < CODE_LENGTH; j++) {
			m_shapes[i].morton_code |=
				(((((a >> (CODE_LENGTH - 1 - j))) & 1) << ((CODE_LENGTH - j) * 3 - 1)) |
				((((b >> (CODE_LENGTH - 1 - j))) & 1) << ((CODE_LENGTH - j) * 3 - 2)) |
				((((c >> (CODE_LENGTH - 1 - j))) & 1) << ((CODE_LENGTH - j) * 3 - 3)));
		}
	}

	// NaiveBVHTree
	//nbtnCnt = 2 * m_shapeCnt; // ceil(log2(m_shapeCnt)) + 1;
	//nbtnCnt = pow(2, depth);
	// For internal nodes, leaf = false
	vnbtn = (NaiveBVHTreeNode *)malloc(sizeof(NaiveBVHTreeNode) * (shapeCnt));
	memset(vnbtn, 0, sizeof(NaiveBVHTreeNode) * (shapeCnt));

	// For leaves, leaf = true
	vnbtl = (NaiveBVHTreeLeafNode *)malloc(sizeof(NaiveBVHTreeLeafNode) * shapeCnt);
	memset(vnbtl, ~0, sizeof(NaiveBVHTreeLeafNode) * shapeCnt);
}

/**
* CLBVH destructor
*/
CLBVH::~CLBVH() 
{
	if (dtn)
		free(dtn);
	if (dtl)
		free(dtl);
	if (vnbtn)
		delete vnbtn;
	if (vnbtl)
		delete vnbtl;
}

void CLBVH::merge_bounds(Bound& b1, Bound& b2, Bound* b3)
{
	b3->min_x = min2(b1.min_x, b2.min_x);
	b3->max_x = max2(b1.max_x, b2.max_x);
	b3->min_y = min2(b1.min_y, b2.min_y);
	b3->max_y = max2(b1.max_y, b2.max_y);
	b3->min_z = min2(b1.min_z, b2.min_z);
	b3->max_z = max2(b1.max_z, b2.max_z);

	return;
}

void CLBVH::calculateOptimalTreelet(int n, TreeNode **leaves, unsigned char *p_opt) 
{
	int num_subsets = pwr(2, n) - 1;

	// 0th element in array should not be used
	float a[128];
	float c_opt[128];
	
	// Calculate surface area for each subset
	for (unsigned char s = 1; s <= num_subsets; s++) 
	{
		a[s] = get_total_area(n, leaves, s);
	}
	
	// Initialize costs of individual leaves
	for (unsigned i = 0; i <= (n - 1); i++) 
	{
		c_opt[pwr(2, i)] = leaves[i]->cost;
	}
	
	// Optimize every subset of leaves
	for (unsigned k = 2; k <= n; k++) 
	{
		for (unsigned char s = 1; s <= num_subsets; s++) 
		{
			if (__popc(s) == k) 
			{
				// Try each way of partitioning the leaves
				float c_s = pos_infinity;
				unsigned char p_s = 0;
				unsigned char d = (s - 1) & s;
				unsigned char p = (-d) & s;
			
				while (p != 0) 
				{
					float c = c_opt[p] + c_opt[s ^ p];
					if (c < c_s) 
					{
						c_s = c;
						p_s = p;
					}
					//printf("p=%x, c=%.0lf, c_s=%.0lf, p_s=%x\n", p & 0xff, c, c_s, p_s & 0xff);
					p = (p - d) & s;
				}
				// Calculate final SAH cost
				c_opt[s] = Ci * a[s] + c_s;
				p_opt[s] = p_s;
			}
		}
	}
}

void CLBVH::propagateAreaCost(TreeNode *root, TreeNode **leaves, int num_leaves) 
{
	for (int i = 0; i < num_leaves; i++) 
	{
		TreeNode *cur = leaves[i];
		cur = cur->parent;
		while (cur != root) 
		{
			if (cur->cost == 0.0) 
			{
				if (cur->left->cost != 0.0 && cur->right->cost != 0.0) 
				{
					// Both left & right propagated
					Bound *bound = &cur->bound;

					merge_bounds(cur->left->bound, cur->right->bound, bound);
					cur->area = getArea(bound->min_x, bound->max_x, bound->min_y,
						bound->max_y, bound->min_z, bound->max_z);
					cur->cost = Ci * cur->area + cur->left->cost + cur->right->cost;
				}
				else 
				{
					// Only one side propagated
					break;
				}
			}
			cur = cur->parent;
		}
	}

	// Propagate root
	Bound *bound = &root->bound;

	merge_bounds(root->left->bound, root->right->bound, bound);

	root->area = getArea(bound->min_x, bound->max_x, bound->min_y, bound->max_y, bound->min_z, bound->max_z);
	root->cost = Ci * root->area + root->left->cost + root->right->cost;
}

void CLBVH::restructTree(TreeNode *parent, TreeNode **leaves,TreeNode **nodes, unsigned char partition, unsigned char *optimal, int &index, bool left, int num_leaves) 
{
	PartitionEntry stack[RESTRUCT_STACK_SIZE];
	int topIndex = RESTRUCT_STACK_SIZE;
	PartitionEntry tmp = { partition, left, parent };

	stack[--topIndex] = tmp;

	// Do while stack is not empty
	while (topIndex != RESTRUCT_STACK_SIZE) 
	{
		PartitionEntry *pe = &stack[topIndex++];
	
		partition = pe->partition;
		left = pe->left;
		parent = pe->parent;

#ifdef DEBUG_PRINT
		printf("parent=%p partition=%x(%s) index=%d left=%d\n", parent, partition & 0xff,
			(__popc(partition) == 1 ? "leaf" : "node"), index, left);
#endif

#ifdef DEBUG_CHECK
		if (partition == 0) {
			printf("partition is 0\n", partition);
			return;
		}
#endif

		if (__popc(partition) == 1) 
		{
			// Leaf
			int leaf_index = __ffs(partition) - 1;
#ifdef DEBUG_PRINT
			printf("parent=%p leaf_index=%d\n", parent, leaf_index);
#endif
			TreeNode *leaf = leaves[leaf_index];
			if (left) 
			{
				parent->left = leaf;
			}
			else 
			{
				parent->right = leaf;
			}
			leaf->parent = parent;
		}
		else 
		{
			// Internal node
#ifdef DEBUG_CHECK
			if (index >= 7) 
			{
				printf("index out of range\n");
				return;
			}
#endif

			TreeNode *node = nodes[index++];

			// Set cost to 0 as a mark
			node->cost = 0.0;

			if (left) 
			{
				parent->left = node;
			}
			else 
			{
				parent->right = node;
			}
			node->parent = parent;

#ifdef DEBUG_CHECK
			if (partition >= 128) 
			{
				printf("partition out of range\n");
				return;
			}
#endif
			unsigned char left_partition = optimal[partition];
			unsigned char right_partition = (~left_partition) & partition;

#ifdef DEBUG_CHECK
			if ((left_partition | partition) != partition) 
			{
				printf("left error: %x vs %x\n", left_partition & 0xff, partition & 0xff);
				return;
			}
			if ((right_partition | partition) != partition) 
			{
				printf("right error: %x vs %x\n", right_partition & 0xff, partition & 0xff);
				return;
			}
#endif

#ifdef DEBUG_CHECK
			if (topIndex < 2) 
			{
				printf("restructTree stack not big enough. Increase RESTRUCT_STACK_SIZE!\n");
			}
#endif
			PartitionEntry tmp1 = { left_partition, true, node };
			stack[--topIndex] = tmp1;
			PartitionEntry tmp2 = { right_partition, false, node };
			stack[--topIndex] = tmp2;
		}
	}

	propagateAreaCost(parent, leaves, num_leaves);
}

/**
* treeletOptimize
* Find the treelet and optimize
*/
void CLBVH::treeletOptimize(TreeNode *root) 
{
	// Don't need to optimize if root is a leaf
	if (root->leaf) return;

	// Find a treelet with max number of leaves being 7
	TreeNode *leaves[7];
	int counter = 0;

	leaves[counter++] = root->left;
	leaves[counter++] = root->right;

	// Also remember the internal nodes
	// Max 7 (leaves) - 1 (root doesn't count) - 1
	TreeNode *nodes[5];
	int nodes_counter = 0;

	float max_area;
	int max_index = 0;

	while (counter < 7 && max_index != -1) 
	{
		max_index = -1;
		max_area = -1.0;

		for (int i = 0; i < counter; i++) 
		{
			if (!(leaves[i]->leaf)) 
			{
				float area = leaves[i]->area;
				if (area > max_area) 
				{
					max_area = area;
					max_index = i;
				}
			}
		}

		if (max_index != -1) 
		{
			TreeNode *tmp = leaves[max_index];

			// Put this node in nodes array
			nodes[nodes_counter++] = tmp;

			// Replace the max node with its children
			leaves[max_index] = leaves[counter - 1];
			leaves[counter - 1] = tmp->left;
			leaves[counter++] = tmp->right;
		}
	}

#ifdef DEBUG_PRINT
	printf("%p counter=%d nodes_counter=%d\n", root, counter, nodes_counter);
	for (int i = 0; i < counter; i++) {
		printf("%p leaf %p\n", root, leaves[i]);
	}
	for (int i = 0; i < nodes_counter; i++) {
		printf("%p node %p\n", root, nodes[i]);
	}
#endif

	unsigned char optimal[128];

	// Call calculateOptimalCost here
	calculateOptimalTreelet(counter, leaves, optimal);

#ifdef DEBUG_PRINT
	printPartition(root, optimal, optimal[(1 << counter) - 1], (1 << counter) - 1);
#endif

	// Use complement on right tree, and use original on left tree
	unsigned char mask = (1 << counter) - 1;    // mask = max index
	int index = 0;                              // index for free nodes
	unsigned char leftIndex = mask;
	unsigned char left = optimal[leftIndex];

	restructTree(root, leaves, nodes, left, optimal, index, true, counter);

	unsigned char right = (~left) & mask;

	restructTree(root, leaves, nodes, right, optimal, index, false, counter);

	// Calculate current node's area & cost
	Bound *bound = &root->bound;
	merge_bounds(root->left->bound, root->right->bound, bound);

	root->area = getArea(bound->min_x, bound->max_x, bound->min_y, bound->max_y, bound->min_z, bound->max_z);
	root->cost = Ci * root->area + root->left->cost + root->right->cost;
}

/**
* CLBVH::makeRadixTree
* Make a radix tree. Required for the construction of BVH tree.
* Described in karras2012 paper.
*/
void CLBVH::buildRadixTree()
{
	int len = m_shapeCnt - 1;

	for (int i = 0; i < len; i++)
	{
		// Run radix tree construction algorithm
		// Determine direction of the range (+1 or -1)
		int d = longestCommonPrefix(i, i + 1, len + 1) - longestCommonPrefix(i, i - 1, len + 1) > 0 ? 1 : -1;

		// Compute upper bound for the length of the range
		int sigMin = longestCommonPrefix(i, i - d, len + 1);
		int lmax = 2;

		while (longestCommonPrefix(i, i + lmax * d, len + 1) > sigMin) 
		{
			lmax *= 2;
		}

		// Find the other end using binary search
		int l = 0;
		int divider = 2;

		for (int t = lmax / divider; t >= 1; divider *= 2) 
		{
			if (longestCommonPrefix(i, i + (l + t) * d, len + 1) > sigMin) 
			{
				l += t;
			}
			t = lmax / divider;
		}

		int j = i + l * d;
		
		//printf("i:%d d:%d lmax:%d l:%d j:%d \n",i , d, lmax, l, j);

		// Find the split position using binary search
		int sigNode = longestCommonPrefix(i, j, len + 1);
		int s = 0;

		divider = 2;
		for (int t = (l + (divider - 1)) / divider; t >= 1; divider *= 2) 
		{
			if (longestCommonPrefix(i, i + (s + t) * d, len + 1) > sigNode) 
			{
				s = s + t;
			}
			t = (l + (divider - 1)) / divider;
		}

		int gamma = i + s * d + min2(d, 0);

		// Output child pointers
		TreeNode *current = dtn + i;
		
		if (min2(i, j) == gamma) 
		{
			current->left = dtl + gamma;
			(dtl + gamma)->parent = current;
		}
		else 
		{
			current->left = dtn + gamma;
			(dtn + gamma)->parent = current;
		}

		if (max2(i, j) == gamma + 1) 
		{
			current->right = dtl + gamma + 1;
			(dtl + gamma + 1)->parent = current;
		}
		else 
		{
			current->right = dtn + gamma + 1;
			(dtn + gamma + 1)->parent = current;
		}

		current->min = min2(i, j);
		current->max = max2(i, j);
	}
}

/**
* CLBVH::buildBVHTree
* Make the BVH tree to be used for ray tracing later.
* Required that a radix tree is generated
*/
void CLBVH::buildBVHTree()
{
	// Sort geometries
	size_t i;

	// initialize host variables
	std::vector<unsigned int> H_keys(m_shapeCnt);
	std::vector<int> H_values(m_shapeCnt);

	// place data into host variables
	for (i = 0; i < m_shapeCnt; i++) {
		H_keys[i] = m_shapes[i].morton_code;
		H_values[i] = i;
	}

	std::sort(H_values.begin(), H_values.end(), [H_keys](size_t i, size_t j)
	{ return H_keys[i] < H_keys[j]; });

	int *sorted_geometry_indices = H_values.data();
	int *nodeCounter = (int *)malloc(sizeof(int) * m_shapeCnt);
	memset(nodeCounter, 0, sizeof(int) * m_shapeCnt);

	for (int index = 0; index < m_shapeCnt; index++)
	{
		TreeNode *leaf = dtl + index;

		// Handle leaf first
		int geometry_index = sorted_geometry_indices[index];
		leaf->bound = m_shapes[geometry_index].b;
		leaf->shape = &(m_shapes[geometry_index]);

		TreeNode *current = leaf->parent;
		int currentIndex = current - dtn;
		int res = nodeCounter[currentIndex]++; // atomicAdd(nodeCounter + currentIndex, 1);

		// Go up and handle internal nodes
		while (1) 
		{
			if (res == 0) 
			{
				break;
			}

			merge_bounds(current->left->bound, current->right->bound, &(current->bound));

			// If current is root, return
			if (current == dtn) 
			{
				break;
			}
			current = current->parent;
			currentIndex = current - dtn;
			res = nodeCounter[currentIndex]++; // atomicAdd(nodeCounter + currentIndex, 1);
		}
	}

	free(nodeCounter);
}

/**
* CLBVH::optimize
* Use treelet reconstruction algorithm described in karras2013hpg_paper.pdf
* to optimize BVH tree
*/
void CLBVH::optimize() 
{
	// nodeCounter makes sure that only 1 thread get to work on a node
	// in BVH construction kernel
	int *nodeCounter;

	nodeCounter = (int *)malloc(sizeof(int) * m_shapeCnt);
	memset(nodeCounter, 0, sizeof(int) * m_shapeCnt);

	for (int index = 0; index < m_shapeCnt; index++)
	{
		TreeNode *leaf = dtl + index;

		// Handle leaf first
		// Leaf's cost is just its bounding volumn's cost
		Bound *bound = &leaf->bound;

		leaf->area = getArea(bound->min_x, bound->max_x, bound->min_y, bound->max_y, bound->min_z, bound->max_z);
		leaf->cost = Ct * leaf->area;

#ifdef DEBUG_PRINT
		printf("%d handled leaf\n", index);
#endif

#ifdef DEBUG_PRINT
		if (index == 0) {
			printf("Launching Print BVH GPU... (before Optimization)\n");
			printBVH(dtn);
			printf("Launched Print BVH GPU... (before Optimization)\n");
		}
#endif
		TreeNode *current = leaf->parent;
		int currentIndex = current - dtn;
		int res = nodeCounter[currentIndex]++; // atomicAdd(nodeCounter + currentIndex, 1);

		// Go up and handle internal nodes
		while (1) 
		{
			if (res == 0) 
			{
				break;
			}

#ifdef DEBUG_PRINT
			printf("%d Going to optimize %p\n", index, current);
#endif
			treeletOptimize(current);

#ifdef DEBUG_PRINT
			printf("%d Optimized %p\n", index, current);
#endif
			// If current is root, return
			if (current == dtn) 
			{
				break;
			}
			current = current->parent;
			currentIndex = current - dtn;
			
			res = nodeCounter[currentIndex]++; // atomicAdd(nodeCounter + currentIndex, 1);
		}
	}
#if 0
#ifdef DEBUG_CHECK
	clErrchk(cudaMalloc(&nodeCounter, sizeof(int) * (this->num_geometries)));
	clErrchk(cudaMemset(nodeCounter, 0, sizeof(int) * (this->num_geometries)));
#else
	cudaMalloc(&nodeCounter, sizeof(int) * (this->num_geometries));
	cudaMemset(nodeCounter, 0, sizeof(int) * (this->num_geometries));
#endif

	// Configure GPU running parameters
	dim3 blockDim(BLOCK_SIZE, 1);
	dim3 gridDim((this->num_geometries + BLOCK_SIZE - 1) / BLOCK_SIZE, 1);

#ifdef DEBUG_PRINT
	std::cout << "Launching BVH Optimization GPU...\n";
#endif

	// Launch the optimize kernel
	kernelOptimize << <gridDim, blockDim >> >(this->num_geometries, nodeCounter,
		cudaDeviceTreeNodes, cudaDeviceTreeLeaves);

#ifdef DEBUG_CHECK
	clErrchk(cudaPeekAtLastError());
	clErrchk(cudaDeviceSynchronize());
#else
	cudaDeviceSynchronize();
#endif

#ifdef DEBUG_PRINT
	std::cout << "Launched BVH Optimization GPU...\n";
#endif

#ifdef DEBUG_CHECK
	clErrchk(cudaFree(nodeCounter));
#else
	cudaDeviceSynchronize();
	cudaFree(nodeCounter);
#endif

#ifdef DEBUG_PRINT
	std::cout << "Launching Print BVH GPU... (after Optimization)\n";

	kernelPrintBVH << <1, 1 >> >(cudaDeviceTreeNodes);
	clErrchk(cudaPeekAtLastError());
	clErrchk(cudaDeviceSynchronize());

	std::cout << "Launched Print BVH GPU... (after Optimization)\n";
#endif
#endif
	free(nodeCounter);
}

Bound CLBVH::getBound(Sphere s)
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

Bound CLBVH::getBound(Triangle t)
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

// Bottom-up approach
void CLBVH::buildNaiveBVHTree()
{
#if 0
	// NaiveTreeNode
	int depth = ceil(log2(m_shapeCnt));
	int leaves = pow(2, depth);
	
	for (int i = 0; i < m_shapeCnt; i++)
	{
		// shape, left
		nbtn[leaves + i].shape = i;
		nbtn[leaves + i].tnt = LEAF;
		
		nbtn[leaves + i].parent = i / 2;

		// bound
		if (m_shapes[i].type == SPHERE)
			nbtn[leaves + i].bound = getBound(m_shapes[i].s);
		else if (m_shapes[i].type == TRIANGLE)
			nbtn[leaves + i].bound = getBound(m_shapes[i].t);
	}

	int lastnode = leaves + m_shapeCnt;

	// Parents
	for (int i = depth; i >= 0; i--)
	{
		int n = pow(2, i - 1 ), j = n;
		for (; j <= 2 * n - 1; j++)
		{
			if (j * 2 >= lastnode)
			{
				nbtn[j].tnt = INVALID;
				continue;
			}
			
			nbtn[j].tnt = VALID;

			nbtn[j].left = j * 2;
			nbtn[j].right = j * 2 + 1;
			
			merge_bounds(nbtn[j * 2].bound, nbtn[j * 2 + 1].bound, &nbtn[j].bound);
		}
		lastnode = j;
	}
#else
	Vec min_p, max_p;

	min_p.x = pos_infinity, min_p.y = pos_infinity, min_p.z = pos_infinity;
	max_p.x = neg_infinity, max_p.y = neg_infinity, max_p.z = neg_infinity;

	for (int i = 0; i < m_shapeCnt; i++)
	{
		vnbtl[i].shape = i;

		if (m_shapes[i].type == SPHERE)
		{
			vnbtl[i].bound = getBound(m_shapes[i].s);
			vnbtl[i].centroid = m_shapes[i].s.p;
		}
		else if (m_shapes[i].type == TRIANGLE)
		{
			vnbtl[i].bound = getBound(m_shapes[i].t);
			vnbtl[i].centroid.x = (vnbtl[i].bound.min_x + vnbtl[i].bound.max_x) / 2.0f,
			vnbtl[i].centroid.y = (vnbtl[i].bound.min_y + vnbtl[i].bound.max_y) / 2.0f,
			vnbtl[i].centroid.z = (vnbtl[i].bound.min_z + vnbtl[i].bound.max_z) / 2.0f;
		}

		if (vnbtl[i].bound.min_x < min_p.x) min_p.x = vnbtl[i].bound.min_x;
		if (vnbtl[i].bound.min_y < min_p.y) min_p.y = vnbtl[i].bound.min_y;
		if (vnbtl[i].bound.min_z < min_p.z) min_p.z = vnbtl[i].bound.min_z;

		if (vnbtl[i].bound.max_x > max_p.x) max_p.x = vnbtl[i].bound.max_x;
		if (vnbtl[i].bound.max_y > max_p.y) max_p.y = vnbtl[i].bound.max_y;
		if (vnbtl[i].bound.max_z > max_p.z) max_p.z = vnbtl[i].bound.max_z;
	}

	// Make the root node
	for (int i = 0; i < m_shapeCnt; i++)
		vnbtn[i].tnt = INVALID;

	vnbtn[1].tnt = VALID;
	vnbtn[1].lFrom = 0, vnbtn[1].lTo = m_shapeCnt - 1;

	vnbtn[1].bound.min_x = min_p.x, vnbtn[1].bound.min_y = min_p.y, vnbtn[1].bound.min_z = min_p.z;
	vnbtn[1].bound.max_x = max_p.x, vnbtn[1].bound.max_y = max_p.y, vnbtn[1].bound.max_z = max_p.z;

	std::vector<NaiveBVHTreeLeafNode> H_values(m_shapeCnt);

	// place data into host variables
	for (int i = 0; i < m_shapeCnt; i++) {
		H_values[i] = vnbtl[i];
	}

	float diffX = vnbtn[1].bound.max_x - vnbtn[1].bound.min_x;
	float diffY = vnbtn[1].bound.max_y - vnbtn[1].bound.min_y;
	float diffZ = vnbtn[1].bound.max_z - vnbtn[1].bound.min_z;
	
	float max_diff = max3(diffX, diffY, diffZ);
	CompareNodes cmp;

	if (max_diff == diffX) cmp.setAxis(X_AXIS);
	else if (max_diff == diffY) cmp.setAxis(Y_AXIS);
	else if (max_diff == diffZ) cmp.setAxis(Z_AXIS);

	std::sort(H_values.begin(), H_values.end(), cmp);

	// place data into host variables
	for (int i = 0; i < m_shapeCnt; i++) {
		vnbtl[i] = H_values[i];
	}

	buildNaiveBVHTreeNodes(1, cmp.getAxis(), 1, m_shapeCnt);
#endif

	return;
}

void CLBVH::getTree(NaiveBVHTreeNode **ppnbtn, NaiveBVHTreeLeafNode **ppnbtl, int *pnbtnCnt)
{
	*ppnbtn = this->vnbtn;
	*ppnbtl = this->vnbtl;
	*pnbtnCnt = this->nbtnCnt;

	return;
}

void CLBVH::buildNaiveBVHTreeNodes(unsigned int node, Axis axis, unsigned int from, unsigned int to)
{
	unsigned int left = 2 * node, right = 2 * node + 1, parent = node / 2;
	int cnt = ((to - from) + 1);

	if (left >= m_shapeCnt || right >= m_shapeCnt)
	{
		vnbtn[node].tnt = LEAF;
		vnbtn[node].parent = node / 2, vnbtn[node].left = from - 1, vnbtn[node].right = to - 1;
		vnbtn[node].lFrom = from - 1, vnbtn[node].lTo = to - 1;
		
		if (node > nbtnCnt) nbtnCnt = node;

		return;
	}

	vnbtn[node].tnt = VALID;
	vnbtn[node].parent = node / 2,  vnbtn[node].left = left, vnbtn[node].right = right;
	vnbtn[node].lFrom = from - 1, vnbtn[node].lTo = to - 1;

	if (axis == X_AXIS)
	{
		vnbtn[left].bound.min_x = vnbtn[node].bound.min_x, vnbtn[left].bound.max_x = vnbtl[from + cnt / 2].bound.max_x;
		vnbtn[left].bound.min_y = vnbtn[node].bound.min_y, vnbtn[left].bound.max_y = vnbtn[node].bound.max_y;
		vnbtn[left].bound.min_z = vnbtn[node].bound.min_z, vnbtn[left].bound.max_z = vnbtn[node].bound.max_z;

		vnbtn[right].bound.min_x = vnbtl[from + cnt / 2 + 1].bound.min_x, vnbtn[right].bound.max_x = vnbtn[node].bound.max_x;
		vnbtn[right].bound.min_y = vnbtn[node].bound.min_y, vnbtn[right].bound.max_y = vnbtn[node].bound.max_y;
		vnbtn[right].bound.min_z = vnbtn[node].bound.min_z, vnbtn[right].bound.max_z = vnbtn[node].bound.max_z;
	}
	else if (axis == Y_AXIS)
	{
		vnbtn[left].bound.min_x = vnbtn[node].bound.min_x, vnbtn[left].bound.max_x = vnbtn[node].bound.max_x;
		vnbtn[left].bound.min_y = vnbtn[node].bound.min_y, vnbtn[left].bound.max_y = vnbtl[from + cnt / 2].bound.max_y;
		vnbtn[left].bound.min_z = vnbtn[node].bound.min_z, vnbtn[left].bound.max_z = vnbtn[node].bound.max_z;

		vnbtn[right].bound.min_x = vnbtn[node].bound.min_x, vnbtn[right].bound.max_x = vnbtn[node].bound.max_x;
		vnbtn[right].bound.min_y = vnbtl[from + cnt / 2 + 1].bound.min_y, vnbtn[right].bound.max_y = vnbtn[node].bound.max_y;
		vnbtn[right].bound.min_z = vnbtn[node].bound.min_z, vnbtn[right].bound.max_z = vnbtn[node].bound.max_z;
	}
	else if (axis == Z_AXIS)
	{
		vnbtn[left].bound.min_x = vnbtn[node].bound.min_x, vnbtn[left].bound.max_x = vnbtn[node].bound.max_x;
		vnbtn[left].bound.min_y = vnbtn[node].bound.min_y, vnbtn[left].bound.max_y = vnbtn[node].bound.max_y;
		vnbtn[left].bound.min_z = vnbtn[node].bound.min_z, vnbtn[left].bound.max_z = vnbtl[from + cnt / 2].bound.max_z;

		vnbtn[right].bound.min_x = vnbtn[node].bound.min_x, vnbtn[right].bound.max_x = vnbtn[node].bound.max_x;
		vnbtn[right].bound.min_y = vnbtn[node].bound.min_y, vnbtn[right].bound.max_y = vnbtn[node].bound.max_y;
		vnbtn[right].bound.min_z = vnbtl[from + cnt / 2 + 1].bound.min_z, vnbtn[right].bound.max_z = vnbtn[node].bound.max_z;
	}
	
	buildNaiveBVHTreeNodes(left, axis, from, from + cnt / 2);
	buildNaiveBVHTreeNodes(right, axis, from + cnt / 2 + 1, to);
	
	return;
}
