
#include "clheader.h"

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

inline int __popc(unsigned int value)
{
	int cnt = 0;
	unsigned int mvalue = (value << 24) >> 24;
	
	if (mvalue & 1) cnt++;
	if (mvalue & 2) cnt++;
	if (mvalue & 4) cnt++;
	if (mvalue & 8) cnt++;
	if (mvalue & 16) cnt++;
	if (mvalue & 32) cnt++;
	if (mvalue & 64) cnt++;
	if (mvalue & 128) cnt++;
	
	mvalue = (value << 16) >> 16;
	
	if (mvalue & 1) cnt++;
	if (mvalue & 2) cnt++;
	if (mvalue & 4) cnt++;
	if (mvalue & 8) cnt++;
	if (mvalue & 16) cnt++;
	if (mvalue & 32) cnt++;
	if (mvalue & 64) cnt++;
	if (mvalue & 128) cnt++;
	
	mvalue = (value << 8) >> 8;
	
	if (mvalue & 1) cnt++;
	if (mvalue & 2) cnt++;
	if (mvalue & 4) cnt++;
	if (mvalue & 8) cnt++;
	if (mvalue & 16) cnt++;
	if (mvalue & 32) cnt++;
	if (mvalue & 64) cnt++;
	if (mvalue & 128) cnt++;
	
	mvalue = value;
	
	if (mvalue & 1) cnt++;
	if (mvalue & 2) cnt++;
	if (mvalue & 4) cnt++;
	if (mvalue & 8) cnt++;
	if (mvalue & 16) cnt++;
	if (mvalue & 32) cnt++;
	if (mvalue & 64) cnt++;
	if (mvalue & 128) cnt++;
	
	return cnt;
}

int min2_i(int i, int j) 
{
        return (i > j) ? j : i;
}

int max2_i(int i, int j) 
{
        return (i > j) ? i : j;
}

inline float min2_f(float a, float b) 
{
	return (a < b) ? a : b;
}

inline float max2_f(float a, float b) 
{
	return (a > b) ? a : b;
}

inline float getArea(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) 
{
	
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z;
	
    return 2 * (dx * dy + dx * dz + dy * dz);
}

inline int fastclz(int iv)
{
	unsigned int v = (unsigned int)iv;
	int x = (0 != (v >> 16)) * 16;

	x += (0 != (v >> (x + 8))) * 8;
	x += (0 != (v >> (x + 4))) * 4;
	x += (0 != (v >> (x + 2))) * 2;
	x += (0 != (v >> (x + 1)));
	x += (0 != (v >> x));

	return 32 - x;
}

void merge_bounds(Bound b1, Bound b2, Bound* b3) 
{
	b3->min_x = min2_f(b1.min_x, b2.min_x);
	b3->max_x = max2_f(b1.max_x, b2.max_x);
	b3->min_y = min2_f(b1.min_y, b2.min_y);
	b3->max_y = max2_f(b1.max_y, b2.max_y);
	b3->min_z = min2_f(b1.min_z, b2.min_z);
	b3->max_z = max2_f(b1.max_z, b2.max_z);

	return;
}

/**
 * Longest common prefix for morton code
 */
int longestCommonPrefix(int i, int j, int len) 
{
    if (0 <= j && j < len) {
        return fastclz(i ^ j);
    } else {
        return -1;
    }
}

/**
 * Radix tree construction kernel
 * Algorithm described in karras2012 paper.
 * Node-wise parallel
 */
 __kernel void kernelConstructRadixTree(int len, __global BVHTreeNode *radixTreeNodes, __global BVHTreeNode *radixTreeLeaves) 
{
    int i = get_global_id(0);

    if (i >= len) return;

    // Run radix tree construction algorithm
    // Determine direction of the range (+1 or -1)
    int d = longestCommonPrefix(i, i + 1, len+1) - 
    longestCommonPrefix(i, i - 1, len+1) > 0 ? 1 : -1;
    
    // Compute upper bound for the length of the range
    int sigMin = longestCommonPrefix(i, i - d, len+1);
    int lmax = 2;

    while (longestCommonPrefix(i, i + lmax * d, len+1) > sigMin) {
        lmax *= 2;
    }

    // Find the other end using binary search
    int l = 0;
    int divider = 2;
    for (int t = lmax / divider; t >= 1; divider *= 2) {
        if (longestCommonPrefix(i, i + (l + t) * d, len+1) > sigMin) {
            l += t;
        }
        t = lmax / divider;
    }
  
    int j = i + l * d;  
    
    // Find the split position using binary search
    int sigNode = longestCommonPrefix(i, j, len+1);
    int s = 0;
    divider = 2;
    for (int t = (l + (divider - 1)) / divider; t >= 1; divider *= 2) {
        if (longestCommonPrefix(i, i + (s + t) * d, len+1) > sigNode) {
            s = s + t;
        }
        t = (l + (divider - 1)) / divider;
    }

    int gamma = i + s * d + min2_i(d, 0);

    // Output child pointers
    __global BVHTreeNode *current = radixTreeNodes + i;

    if (min2_i(i, j) == gamma) {
        current->nLeft = gamma;
		current->leaf = 2;
        (radixTreeLeaves + gamma)->nParent = i;
    } else {
        current->nLeft = gamma;
        (radixTreeNodes + gamma)->nParent = i;
    }

    if (max2_i(i, j) == gamma + 1) {
        current->nRight = gamma + 1;
		current->leaf = 2;
        (radixTreeLeaves + gamma + 1)->nParent = i;
    } else {
        current->nRight = gamma + 1;
        (radixTreeNodes + gamma + 1)->nParent = i;
    }

    current->min = min2_i(i, j);
    current->max = max2_i(i, j);
}

/**
 * BVH Construction kernel
 * Algorithm described in karras2012 paper (bottom-up approach).
 */
__kernel void kernelConstructBVHTree(int len, __global BVHTreeNode *treeNodes, __global BVHTreeNode *treeLeaves, __global int *nodeCounter, __constant int *sorted_geometry_indices, __global Shape *shapes) 
{
    int index = get_global_id(0);
    
    if (index >= len) return;

    __global BVHTreeNode *leaf = &treeLeaves[index];

    // Handle leaf first
    int geometry_index = sorted_geometry_indices[index];

	leaf->bound = shapes[geometry_index].b;
	leaf->nShape = geometry_index;
	
	__global BVHTreeNode *current = &treeNodes[leaf->nParent];

	int currentIndex = leaf->nParent;
	int res = atomic_add(nodeCounter + currentIndex, 1);

    // Go up and handle internal nodes
    while (1) {
        if (res == 0) {
            return;
        }		
		
		Bound bound;
		
		if (current->leaf == 2) 
			merge_bounds(treeLeaves[current->nLeft].bound, treeLeaves[current->nRight].bound, &bound);
		else 
			merge_bounds(treeNodes[current->nLeft].bound, treeNodes[current->nRight].bound, &bound);
		
		current->bound = bound;

        // If current is root, return
        if (current == treeNodes) {
            return;
        }

		current = &treeNodes[current->nParent];
		currentIndex = current->nParent;
		
        res = atomic_add(nodeCounter + currentIndex, 1);
    }
}

float get_total_area(int n, int *leaves, __global BVHTreeNode *treeNodes, unsigned s) 
{
    float lmin_x, lmin_y, lmin_z, lmax_x, lmax_y, lmax_z;
    float min_x = FLT_MAX;
    float max_x = FLT_MIN;
    float min_y = FLT_MAX;
    float max_y = FLT_MIN;
    float min_z = FLT_MAX;
    float max_z = FLT_MIN;
	
    for (int i = 0; i < n; i++) {
        if ((s >> i) & 1 == 1) {
            lmin_x = treeNodes[leaves[i]].bound.min_x;
            lmin_y = treeNodes[leaves[i]].bound.min_y;
            lmin_z = treeNodes[leaves[i]].bound.min_z;
            lmax_x = treeNodes[leaves[i]].bound.max_x;
            lmax_y = treeNodes[leaves[i]].bound.max_y;
            lmax_z = treeNodes[leaves[i]].bound.max_z;
			
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

void calculateOptimalTreelet(int n, int *leaves, __global BVHTreeNode *treeNodes, unsigned char *p_opt) 
{
    int num_subsets = (float) pow((float)2, (float)n) - 1;
	
    // 0th element in array should not be used
    float a[128];
    float c_opt[128];
	
    // Calculate surface area for each subset
    for (unsigned char s = 1; s <= num_subsets; s++) {
        a[s] = get_total_area(n, leaves, treeNodes, s);
    }
	
    // Initialize costs of individual leaves
    for (unsigned i = 0; i <= (n-1); i++) {
		int index = (float)pow((float)2, (float)i);
        c_opt[index] = treeNodes[leaves[i]].cost;
    }
    // Optimize every subset of leaves
    for (unsigned k = 2; k <= n; k++) {
        for (unsigned char s = 1; s <= num_subsets; s++) {
            if (__popc(s) == k) {
                // Try each way of partitioning the leaves
                float c_s = FLT_MAX;//pos_infinity;
                unsigned char p_s = 0;
                unsigned char d = (s - 1) & s;
                unsigned char p = (-d) & s;
                while (p != 0) {
                    float c = c_opt[p] + c_opt[s ^ p];
                    if (c < c_s) {
                        c_s = c;
                        p_s = p;
                    }
                    p = (p - d) & s;
                }
				
                // Calculate final SAH cost
                c_opt[s] = Ci * a[s] + c_s;
                p_opt[s] = p_s;
            }
        }
    }
}

void restructTree(int root, __global BVHTreeNode *treeNodes, int *leaves, int *nodes, unsigned char partition, unsigned char *optimal, int index, int left, int num_leaves) 
{
	int nParent = root;
	__global BVHTreeNode *parent = &treeNodes[root];
    PartitionEntry stack[RESTRUCT_STACK_SIZE];
    int topIndex = RESTRUCT_STACK_SIZE;
    PartitionEntry tmp = {partition, left, root};
	
    stack[--topIndex] = tmp;

    // Do while stack is not empty
    while (topIndex != RESTRUCT_STACK_SIZE) {
        PartitionEntry *pe = &stack[topIndex++];
        partition = pe->partition;
        left = pe->left;
		parent = &treeNodes[pe->nParent];
		nParent = pe->nParent;
		
        if (__popc(partition) == 1) {
            // Leaf
            int leaf_index = __ffs(partition) - 1;

            __global BVHTreeNode *leaf = &treeNodes[leaves[leaf_index]];
            if (left) {
				parent->nLeft = leaf_index;
            } else {
				parent->nRight = leaf_index;
            }
			leaf->nParent = nParent;
        } else {
            // Internal node

#if 0
            __global BVHTreeNode *node = &treeNodes[nodes[index++]];

            // Set cost to 0 as a mark
            node->cost = 0.0;

            if (left) {
				parent->nLeft = index - 1;
            } else {
				parent->nRight = index - 1;
            }

			node->nParent = nParent;

            unsigned char left_partition = optimal[partition];
            unsigned char right_partition = (~left_partition) & partition;

            PartitionEntry tmp1 = {left_partition, 1, nodes[index-1]};
            stack[--topIndex] = tmp1;
            PartitionEntry tmp2 = {right_partition, 0, nodes[index-1]};
            stack[--topIndex] = tmp2;
#endif
        }
    }
#if 0
    propagateAreaCost(nParent, treeNodes, leaves, num_leaves);
#endif
}

/**
 * treeletOptimize
 * Find the treelet and optimize
 */
void treeletOptimize(int root, __global BVHTreeNode *treeNodes) 
{
    // Don't need to optimize if root is a leaf
    if (treeNodes[root].leaf == 1) return;

    // Find a treelet with max number of leaves being 7
	int leaves[7];
    int counter = 0;
	
    leaves[counter++] = treeNodes[root].nLeft;//root->left;
    leaves[counter++] = treeNodes[root].nRight;//root->right;

    // Also remember the internal nodes
    // Max 7 (leaves) - 1 (root doesn't count) - 1
	int nodes[5];
    int nodes_counter = 0;

    float max_area;
    int max_index = 0;

    while (counter < 7 && max_index != -1) {
        max_index = -1;
        max_area = -1.0;

        for (int i = 0; i < counter; i++) {
			if (treeNodes[leaves[i]].leaf != 1) {
				float area = treeNodes[leaves[i]].area;
                if (area > max_area) {
                    max_area = area;
                    max_index = i;
                }
            }
        }

        if (max_index != -1) {
			int tmp = leaves[max_index];

            // Put this node in nodes array
            nodes[nodes_counter++] = tmp;

            // Replace the max node with its children
            leaves[max_index] = leaves[counter - 1];
			leaves[counter - 1] = treeNodes[tmp].nLeft;
			leaves[counter++] = treeNodes[tmp].nRight;
        }
    }

    unsigned char optimal[128];

    // Call calculateOptimalCost here
    calculateOptimalTreelet(counter, leaves, treeNodes, optimal);

    // Use complement on right tree, and use original on left tree
    unsigned char mask = (1 << counter) - 1;    // mask = max index
    int index = 0;                              // index for free nodes
    unsigned char leftIndex = mask;

    unsigned char left = optimal[leftIndex];	
    restructTree(root, treeNodes, leaves, nodes, left, optimal, index, 1, counter);

    unsigned char right = (~left) & mask;
    restructTree(root, treeNodes, leaves, nodes, right, optimal, index, 0, counter);

    // Calculate current node's area & cost
	Bound bound;
	
	merge_bounds(treeNodes[treeNodes[root].nLeft].bound, treeNodes[treeNodes[root].nRight].bound, &bound);
	
	treeNodes[root].area = getArea(bound.min_x, bound.max_x, bound.min_y,
          bound.max_y, bound.min_z, bound.max_z);
	treeNodes[root].cost = Ci * treeNodes[root].area + treeNodes[treeNodes[root].nLeft].cost + treeNodes[treeNodes[root].nRight].cost;
}

void propagateAreaCost(int nParent, __global BVHTreeNode *treeNodes, int *leaves, int num_leaves) 
{

    for (int i = 0; i < num_leaves; i++) {
        __global BVHTreeNode *cur = &treeNodes[leaves[i]];
		cur = &treeNodes[cur->nParent];
		
		while (cur->nParent != nParent) {
            if (cur->cost == 0.0) {
				if (treeNodes[cur->nLeft].cost != 0.0 && treeNodes[cur->nRight].cost != 0.0) {
                    // Both left & right propagated
					Bound bound;
					merge_bounds(treeNodes[cur->nLeft].bound, treeNodes[cur->nRight].bound, &bound);
					cur->bound = bound;
					
					cur->area = getArea(bound.min_x, bound.max_x, bound.min_y,
                          bound.max_y, bound.min_z, bound.max_z);
					cur->cost = Ci * cur->area + treeNodes[cur->nLeft].cost + treeNodes[cur->nRight].cost;
                } else {
                    // Only one side propagated
                    break;
                }
            }
			cur = &treeNodes[cur->nParent];
        }
    }

    // Propagate root
	Bound bound;
	
	merge_bounds(treeNodes[treeNodes[nParent].nLeft].bound, treeNodes[treeNodes[nParent].nRight].bound, &bound);
	treeNodes[nParent].bound = bound;	

    treeNodes[nParent].area = getArea(bound.min_x, bound.max_x, bound.min_y, bound.max_y, bound.min_z, bound.max_z);	

	treeNodes[nParent].cost = Ci * treeNodes[nParent].area + treeNodes[treeNodes[nParent].nLeft].cost + treeNodes[treeNodes[nParent].nRight].cost;
}

/**
 * BVH Optimization kernel
 */
__kernel void kernelOptimize(int num_leaves, __global int *nodeCounter, __global BVHTreeNode *treeNodes, __global BVHTreeNode *treeLeaves) 
{
    int index = get_global_id(0);
    
    if (index >= num_leaves) return;
    
    // Handle leaf first
    // Leaf's cost is just its bounding volumn's cost
	treeLeaves[index].area = getArea(treeLeaves[index].bound.min_x, treeLeaves[index].bound.max_x, treeLeaves[index].bound.min_y, treeLeaves[index].bound.max_y, treeLeaves[index].bound.min_z, treeLeaves[index].bound.max_z);
	treeLeaves[index].cost = Ct * treeLeaves[index].area;
	
	int current = treeLeaves[index].nParent;
	int res = atomic_add(nodeCounter + current, 1);

    // Go up and handle internal nodes
    while (1) {
        if (res == 0) {
            return;
        }

		treeletOptimize(current, treeNodes);

        // If current is root, return
		if (current == 0) {
            return;
        }
		current = treeNodes[current].nParent;
		res = atomic_add(nodeCounter + current, 1);
    }
}
