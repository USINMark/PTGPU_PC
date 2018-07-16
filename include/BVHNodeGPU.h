#ifndef BVHNODEGPU_H
#define BVHNODEGPU_H

typedef struct BVHNodeGPU {
	int min;
	int max;

	int nShape;
	int leaf;
	
	int nLeft, nRight, nParent;

	Bound bound;
	float cost;
	float area;		
} BVHNodeGPU;

typedef struct {
    unsigned char partition;
    int left;	
    //TreeNode *parent;
	int nParent;
}PartitionEntry;

#endif //BVHNODEGPU_H