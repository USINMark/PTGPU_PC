
typedef struct KDNodeGPU {
	int leaf;
	int nLeft, nRight, nParent;

	Bound bound;
	
	int min;
	int max;

	int nShape;
} KDNodeGPU;
