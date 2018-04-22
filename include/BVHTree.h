
typedef struct BVHTreeNode {
	int min;
	int max;

	int nShape;
	int leaf;
	
	int nLeft, nRight, nParent;

	Bound bound;
	float cost;
	float area;		
} BVHTreeNode;

typedef struct {
    unsigned char partition;
    int left;	
    //TreeNode *parent;
	int nParent;
}PartitionEntry;