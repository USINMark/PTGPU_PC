#ifndef KDNODEGPU_H
#define KDNODEGPU_H

typedef struct KDNodeGPU {
	int leaf;
	int nLeft, nRight, nParent;

	Bound bound;
	
	int min;
	int max;

	int nShape;
} KDNodeGPU;

#endif //KDNODEGPU_H