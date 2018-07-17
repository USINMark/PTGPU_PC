#ifndef KDNODEGPU_H
#define KDNODEGPU_H

typedef struct KDNodeGPU {
	short leaf;
	short nLeft, nRight, nParent;

	Bound bound;
	
	short min;
	short max;
} KDNodeGPU;

#endif //KDNODEGPU_H