#ifndef COLLISIONKDTREE2D_H
#define COLLISIONKDTREE2D_H

class CollisionKdTree;
class CollisionKdNode;

#include <qglobal.h>

class CollisionKdTree2D
{
	int splitAxis;
	double splitCost;
	double splitCoord;
public:
	double threshold;
	int minPrimitivesPerNode;
	CollisionKdTree *parentTree;
	CollisionKdNode *parentNode;
	CollisionKdNode *rootNode;
	CollisionKdTree2D(CollisionKdTree *parentTree, CollisionKdNode *parentNode);
	~CollisionKdTree2D();
	void createTree();

private:
	void splitNode(CollisionKdNode *node);
	void checkNodeAxis(CollisionKdNode *node, int axis, int numSplitCoords, QVector<double> *splitCoords, double bb[6]);
};


#endif // COLLISIONKDTREE2D_H
