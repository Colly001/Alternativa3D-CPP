#ifndef COLLISIONKDNODE_H
#define COLLISIONKDNODE_H

#include <qglobal.h>
#include "alternativa/physics/collision/types/boundbox.h"

class CollisionKdTree2D;


class CollisionKdNode
{
public:
	CollisionKdNode();
	~CollisionKdNode();
	QVector<int> *indices;
	QVector<int> *splitIndices;
	BoundBox boundBox;
	CollisionKdNode *parent;
	CollisionKdTree2D *splitTree;
	CollisionKdNode *positiveNode;
	CollisionKdNode *negativeNode;
	double coord;
	int axis;
};

#endif // COLLISIONKDNODE_H
