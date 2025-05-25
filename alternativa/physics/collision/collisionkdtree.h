#ifndef COLLISIONKDTREE_H
#define COLLISIONKDTREE_H

#include <qglobal.h>
#include <memory>

class CollisionPrimitive;
class BoundBox;
class CollisionKdNode;


class CollisionKdTree
{
	int splitAxis;
	double splitCoord;
	double splitCost;

	void splitNode(CollisionKdNode *node);
	void checkNodeAxis(CollisionKdNode *node, int axis, int numSplitCoords, QVector<double> *splitCoords, double bb[6]);
	void traceNode(const QString &str, CollisionKdNode *node);

public:

	double threshold;
	int minPrimitivesPerNode;
	CollisionKdNode *rootNode;
	std::shared_ptr<QVector<std::shared_ptr<CollisionPrimitive>>> _staticChildren;
	QVector<CollisionPrimitive*> *staticChildren;
	int numStaticChildren;
	QVector<BoundBox*> *staticBoundBoxes;
	CollisionKdTree();
	~CollisionKdTree();

	void createTree(std::shared_ptr<QVector<std::shared_ptr<CollisionPrimitive>>> collisionPrimitives, BoundBox *boundBox = nullptr);
};

#endif // COLLISIONKDTREE_H
