#ifndef KDTREECOLLISIONDETECTOR_H
#define KDTREECOLLISIONDETECTOR_H

#include "ICollisionDetector.h"
#include "MinMax.h"
#include <qglobal.h>
#include <memory>


class CollisionKdTree;
class RayIntersection;
class ICollider;
class Body;
class CollisionKdNode;
class BoundBox;


class KdTreeCollisionDetector : public ICollisionDetector
{
	ICollider *colliders[18];
	MinMax _time;
	RayIntersection *_dynamicIntersection;

public:
	KdTreeCollisionDetector();
	virtual ~KdTreeCollisionDetector();
	CollisionKdTree *tree;
	QVector<CollisionPrimitive*> *dynamicPrimitives;
	int dynamicPrimitivesNum;
	double threshold;

	bool addPrimitive(CollisionPrimitive *primitive, bool isStatic = true);
	bool removePrimitive(CollisionPrimitive *primitive, bool isStatic = true);
	void init(std::shared_ptr<QVector<std::shared_ptr<CollisionPrimitive>>> collisionPrimitives);
	Contact* getAllContacts(Contact *contacts);
	bool getContact(CollisionPrimitive *prim1, CollisionPrimitive *prim2, Contact *contact);
	bool testCollision(CollisionPrimitive *prim1, CollisionPrimitive *prim2);
	bool intersectRay(const Vector3 *origin, const Vector3 *dir, int collisionGroup, double maxTime, IRayCollisionPredicate *predicate, RayIntersection *result);
	bool intersectRayNecessarilyDynamic(const Vector3*, const Vector3*, int, double, IRayCollisionPredicate*, RayIntersection*);
	bool intersectRayWithStatic(const Vector3 *origin, const Vector3 *dir, int collisionGroup, double maxTime, IRayCollisionPredicate *predicate, RayIntersection *result);
	bool testBodyPrimitiveCollision(Body *body, CollisionPrimitive *primitive);

private:
	void addCollider(int type1, int type2, ICollider *collider);

	Contact* getPrimitiveNodeCollisions(CollisionKdNode *node, CollisionPrimitive *primitive, Contact *contacts);
	bool intersectRayWithDynamic(const Vector3 *origin, const Vector3 *dir, int collisionGroup, double maxTime, IRayCollisionPredicate *predicate, RayIntersection *result);
	bool getRayBoundBoxIntersection(const Vector3 *origin, const Vector3 *dir, BoundBox *bb, MinMax *time);
	bool testRayAgainstNode(CollisionKdNode *node, const Vector3 *origin, const Vector3 *localOrigin, const Vector3 *dir, int collisionGroup, double t1, double t2, IRayCollisionPredicate *predicate, RayIntersection *result);
	bool getRayNodeIntersection(const Vector3 *origin, const Vector3 *dir, int collisionGroup, QVector<CollisionPrimitive*> *primitives, QVector<int> *indices, IRayCollisionPredicate *predicate, RayIntersection *intersection);
	void destroy();
};

#endif // KDTREECOLLISIONDETECTOR_H
