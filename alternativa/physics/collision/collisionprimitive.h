#ifndef COLLISIONPRIMITIVE_H
#define COLLISIONPRIMITIVE_H

class ICollisionPredicate;
class Body;
class Matrix4;
class BoundBox;
class Vector3;

#include "alternativa/math/matrix4.h"
#include "alternativa/physics/collision/types/boundbox.h"

class CollisionPrimitive
{
public:
	static const int BOX;
	static const int PLANE;
	static const int SPHERE;
	static const int RECT;
	static const int TRIANGLE;

	int type;
	int collisionGroup;
	ICollisionPredicate *postCollisionPredicate;
	Body *body;
	Matrix4 *localTransform;
	Matrix4 transform;
	BoundBox aabb;

	CollisionPrimitive(int type, int collisionGroup);
	~CollisionPrimitive();
	void setBody(Body *body, const Matrix4 *localTransform = nullptr);
	virtual BoundBox* calculateAABB();
	virtual double getRayIntersection(const Vector3 *origin, const Vector3 *vector, double epsilon, Vector3 *normal);
	virtual void copyFrom(CollisionPrimitive *source);
	virtual void destroy() = 0;

protected:
	virtual CollisionPrimitive* createPrimitive();
};


#endif // COLLISIONPRIMITIVE_H
