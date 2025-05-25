#ifndef COLLISIONTRIANGLE_H
#define COLLISIONTRIANGLE_H

#include "../collisionprimitive.h"
#include "alternativa/math/vector3.h"

class CollisionTriangle : public CollisionPrimitive
{
public:

	Vector3 v0;
	Vector3 v1;
	Vector3 v2;

	Vector3 e0;
	Vector3 e1;
	Vector3 e2;

	CollisionTriangle(Vector3 *v0, Vector3 *v1, Vector3 *v2, int collisionGroup);
	virtual ~CollisionTriangle();
	BoundBox* calculateAABB() override;
	double getRayIntersection(const Vector3 *origin, const Vector3 *vector, double epsilon, Vector3 *normal) override;
	void copyFrom(CollisionPrimitive *source) override;
	void destroy() override;


protected:
	CollisionPrimitive* createPrimitive() override;

private:
	void initVertices(Vector3 *v0, Vector3 *v1, Vector3 *v2);
};

#endif // COLLISIONTRIANGLE_H
