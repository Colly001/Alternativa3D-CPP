#ifndef COLLISIONRECT_H
#define COLLISIONRECT_H

#include "../collisionprimitive.h"
#include "alternativa/math/vector3.h"


class CollisionRect : public CollisionPrimitive
{
public:
	Vector3 hs;
	bool twoSided;

	CollisionRect(Vector3 *hs, int collisionGroup);
	virtual ~CollisionRect();
	BoundBox* calculateAABB() override;
	void copyFrom(CollisionPrimitive *source) override;
	double getRayIntersection(const Vector3 *origin, const Vector3 *vector, double threshold, Vector3 *normal) override;

	void destroy() override;

protected:
	CollisionPrimitive* createPrimitive() override;
};

#endif // COLLISIONRECT_H
