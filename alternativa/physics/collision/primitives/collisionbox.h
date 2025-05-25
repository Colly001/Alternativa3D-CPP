#ifndef COLLISIONBOX_H
#define COLLISIONBOX_H

#include "alternativa/math/vector3.h"
#include "../collisionprimitive.h"


class CollisionBox : public CollisionPrimitive
{
public:
	Vector3 hs;
	int excludedFaces;


	CollisionBox(const Vector3 *hs, int collisionGroup);
	virtual ~CollisionBox();
	BoundBox* calculateAABB() override;
	void copyFrom(CollisionPrimitive *source) override;
	double getRayIntersection(const Vector3 *origin, const Vector3 *vector, double epsilon, Vector3 *normal) override;
	void destroy() override;

protected:
	CollisionPrimitive* createPrimitive() override;
};

#endif // COLLISIONBOX_H
