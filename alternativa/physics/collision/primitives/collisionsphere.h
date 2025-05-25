#ifndef COLLISIONSPHERE_H
#define COLLISIONSPHERE_H

#include "../collisionprimitive.h"

class CollisionSphere : public CollisionPrimitive
{
public:
	double r;

	CollisionSphere(double r, int collisionGroup);
	virtual ~CollisionSphere();

	BoundBox *calculateAABB() override;
	double getRayIntersection(const Vector3 *origin, const Vector3 *vector, double threshold, Vector3 *normal) override;
	void copyFrom(CollisionPrimitive *source) override;
	void destroy() override;


protected:
	CollisionPrimitive* createPrimitive() override;
};

#endif // COLLISIONSPHERE_H
