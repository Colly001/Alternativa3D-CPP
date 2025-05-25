#ifndef SPHERESPHERECOLLIDER_H
#define SPHERESPHERECOLLIDER_H

#include "../ICollider.h"
#include "alternativa/math/vector3.h"

class SphereSphereCollider : public ICollider
{
public:
	SphereSphereCollider();
	virtual ~SphereSphereCollider();
	bool getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact);
	bool haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2);
	void destroy() override;
};

#endif // SPHERESPHERECOLLIDER_H
