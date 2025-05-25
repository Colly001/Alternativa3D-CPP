#ifndef BOXSPHERECOLLIDER_H
#define BOXSPHERECOLLIDER_H

#include "../ICollider.h"
#include "alternativa/math/vector3.h"

class BoxSphereCollider : public ICollider
{
	Vector3 center;
	Vector3 closestPt;
	Vector3 bPos;
	Vector3 sPos;
public:
	virtual ~BoxSphereCollider();
	bool getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact);
	bool haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2);
	void destroy() override;
};

#endif // BOXSPHERECOLLIDER_H
