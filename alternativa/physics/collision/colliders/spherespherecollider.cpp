#include "spherespherecollider.h"
#include "alternativa/physics/collision/collisionprimitive.h"
#include "alternativa/physics/collision/primitives/collisionsphere.h"
#include "alternativa/physics/contact.h"
#include "alternativa/physics/contactpoint.h"
#include "mymath.h"

#include <QDebug>

SphereSphereCollider::SphereSphereCollider()
{

}

SphereSphereCollider::~SphereSphereCollider()
{

}

bool SphereSphereCollider::getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact)
{
	CollisionSphere *s1;
	CollisionSphere *s2;

	if (prim1->body != nullptr)
	{
		s1 = (CollisionSphere*)prim1;
		s2 = (CollisionSphere*)prim2;
	}
	else
	{
		s1 = (CollisionSphere*)prim2;
		s2 = (CollisionSphere*)prim1;
	}

	Vector3 p1, p2;

	s1->transform.getAxis(3, &p1);
	s2->transform.getAxis(3, &p2);

	double dx = (p1.x - p2.x);
	double dy = (p1.y - p2.y);
	double dz = (p1.z - p2.z);
	double len = (((dx * dx) + (dy * dy)) + (dz * dz));
	double sum = (s1->r + s2->r);
	if (len > (sum * sum))
	{
		return false;
	}
	len = MyMath::sqrt(len);
	dx = (dx / len);
	dy = (dy / len);
	dz = (dz / len);
	contact->body1 = s1->body;
	contact->body2 = s2->body;
	contact->normal.x = dx;
	contact->normal.y = dy;
	contact->normal.z = dz;
	contact->pcount = 1;
	ContactPoint *cp = contact->points[0];
	cp->penetration = (sum - len);
	cp->pos.x = (p1.x - (dx * s1->r));
	cp->pos.y = (p1.y - (dy * s1->r));
	cp->pos.z = (p1.z - (dz * s1->r));
	cp->r1.diff(&cp->pos, &p1);
	cp->r2.diff(&cp->pos, &p2);
	return true;
}

bool SphereSphereCollider::haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2)
{
	(void)prim1;
	(void)prim2;
	return false;
}

void SphereSphereCollider::destroy()
{
	delete this;
}
