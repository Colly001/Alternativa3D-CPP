#include "boxspherecollider.h"
#include "alternativa/physics/collision/primitives/collisionsphere.h"
#include "alternativa/physics/collision/primitives/collisionbox.h"
#include "alternativa/physics/contact.h"
#include "alternativa/physics/contactpoint.h"
#include "mymath.h"
#include <QDebug>

BoxSphereCollider::~BoxSphereCollider()
{

}

bool BoxSphereCollider::getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact)
{
	CollisionBox *box;
	CollisionSphere *sphere;

	if (prim1->type != CollisionPrimitive::SPHERE)
	{
		box = (CollisionBox*)prim1;
		sphere = (CollisionSphere*)prim2;
	}
	else
	{
		box = (CollisionBox*)prim2;
		sphere = (CollisionSphere*)prim1;
	}

	sphere->transform.getAxis(3, &this->sPos);
	box->transform.getAxis(3, &this->bPos);
	box->transform.transformVectorInverse(&this->sPos, &this->center);
	const Vector3 &hs = box->hs;
	double sx = (hs.x + sphere->r);
	double sy = (hs.y + sphere->r);
	double sz = (hs.z + sphere->r);

	if (((((((this->center.x > sx) || (this->center.x < -(sx))) || (this->center.y > sy)) || (this->center.y < -(sy))) || (this->center.z > sz)) || (this->center.z < -(sz))))
	{
		return false;
	}
	if (this->center.x > hs.x)
	{
		this->closestPt.x = hs.x;
	}
	else
	{
		if (this->center.x < -(hs.x))
		{
			this->closestPt.x = -(hs.x);
		}
		else
		{
			this->closestPt.x = this->center.x;
		}
	}
	if (this->center.y > hs.y)
	{
		this->closestPt.y = hs.y;
	}
	else
	{
		if (this->center.y < -(hs.y))
		{
			this->closestPt.y = -(hs.y);
		}
		else
		{
			this->closestPt.y = this->center.y;
		}
	}
	if (this->center.z > hs.z)
	{
		this->closestPt.z = hs.z;
	}
	else
	{
		if (this->center.z < -(hs.z))
		{
			this->closestPt.z = -(hs.z);
		}
		else
		{
			this->closestPt.z = this->center.z;
		}
	}
	this->center.subtract(&this->closestPt);
	double distSqr = this->center.lengthSqr();
	if (distSqr > (sphere->r * sphere->r))
	{
		return false;
	}
	contact->body1 = sphere->body;
	contact->body2 = box->body;
	contact->normal.copy(&this->closestPt);
	contact->normal.transformBy4(&box->transform);
	contact->normal.subtract(&this->sPos);
	contact->normal.normalize();
	contact->normal.reverse();
	//vTransformBy4(box.transform).vSubtract(this->sPos).vNormalize().vReverse();


	contact->pcount = 1;
	ContactPoint *cp = contact->points[0];
	cp->penetration = (sphere->r - MyMath::sqrt(distSqr));
	cp->pos.copy(&contact->normal);
	cp->pos.scale(-sphere->r);
	cp->pos.add(&this->sPos);
	cp->r1.diff(&cp->pos, &this->sPos);
	cp->r2.diff(&cp->pos, &this->bPos);
	return true;
}

bool BoxSphereCollider::haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2)
{
	CollisionBox *box;
	CollisionSphere *sphere;

	if (prim1->type != CollisionPrimitive::SPHERE)
	{
		box = (CollisionBox*)prim1;
		sphere = (CollisionSphere*)prim2;
	}
	else
	{
		box = (CollisionBox*)prim2;
		sphere = (CollisionSphere*)prim1;
	}

	sphere->transform.getAxis(3, &this->sPos);
	box->transform.getAxis(3, &this->bPos);
	box->transform.transformVectorInverse(&this->sPos, &this->center);
	const Vector3 &hs = box->hs;
	double sx = (hs.x + sphere->r);
	double sy = (hs.y + sphere->r);
	double sz = (hs.z + sphere->r);
	if (((((((this->center.x > sx) || (this->center.x < -(sx))) || (this->center.y > sy)) || (this->center.y < -(sy))) || (this->center.z > sz)) || (this->center.z < -(sz))))
	{
		return false;
	}
	if (this->center.x > hs.x)
	{
		this->closestPt.x = hs.x;
	}
	else
	{
		if (this->center.x < -(hs.x))
		{
			this->closestPt.x = -(hs.x);
		}
		else
		{
			this->closestPt.x = this->center.x;
		}
	}
	if (this->center.y > hs.y)
	{
		this->closestPt.y = hs.y;
	}
	else
	{
		if (this->center.y < -(hs.y))
		{
			this->closestPt.y = -(hs.y);
		}
		else
		{
			this->closestPt.y = this->center.y;
		}
	}
	if (this->center.z > hs.z)
	{
		this->closestPt.z = hs.z;
	}
	else
	{
		if (this->center.z < -(hs.z))
		{
			this->closestPt.z = -(hs.z);
		}
		else
		{
			this->closestPt.z = this->center.z;
		}
	}

	this->center.subtract(&this->closestPt);
	double distSqr = this->center.lengthSqr();

	if (distSqr > (sphere->r * sphere->r))
	{
		return false;
	}
	return true;
}

void BoxSphereCollider::destroy()
{
	delete this;
}
