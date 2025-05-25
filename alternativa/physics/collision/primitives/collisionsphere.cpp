#include "collisionsphere.h"
#include "alternativa/physics/collision/types/boundbox.h"
#include "alternativa/math/vector3.h"
#include "mymath.h"


CollisionSphere::CollisionSphere(double r, int collisionGroup) :
	CollisionPrimitive(SPHERE, collisionGroup)
{
	this->r = r;
}

CollisionSphere::~CollisionSphere()
{

}

BoundBox *CollisionSphere::calculateAABB()
{
	aabb.maxX = (transform.d + this->r);
	aabb.minX = (transform.d - this->r);
	aabb.maxY = (transform.h + this->r);
	aabb.minY = (transform.h - this->r);
	aabb.maxZ = (transform.l + this->r);
	aabb.minZ = (transform.l - this->r);
	return &aabb;
}

double CollisionSphere::getRayIntersection(const Vector3 *origin, const Vector3 *vector, double threshold, Vector3 *normal)
{
	(void)threshold;
	(void)normal;
	double px = (origin->x - transform.d);
	double py = (origin->y - transform.h);
	double pz = (origin->z - transform.l);
	double k = (((vector->x * px) + (vector->y * py)) + (vector->z * pz));
	if (k > 0)
	{
		return (-1);
	}
	double a = (((vector->x * vector->x) + (vector->y * vector->y)) + (vector->z * vector->z));
	double D = ((k * k) - (a * ((((px * px) + (py * py)) + (pz * pz)) - (this->r * this->r))));
	if (D < 0)
	{
		return (-1);
	}
	return (-(k + MyMath::sqrt(D)) / a);
}

void CollisionSphere::copyFrom(CollisionPrimitive *source)
{
	if (source->type != CollisionPrimitive::SPHERE)
		return;

	CollisionSphere *sphere = (CollisionSphere*)source;
	CollisionPrimitive::copyFrom(sphere);
	this->r = sphere->r;
}

void CollisionSphere::destroy()
{
	delete this;
}

CollisionPrimitive *CollisionSphere::createPrimitive()
{
	return new CollisionSphere(this->r, collisionGroup);
}
