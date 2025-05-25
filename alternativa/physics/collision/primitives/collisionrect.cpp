#include "collisionrect.h"
#include "alternativa/physics/collision/types/boundbox.h"

#define EPSILON 0.005


CollisionRect::CollisionRect(Vector3 *hs, int collisionGroup) :
	CollisionPrimitive(RECT, collisionGroup)
{
	this->twoSided = true;
	this->hs.copy(hs);
}

CollisionRect::~CollisionRect()
{

}

BoundBox *CollisionRect::calculateAABB()
{
	Matrix4 &t = transform;
	double xx = ((t.a < 0) ? -(t.a) : t.a);
	double yy = ((t.b < 0) ? -(t.b) : t.b);
	double zz = ((t.c < 0) ? -(t.c) : t.c);
	aabb.maxX = (((this->hs.x * xx) + (this->hs.y * yy)) + (EPSILON * zz));
	aabb.minX = -(aabb.maxX);
	xx = ((t.e < 0) ? -(t.e) : t.e);
	yy = ((t.f < 0) ? -(t.f) : t.f);
	zz = ((t.g < 0) ? -(t.g) : t.g);
	aabb.maxY = (((this->hs.x * xx) + (this->hs.y * yy)) + (EPSILON * zz));
	aabb.minY = -(aabb.maxY);
	xx = ((t.i < 0) ? -(t.i) : t.i);
	yy = ((t.j < 0) ? -(t.j) : t.j);
	zz = ((t.k < 0) ? -(t.k) : t.k);
	aabb.maxZ = (((this->hs.x * xx) + (this->hs.y * yy)) + (EPSILON * zz));
	aabb.minZ = -(aabb.maxZ);
	aabb.minX = (aabb.minX + t.d);
	aabb.maxX = (aabb.maxX + t.d);
	aabb.minY = (aabb.minY + t.h);
	aabb.maxY = (aabb.maxY + t.h);
	aabb.minZ = (aabb.minZ + t.l);
	aabb.maxZ = (aabb.maxZ + t.l);
	return &aabb;
}

void CollisionRect::copyFrom(CollisionPrimitive *source)
{
	if (source->type != CollisionPrimitive::RECT)
		return;
	CollisionRect *rect = (CollisionRect*)source;
	CollisionPrimitive::copyFrom(rect);
	this->hs.copy(&rect->hs);
}

double CollisionRect::getRayIntersection(const Vector3 *origin, const Vector3 *vector, double threshold, Vector3 *normal)
{
	double vx = (origin->x - transform.d);
	double vy = (origin->y - transform.h);
	double vz = (origin->z - transform.l);
	double ox = (((transform.a * vx) + (transform.e * vy)) + (transform.i * vz));
	double oy = (((transform.b * vx) + (transform.f * vy)) + (transform.j * vz));
	double oz = (((transform.c * vx) + (transform.g * vy)) + (transform.k * vz));
	vx = (((transform.a * vector->x) + (transform.e * vector->y)) + (transform.i * vector->z));
	vy = (((transform.b * vector->x) + (transform.f * vector->y)) + (transform.j * vector->z));
	vz = (((transform.c * vector->x) + (transform.g * vector->y)) + (transform.k * vector->z));
	if (((vz > -(threshold)) && (vz < threshold)))
	{
		return -1;
	}
	double t = (-(oz) / vz);
	if (t < 0)
	{
		return (-1);
	}
	ox = (ox + (vx * t));
	oy = (oy + (vy * t));
	oz = 0;
	if (((((ox < (-(this->hs.x) - threshold)) || (ox > (this->hs.x + threshold))) || (oy < (-(this->hs.y) - threshold))) || (oy > (this->hs.y + threshold))))
	{
		return -1;
	}
	normal->x = transform.c;
	normal->y = transform.g;
	normal->z = transform.k;
	return t;
}

void CollisionRect::destroy()
{
	delete this;
}

CollisionPrimitive *CollisionRect::createPrimitive()
{
	return new CollisionRect(&this->hs, collisionGroup);
}
