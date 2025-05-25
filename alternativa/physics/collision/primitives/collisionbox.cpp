#include "collisionbox.h"
#include "alternativa/physics/collision/types/boundbox.h"
#include "_global.h"

CollisionBox::CollisionBox(const Vector3 *hs, int collisionGroup) :
	CollisionPrimitive(BOX, collisionGroup)
{
	this->hs.copy(hs);
	this->excludedFaces = 0;
}

CollisionBox::~CollisionBox()
{

}



BoundBox *CollisionBox::calculateAABB()
{
	Matrix4 &t = transform;
	double xx = (t.a < 0) ? -t.a : t.a;
	double yy = (t.b < 0) ? -t.b : t.b;
	double zz = (t.c < 0) ? -t.c : t.c;
	aabb.maxX = (this->hs.x * xx) + (this->hs.y * yy) + (this->hs.z * zz);
	aabb.minX = -aabb.maxX;
	xx = (t.e < 0) ? -t.e : t.e;
	yy = (t.f < 0) ? -t.f : t.f;
	zz = (t.g < 0) ? -t.g : t.g;
	aabb.maxY = (this->hs.x * xx) + (this->hs.y * yy) + (this->hs.z * zz);
	aabb.minY = -(aabb.maxY);
	xx = (t.i < 0) ? -t.i : t.i;
	yy = (t.j < 0) ? -t.j : t.j;
	zz = (t.k < 0) ? -t.k : t.k;
	aabb.maxZ = (this->hs.x * xx) + (this->hs.y * yy) + (this->hs.z * zz);
	aabb.minZ = -aabb.maxZ;
	aabb.minX = aabb.minX + t.d;
	aabb.maxX = aabb.maxX + t.d;
	aabb.minY = aabb.minY + t.h;
	aabb.maxY = aabb.maxY + t.h;
	aabb.minZ = aabb.minZ + t.l;
	aabb.maxZ = aabb.maxZ + t.l;
	return &aabb;
}

void CollisionBox::copyFrom(CollisionPrimitive *source)
{
	if (source->type != CollisionPrimitive::BOX)
		return;

	CollisionBox *box = (CollisionBox*)source;
	CollisionPrimitive::copyFrom(box);
	this->hs.copy(&box->hs);
}

CollisionPrimitive* CollisionBox::createPrimitive()
{
	return new CollisionBox(&this->hs, collisionGroup);
}

double CollisionBox::getRayIntersection(const Vector3 *origin, const Vector3 *vector, double epsilon, Vector3 *normal)
{
	normal->reset();

	double tMin = -1.;
	double tMax = 1E308;
	double vx = origin->x - transform.d;
	double vy = origin->y - transform.h;
	double vz = origin->z - transform.l;
	double ox = (transform.a * vx) + (transform.e * vy) + (transform.i * vz);
	double oy = (transform.b * vx) + (transform.f * vy) + (transform.j * vz);
	double oz = (transform.c * vx) + (transform.g * vy) + (transform.k * vz);
	vx = (transform.a * vector->x) + (transform.e * vector->y) + (transform.i * vector->z);
	vy = (transform.b * vector->x) + (transform.f * vector->y) + (transform.j * vector->z);
	vz = (transform.c * vector->x) + (transform.g * vector->y) + (transform.k * vector->z);
	if (ox >= -this->hs.x && ox <= this->hs.x && oy >= -this->hs.y && oy <= this->hs.y && oz >= -this->hs.z && oz <= this->hs.z)
	{
		normal->x = -vector->x;
		normal->y = -vector->y;
		normal->z = -vector->z;
		normal->normalize();
		return 0;
	}
	if (vx < epsilon && vx > -epsilon)
	{
		if (ox < -this->hs.x || ox > this->hs.x)
		{
			return -1;
		}
	}
	else
	{
		double t1 = (-this->hs.x - ox) / vx;
		double t2 = (this->hs.x - ox) / vx;
		if (t1 < t2)
		{
			if (t1 > tMin)
			{
				tMin = t1;
				normal->x = -1;
				normal->y = normal->z = 0;
			}
			if (t2 < tMax)
			{
				tMax = t2;
			}
		}
		else
		{
			if (t2 > tMin)
			{
				tMin = t2;
				normal->x = 1;
				normal->y = normal->z = 0;
			}
			if (t1 < tMax)
			{
				tMax = t1;
			}
		}
		if (tMax < tMin)
		{
			return -1;
		}
	}
	if (vy < epsilon && vy > -epsilon)
	{
		if (oy < -this->hs.y || oy > this->hs.y)
		{
			return -1;
		}
	}
	else
	{
		double t1 = (-this->hs.y - oy) / vy;
		double t2 = (this->hs.y - oy) / vy;
		if (t1 < t2)
		{
			if (t1 > tMin)
			{
				tMin = t1;
				normal->y = -1;
				normal->x = normal->z = 0;
			}
			if (t2 < tMax)
			{
				tMax = t2;
			}
		}
		else
		{
			if (t2 > tMin)
			{
				tMin = t2;
				normal->y = 1;
				normal->x = normal->z = 0;
			}
			if (t1 < tMax)
			{
				tMax = t1;
			}
		}
		if (tMax < tMin)
		{
			return -1;
		}
	}
	if (vz < epsilon && vz > -epsilon)
	{
		if (oz < -this->hs.z || oz > this->hs.z)
		{
			return -1;
		}
	}
	else
	{
		double t1 = (-this->hs.z - oz) / vz;
		double t2 = (this->hs.z - oz) / vz;
		if (t1 < t2)
		{
			if (t1 > tMin)
			{
				tMin = t1;
				normal->z = -1;
				normal->x = normal->y = 0;
			}
			if (t2 < tMax)
			{
				tMax = t2;
			}
		}
		else
		{
			if (t2 > tMin)
			{
				tMin = t2;
				normal->z = 1;
				normal->x = normal->y = 0;
			}
			if (t1 < tMax)
			{
				tMax = t1;
			}
		}
		if (tMax < tMin)
		{
			return -1;
		}
	}
	vx = normal->x;
	vy = normal->y;
	vz = normal->z;
	normal->x = (transform.a * vx) + (transform.b * vy) + (transform.c * vz);
	normal->y = (transform.e * vx) + (transform.f * vy) + (transform.g * vz);
	normal->z = (transform.i * vx) + (transform.j * vy) + (transform.k * vz);
	return tMin;
}

void CollisionBox::destroy()
{
	delete this;
}
