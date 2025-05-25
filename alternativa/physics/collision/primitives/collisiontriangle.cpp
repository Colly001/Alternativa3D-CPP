#include "collisiontriangle.h"
#include "alternativa/physics/collision/types/boundbox.h"


CollisionTriangle::CollisionTriangle(Vector3 *v0, Vector3 *v1, Vector3 *v2, int collisionGroup) :
	CollisionPrimitive(TRIANGLE, collisionGroup)
{
	this->initVertices(v0, v1, v2);
}

CollisionTriangle::~CollisionTriangle()
{

}

BoundBox* CollisionTriangle::calculateAABB()
{
	double epsilon = 0.005;
	double eps_c = (epsilon * transform.c);
	double eps_g = (epsilon * transform.g);
	double eps_k = (epsilon * transform.k);
	double a = ((this->v0.x * transform.a) + (this->v0.y * transform.b));
	aabb.minX = (aabb.maxX = (a + eps_c));
	double b = (a - eps_c);
	if (b > aabb.maxX)
	{
		aabb.maxX = b;
	}
	else
	{
		if (b < aabb.minX)
		{
			aabb.minX = b;
		}
	}
	a = ((this->v0.x * transform.e) + (this->v0.y * transform.f));
	aabb.minY = (aabb.maxY = (a + eps_g));
	b = (a - eps_g);
	if (b > aabb.maxY)
	{
		aabb.maxY = b;
	}
	else
	{
		if (b < aabb.minY)
		{
			aabb.minY = b;
		}
	}
	a = ((this->v0.x * transform.i) + (this->v0.y * transform.j));
	aabb.minZ = (aabb.maxZ = (a + eps_k));
	b = (a - eps_k);
	if (b > aabb.maxZ)
	{
		aabb.maxZ = b;
	}
	else
	{
		if (b < aabb.minZ)
		{
			aabb.minZ = b;
		}
	}
	a = ((this->v1.x * transform.a) + (this->v1.y * transform.b));
	b = (a + eps_c);
	if (b > aabb.maxX)
	{
		aabb.maxX = b;
	}
	else
	{
		if (b < aabb.minX)
		{
			aabb.minX = b;
		}
	}
	b = (a - eps_c);
	if (b > aabb.maxX)
	{
		aabb.maxX = b;
	}
	else
	{
		if (b < aabb.minX)
		{
			aabb.minX = b;
		}
	}
	a = ((this->v1.x * transform.e) + (this->v1.y * transform.f));
	b = (a + eps_g);
	if (b > aabb.maxY)
	{
		aabb.maxY = b;
	}
	else
	{
		if (b < aabb.minY)
		{
			aabb.minY = b;
		}
	}
	b = (a - eps_g);
	if (b > aabb.maxY)
	{
		aabb.maxY = b;
	}
	else
	{
		if (b < aabb.minY)
		{
			aabb.minY = b;
		}
	}
	a = ((this->v1.x * transform.i) + (this->v1.y * transform.j));
	b = (a + eps_k);
	if (b > aabb.maxZ)
	{
		aabb.maxZ = b;
	}
	else
	{
		if (b < aabb.minZ)
		{
			aabb.minZ = b;
		}
	}
	b = (a - eps_k);
	if (b > aabb.maxZ)
	{
		aabb.maxZ = b;
	}
	else
	{
		if (b < aabb.minZ)
		{
			aabb.minZ = b;
		}
	}
	a = ((this->v2.x * transform.a) + (this->v2.y * transform.b));
	b = (a + eps_c);
	if (b > aabb.maxX)
	{
		aabb.maxX = b;
	}
	else
	{
		if (b < aabb.minX)
		{
			aabb.minX = b;
		}
	}
	b = (a - eps_c);
	if (b > aabb.maxX)
	{
		aabb.maxX = b;
	}
	else
	{
		if (b < aabb.minX)
		{
			aabb.minX = b;
		}
	}
	a = ((this->v2.x * transform.e) + (this->v2.y * transform.f));
	b = (a + eps_g);
	if (b > aabb.maxY)
	{
		aabb.maxY = b;
	}
	else
	{
		if (b < aabb.minY)
		{
			aabb.minY = b;
		}
	}
	b = (a - eps_g);
	if (b > aabb.maxY)
	{
		aabb.maxY = b;
	}
	else
	{
		if (b < aabb.minY)
		{
			aabb.minY = b;
		}
	}
	a = ((this->v2.x * transform.i) + (this->v2.y * transform.j));
	b = (a + eps_k);
	if (b > aabb.maxZ)
	{
		aabb.maxZ = b;
	}
	else
	{
		if (b < aabb.minZ)
		{
			aabb.minZ = b;
		}
	}
	b = (a - eps_k);
	if (b > aabb.maxZ)
	{
		aabb.maxZ = b;
	}
	else
	{
		if (b < aabb.minZ)
		{
			aabb.minZ = b;
		}
	}
	aabb.minX = aabb.minX + transform.d;
	aabb.maxX = aabb.maxX + transform.d;
	aabb.minY = aabb.minY + transform.h;
	aabb.maxY = aabb.maxY + transform.h;
	aabb.minZ = aabb.minZ + transform.l;
	aabb.maxZ = aabb.maxZ + transform.l;
	return &aabb;
}

double CollisionTriangle::getRayIntersection(const Vector3 *origin, const Vector3 *vector, double epsilon, Vector3 *normal)
{
	double vz = (((vector->x * transform.c) + (vector->y * transform.g)) + (vector->z * transform.k));
	if (((vz < epsilon) && (vz > -(epsilon))))
	{
		return -1;
	}
	double tx = (origin->x - transform.d);
	double ty = (origin->y - transform.h);
	double tz = (origin->z - transform.l);
	double oz = (((tx * transform.c) + (ty * transform.g)) + (tz * transform.k));
	double t = -oz / vz;
	if (t < 0)
	{
		return -1;
	}
	double ox = (((tx * transform.a) + (ty * transform.e)) + (tz * transform.i));
	double oy = (((tx * transform.b) + (ty * transform.f)) + (tz * transform.j));
	tx = (ox + (t * (((vector->x * transform.a) + (vector->y * transform.e)) + (vector->z * transform.i))));
	ty = (oy + (t * (((vector->x * transform.b) + (vector->y * transform.f)) + (vector->z * transform.j))));
	tz = (oz + (t * vz));
	if ((((((this->e0.x * (ty - this->v0.y)) - (this->e0.y * (tx - this->v0.x))) < 0) || (((this->e1.x * (ty - this->v1.y)) - (this->e1.y * (tx - this->v1.x))) < 0)) || (((this->e2.x * (ty - this->v2.y)) - (this->e2.y * (tx - this->v2.x))) < 0)))
	{
		return -1;
	}
	normal->x = transform.c;
	normal->y = transform.g;
	normal->z = transform.k;
	return t;
}

void CollisionTriangle::copyFrom(CollisionPrimitive *source)
{
	CollisionPrimitive::copyFrom(source);
	if (source->type == CollisionPrimitive::TRIANGLE)
	{
		CollisionTriangle *tri = (CollisionTriangle*)source;
		this->v0.copy(&tri->v0);
		this->v1.copy(&tri->v1);
		this->v2.copy(&tri->v2);
		this->e0.copy(&tri->e0);
		this->e1.copy(&tri->e1);
		this->e2.copy(&tri->e2);
	}
}

void CollisionTriangle::destroy()
{
	delete this;
}

CollisionPrimitive* CollisionTriangle::createPrimitive()
{
	return new CollisionTriangle(&this->v0, &this->v1, &this->v2, collisionGroup);
}

void CollisionTriangle::initVertices(Vector3 *v0, Vector3 *v1, Vector3 *v2)
{
	this->v0.copy(v0);
	this->v1.copy(v1);
	this->v2.copy(v2);
	this->e0.diff(v1, v0);
	this->e0.normalize();
	this->e1.diff(v2, v1);
	this->e1.normalize();
	this->e2.diff(v0, v2);
	this->e2.normalize();
}
