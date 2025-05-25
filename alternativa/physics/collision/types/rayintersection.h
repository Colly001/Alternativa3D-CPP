#ifndef RAYINTERSECTION_H
#define RAYINTERSECTION_H

#include "alternativa/math/vector3.h"

class CollisionPrimitive;


class RayIntersection
{
public:
	//RayIntersection();

	CollisionPrimitive *primitive;
	Vector3 pos;
	Vector3 normal;
	double t;

	void copy(RayIntersection *source);
};

#endif // RAYINTERSECTION_H
