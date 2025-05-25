#ifndef RAYINTERSECTIONDATA_H
#define RAYINTERSECTIONDATA_H

#include "flash/geom/point.h"
#include "alternativa/math/vector3.h"

class Object3D;
class Face;

class RayIntersectionData
{
public:
	Object3D *object;
	Face *face;
	Vector3 point;
	Point uv;
	double time;

	void copy(const RayIntersectionData *other);
};

#endif // RAYINTERSECTIONDATA_H
