#include "rayintersectiondata.h"


void RayIntersectionData::copy(const RayIntersectionData *other)
{
	this->object = other->object;
	this->face = other->face;
	this->point.copy(&other->point);
	this->uv.copy(&other->uv);
	this->time = other->time;
}
