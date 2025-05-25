#include "rayintersection.h"

/*
RayIntersection::RayIntersection()
{
	this->t = 0.;
	this->primitive = nullptr;
	pos.reset();
	normal.reset();
}
*/

void RayIntersection::copy(RayIntersection *source)
{
	this->primitive = source->primitive;
	this->pos.copy(&source->pos);
	this->normal.copy(&source->normal);
	this->t = source->t;
}
