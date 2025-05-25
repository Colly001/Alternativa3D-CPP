#include "contactpoint.h"
#include "alternativa/math/vector3.h"

ContactPoint::ContactPoint()
{
	pos.reset();
	r1.reset();
	r2.reset();
	penetration = 0;
	feature1 = 0;
	feature2 = 0;
	normalVel = 0;
	minSepVel = 0;
	velByUnitImpulseN = 0;
	angularInertia1 = 0;
	angularInertia2 = 0;
	accumImpulseN = 0;
	satisfied = false;
}


void ContactPoint::copyFrom(ContactPoint *cp)
{
	this->pos.copy(&cp->pos);
	this->penetration = cp->penetration;
	this->feature1 = cp->feature1;
	this->feature2 = cp->feature2;
	this->r1.copy(&cp->r1);
	this->r2.copy(&cp->r2);
}
