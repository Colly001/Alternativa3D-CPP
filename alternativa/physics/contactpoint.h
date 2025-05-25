#ifndef CONTACTPOINT_H
#define CONTACTPOINT_H

class Vector3;
#include "alternativa/math/vector3.h"

class ContactPoint
{
public:
	ContactPoint();

	Vector3 pos;
	Vector3 r1;
	Vector3 r2;

	double penetration;
	double normalVel;
	double minSepVel;
	double velByUnitImpulseN;
	double angularInertia1;
	double angularInertia2;
	double accumImpulseN;

	int feature1;
	int feature2;

	bool satisfied;

	void copyFrom(ContactPoint *cp);
};

#endif // CONTACTPOINT_H
