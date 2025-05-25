#include "physicsutils.h"
#include "alternativa/math/vector3.h"
#include "alternativa/math/matrix3.h"


void PhysicsUtils::getBoxInvInertia(double mass, const Vector3 *halfSize, Matrix3 *result)
{
	if (mass <= 0)
		throw 142343;

	result->copy(&Matrix3::ZERO);
	//if (mass == Infinity)
		//return;

	double xx = halfSize->x * halfSize->x;
	double yy = halfSize->y * halfSize->y;
	double zz = halfSize->z * halfSize->z;
	result->a = 3. / (mass * (yy + zz));
	result->f = 3. / (mass * (zz + xx));
	result->k = 3. / (mass * (xx + yy));
}

void PhysicsUtils::getCylinderInvInertia(double mass, double r, double h, Matrix3 *result)
{
	if (mass <= 0)
		throw 142345;

	result->copy(&Matrix3::ZERO);
	//if (mass == Infinity)
		//return;

	result->a = result->f = 1. / (mass * (((h * h) / 12.) + ((r * r) / 4.)));
	result->k = 2. / ((mass * r) * r);
}
