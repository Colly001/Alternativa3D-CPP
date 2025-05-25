#ifndef PHYSICSUTILS_H
#define PHYSICSUTILS_H

class Vector3;
class Matrix3;

class PhysicsUtils
{
public:
	static void getBoxInvInertia(double mass, const Vector3 *halfSize, Matrix3 *result);
	static void getCylinderInvInertia(double mass, double r, double h, Matrix3 *result);
};

#endif // PHYSICSUTILS_H
