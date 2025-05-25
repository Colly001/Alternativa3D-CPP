#ifndef BODYSTATE_H
#define BODYSTATE_H

#include "alternativa/math/vector3.h"
#include "alternativa/math/quaternion.h"

class BodyState
{
public:
	BodyState();

	//double posX;
	//double posY;
	//double posZ;
	//double velX;
	//double velY;
	//double velZ;

	Vector3 pos;
	Quaternion orientation;
	Vector3 velocity;
	Vector3 rotation;
	void copy(BodyState *state);
};

#endif // BODYSTATE_H
