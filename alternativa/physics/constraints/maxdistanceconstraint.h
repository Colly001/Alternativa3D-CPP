#ifndef MAXDISTANCECONSTRAINT_H
#define MAXDISTANCECONSTRAINT_H

#include "constraint.h"
#include "alternativa/math/vector3.h"

class Body;


class MaxDistanceConstraint : public Constraint
{
	Body *body1;
	Body *body2;

	Vector3 r1;
	Vector3 r2;
	Vector3 wr1;
	Vector3 wr2;
	double minClosingVel;
	double velByUnitImpulseN;
	Vector3 impulseDirection;
	double maxDistance;

public:

	MaxDistanceConstraint(Body *body1, Body *body2, const Vector3 *r1, const Vector3 *r2, double maxDistance);
	virtual ~MaxDistanceConstraint();

	void preProcess(double dt) override;
	void apply(double dt) override;
};

#endif // MAXDISTANCECONSTRAINT_H
