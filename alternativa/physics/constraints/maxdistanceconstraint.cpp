#include "maxdistanceconstraint.h"
#include "alternativa/physics/body.h"
#include "alternativa/physics/physicsscene.h"
#include "mymath.h"



MaxDistanceConstraint::MaxDistanceConstraint(Body *body1, Body *body2, const Vector3 *r1, const Vector3 *r2, double maxDistance)
{
	this->body1 = body1;
	this->body2 = body2;
	this->r1.copy(r1);
	this->r2.copy(r2);
	this->maxDistance = maxDistance;
}

MaxDistanceConstraint::~MaxDistanceConstraint()
{

}

void MaxDistanceConstraint::preProcess(double dt)
{
	const Matrix3 &m = this->body1->baseMatrix;
	this->wr1.x = (((m.a * this->r1.x) + (m.b * this->r1.y)) + (m.c * this->r1.z));
	this->wr1.y = (((m.e * this->r1.x) + (m.f * this->r1.y)) + (m.g * this->r1.z));
	this->wr1.z = (((m.i * this->r1.x) + (m.j * this->r1.y)) + (m.k * this->r1.z));

	if (this->body2 != nullptr)
	{
		const Matrix3 &m = this->body2->baseMatrix;
		this->wr2.x = (((m.a * this->r2.x) + (m.b * this->r2.y)) + (m.c * this->r2.z));
		this->wr2.y = (((m.e * this->r2.x) + (m.f * this->r2.y)) + (m.g * this->r2.z));
		this->wr2.z = (((m.i * this->r2.x) + (m.j * this->r2.y)) + (m.k * this->r2.z));
	}
	else
	{
		this->wr2.x = this->r2.x;
		this->wr2.y = this->r2.y;
		this->wr2.z = this->r2.z;
	}

	const Vector3 &p1 = this->body1->state.pos;
	this->impulseDirection.x = ((this->wr2.x - this->wr1.x) - p1.x);
	this->impulseDirection.y = ((this->wr2.y - this->wr1.y) - p1.y);
	this->impulseDirection.z = ((this->wr2.z - this->wr1.z) - p1.z);

	if (this->body2 != nullptr)
	{
		const Vector3 &p2 = this->body2->state.pos;
		this->impulseDirection.x = (this->impulseDirection.x + p2.x);
		this->impulseDirection.y = (this->impulseDirection.y + p2.y);
		this->impulseDirection.z = (this->impulseDirection.z + p2.z);
	}

	double len = MyMath::sqrt((((this->impulseDirection.x * this->impulseDirection.x) + (this->impulseDirection.y * this->impulseDirection.y)) + (this->impulseDirection.z * this->impulseDirection.z)));
	double delta = len - this->maxDistance;
	if (delta > 0)
	{
		satisfied = false;
		if (len < 0.001)
		{
			this->impulseDirection.x = 1;
		}
		else
		{
			len = 1. / len;
			this->impulseDirection.x = (this->impulseDirection.x * len);
			this->impulseDirection.y = (this->impulseDirection.y * len);
			this->impulseDirection.z = (this->impulseDirection.z * len);
		}
		this->minClosingVel = (delta / (world->penResolutionSteps * dt));
		if (this->minClosingVel > world->maxPenResolutionSpeed)
		{
			this->minClosingVel = world->maxPenResolutionSpeed;
		}
		this->velByUnitImpulseN = 0;
		if (this->body1->movable)
		{
			double vx = ((this->wr1.y * this->impulseDirection.z) - (this->wr1.z * this->impulseDirection.y));
			double vy = ((this->wr1.z * this->impulseDirection.x) - (this->wr1.x * this->impulseDirection.z));
			double vz = ((this->wr1.x * this->impulseDirection.y) - (this->wr1.y * this->impulseDirection.x));
			const Matrix3 &m = this->body1->invInertiaWorld;
			double x = (((m.a * vx) + (m.b * vy)) + (m.c * vz));
			double y = (((m.e * vx) + (m.f * vy)) + (m.g * vz));
			double z = (((m.i * vx) + (m.j * vy)) + (m.k * vz));
			vx = ((y * this->wr1.z) - (z * this->wr1.y));
			vy = ((z * this->wr1.x) - (x * this->wr1.z));
			vz = ((x * this->wr1.y) - (y * this->wr1.x));
			this->velByUnitImpulseN = (this->velByUnitImpulseN + (((this->body1->invMass + (vx * this->impulseDirection.x)) + (vy * this->impulseDirection.y)) + (vz * this->impulseDirection.z)));
		}
		if (((!(this->body2 == nullptr)) && (this->body2->movable)))
		{
			double vx = ((this->wr2.y * this->impulseDirection.z) - (this->wr2.z * this->impulseDirection.y));
			double vy = ((this->wr2.z * this->impulseDirection.x) - (this->wr2.x * this->impulseDirection.z));
			double vz = ((this->wr2.x * this->impulseDirection.y) - (this->wr2.y * this->impulseDirection.x));
			const Matrix3 &m = this->body2->invInertiaWorld;
			double x = (((m.a * vx) + (m.b * vy)) + (m.c * vz));
			double y = (((m.e * vx) + (m.f * vy)) + (m.g * vz));
			double z = (((m.i * vx) + (m.j * vy)) + (m.k * vz));
			vx = ((y * this->wr2.z) - (z * this->wr2.y));
			vy = ((z * this->wr2.x) - (x * this->wr2.z));
			vz = ((x * this->wr2.y) - (y * this->wr2.x));
			this->velByUnitImpulseN = (this->velByUnitImpulseN + (((this->body2->invMass + (vx * this->impulseDirection.x)) + (vy * this->impulseDirection.y)) + (vz * this->impulseDirection.z)));
		}
	}
	else
	{
		satisfied = true;
	}
}

void MaxDistanceConstraint::apply(double dt)
{
	(void)dt;

	if (satisfied)
		return;

	const Vector3 &vel = this->body1->state.velocity;
	const Vector3 &rot = this->body1->state.rotation;
	double vx = ((vel.x + (rot.y * this->wr1.z)) - (rot.z * this->wr1.y));
	double vy = ((vel.y + (rot.z * this->wr1.x)) - (rot.x * this->wr1.z));
	double vz = ((vel.z + (rot.x * this->wr1.y)) - (rot.y * this->wr1.x));
	if (this->body2 != nullptr)
	{
		const Vector3 &vel = this->body2->state.velocity;
		const Vector3 &rot = this->body2->state.rotation;
		vx = (vx - ((vel.x + (rot.y * this->wr2.z)) - (rot.z * this->wr2.y)));
		vy = (vy - ((vel.y + (rot.z * this->wr2.x)) - (rot.x * this->wr2.z)));
		vz = (vz - ((vel.z + (rot.x * this->wr2.y)) - (rot.y * this->wr2.x)));
	}
	double closingVel = (((vx * this->impulseDirection.x) + (vy * this->impulseDirection.y)) + (vz * this->impulseDirection.z));
	if (closingVel > this->minClosingVel)
	{
		return;
	}
	double impulse = ((this->minClosingVel - closingVel) / this->velByUnitImpulseN);
	if (this->body1->movable)
	{
		this->body1->applyRelPosWorldImpulse(&this->wr1, &this->impulseDirection, impulse);
	}
	if (((!(this->body2 == nullptr)) && (this->body2->movable)))
	{
		this->body2->applyRelPosWorldImpulse(&this->wr2, &this->impulseDirection, -(impulse));
	}
}
