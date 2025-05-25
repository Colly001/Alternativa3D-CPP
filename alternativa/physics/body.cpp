#include "body.h"
#include "alternativa/math/quaternion.h"
#include "collision/types/boundbox.h"
#include "collisionprimitivelistitem.h"
#include "collisionprimitivelist.h"
#include "alternativa/math/matrix4.h"
#include "collision/collisionprimitive.h"
#include "bodystate.h"
#include "alternativa/math/quaternion.h"
#include "_global.h"
#include "mymath.h"
//#include <QDebug>

const Body::T *Body::TYPE = new Body::T("Body");

double Body::linDamping = 0.997;
double Body::rotDamping = 0.997;
//const int Body::MAX_CONTACTS = 20;
static const double MAX_SPEED_Z = 1500;


Body::Body(const T *type, double invMass, const Matrix3 *invInertia, bool localTank)
{
	(void)localTank;
	this->type = type;
	this->contactsNum = 0;
	this->freezeCounter = 0;
	this->canFreeze = false;
	this->frozen = false;
	this->movable = true;
	this->invMass = invMass;
	this->invInertia.copy(invInertia);
	state.pos.reset();
	state.orientation.reset();
	state.rotation.reset();
	state.velocity.reset();
	prevState.copy(&state);
	accel.reset();
	angleAccel.reset();
	torqueAccum.reset();
	forceAccum.reset();
	baseMatrix.copy(&Matrix3::IDENTITY);
	aabb.infinity();
	invInertiaWorld.copy(&Matrix3::IDENTITY);
	collisionPrimitives = nullptr;
	postCollisionPredicate = nullptr;
	maxSpeedXY = 0;
	previousVelocity.reset();
	minZAcceleration = -1100;
	maxZSpeedDelta = 600;
	slipperyMode = false;
#ifdef L_MT
	mass = 1;
#endif
}

Body::~Body()
{
	if (this->collisionPrimitives != nullptr) {
		this->collisionPrimitives->clear();
		delete this->collisionPrimitives;
	}
}

void Body::destroy()
{
	delete this;
}


void Body::calcAccelerations()
{
	this->accel.x = this->forceAccum.x * this->invMass;
	this->accel.y = this->forceAccum.y * this->invMass;
	this->accel.z = this->forceAccum.z * this->invMass;
	this->angleAccel.x = (this->invInertiaWorld.a * this->torqueAccum.x) + (this->invInertiaWorld.b * this->torqueAccum.y) + (this->invInertiaWorld.c * this->torqueAccum.z);
	this->angleAccel.y = (this->invInertiaWorld.e * this->torqueAccum.x) + (this->invInertiaWorld.f * this->torqueAccum.y) + (this->invInertiaWorld.g * this->torqueAccum.z);
	this->angleAccel.z = (this->invInertiaWorld.i * this->torqueAccum.x) + (this->invInertiaWorld.j * this->torqueAccum.y) + (this->invInertiaWorld.k * this->torqueAccum.z);
}

void Body::saveState()
{
	this->prevState.copy(&this->state);
}


void Body::integrateVelocity(double dt)
{
#ifndef L_MT
	this->state.velocity.x = this->state.velocity.x + (this->accel.x * dt);
	this->state.velocity.y = this->state.velocity.y + (this->accel.y * dt);
	this->state.velocity.z = this->state.velocity.z + (this->accel.z * dt);

	this->state.rotation.x = this->state.rotation.x + (this->angleAccel.x * dt);
	this->state.rotation.y = this->state.rotation.y + (this->angleAccel.y * dt);
	this->state.rotation.z = this->state.rotation.z + (this->angleAccel.z * dt);

	this->state.velocity.x = this->state.velocity.x * linDamping;
	this->state.velocity.y = this->state.velocity.y * linDamping;
	this->state.velocity.z = this->state.velocity.z * linDamping;

	this->state.rotation.x = this->state.rotation.x * rotDamping;
	this->state.rotation.y = this->state.rotation.y * rotDamping;
	this->state.rotation.z = this->state.rotation.z * rotDamping;
#else
	this->integrateLinearVelocity(dt);
	this->integrateAngularVelocity(dt);
#endif
}


void Body::integratePosition(double dt)
{
	this->state.pos.x = this->state.pos.x + (this->state.velocity.x * dt);
	this->state.pos.y = this->state.pos.y + (this->state.velocity.y * dt);
	this->state.pos.z = this->state.pos.z + (this->state.velocity.z * dt);
	this->state.orientation.addScaledVector(&this->state.rotation, dt);
}

void Body::calcDerivedData()
{
	_function_name("Body::calcDerivedData");
	this->state.orientation.toMatrix3(&this->baseMatrix);
	this->invInertiaWorld.copy(&this->invInertia);
	this->invInertiaWorld.append(&this->baseMatrix);
	this->invInertiaWorld.prependTransposed(&this->baseMatrix);

	if (this->collisionPrimitives != nullptr)
	{
		this->aabb.infinity();
		CollisionPrimitiveListItem *item = this->collisionPrimitives->head;
		while (item != nullptr)
		{
			CollisionPrimitive *primitive = item->primitive;

			primitive->transform.setFromMatrix3(&this->baseMatrix, &this->state.pos);
			if (primitive->localTransform != nullptr)
			{
				primitive->transform.prepend(primitive->localTransform);
			}

			primitive->calculateAABB();

			this->aabb.addBoundBox(&primitive->aabb);
			item = item->next;
		}
	}
}



void Body::restoreState()
{
	this->state.copy(&this->prevState);
}


void Body::applyRelPosWorldImpulse(const Vector3 *r, const Vector3 *dir, double magnitude)
{
	//r -45.2822 -87.4799 -73.8636
	//dir 0 0 1
	//dir -0.956371 0.292156 0

	double d = magnitude * this->invMass;
	this->state.velocity.x = (this->state.velocity.x + (d * dir->x));
	this->state.velocity.y = (this->state.velocity.y + (d * dir->y));
	this->state.velocity.z = (this->state.velocity.z + (d * dir->z));
	double x = ((r->y * dir->z) - (r->z * dir->y)) * magnitude;
	double y = ((r->z * dir->x) - (r->x * dir->z)) * magnitude;
	double z = ((r->x * dir->y) - (r->y * dir->x)) * magnitude;
	this->state.rotation.x = this->state.rotation.x + (((this->invInertiaWorld.a * x) + (this->invInertiaWorld.b * y)) + (this->invInertiaWorld.c * z));
	this->state.rotation.y = this->state.rotation.y + (((this->invInertiaWorld.e * x) + (this->invInertiaWorld.f * y)) + (this->invInertiaWorld.g * z));
	this->state.rotation.z = this->state.rotation.z + (((this->invInertiaWorld.i * x) + (this->invInertiaWorld.j * y)) + (this->invInertiaWorld.k * z));
}

void Body::clearAccumulators()
{
	this->forceAccum.reset();
	this->torqueAccum.reset();
}

void Body::addCollisionPrimitive(CollisionPrimitive *primitive, const Matrix4 *localTransform)
{
	if (primitive == nullptr)
		throw 455234;

	if (this->collisionPrimitives == nullptr)
		this->collisionPrimitives = new CollisionPrimitiveList();

	this->collisionPrimitives->append(primitive);
	primitive->setBody(this, localTransform);
}

void Body::addWorldForce(const Vector3 *pos, const Vector3 *force)
{
	this->forceAccum.add(force);
	Vector3 _r;
	_r.diff(pos, &this->state.pos);
	_r.cross(force);
	this->torqueAccum.add(&_r);
}

void Body::interpolate(double t, Vector3 *pos, Quaternion *orientation) const
{
	double t1 = 1. - t;
	pos->x = (this->prevState.pos.x * t1) + (this->state.pos.x * t);
	pos->y = (this->prevState.pos.y * t1) + (this->state.pos.y * t);
	pos->z = (this->prevState.pos.z * t1) + (this->state.pos.z * t);
	orientation->w = (this->prevState.orientation.w * t1) + (this->state.orientation.w * t);
	orientation->x = (this->prevState.orientation.x * t1) + (this->state.orientation.x * t);
	orientation->y = (this->prevState.orientation.y * t1) + (this->state.orientation.y * t);
	orientation->z = (this->prevState.orientation.z * t1) + (this->state.orientation.z * t);
}


void Body::addForce(const Vector3 *f)
{
	this->forceAccum.add(f);
}


void Body::addWorldForceScaled(const Vector3 *pos, const Vector3 *force, double scale)
{
	Vector3 f;
	f.x = scale * force->x;
	f.y = scale * force->y;
	f.z = scale * force->z;
	this->forceAccum.add(&f);
	Vector3 r;
	r.diff(pos, &this->state.pos);
	r.cross(&f);
	this->torqueAccum.add(&r);
}

void Body::setMaxSpeedXY(double value)
{
	this->maxSpeedXY = value;
}

void Body::integrateLinearVelocity(double dt)
{
	this->previousVelocity.copy(&this->state.velocity);
	if(this->accel.z < this->minZAcceleration)
	{
		this->accel.z = this->minZAcceleration;
	}
	//this->state.velocity.copy(&this->state.velocity);
	this->state.velocity.x = this->state.velocity.x + this->accel.x * dt;
	this->state.velocity.y = this->state.velocity.y + this->accel.y * dt;
	this->state.velocity.z = this->state.velocity.z + this->accel.z * dt;
	this->state.velocity.x = this->state.velocity.x * linDamping;
	this->state.velocity.y = this->state.velocity.y * linDamping;
	this->state.velocity.z = this->state.velocity.z * linDamping;
	double _loc2_ = MyMath::abs(this->state.velocity.z);
	if (_loc2_ > MAX_SPEED_Z)
	{
		this->state.velocity.z = this->state.velocity.z * (MAX_SPEED_Z / _loc2_);
	}
	if (this->state.velocity.z - this->previousVelocity.z > this->maxZSpeedDelta)
	{
		this->state.velocity.z = this->previousVelocity.z + this->maxZSpeedDelta;
	}
	double _loc3_ = this->state.velocity.x;
	double _loc4_ = this->state.velocity.y;
	double _loc5_ = MyMath::sqrt(_loc3_ * _loc3_ + _loc4_ * _loc4_);
	double _loc6_ = this->maxSpeedXY;
	if (_loc5_ > _loc6_)
	{
		double _loc7_ = _loc6_ / _loc5_;
		this->state.velocity.x = this->state.velocity.x * _loc7_;
		this->state.velocity.y = this->state.velocity.y * _loc7_;
	}
}

void Body::integrateAngularVelocity(double dt)
{
	this->state.rotation.x = this->state.rotation.x + this->angleAccel.x * dt;
	this->state.rotation.y = this->state.rotation.y + this->angleAccel.y * dt;
	this->state.rotation.z = this->state.rotation.z + this->angleAccel.z * dt;
	this->state.rotation.x = this->state.rotation.x * rotDamping;
	this->state.rotation.y = this->state.rotation.y * rotDamping;
	this->state.rotation.z = this->state.rotation.z * rotDamping;
	if (this->state.rotation.length() > 10)
	{
		this->state.rotation.setLength(10);
	}
}
