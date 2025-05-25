#include "physicsscene.h"
#include "alternativa/math/vector3.h"
#include "alternativa/physics/bodylist.h"
#include "bodylistitem.h"
#include "body.h"
#include "contactpoint.h"
#include "collision/ICollisionDetector.h"
#include "contact.h"
#include "bodystate.h"
#include "bodymaterial.h"
#include "constraints/constraint.h"
#include "alternativa/math/matrix3.h"
#include "collision/kdtreecollisiondetector.h"
#include "_global.h"
#include <QVector>


static int lastBodyId = 1;


#define MAX_CONTACTS 1000

PhysicsScene::PhysicsScene()
{
	_function_name("PhysicsScene::PhysicsScene");

	_gravity.reset(pseudoX, pseudoY, pseudoZ);
	bodies = new BodyList();
	constraints = new QVector<Constraint*>;
	//constraintsNum = 0;

	//PhysicsScene 1 12 4
	//PhysicsScene 2 112 120

	//this->timeStamp = 0;
	//this->time = 0;

	//no free
	//Contact

	this->contacts = new Contact(0);
	Contact *contact = this->contacts;

	for (int i = 1; i < MAX_CONTACTS; i++) {
		contact->next = new Contact(i);
		contact = contact->next;
	}

	this->_staticCD = new KdTreeCollisionDetector();
	this->collisionDetector = _staticCD;
}

PhysicsScene::~PhysicsScene()
{
	_function_name("PhysicsScene::~PhysicsScene");

	Contact *contact = this->contacts;
	while (contact != nullptr)
	{
		Contact *next = contact->next;
		delete contact;
		contact = next;
	}

	delete bodies;
	delete constraints;
	delete _staticCD;
}


void PhysicsScene::setGravity(const Vector3 *value)
{
	_function_name("PhysicsScene::setGravity");
	this->_gravity.copy(value);
	this->_gravityMagnitude = this->_gravity.length();
}

const Vector3* PhysicsScene::getGravity()
{
	_function_name("PhysicsScene::getGravity");
	return &this->_gravity;
}

void PhysicsScene::addBody(Body *body)
{
	_function_name("PhysicsScene::addBody");
	if (this->bodies->findItem(body) != nullptr)
		return;
	body->id = lastBodyId++;
	body->world = this;
	this->bodies->append(body);
}

void PhysicsScene::removeBody(Body *body)
{
	_function_name("PhysicsScene::removeBody");
	if (this->bodies->remove(body))
		body->world = nullptr;
}

void PhysicsScene::update(int delta)
{	
	_function_name("PhysicsScene::update");

	//this->timeStamp++;
	//this->time = this->time + delta;
	double dt = 0.001 * delta;
	this->applyForces(dt);
	this->detectCollisions(dt);
	this->preProcessContacts(dt);
	this->processContacts(dt, false);
	this->intergateVelocities(dt);
	this->processContacts(dt, true);
	this->integratePositions(dt);
	this->postPhysics();
}


void PhysicsScene::applyForces(double dt)
{
	_function_name("PhysicsScene::applyForces");

	BodyListItem *item = this->bodies->head;
	while (item != nullptr)
	{
		Body *body = item->body;
		body->beforePhysicsStep(dt);
		body->calcAccelerations();
		if (body->movable && !body->frozen)
		{
			body->accel.x = body->accel.x + this->_gravity.x;
			body->accel.y = body->accel.y + this->_gravity.y;
			body->accel.z = body->accel.z + this->_gravity.z;
		}
		item = item->next;
	}
}


void PhysicsScene::detectCollisions(double dt)
{
	_function_name("PhysicsScene::detectCollisions");
	BodyListItem *item = this->bodies->head;
	while (item != nullptr)
	{
		Body *body = item->body;
		if ((!(body->frozen)))
		{
			body->contactsNum = 0;
			body->saveState();
			if (this->usePrediction)
			{
				body->integrateVelocity(dt);
				body->integratePosition(dt);
			}
			body->calcDerivedData();
		}
		item = item->next;
	}

	this->borderContact = this->collisionDetector->getAllContacts(this->contacts);
	Contact *contact = this->contacts;
	while (contact != this->borderContact)
	{
		Body *b1 = contact->body1;
		Body *b2 = contact->body2;
		int j = 0;
		while (j < contact->pcount)
		{
			ContactPoint *cp = contact->points[j];
			Vector3 *bPos = &b1->state.pos;
			cp->r1.x = cp->pos.x - bPos->x;
			cp->r1.y = cp->pos.y - bPos->y;
			cp->r1.z = cp->pos.z - bPos->z;
			if (b2 != nullptr)
			{
				bPos = &b2->state.pos;
				cp->r2.x = cp->pos.x - bPos->x;
				cp->r2.y = cp->pos.y - bPos->y;
				cp->r2.z = cp->pos.z - bPos->z;
			}
			j++;
		}
		contact = contact->next;
	}

	if (this->usePrediction)
	{
		item = this->bodies->head;
		while (item != nullptr)
		{
			Body *body = item->body;
			if (!body->frozen)
			{
				body->restoreState();
				body->calcDerivedData();
			}
			item = item->next;
		}
	}
}



void PhysicsScene::preProcessContacts(double dt)
{
	_function_name("PhysicsScene::preProcessContacts");

	Contact *contact = this->contacts;
	while (contact != this->borderContact)
	{
		Body *b1 = contact->body1;
		Body *b2 = contact->body2;
		if (b1->frozen)
		{
			b1->frozen = false;
			b1->freezeCounter = 0;
		}
		if (b2 != nullptr && b2->frozen)
		{
			b2->frozen = false;
			b2->freezeCounter = 0;
		}
		contact->restitution = b1->material.restitution;
		if (b2 != nullptr && b2->material.restitution < contact->restitution)
		{
			contact->restitution = b2->material.restitution;
		}
		contact->friction = b1->material.friction;
		if (b2 != nullptr && b2->material.friction < contact->friction)
		{
			contact->friction = b2->material.friction;
		}
		int j = 0;
		while (j < contact->pcount)
		{
			ContactPoint *cp = contact->points[j];
			cp->accumImpulseN = 0;
			cp->velByUnitImpulseN = 0;
			if (b1->movable)
			{
				Vector3 v;
				v.cross2(&cp->r1, &contact->normal);
				v.transformBy3(&b1->invInertiaWorld);
				v.cross(&cp->r1);
				cp->angularInertia1 = v.dot(&contact->normal);
				cp->velByUnitImpulseN = (cp->velByUnitImpulseN + (b1->invMass + cp->angularInertia1));
			}
			if (b2 != nullptr && b2->movable)
			{
				Vector3 v;
				v.cross2(&cp->r2, &contact->normal);
				v.transformBy3(&b2->invInertiaWorld);
				v.cross(&cp->r2);
				cp->angularInertia2 = v.dot(&contact->normal);
				cp->velByUnitImpulseN = (cp->velByUnitImpulseN + (b2->invMass + cp->angularInertia2));
			}

			Vector3 _v;
			this->calcSepVelocity(b1, b2, cp, &_v);
			cp->normalVel = _v.dot(&contact->normal);
			if (cp->normalVel < 0)
			{
				cp->normalVel = (-(contact->restitution) * cp->normalVel);
			}
			cp->minSepVel = ((cp->penetration > this->allowedPenetration) ? ((cp->penetration - this->allowedPenetration) / (this->penResolutionSteps * dt)) : 0);
			if (cp->minSepVel > this->maxPenResolutionSpeed)
			{
				cp->minSepVel = this->maxPenResolutionSpeed;
			}
			j++;
		}
		contact = contact->next;
	}

	int constraintsNum = this->constraints->length();
	for (int i = 0; i < constraintsNum; i++)
		this->constraints->at(i)->preProcess(dt);
}

void PhysicsScene::processContacts(double dt, bool forceInelastic)
{
	_function_name("PhysicsScene::processContacts");

	int iterNum = forceInelastic ? this->contactIterations : this->collisionIterations;
	bool forwardLoop = false;
	int iter = 0;

	while (iter < iterNum)
	{
		forwardLoop = !forwardLoop;
		Contact *contact = this->contacts;
		while (contact != this->borderContact)
		{
			this->resolveContact(contact, forceInelastic, forwardLoop);
			contact = contact->next;
		}
		int constraintsNum = this->constraints->length();
		for (int i = 0; i < constraintsNum; i++)
			this->constraints->at(i)->apply(dt);
		iter++;
	}
}


void PhysicsScene::calcSepVelocity(Body *body1, Body *body2, ContactPoint *cp, Vector3 *result)
{
	_function_name("PhysicsScene::calcSepVelocity");

	const Vector3 &rot = body1->state.rotation;
	const Vector3 &v1 = cp->r1;
	double x = (rot.y * v1.z) - (rot.z * v1.y);
	double y = (rot.z * v1.x) - (rot.x * v1.z);
	double z = (rot.x * v1.y) - (rot.y * v1.x);
	Vector3 &v2 = body1->state.velocity;
	result->x = (v2.x + x);
	result->y = (v2.y + y);
	result->z = (v2.z + z);
	if (body2 != nullptr)
	{
		Vector3 &rot = body2->state.rotation;
		Vector3 &v3 = cp->r2;
		x = (rot.y * v3.z) - (rot.z * v3.y);
		y = (rot.z * v3.x) - (rot.x * v3.z);
		z = (rot.x * v3.y) - (rot.y * v3.x);
		Vector3 &v4 = body2->state.velocity;
		result->x = result->x - (v4.x + x);
		result->y = result->y - (v4.y + y);
		result->z = result->z - (v4.z + z);
	}
}

void PhysicsScene::resolveContact(Contact *contactInfo, bool forceInelastic, bool forwardLoop)
{
	_function_name("PhysicsScene::resolveContact");

	Body *b1 = contactInfo->body1;
	Body *b2 = contactInfo->body2;
	const Vector3 *normal = &contactInfo->normal;
	if (forwardLoop)
	{
		int i = 0;
		while (i < contactInfo->pcount)
		{
			this->resolveContactPoint(i, b1, b2, contactInfo, normal, forceInelastic);
			i++;
		}
	}
	else
	{
		int i = contactInfo->pcount - 1;
		while (i >= 0)
		{
			this->resolveContactPoint(i, b1, b2, contactInfo, normal, forceInelastic);
			i--;
		}
	}
}


void PhysicsScene::resolveContactPoint(int idx, Body *b1, Body *b2, Contact *contact, const Vector3 *normal, bool forceInelastic)
{
	_function_name("PhysicsScene::resolveContactPoint");

	ContactPoint *cp = contact->points[idx];
	if (!forceInelastic)
	{
		cp->satisfied = true;
	}

	double newVel = 0.;
	Vector3 _v;

	this->calcSepVelocity(b1, b2, cp, &_v);
	Vector3 *cnormal = &contact->normal;
	double sepVel = (_v.x * cnormal->x) + (_v.y * cnormal->y) + (_v.z * cnormal->z);
	if (forceInelastic)
	{
		double minSpeVel = this->useStaticSeparation ? 0 : cp->minSepVel;
		if (sepVel < minSpeVel)
		{
			cp->satisfied = false;
		}
		else
		{
			if (cp->satisfied)
			{
				return;
			}
		}
		newVel = minSpeVel;
	}
	else
	{
		newVel = cp->normalVel;
	}
	double deltaVel = newVel - sepVel;
	double impulse = deltaVel / cp->velByUnitImpulseN;
	double accumImpulse = cp->accumImpulseN + impulse;
	if (accumImpulse < 0)
	{
		accumImpulse = 0;
	}
	double deltaImpulse = accumImpulse - cp->accumImpulseN;
	cp->accumImpulseN = accumImpulse;
	if (b1->movable)
	{
		b1->applyRelPosWorldImpulse(&cp->r1, normal, deltaImpulse);
	}
	if (b2 != nullptr && b2->movable)
	{
		b2->applyRelPosWorldImpulse(&cp->r2, normal, -deltaImpulse);
	}
	this->calcSepVelocity(b1, b2, cp, &_v);
	double tanSpeedByUnitImpulse = 0;
	double dot = (_v.x * cnormal->x) + (_v.y * cnormal->y) + (_v.z * cnormal->z);
	_v.x = _v.x - (dot * cnormal->x);
	_v.y = _v.y - (dot * cnormal->y);
	_v.z = _v.z - (dot * cnormal->z);
	double tanSpeed = _v.length();
	if (tanSpeed < 0.001)
	{
		return;
	}

	Vector3 _t;

	_t.x = -(_v.x);
	_t.y = -(_v.y);
	_t.z = -(_v.z);
	_t.normalize();
	if (b1->movable)
	{
		Vector3 *r = &cp->r1;
		Matrix3 *m = &b1->invInertiaWorld;
		_v.x = ((r->y * _t.z) - (r->z * _t.y));
		_v.y = ((r->z * _t.x) - (r->x * _t.z));
		_v.z = ((r->x * _t.y) - (r->y * _t.x));
		double xx = (((m->a * _v.x) + (m->b * _v.y)) + (m->c * _v.z));
		double yy = (((m->e * _v.x) + (m->f * _v.y)) + (m->g * _v.z));
		double zz = (((m->i * _v.x) + (m->j * _v.y)) + (m->k * _v.z));
		_v.x = ((yy * r->z) - (zz * r->y));
		_v.y = ((zz * r->x) - (xx * r->z));
		_v.z = ((xx * r->y) - (yy * r->x));
		tanSpeedByUnitImpulse = (tanSpeedByUnitImpulse + (((b1->invMass + (_v.x * _t.x)) + (_v.y * _t.y)) + (_v.z * _t.z)));
	}
	if (((!(b2 == nullptr)) && (b2->movable)))
	{
		Vector3 *r = &cp->r2;
		Matrix3 *m = &b2->invInertiaWorld;
		_v.x = ((r->y * _t.z) - (r->z * _t.y));
		_v.y = ((r->z * _t.x) - (r->x * _t.z));
		_v.z = ((r->x * _t.y) - (r->y * _t.x));
		double xx = (((m->a * _v.x) + (m->b * _v.y)) + (m->c * _v.z));
		double yy = (((m->e * _v.x) + (m->f * _v.y)) + (m->g * _v.z));
		double zz = (((m->i * _v.x) + (m->j * _v.y)) + (m->k * _v.z));
		_v.x = ((yy * r->z) - (zz * r->y));
		_v.y = ((zz * r->x) - (xx * r->z));
		_v.z = ((xx * r->y) - (yy * r->x));
		tanSpeedByUnitImpulse = tanSpeedByUnitImpulse + (b2->invMass + (_v.x * _t.x) + (_v.y * _t.y) + (_v.z * _t.z));
	}
	double tanImpulse = tanSpeed / tanSpeedByUnitImpulse;
	double max = contact->friction * cp->accumImpulseN;
	if (max < 0)
	{
		if (tanImpulse < max)
		{
			tanImpulse = max;
		}
	}
	else
	{
		if (tanImpulse > max)
		{
			tanImpulse = max;
		}
	}
	if (b1->movable)
	{
		b1->applyRelPosWorldImpulse(&cp->r1, &_t, tanImpulse);
	}
	if (b2 != nullptr && b2->movable)
	{
		b2->applyRelPosWorldImpulse(&cp->r2, &_t, -tanImpulse);
	}
}




void PhysicsScene::intergateVelocities(double dt)
{
	_function_name("PhysicsScene::intergateVelocities");

	BodyListItem *item = this->bodies->head;
	while (item != nullptr)
	{
		item->body->integrateVelocity(dt);
		item = item->next;
	}
}


void PhysicsScene::integratePositions(double dt)
{
	_function_name("PhysicsScene::integratePositions");

	BodyListItem *item = this->bodies->head;
	while (item != nullptr)
	{
		Body *body = item->body;
		if (body->movable && !body->frozen)
		{
			body->integratePosition(dt);
		}
		item = item->next;
	}
}


void PhysicsScene::postPhysics()
{
	_function_name("PhysicsScene::postPhysics");

	BodyListItem *item = this->bodies->head;
	while (item != nullptr)
	{
		Body *body = item->body;
		body->clearAccumulators();
		body->calcDerivedData();
		if (body->canFreeze)
		{
			if ((body->state.velocity.length() < this->linSpeedFreezeLimit) && (body->state.rotation.length() < this->angSpeedFreezeLimit))
			{
				if (!body->frozen)
				{
					body->freezeCounter++;
					if (body->freezeCounter >= this->freezeSteps)
					{
						body->frozen = true;
					}
				}
			}
			else
			{
				body->freezeCounter = 0;
				body->frozen = false;
			}
		}
		item = item->next;
	}
}


void PhysicsScene::addConstraint(Constraint *c)
{
	if (!this->constraints->contains(c)) {
		this->constraints->append(c);
		c->world = this;
	}
}

bool PhysicsScene::removeConstraint(Constraint *c)
{
	if (this->constraints->removeOne(c)) {
		c->world = nullptr;
		return true;
	} else {
		return false;
	}
}

