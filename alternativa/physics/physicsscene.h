#ifndef PHYSICSSCENE_H
#define PHYSICSSCENE_H

class Contact;
class ICollisionDetector;
class KdTreeCollisionDetector;
class BodyList;
class Vector3;
class Body;
class ContactPoint;
class Constraint;


#include <qglobal.h>
#include "alternativa/math/vector3.h"

class PhysicsScene
{
	double pseudoX = 0;
	double pseudoY = 0;
	double pseudoZ = -9.8;
public:	
	Contact *borderContact;

	const int MAX_CONTACTS = 1000;
	int penResolutionSteps = 10;
	int allowedPenetration = 0.01;
	double maxPenResolutionSpeed = 0.5;
	int collisionIterations = 4;
	int contactIterations = 4;
	bool usePrediction = false;
	int freezeSteps = 10;
	double linSpeedFreezeLimit = 1;
	double angSpeedFreezeLimit = 0.01;
	int staticSeparationIterations = 10;
	int staticSeparationSteps = 10;
	double maxAngleMove = 10;
	bool useStaticSeparation = false;
	double _gravityMagnitude = 9.8;
	ICollisionDetector *collisionDetector;
	Contact *contacts;
	//int constraintsNum;
	//int timeStamp;
	//int time;

	KdTreeCollisionDetector *_staticCD;
	
	double realX = 0;
	double realY = 0;
	double realZ = -1000;


	Vector3 _gravity;
	BodyList *bodies;

	QVector<Constraint*> *constraints;
	//altphysics var constraints:Vector.<Constraint> = new Vector.<Constraint>();

	PhysicsScene();
	~PhysicsScene();

	void update(int delta);
	void setGravity(const Vector3 *value);
	const Vector3 *getGravity();

	void addBody(Body *body);
	void removeBody(Body *body);

	void addConstraint(Constraint *c);
	bool removeConstraint(Constraint *c);

private:
	void applyForces(double dt);
	void detectCollisions(double dt);
	void preProcessContacts(double dt);
	void processContacts(double dt, bool forceInelastic);
	void calcSepVelocity(Body *body1, Body *body2, ContactPoint *cp, Vector3 *result);
	void resolveContact(Contact *contactInfo, bool forceInelastic, bool forwardLoop);
	void resolveContactPoint(int idx, Body *b1, Body *b2, Contact *contact, const Vector3 *normal, bool forceInelastic);
	void intergateVelocities(double dt);
	void integratePositions(double dt);
	void postPhysics();
};

#endif // PHYSICSSCENE_H
