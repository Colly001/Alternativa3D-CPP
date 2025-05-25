#ifndef ICOLLISIONDETECTOR_H
#define ICOLLISIONDETECTOR_H

class Contact;
class IRayCollisionPredicate;
class RayIntersection;
class Vector3;
class CollisionPrimitive;


class ICollisionDetector
{
public:
	virtual Contact* getAllContacts(Contact*) = 0;
	virtual bool intersectRay(const Vector3*, const Vector3*, int, double, IRayCollisionPredicate*, RayIntersection*) = 0;
	virtual bool intersectRayNecessarilyDynamic(const Vector3*, const Vector3*, int, double, IRayCollisionPredicate*, RayIntersection*) = 0;
	virtual bool intersectRayWithStatic(const Vector3*, const Vector3*, int, double, IRayCollisionPredicate*, RayIntersection*) = 0;
	virtual bool getContact(CollisionPrimitive*, CollisionPrimitive*, Contact*) = 0;
	virtual bool testCollision(CollisionPrimitive*, CollisionPrimitive*) = 0;
	virtual void destroy() = 0;
};

#endif // ICOLLISIONDETECTOR_H
