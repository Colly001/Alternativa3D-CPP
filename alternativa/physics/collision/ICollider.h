#ifndef ICOLLIDER_H
#define ICOLLIDER_H

class CollisionPrimitive;
class Contact;


class ICollider
{
public:
	virtual bool getContact(const CollisionPrimitive*, const CollisionPrimitive*, Contact*) = 0;
	virtual bool haveCollision(const CollisionPrimitive*, const CollisionPrimitive*) = 0;
	virtual void destroy() = 0;
};

#endif // ICOLLIDER_H
