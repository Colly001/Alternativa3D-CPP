#ifndef ICOLLISIONPREDICATE_H
#define ICOLLISIONPREDICATE_H

class CollisionPrimitive;

class ICollisionPredicate
{
public:
	virtual bool considerCollision(CollisionPrimitive*) = 0;
};

#endif // ICOLLISIONPREDICATE_H
