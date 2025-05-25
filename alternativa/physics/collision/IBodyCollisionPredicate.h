#ifndef IBODYCOLLISIONPREDICATE_H
#define IBODYCOLLISIONPREDICATE_H

class Body;

class IBodyCollisionPredicate
{
public:
	virtual bool considerBodies(Body*, Body*) = 0;
};

#endif // IBODYCOLLISIONPREDICATE_H
