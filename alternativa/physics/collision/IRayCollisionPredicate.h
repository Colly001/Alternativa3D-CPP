#ifndef IRAYCOLLISIONPREDICATE_H
#define IRAYCOLLISIONPREDICATE_H

class Body;

class IRayCollisionPredicate
{
public:
	virtual bool considerBody(Body*) = 0;
	virtual ~IRayCollisionPredicate() {}
};

#endif // IRAYCOLLISIONPREDICATE_H
