#ifndef CONSTRAINT_H
#define CONSTRAINT_H

class PhysicsScene;

class Constraint
{
public:
	Constraint();
	virtual ~Constraint();

	bool satisfied;
	PhysicsScene *world;
	virtual void preProcess(double dt);
	virtual void apply(double dt);
};

#endif // CONSTRAINT_H
