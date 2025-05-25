#ifndef CONTACT_H
#define CONTACT_H

class Vector3;
class ContactPoint;
class Body;

#include "alternativa/math/vector3.h"

class Contact
{
public:
	Contact(int index);
	~Contact();
	Body *body1;
	Body *body2;
	Contact *next;
	ContactPoint *points[8];
	Vector3 normal;
	double maxPenetration;
	double restitution;
	double friction;
	int pcount;
	int index;	
	bool satisfied;
};

#endif // CONTACT_H
