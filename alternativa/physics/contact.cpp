#include "contact.h"
#include "alternativa/math/vector3.h"
#include "contactpoint.h"

#define MAX_POINTS 8

Contact::Contact(int index)
{
	pcount = 0;
	body1 = nullptr;
	body2 = nullptr;
	next = nullptr;
	this->index = index;
	restitution = 0;
	friction = 0;
	normal.reset();
	maxPenetration = 0;
	satisfied = false;

	for (int i = 0; i < MAX_POINTS; i++)
		this->points[i] = new ContactPoint();
}

Contact::~Contact()
{
	for (int i = 0; i < MAX_POINTS; i++)
		delete this->points[i];
}
