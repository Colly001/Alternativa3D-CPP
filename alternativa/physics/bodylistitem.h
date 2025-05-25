#ifndef BODYLISTITEM_H
#define BODYLISTITEM_H

class Body;

class BodyListItem
{
public:
	Body *body;
	BodyListItem *next;
	BodyListItem *prev;
	BodyListItem(Body *body);
	static BodyListItem* create(Body *body);
	static void clearPool();
	void dispose();
};

#endif // BODYLISTITEM_H
