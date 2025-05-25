#ifndef BODYLIST_H
#define BODYLIST_H

class BodyListItem;
class Body;


class BodyList
{
public:
	BodyList();
	~BodyList();
	BodyListItem *head;
	BodyListItem *tail;
	int size;
	void append(Body *body);
	bool remove(Body *body);
	BodyListItem* findItem(Body *body);
};

#endif // BODYLIST_H
