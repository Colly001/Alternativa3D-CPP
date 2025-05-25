#ifndef COLLISIONPRIMITIVELIST_H
#define COLLISIONPRIMITIVELIST_H

class CollisionPrimitiveListItem;
class CollisionPrimitive;


class CollisionPrimitiveList
{
public:
	CollisionPrimitiveList();
	~CollisionPrimitiveList();
	CollisionPrimitiveListItem *head;
	CollisionPrimitiveListItem *tail;
	int size;
	void append(CollisionPrimitive *primitive);
	void remove(CollisionPrimitive *primitve);
	CollisionPrimitiveListItem* findItem(CollisionPrimitive *primitive);
	void clear();
};

#endif // COLLISIONPRIMITIVELIST_H
