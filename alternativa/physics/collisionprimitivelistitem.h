#ifndef COLLISIONPRIMITIVELISTITEM_H
#define COLLISIONPRIMITIVELISTITEM_H

class CollisionPrimitive;

class CollisionPrimitiveListItem
{
public:
	CollisionPrimitive *primitive;
	CollisionPrimitiveListItem *next;
	CollisionPrimitiveListItem *prev;
	CollisionPrimitiveListItem(CollisionPrimitive *primitive);
	static CollisionPrimitiveListItem* create(CollisionPrimitive *primitive);
	void dispose();
};

#endif // COLLISIONPRIMITIVELISTITEM_H
