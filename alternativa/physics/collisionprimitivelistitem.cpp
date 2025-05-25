#include "collisionprimitivelistitem.h"
#include "_global.h"

CollisionPrimitiveListItem::CollisionPrimitiveListItem(CollisionPrimitive *primitive)
{
	_function_name("CollisionPrimitiveListItem::CollisionPrimitiveListItem");
	this->primitive = primitive;
	this->next = nullptr;
	this->prev = nullptr;
}

CollisionPrimitiveListItem* CollisionPrimitiveListItem::create(CollisionPrimitive *primitive)
{
	_function_name("CollisionPrimitiveListItem::create");
	return new CollisionPrimitiveListItem(primitive);
}

void CollisionPrimitiveListItem::dispose()
{
	_function_name("CollisionPrimitiveListItem::dispose");
	delete this;
	//this->primitive = nullptr;
	//this->prev = nullptr;
	//this->next = poolTop;
	//poolTop = this;
}
