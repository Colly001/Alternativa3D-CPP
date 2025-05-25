#include "collisionprimitivelist.h"
#include "collisionprimitivelistitem.h"
#include "_global.h"


CollisionPrimitiveList::CollisionPrimitiveList()
{
	_function_name("CollisionPrimitiveList::CollisionPrimitiveList");
	head = nullptr;
	tail = nullptr;
	size = 0;
}

CollisionPrimitiveList::~CollisionPrimitiveList()
{
	_function_name("CollisionPrimitiveList::~CollisionPrimitiveList");
}


void CollisionPrimitiveList::append(CollisionPrimitive *primitive)
{
	_function_name("CollisionPrimitiveList::append");

	CollisionPrimitiveListItem *item = CollisionPrimitiveListItem::create(primitive);
	item->next = nullptr;

	if (this->head == nullptr)
	{
		this->head = this->tail = item;
	}
	else
	{
		this->tail->next = item;
		item->prev = this->tail;
		this->tail = item;
	}
	this->size++;
}

void CollisionPrimitiveList::remove(CollisionPrimitive *primitve)
{
	_function_name("CollisionPrimitiveList::remove");

	CollisionPrimitiveListItem *item = this->findItem(primitve);
	if (item == nullptr)
	{
		return;
	}
	if (item == this->head)
	{
		if (this->size == 1)
		{
			this->head = this->tail = nullptr;
		}
		else
		{
			this->head = item->next;
			this->head->prev = nullptr;
		}
	}
	else
	{
		if (item == this->tail)
		{
			this->tail = this->tail->prev;
			this->tail->next = nullptr;
		}
		else
		{
			item->prev->next = item->next;
			item->next->prev = item->prev;
		}
	}
	item->dispose();
	this->size--;
}

CollisionPrimitiveListItem *CollisionPrimitiveList::findItem(CollisionPrimitive *primitive)
{
	_function_name("CollisionPrimitiveList::findItem");

	CollisionPrimitiveListItem *item = this->head;
	while (item != nullptr && item->primitive != primitive)
	{
		item = item->next;
	}
	return item;
}

void CollisionPrimitiveList::clear()
{
	_function_name("CollisionPrimitiveList::clear");

	while (this->head != nullptr)
	{
		CollisionPrimitiveListItem *item = this->head;
		this->head = this->head->next;
		item->dispose();
	}
	this->tail = nullptr;
	this->size = 0;
}
