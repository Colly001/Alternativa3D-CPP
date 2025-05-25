#include "bodylistitem.h"

static BodyListItem *poolTop;


BodyListItem::BodyListItem(Body *body)
{
	this->body = body;
	this->next = nullptr;
	this->prev = nullptr;
}

BodyListItem *BodyListItem::create(Body *body)
{
	BodyListItem *item;
	if (poolTop == nullptr)
	{
		item = new BodyListItem(body);
	}
	else
	{
		item = poolTop;
		poolTop = item->next;
		item->next = nullptr;
		item->prev = nullptr;
		item->body = body;
	}
	return item;
}

void BodyListItem::clearPool()
{
	BodyListItem *item = poolTop;
	while (item != nullptr)
	{
		BodyListItem *i = item->next;
		delete item;
		item = i;
	}
	poolTop = nullptr;
}

void BodyListItem::dispose()
{
	this->body = nullptr;
	this->prev = nullptr;
	this->next = poolTop;
	poolTop = this;
}
