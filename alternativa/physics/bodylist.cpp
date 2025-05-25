#include "bodylist.h"
#include "bodylistitem.h"


BodyList::BodyList()
{
	head = nullptr;
	tail = nullptr;
	size = 0;
}

BodyList::~BodyList()
{
	BodyListItem *item = head;
	while (item != nullptr)
	{
		BodyListItem *i = item->next;
		item->dispose();
		item = i;
	}
}

void BodyList::append(Body *body)
{
	BodyListItem *item = BodyListItem::create(body);
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

bool BodyList::remove(Body *body)
{
	BodyListItem *item = this->findItem(body);
	if (item == nullptr)
	{
		return false;
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
			this->tail = item->prev;
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
	return true;
}


BodyListItem *BodyList::findItem(Body *body)
{
	BodyListItem *item = this->head;
	while (item != nullptr && item->body != body)
	{
		item = item->next;
	}
	return item;
}
