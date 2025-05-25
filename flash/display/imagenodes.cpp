#include "imagenodes.h"
#include "imageitem.h"

namespace
{
	typedef ImageItem Node;

	void insertNodeLast(ImageNodes *n, Node *node)
	{
		node->next = nullptr;
		node->prev = n->last;
		if (n->first == nullptr)
			n->first = node;
		else
			n->last->next = node;
		n->last = node;
	}

	void insertNodeNext(ImageNodes *n, Node *node_current, Node *node_next_new)
	{
		Node *next = node_current->next;
		node_next_new->next = next;
		node_next_new->prev = node_current;
		node_current->next = node_next_new;
		if (next != nullptr)
			next->prev = node_next_new;
		else
			n->last = node_next_new;
	}

	void insertNodePrev(ImageNodes *n, Node *node_current, Node *node_prev_new)
	{
		Node *prev = node_current->prev;
		node_prev_new->prev = prev;
		node_prev_new->next = node_current;
		node_current->prev = node_prev_new;
		if (prev != nullptr)
			prev->next = node_prev_new;
		else
			n->first = node_prev_new;
	}

}

ImageNodes::ImageNodes()
{
	this->first = nullptr;
	this->last = nullptr;
}

void ImageNodes::add(ImageItem *item_new)
{
	auto node_new = item_new;
	auto *node = first;

	if (node != nullptr)
	{
		if (node->level > node_new->level)
		{
			insertNodePrev(this, node, node_new);
			return;
		}

		if (last->level < node_new->level)
		{
			insertNodeNext(this, last, node_new);
			return;
		}
	}

	while (node != nullptr)
	{
		if (node->level > node_new->level)
		{
			insertNodeNext(this, node->prev, node_new);
			return;
		}
		node = node->next;
	}

	if (node == nullptr)
	{
		insertNodeLast(this, node_new);
	}
}

void ImageNodes::remove(ImageItem *node)
{
	ImageItem *prev = node->prev;
	ImageItem *next = node->next;

	if (prev != nullptr)
		prev->next = next;
	else
		first = next;

	if (next != nullptr)
		next->prev = prev;
	else
		last = prev;
}
