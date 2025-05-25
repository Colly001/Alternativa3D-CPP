#include "collisionkdnode.h"
#include "collisionkdtree2d.h"
#include <QVector>

CollisionKdNode::CollisionKdNode()
{
	axis = -1;
	indices = nullptr;
	splitIndices = nullptr;
	parent = nullptr;
	splitTree = nullptr;
	coord = 0;
	positiveNode = nullptr;
	negativeNode = nullptr;
}

CollisionKdNode::~CollisionKdNode()
{
	CollisionKdNode *current = this;

	for (;;)
	{
		if (current == nullptr)
			break;

		if (current->positiveNode != nullptr)
		{
			current = current->positiveNode;
			continue;
		}

		if (current->negativeNode != nullptr)
		{
			current = current->negativeNode;
			continue;
		}

		if (current == this)
			break;

		auto parent = current->parent;
		if (parent != nullptr)
		{
			parent->positiveNode = nullptr;
			parent->negativeNode = nullptr;
		}

		delete current;
		current = parent;
	}

	//QVector<int>
	if (indices != nullptr)
		delete indices;

	//QVector<int>
	if (splitIndices != nullptr)
		delete splitIndices;

	//CollisionKdTree2D
	if (splitTree != nullptr)
		delete splitTree;

	/*
	//CollisionKdNode
	if (positiveNode != nullptr)
		delete positiveNode;

	//CollisionKdNode
	if (negativeNode != nullptr)
		delete negativeNode;
		*/
}
