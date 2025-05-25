#include "collisionkdtree2d.h"
#include "collisionkdnode.h"
#include "collisionkdtree.h"
#include "types/boundbox.h"
#include "mymath.h"
#include <QVector>
#include "_global.h"



CollisionKdTree2D::CollisionKdTree2D(CollisionKdTree *parentTree, CollisionKdNode *parentNode)
{
	this->splitAxis = 0;
	this->splitCost = 0;
	this->splitCoord = 0;
	this->threshold = 0.1;
	this->minPrimitivesPerNode = 1;
	this->parentTree = parentTree;
	this->parentNode = parentNode;
	this->rootNode = nullptr;
}

CollisionKdTree2D::~CollisionKdTree2D()
{
	if (this->rootNode != nullptr)
		delete this->rootNode;
}

void CollisionKdTree2D::createTree()
{
	this->rootNode = new CollisionKdNode();
	this->rootNode->boundBox.copyFrom(&this->parentNode->boundBox);
	this->rootNode->indices = new QVector<int>;
	*this->rootNode->indices = *this->parentNode->splitIndices;
	this->splitNode(this->rootNode);
}

void CollisionKdTree2D::splitNode(CollisionKdNode *node)
{
	if (node->indices->length() <= this->minPrimitivesPerNode)
	{
		return;
	}

	QVector<int> *objects = node->indices;
	const BoundBox *nodeBoundBox = &node->boundBox;
	BoundBox nodeBoundBoxThreshold;
	nodeBoundBoxThreshold.minX = (nodeBoundBox->minX + this->threshold);
	nodeBoundBoxThreshold.minY = (nodeBoundBox->minY + this->threshold);
	nodeBoundBoxThreshold.minZ = (nodeBoundBox->minZ + this->threshold);
	nodeBoundBoxThreshold.maxX = (nodeBoundBox->maxX - this->threshold);
	nodeBoundBoxThreshold.maxY = (nodeBoundBox->maxY - this->threshold);
	nodeBoundBoxThreshold.maxZ = (nodeBoundBox->maxZ - this->threshold);
	//double doubleThreshold = (this->threshold * 2);
	QVector<BoundBox*> *staticBoundBoxes = this->parentTree->staticBoundBoxes;
	int numObjects = objects->length();

	QVector<double> splitCoordsX;
	QVector<double> splitCoordsY;
	QVector<double> splitCoordsZ;


	int i = 0;
	while (i < numObjects)
	{
		BoundBox *bb = staticBoundBoxes->at(objects->at(i));
		if (this->parentNode->axis != 0)
		{
			if (bb->minX > nodeBoundBoxThreshold.minX)
			{
				splitCoordsX.append(bb->minX);
			}
			if (bb->maxX < nodeBoundBoxThreshold.maxX)
			{
				splitCoordsX.append(bb->maxX);
			}
		}
		if (this->parentNode->axis != 1)
		{
			if (bb->minY > nodeBoundBoxThreshold.minY)
			{
				splitCoordsY.append(bb->minY);
			}
			if (bb->maxY < nodeBoundBoxThreshold.maxY)
			{
				splitCoordsY.append(bb->maxY);
			}
		}
		if (this->parentNode->axis != 2)
		{
			if (bb->minZ > nodeBoundBoxThreshold.minZ)
			{
				splitCoordsZ.append(bb->minZ);
			}
			if (bb->maxZ < nodeBoundBoxThreshold.maxZ)
			{
				splitCoordsZ.append(bb->maxZ);
			}
		}
		i++;
	}
	this->splitAxis = -1;
	this->splitCost = 1E308;
	double _nodeBB[6];
	_nodeBB[0] = nodeBoundBox->minX;
	_nodeBB[1] = nodeBoundBox->minY;
	_nodeBB[2] = nodeBoundBox->minZ;
	_nodeBB[3] = nodeBoundBox->maxX;
	_nodeBB[4] = nodeBoundBox->maxY;
	_nodeBB[5] = nodeBoundBox->maxZ;
	if (this->parentNode->axis != 0)
	{
		this->checkNodeAxis(node, 0, splitCoordsX.length(), &splitCoordsX, _nodeBB);
	}
	if (this->parentNode->axis != 1)
	{
		this->checkNodeAxis(node, 1, splitCoordsY.length(), &splitCoordsY, _nodeBB);
	}
	if (this->parentNode->axis != 2)
	{
		this->checkNodeAxis(node, 2, splitCoordsZ.length(), &splitCoordsZ, _nodeBB);
	}
	if (this->splitAxis < 0)
	{
		return;
	}
	bool axisX = this->splitAxis == 0;
	bool axisY = this->splitAxis == 1;
	node->axis = this->splitAxis;
	node->coord = this->splitCoord;


	if (node->negativeNode != nullptr)
		delete node->negativeNode;

	if (node->positiveNode != nullptr)
		delete node->positiveNode;

	node->negativeNode = new CollisionKdNode();
	node->negativeNode->parent = node;
	node->negativeNode->boundBox.copyFrom(nodeBoundBox);
	node->positiveNode = new CollisionKdNode();
	node->positiveNode->parent = node;
	node->positiveNode->boundBox.copyFrom(nodeBoundBox);
	if (axisX)
	{
		node->negativeNode->boundBox.maxX = node->positiveNode->boundBox.minX = this->splitCoord;
	}
	else
	{
		if (axisY)
		{
			node->negativeNode->boundBox.maxY = node->positiveNode->boundBox.minY = this->splitCoord;
		}
		else
		{
			node->negativeNode->boundBox.maxZ = node->positiveNode->boundBox.minZ = this->splitCoord;
		}
	}
	double coordMin = (this->splitCoord - this->threshold);
	double coordMax = (this->splitCoord + this->threshold);
	i = 0;
	while (i < numObjects)
	{
		BoundBox *bb = staticBoundBoxes->at(objects->at(i));
		double min = ((axisX) ? bb->minX : ((axisY) ? bb->minY : bb->minZ));
		double max = ((axisX) ? bb->maxX : ((axisY) ? bb->maxY : bb->maxZ));
		if (max <= coordMax)
		{
			if (min < coordMin)
			{
				if (node->negativeNode->indices == nullptr)
				{
					node->negativeNode->indices = new QVector<int>;
				}
				node->negativeNode->indices->append(objects->at(i));
				objects->replace(i, -1);
			}
		}
		else
		{
			if (min >= coordMin)
			{
				if (max > coordMax)
				{
					if (node->positiveNode->indices == nullptr)
					{
						node->positiveNode->indices = new QVector<int>;
					}
					node->positiveNode->indices->append(objects->at(i));
					objects->replace(i, -1);
				}
			}
		}
		i++;
	}
	i = 0;
	int j = 0;
	while (i < numObjects)
	{
		if (objects->at(i) >= 0)
		{
			int _local_19 = j++;
			objects->replace(_local_19, objects->at(i));
		}
		i++;
	}
	if (j > 0)
	{
		objects->resize(j);
	}
	else
	{
		if (node->indices != nullptr) {
			delete node->indices;
			node->indices = nullptr;
		}
	}

	if (node->negativeNode->indices != nullptr)
	{
		this->splitNode(node->negativeNode);
	}

	if (node->positiveNode->indices != nullptr)
	{
		this->splitNode(node->positiveNode);
	}
}

void CollisionKdTree2D::checkNodeAxis(CollisionKdNode *node, int axis, int numSplitCoords, QVector<double> *splitCoords, double bb[6])
{
	int j = 0;
	int axis1 = ((axis + 1) % 3);
	int axis2 = ((axis + 2) % 3);
	double area = ((bb[(axis1 + 3)] - bb[axis1]) * (bb[(axis2 + 3)] - bb[axis2]));
	QVector<BoundBox*> *staticBoundBoxes = this->parentTree->staticBoundBoxes;
	int i = 0;
	while (i < numSplitCoords)
	{
		double currSplitCoord = splitCoords->at(i);
		if (!MyMath::isNaN(currSplitCoord))
		{
			double minCoord = (currSplitCoord - this->threshold);
			double maxCoord = (currSplitCoord + this->threshold);
			double areaNegative = (area * (currSplitCoord - bb[axis]));
			double areaPositive = (area * (bb[int((axis + 3))] - currSplitCoord));
			int numNegative = 0;
			int numPositive = 0;
			bool conflict = false;
			int numObjects = node->indices->length();
			j = 0;
			while (j < numObjects)
			{
				BoundBox *boundBox = staticBoundBoxes->at(node->indices->at(j));
				double _bb[6];
				_bb[0] = boundBox->minX;
				_bb[1] = boundBox->minY;
				_bb[2] = boundBox->minZ;
				_bb[3] = boundBox->maxX;
				_bb[4] = boundBox->maxY;
				_bb[5] = boundBox->maxZ;
				if (_bb[(axis + 3)] <= maxCoord)
				{
					if (_bb[axis] < minCoord)
					{
						numNegative++;
					}
				}
				else
				{
					if (_bb[axis] >= minCoord)
					{
						numPositive++;
					}
					else
					{
						conflict = true;
						break;
					}
				}
				j++;
			}
			double cost = ((areaNegative * numNegative) + (areaPositive * numPositive));
			if (((!(conflict)) && (cost < this->splitCost)))
			{
				this->splitAxis = axis;
				this->splitCost = cost;
				this->splitCoord = currSplitCoord;
			}
			j = (i + 1);
			while (j < numSplitCoords)
			{
				double v = splitCoords->at(j);
				if (!MyMath::isNaN(v) && (v >= (currSplitCoord - this->threshold)) && (v <= (currSplitCoord + this->threshold)))
				{
					splitCoords->replace(j, MyMath::nan());
				}
				j++;
			}
		}
		i++;
	}
}
