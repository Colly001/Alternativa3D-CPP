#include "collisionkdtree.h"
#include "collisionkdnode.h"
#include "types/boundbox.h"
#include "collisionprimitive.h"
#include "collisionkdtree2d.h"
#include "_global.h"
#include "mymath.h"
#include <QVector>



CollisionKdTree::CollisionKdTree()
{
	_function_name("CollisionKdTree::CollisionKdTree");
	staticBoundBoxes = new QVector<BoundBox*>;
	staticChildren = new QVector<CollisionPrimitive*>;
	splitAxis = 0;
	splitCoord = 0;
	splitCost = 0;
	threshold = 0.1;
	minPrimitivesPerNode = 1;
	numStaticChildren = 0;
	rootNode = nullptr;
}

CollisionKdTree::~CollisionKdTree()
{
	_function_name("CollisionKdTree::~CollisionKdTree");
	if (rootNode != nullptr)
		delete rootNode;

	delete staticChildren;

	delete staticBoundBoxes;
	this->_staticChildren = nullptr;

	/*
	if (this->staticChildren != nullptr) {
		for (int i = 0; i < this->staticChildren->length(); i++)
			this->staticChildren->at(i)->destroy();
		delete this->staticChildren;
	}
	*/
}

void CollisionKdTree::createTree(std::shared_ptr<QVector<std::shared_ptr<CollisionPrimitive>>> collisionPrimitives, BoundBox *boundBox)
{
	_function_name("CollisionKdTree::createTree");

	this->numStaticChildren = collisionPrimitives->length();

	this->_staticChildren = collisionPrimitives;
	this->staticChildren->reserve(this->numStaticChildren);

	this->staticBoundBoxes->reserve(this->numStaticChildren);

	this->rootNode = new CollisionKdNode();
	this->rootNode->indices = new QVector<int>;
	this->rootNode->indices->reserve(this->numStaticChildren);


	if (boundBox == nullptr) {
		this->rootNode->boundBox.infinity();
	} else {
		this->rootNode->boundBox.copyFrom(boundBox);
	}


	for (int i = 0; i < this->numStaticChildren; i++)
	{
		CollisionPrimitive *child = this->_staticChildren->at(i).get();
		this->staticChildren->append(child);
		BoundBox *childBoundBox = child->calculateAABB();
		this->staticBoundBoxes->append(childBoundBox);
		this->rootNode->boundBox.addBoundBox(childBoundBox);
		this->rootNode->indices->append(i);
	}

	this->splitNode(this->rootNode);
}


void CollisionKdTree::splitNode(CollisionKdNode *node)
{
	_function_name("CollisionKdTree::splitNode");
	QVector<int> *indices = node->indices;
	int numPrimitives = indices->length();

	if (numPrimitives <= this->minPrimitivesPerNode)
		return;

	const BoundBox &nodeBoundBox = node->boundBox;
	BoundBox nodeBoundBoxThreshold;

	nodeBoundBoxThreshold.minX = nodeBoundBox.minX + this->threshold;
	nodeBoundBoxThreshold.minY = nodeBoundBox.minY + this->threshold;
	nodeBoundBoxThreshold.minZ = nodeBoundBox.minZ + this->threshold;
	nodeBoundBoxThreshold.maxX = nodeBoundBox.maxX - this->threshold;
	nodeBoundBoxThreshold.maxY = nodeBoundBox.maxY - this->threshold;
	nodeBoundBoxThreshold.maxZ = nodeBoundBox.maxZ - this->threshold;
	double doubleThreshold = this->threshold * 2.;

	QVector<double> splitCoordsX;
	QVector<double> splitCoordsY;
	QVector<double> splitCoordsZ;


	for (int i = 0; i < numPrimitives; i++)
	{
		BoundBox *boundBox = this->staticBoundBoxes->at(indices->at(i));
		if ((boundBox->maxX - boundBox->minX) <= doubleThreshold)
		{
			if (boundBox->minX <= nodeBoundBoxThreshold.minX)
			{
				splitCoordsX.append(nodeBoundBox.minX);
			}
			else
			{
				if (boundBox->maxX >= nodeBoundBoxThreshold.maxX)
				{
					splitCoordsX.append(nodeBoundBox.maxX);
				}
				else
				{
					splitCoordsX.append((boundBox->minX + boundBox->maxX) * 0.5);
				}
			}
		}
		else
		{
			if (boundBox->minX > nodeBoundBoxThreshold.minX)
			{
				splitCoordsX.append(boundBox->minX);
			}
			if (boundBox->maxX < nodeBoundBoxThreshold.maxX)
			{
				splitCoordsX.append(boundBox->maxX);
			}
		}

		if ((boundBox->maxY - boundBox->minY) <= doubleThreshold)
		{
			if (boundBox->minY <= nodeBoundBoxThreshold.minY)
			{
				splitCoordsY.append(nodeBoundBox.minY);
			}
			else
			{
				if (boundBox->maxY >= nodeBoundBoxThreshold.maxY)
				{
					splitCoordsY.append(nodeBoundBox.maxY);
				}
				else
				{
					splitCoordsY.append((boundBox->minY + boundBox->maxY) * 0.5);
				}
			}
		}
		else
		{
			if (boundBox->minY > nodeBoundBoxThreshold.minY)
			{
				splitCoordsY.append(boundBox->minY);
			}
			if (boundBox->maxY < nodeBoundBoxThreshold.maxY)
			{
				splitCoordsY.append(boundBox->maxY);
			}
		}

		if ((boundBox->maxZ - boundBox->minZ) <= doubleThreshold)
		{
			if (boundBox->minZ <= nodeBoundBoxThreshold.minZ)
			{
				splitCoordsZ.append(nodeBoundBox.minZ);
			}
			else
			{
				if (boundBox->maxZ >= nodeBoundBoxThreshold.maxZ)
				{
					splitCoordsZ.append(nodeBoundBox.maxZ);
				}
				else
				{
					splitCoordsZ.append((boundBox->minZ + boundBox->maxZ) * 0.5);
				}
			}
		}
		else
		{
			if (boundBox->minZ > nodeBoundBoxThreshold.minZ)
			{
				splitCoordsZ.append(boundBox->minZ);
			}
			if (boundBox->maxZ < nodeBoundBoxThreshold.maxZ)
			{
				splitCoordsZ.append(boundBox->maxZ);
			}
		}
	}

	this->splitAxis = -1;
	this->splitCost = 1E308;
	double nodeBB[6];
	nodeBB[0] = nodeBoundBox.minX;
	nodeBB[1] = nodeBoundBox.minY;
	nodeBB[2] = nodeBoundBox.minZ;
	nodeBB[3] = nodeBoundBox.maxX;
	nodeBB[4] = nodeBoundBox.maxY;
	nodeBB[5] = nodeBoundBox.maxZ;
	this->checkNodeAxis(node, 0, splitCoordsX.length(), &splitCoordsX, nodeBB);
	this->checkNodeAxis(node, 1, splitCoordsY.length(), &splitCoordsY, nodeBB);
	this->checkNodeAxis(node, 2, splitCoordsZ.length(), &splitCoordsZ, nodeBB);

	if (this->splitAxis < 0)
		return;

	bool axisX = this->splitAxis == 0;
	bool axisY = this->splitAxis == 1;
	node->axis = this->splitAxis;
	node->coord = this->splitCoord;
	node->negativeNode = new CollisionKdNode();
	node->negativeNode->parent = node;
	node->negativeNode->boundBox.copyFrom(&nodeBoundBox);
	node->positiveNode = new CollisionKdNode();
	node->positiveNode->parent = node;
	node->positiveNode->boundBox.copyFrom(&nodeBoundBox);

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

	const double coordMin = this->splitCoord - this->threshold;
	const double coordMax = this->splitCoord + this->threshold;

	for (int i = 0; i < numPrimitives; i++)
	{
		BoundBox *boundBox = this->staticBoundBoxes->at(indices->at(i));
		double min = axisX ? boundBox->minX : (axisY ? boundBox->minY : boundBox->minZ);
		double max = axisX ? boundBox->maxX : (axisY ? boundBox->maxY : boundBox->maxZ);
		if (max <= coordMax)
		{
			if (min < coordMin)
			{
				if (node->negativeNode->indices == nullptr)
					node->negativeNode->indices = new QVector<int>;

				node->negativeNode->indices->append(indices->at(i));
				indices->replace(i, -1);
			}
			else
			{
				if (node->splitIndices == nullptr)
					node->splitIndices = new QVector<int>;

				node->splitIndices->append(indices->at(i));
				indices->replace(i, -1);
			}
		}
		else
		{
			if (min >= coordMin)
			{
				if (node->positiveNode->indices == nullptr)
				{
					node->positiveNode->indices = new QVector<int>;
				}
				node->positiveNode->indices->append(indices->at(i));
				indices->replace(i, -1);
			}
		}
	}


	int j = 0;
	for (int i = 0; i < numPrimitives; i++)
	{
		if (indices->at(i) >= 0)
		{
			indices->replace(j++, indices->at(i));
		}
	}

	if (j > 0)
		indices->resize(j);
	else {
		if (node->indices != nullptr) {
			delete node->indices;
			node->indices = nullptr;
		}
	}

	if (node->splitIndices != nullptr)
	{
		node->splitTree = new CollisionKdTree2D(this, node);
		node->splitTree->createTree();
	}

	if (node->negativeNode->indices != nullptr)
		this->splitNode(node->negativeNode);

	if (node->positiveNode->indices != nullptr)
		this->splitNode(node->positiveNode);
}

void CollisionKdTree::checkNodeAxis(CollisionKdNode *node, int axis, int numSplitCoords, QVector<double> *splitCoords, double bb[6])
{
	_function_name("CollisionKdTree::checkNodeAxis");

	int axis1 = (axis + 1) % 3;
	int axis2 = (axis + 2) % 3;
	double area = (bb[axis1 + 3] - bb[axis1]) * (bb[axis2 + 3] - bb[axis2]);

	for (int i = 0; i < numSplitCoords; i++)
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
			int j = 0;
			while (j < numObjects)
			{
				const BoundBox &boundBox = *this->staticBoundBoxes->at(node->indices->at(j));
				double bb[6];
				bb[0] = boundBox.minX;
				bb[1] = boundBox.minY;
				bb[2] = boundBox.minZ;
				bb[3] = boundBox.maxX;
				bb[4] = boundBox.maxY;
				bb[5] = boundBox.maxZ;
				if (bb[axis + 3] <= maxCoord)
				{
					if (bb[axis] < minCoord)
					{
						numNegative++;
					}
				}
				else
				{
					if (bb[axis] >= minCoord)
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
			double cost = (areaNegative * numNegative) + (areaPositive * numPositive);
			if (!conflict && cost < this->splitCost)
			{
				this->splitAxis = axis;
				this->splitCost = cost;
				this->splitCoord = currSplitCoord;
			}
			j = i + 1;
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
	}
}
