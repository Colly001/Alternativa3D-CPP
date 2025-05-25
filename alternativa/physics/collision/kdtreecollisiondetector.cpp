#include "kdtreecollisiondetector.h"
#include "collisionprimitive.h"
#include "colliders/boxboxcollider.h"
#include "colliders/boxspherecollider.h"
#include "colliders/boxrectcollider.h"
#include "colliders/boxtrianglecollider.h"
#include "colliders/spherespherecollider.h"
#include "collisionkdtree.h"
#include "types/boundbox.h"
#include "types/rayintersection.h"
#include "ICollisionPredicate.h"
#include "collisionkdnode.h"
#include "IRayCollisionPredicate.h"
#include "collisionkdtree2d.h"
#include <QVector>
#include <QHash>


static ICollider* getCollider(ICollider **colliders, CollisionPrimitive *prim1, CollisionPrimitive *prim2)
{
	return colliders[prim1->type | prim2->type];
}


KdTreeCollisionDetector::KdTreeCollisionDetector()
{
	threshold = 0.0001;
	_dynamicIntersection = new RayIntersection;
	for (int i = 0; i < 18; i++)
		colliders[i] = 0;

	this->tree = new CollisionKdTree();
	this->addCollider(CollisionPrimitive::BOX, CollisionPrimitive::BOX, new BoxBoxCollider());
	this->addCollider(CollisionPrimitive::BOX, CollisionPrimitive::SPHERE, new BoxSphereCollider());
	this->addCollider(CollisionPrimitive::BOX, CollisionPrimitive::RECT, new BoxRectCollider());
	this->addCollider(CollisionPrimitive::BOX, CollisionPrimitive::TRIANGLE, new BoxTriangleCollider());
	this->addCollider(CollisionPrimitive::SPHERE, CollisionPrimitive::SPHERE, new SphereSphereCollider());
}

KdTreeCollisionDetector::~KdTreeCollisionDetector()
{
	delete _dynamicIntersection;
	for (int i = 0; i < 18; i++) {
		ICollider *collider = colliders[i];
		if (collider != nullptr)
			collider->destroy();
	}
	delete tree;
}


void KdTreeCollisionDetector::addCollider(int type1, int type2, ICollider *collider)
{
	this->colliders[type1 | type2] = collider;
}

bool KdTreeCollisionDetector::addPrimitive(CollisionPrimitive *primitive, bool isStatic)
{
	(void)primitive;
	(void)isStatic;
	return true;
}

bool KdTreeCollisionDetector::removePrimitive(CollisionPrimitive *primitive, bool isStatic)
{
	(void)primitive;
	(void)isStatic;
	return true;
}

void KdTreeCollisionDetector::init(std::shared_ptr<QVector<std::shared_ptr<CollisionPrimitive>>> collisionPrimitives)
{
	this->tree->createTree(collisionPrimitives);
}

Contact* KdTreeCollisionDetector::getAllContacts(Contact *contacts)
{
	(void)contacts;
	return nullptr;
}

bool KdTreeCollisionDetector::getContact(CollisionPrimitive *prim1, CollisionPrimitive *prim2, Contact *contact)
{
	if ((prim1->collisionGroup & prim2->collisionGroup) == 0)
	{
		return false;
	}
	if (((!(prim1->body == nullptr)) && (prim1->body == prim2->body)))
	{
		return false;
	}
	if ((!(prim1->aabb.intersects(&prim2->aabb, 0.01))))
	{
		return false;
	}

	ICollider *collider = getCollider(this->colliders, prim1, prim2);

	if (((!(collider == nullptr)) && (collider->getContact(prim1, prim2, contact))))
	{
		if (((!(prim1->postCollisionPredicate == nullptr)) && (!(prim1->postCollisionPredicate->considerCollision(prim2)))))
		{
			return false;
		}
		if (((!(prim2->postCollisionPredicate == nullptr)) && (!(prim2->postCollisionPredicate->considerCollision(prim1)))))
		{
			return false;
		}
		return true;
	}

	return false;
}

bool KdTreeCollisionDetector::testCollision(CollisionPrimitive *prim1, CollisionPrimitive *prim2)
{
	if ((prim1->collisionGroup & prim2->collisionGroup) == 0)
	{
		return false;
	}
	if (((!(prim1->body == nullptr)) && (prim1->body == prim2->body)))
	{
		return false;
	}
	if ((!(prim1->aabb.intersects(&prim2->aabb, 0.01))))
	{
		return false;
	}

	ICollider *collider = getCollider(this->colliders, prim1, prim2);
	if (((!(collider == nullptr)) && (collider->haveCollision(prim1, prim2))))
	{
		if (((!(prim1->postCollisionPredicate == nullptr)) && (!(prim1->postCollisionPredicate->considerCollision(prim2)))))
		{
			return false;
		}
		if (((!(prim2->postCollisionPredicate == nullptr)) && (!(prim2->postCollisionPredicate->considerCollision(prim1)))))
		{
			return false;
		}
		return true;
	}
	return false;
}

bool KdTreeCollisionDetector::intersectRay(const Vector3 *origin, const Vector3 *dir, int collisionGroup, double maxTime, IRayCollisionPredicate *predicate, RayIntersection *result)
{
	bool hasStaticIntersection = this->intersectRayWithStatic(origin, dir, collisionGroup, maxTime, predicate, result);
	bool hasDynamicIntersection = this->intersectRayWithDynamic(origin, dir, collisionGroup, maxTime, predicate, this->_dynamicIntersection);
	if ((!((hasDynamicIntersection) || (hasStaticIntersection))))
	{
		return false;
	}
	if (((hasDynamicIntersection) && (hasStaticIntersection)))
	{
		if (result->t > this->_dynamicIntersection->t)
		{
			result->copy(this->_dynamicIntersection);
		}
		return true;
	}
	if (hasStaticIntersection)
	{
		return true;
	}
	result->copy(this->_dynamicIntersection);
	return true;
}

bool KdTreeCollisionDetector::intersectRayNecessarilyDynamic(const Vector3 *, const Vector3 *, int, double, IRayCollisionPredicate *, RayIntersection *)
{
	return false;
}

bool KdTreeCollisionDetector::intersectRayWithStatic(const Vector3 *origin, const Vector3 *dir, int collisionGroup, double maxTime, IRayCollisionPredicate *predicate, RayIntersection *result)
{
	if ((!(this->getRayBoundBoxIntersection(origin, dir, &this->tree->rootNode->boundBox, &this->_time))))
	{
		return false;
	}
	if (((this->_time.max < 0) || (this->_time.min > maxTime)))
	{
		return false;
	}

	Vector3 _o;

	if (this->_time.min <= 0)
	{
		this->_time.min = 0;
		_o.x = origin->x;
		_o.y = origin->y;
		_o.z = origin->z;
	}
	else
	{
		_o.x = (origin->x + (this->_time.min * dir->x));
		_o.y = (origin->y + (this->_time.min * dir->y));
		_o.z = (origin->z + (this->_time.min * dir->z));
	}
	if (this->_time.max > maxTime)
	{
		this->_time.max = maxTime;
	}
	bool hasIntersection = this->testRayAgainstNode(this->tree->rootNode, origin, &_o, dir, collisionGroup, this->_time.min, this->_time.max, predicate, result);
	return ((hasIntersection) ? (result->t <= maxTime) : false);
}

bool KdTreeCollisionDetector::testBodyPrimitiveCollision(Body *body, CollisionPrimitive *primitive)
{
	(void)body;
	(void)primitive;
	return false;
}


Contact *KdTreeCollisionDetector::getPrimitiveNodeCollisions(CollisionKdNode *node, CollisionPrimitive *primitive, Contact *contacts)
{
	(void)node;
	(void)primitive;
	(void)contacts;
	return nullptr;
}

bool KdTreeCollisionDetector::intersectRayWithDynamic(const Vector3 *origin, const Vector3 *dir, int collisionGroup, double maxTime, IRayCollisionPredicate *predicate, RayIntersection *result)
{
	double xx = origin->x + (dir->x * maxTime);
	double yy = origin->y + (dir->y * maxTime);
	double zz = origin->z + (dir->z * maxTime);

	BoundBox rayAABB;
	if (xx < origin->x)
	{
		rayAABB.minX = xx;
		rayAABB.maxX = origin->x;
	}
	else
	{
		rayAABB.minX = origin->x;
		rayAABB.maxX = xx;
	}
	if (yy < origin->y)
	{
		rayAABB.minY = yy;
		rayAABB.maxY = origin->y;
	}
	else
	{
		rayAABB.minY = origin->y;
		rayAABB.maxY = yy;
	}
	if (zz < origin->z)
	{
		rayAABB.minZ = zz;
		rayAABB.maxZ = origin->z;
	}
	else
	{
		rayAABB.minZ = origin->z;
		rayAABB.maxZ = zz;
	}

	double minTime = (maxTime + 1);

	for (int i = 0; i < this->dynamicPrimitivesNum; i++)
	{
		CollisionPrimitive *primitive = this->dynamicPrimitives->at(i);
		if ((primitive->collisionGroup & collisionGroup) != 0)
		{
			const BoundBox &paabb = primitive->aabb;
			if (!((((((rayAABB.maxX < paabb.minX) || (rayAABB.minX > paabb.maxX)) || (rayAABB.maxY < paabb.minY)) || (rayAABB.minY > paabb.maxY)) || (rayAABB.maxZ < paabb.minZ)) || (rayAABB.minZ > paabb.maxZ)))
			{
				if (!(((!(predicate == nullptr)) && (!(primitive->body == nullptr))) && (!(predicate->considerBody(primitive->body)))))
				{
					Vector3 _n;
					double t = primitive->getRayIntersection(origin, dir, this->threshold, &_n);
					if (((t > 0) && (t < minTime)))
					{
						minTime = t;
						result->primitive = primitive;
						result->normal.x = _n.x;
						result->normal.y = _n.y;
						result->normal.z = _n.z;
					}
				}
			}
		}
	}
	if (minTime > maxTime)
	{
		return false;
	}
	result->pos.x = (origin->x + (dir->x * minTime));
	result->pos.y = (origin->y + (dir->y * minTime));
	result->pos.z = (origin->z + (dir->z * minTime));
	result->t = minTime;

	return true;
}

bool KdTreeCollisionDetector::getRayBoundBoxIntersection(const Vector3 *origin, const Vector3 *dir, BoundBox *bb, MinMax *time)
{
	double t1;
	double t2;
	time->min = -1;
	time->max = 1E308;

	for (int i = 0; i < 3; i++)
	{
		switch (i)
		{
			case 0:
				if (((dir->x < this->threshold) && (dir->x > -(this->threshold))))
				{
					if (((origin->x < bb->minX) || (origin->x > bb->maxX)))
					{
						return false;
					}
					continue;
				}
				t1 = ((bb->minX - origin->x) / dir->x);
				t2 = ((bb->maxX - origin->x) / dir->x);
				break;
			case 1:
				if (((dir->y < this->threshold) && (dir->y > -(this->threshold))))
				{
					if (((origin->y < bb->minY) || (origin->y > bb->maxY)))
					{
						return false;
					}
					continue;
				}
				t1 = ((bb->minY - origin->y) / dir->y);
				t2 = ((bb->maxY - origin->y) / dir->y);
				break;
			case 2:
				if (((dir->z < this->threshold) && (dir->z > -(this->threshold))))
				{
					if (((origin->z < bb->minZ) || (origin->z > bb->maxZ)))
					{
						return false;
					}
					continue;
				}
				t1 = ((bb->minZ - origin->z) / dir->z);
				t2 = ((bb->maxZ - origin->z) / dir->z);
				break;
		}
		if (t1 < t2)
		{
			if (t1 > time->min)
			{
				time->min = t1;
			}
			if (t2 < time->max)
			{
				time->max = t2;
			}
		}
		else
		{
			if (t2 > time->min)
			{
				time->min = t2;
			}
			if (t1 < time->max)
			{
				time->max = t1;
			}
		}
		if (time->max < time->min)
		{
			return false;
		}
	}

	return true;
}

bool KdTreeCollisionDetector::testRayAgainstNode(CollisionKdNode *node, const Vector3 *origin, const Vector3 *localOrigin, const Vector3 *dir, int collisionGroup, double t1, double t2, IRayCollisionPredicate *predicate, RayIntersection *result)
{
	double splitTime;
	CollisionKdNode *currChildNode;
	if (((!(node->indices == nullptr)) && (this->getRayNodeIntersection(origin, dir, collisionGroup, this->tree->staticChildren, node->indices, predicate, result))))
	{
		return true;
	}
	if (node->axis == -1)
	{
		return false;
	}
	switch (node->axis)
	{
		case 0:
			if (((dir->x > -(this->threshold)) && (dir->x < this->threshold)))
			{
				splitTime = (t2 + 1);
			}
			else
			{
				splitTime = ((node->coord - origin->x) / dir->x);
			}
			currChildNode = ((localOrigin->x < node->coord) ? node->negativeNode : node->positiveNode);
			break;
		case 1:
			if (((dir->y > -(this->threshold)) && (dir->y < this->threshold)))
			{
				splitTime = (t2 + 1);
			}
			else
			{
				splitTime = ((node->coord - origin->y) / dir->y);
			}
			currChildNode = ((localOrigin->y < node->coord) ? node->negativeNode : node->positiveNode);
			break;
		case 2:
			if (((dir->z > -(this->threshold)) && (dir->z < this->threshold)))
			{
				splitTime = (t2 + 1);
			}
			else
			{
				splitTime = ((node->coord - origin->z) / dir->z);
			}
			currChildNode = ((localOrigin->z < node->coord) ? node->negativeNode : node->positiveNode);
			break;
		default:
			throw 4543;
	}


	volatile double _splitTime = splitTime;

	if (_splitTime < t1 || _splitTime > t2)
	{
		return (this->testRayAgainstNode(currChildNode, origin, localOrigin, dir, collisionGroup, t1, t2, predicate, result));
	}
	bool intersects = this->testRayAgainstNode(currChildNode, origin, localOrigin, dir, collisionGroup, t1, splitTime, predicate, result);
	if (intersects)
		return true;

	Vector3 _o;
	_o.x = (origin->x + (splitTime * dir->x));
	_o.y = (origin->y + (splitTime * dir->y));
	_o.z = (origin->z + (splitTime * dir->z));
	if (node->splitTree != nullptr)
	{
		CollisionKdNode *splitNode = node->splitTree->rootNode;
		while (((!(splitNode == nullptr)) && (!(splitNode->axis == -1))))
		{
			switch (splitNode->axis)
			{
				case 0:
					splitNode = ((_o.x < splitNode->coord) ? splitNode->negativeNode : splitNode->positiveNode);
					break;
				case 1:
					splitNode = ((_o.y < splitNode->coord) ? splitNode->negativeNode : splitNode->positiveNode);
					break;
				case 2:
					splitNode = ((_o.z < splitNode->coord) ? splitNode->negativeNode : splitNode->positiveNode);
					break;
			}
		}
		if (((!(splitNode == nullptr)) && (!(splitNode->indices == nullptr))))
		{
			for (int i = splitNode->indices->length() - 1; i >= 0; i--)
			{
				CollisionPrimitive *primitive = this->tree->staticChildren->at(splitNode->indices->at(i));
				if ((primitive->collisionGroup & collisionGroup) != 0)
				{
					if (!(predicate != nullptr && primitive->body != nullptr && !predicate->considerBody(primitive->body)))
					{
						result->t = primitive->getRayIntersection(origin, dir, this->threshold, &result->normal);
						if (result->t >= 0)
						{
							result->pos.copy(&_o);
							result->primitive = primitive;
							return true;
						}
					}
				}
			}
		}
	}
	return this->testRayAgainstNode(((currChildNode == node->negativeNode) ? node->positiveNode : node->negativeNode), origin, &_o, dir, collisionGroup, splitTime, t2, predicate, result);
}

bool KdTreeCollisionDetector::getRayNodeIntersection(const Vector3 *origin, const Vector3 *dir, int collisionGroup, QVector<CollisionPrimitive *> *primitives, QVector<int> *indices, IRayCollisionPredicate *predicate, RayIntersection *intersection)
{
	int pnum = indices->length();
	double minTime = 1E308;
	for (int i = 0; i < pnum; i++)
	{
		CollisionPrimitive *primitive = primitives->at(indices->at(i));
		if ((primitive->collisionGroup & collisionGroup) != 0)
		{
			if (!(((!(predicate == nullptr)) && (!(primitive->body == nullptr))) && (!(predicate->considerBody(primitive->body)))))
			{
				Vector3 _n;
				double t = primitive->getRayIntersection(origin, dir, this->threshold, &_n);
				if (((t > 0) && (t < minTime)))
				{
					minTime = t;
					intersection->primitive = primitive;
					intersection->normal.x = _n.x;
					intersection->normal.y = _n.y;
					intersection->normal.z = _n.z;
				}
			}
		}
	}
	if (minTime == 1E308)
	{
		return false;
	}
	intersection->pos.x = (origin->x + (dir->x * minTime));
	intersection->pos.y = (origin->y + (dir->y * minTime));
	intersection->pos.z = (origin->z + (dir->z * minTime));
	intersection->t = minTime;
	return true;
}

void KdTreeCollisionDetector::destroy()
{
	delete this;
}
