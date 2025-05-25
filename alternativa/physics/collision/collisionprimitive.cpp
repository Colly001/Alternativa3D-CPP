#include "collisionprimitive.h"
#include "alternativa/math/matrix4.h"
#include "alternativa/physics/collision/types/boundbox.h"
#include "alternativa/math/matrix4.h"

#if 1
const int CollisionPrimitive::BOX = 1;
const int CollisionPrimitive::PLANE = 2;
const int CollisionPrimitive::SPHERE = 4;
const int CollisionPrimitive::RECT = 8;
const int CollisionPrimitive::TRIANGLE = 16;
#else
const int CollisionPrimitive::BOX = 1;
const int CollisionPrimitive::PLANE = 2;
const int CollisionPrimitive::SPHERE = 3;
const int CollisionPrimitive::RECT = 4;
const int CollisionPrimitive::TRIANGLE = 5;
#endif


CollisionPrimitive::CollisionPrimitive(int type, int collisionGroup)
{
	this->transform.toIdentity();
	this->aabb.infinity();
	this->type = type;
	this->collisionGroup = collisionGroup;
	this->localTransform = nullptr;
	this->postCollisionPredicate = nullptr;
	this->body = nullptr;
}

CollisionPrimitive::~CollisionPrimitive()
{
	if (this->localTransform != nullptr)
		delete this->localTransform;
}

void CollisionPrimitive::setBody(Body *body, const Matrix4 *localTransform)
{
	if (this->body == body)
		return;

	this->body = body;
	if (body != nullptr)
	{
		if (localTransform != nullptr)
		{
			if (this->localTransform == nullptr)
			{
				this->localTransform = new Matrix4();
			}
			this->localTransform->copy(localTransform);
		}
		else
		{
			this->localTransform = nullptr;
		}
	}
}

BoundBox* CollisionPrimitive::calculateAABB()
{
	return &this->aabb;
}

double CollisionPrimitive::getRayIntersection(const Vector3 *origin, const Vector3 *vector, double epsilon, Vector3 *normal)
{
	(void)origin;
	(void)vector;
	(void)epsilon;
	(void)normal;
	return -1.;
}

void CollisionPrimitive::copyFrom(CollisionPrimitive *source)
{
	if (source == nullptr)
		throw 10001;

	this->type = source->type;
	this->transform.copy(&source->transform);
	this->collisionGroup = source->collisionGroup;
	this->setBody(source->body, source->localTransform);
	this->aabb.copyFrom(&source->aabb);
}

CollisionPrimitive* CollisionPrimitive::createPrimitive()
{
	throw 141343;
	//return new CollisionPrimitive(this->type, this->collisionGroup);
}


