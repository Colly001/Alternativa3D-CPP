#ifndef BOXBOXCOLLIDER_H
#define BOXBOXCOLLIDER_H

#include "boxcollider.h"
#include "alternativa/math/vector3.h"

struct CollisionBox;
struct Vector3;
class ShapeContact;
class ContactPoint;


class BoxBoxCollider : public BoxCollider
{
	double epsilon;
	double minOverlap;
	int bestAxisIndex;
	ContactPoint *tmpPoints[8];
	Vector3 *points1[8];
	Vector3 *points2[8];
	Vector3 vectorToBox1;
	int pcount;

public:
	BoxBoxCollider();
	virtual ~BoxBoxCollider();
	bool getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact) override;
	bool haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2) override;
	double overlapOnAxis(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *axis, const Vector3 *vectorToBox1);
	void destroy() override;

private:
	bool findFaceContactPoints(const CollisionBox *box1, const CollisionBox *box2, Vector3 *vectorToBox1, int faceAxisIdx, Contact *contact);
	void reducePoints();
	double getPointBoxPenetration(const Vector3 *hs, const Vector3 *p, int faceAxisIdx, bool reverse);
	int clip(const Vector3 *hs, int faceAxisIdx);
	void findEdgesIntersection(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *vectorToBox1, int axisIdx1, int axisIdx2, Contact *contact);
	bool testMainAxis(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *axis, int axisIndex, const Vector3 *toBox1);
	bool testDerivedAxis(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *axis1, const Vector3 *axis2, int axisIndex, const Vector3 *toBox1);
};

#endif // BOXBOXCOLLIDER_H
