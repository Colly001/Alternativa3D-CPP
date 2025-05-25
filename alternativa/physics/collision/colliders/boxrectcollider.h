#ifndef BOXRECTCOLLIDER_H
#define BOXRECTCOLLIDER_H

#include "boxcollider.h"
#include "alternativa/math/vector3.h"

struct CollisionBox;
struct CollisionRect;

class BoxRectCollider : public BoxCollider
{
	double epsilon;
	double minOverlap;
	int bestAxisIndex;
	Vector3 *points1[8];
	Vector3 *points2[8];
	Vector3 vectorToBox;

public:

	BoxRectCollider();
	virtual ~BoxRectCollider();

	bool getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact) override;
	bool haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2) override;
	double overlapOnAxis(CollisionBox *box, CollisionRect *rect, const Vector3 *axis, const Vector3 *vectorToBox);
	void destroy() override;

private:
	bool findFaceContactPoints(CollisionBox *box, CollisionRect *rect, Vector3 *vectorToBox, int faceAxisIdx, Contact *contact);
	double getPointBoxPenetration(Vector3 *hs, Vector3 *p, int faceAxisIdx, bool reverse);
	int clipByBox(Vector3 *hs, int faceAxisIdx);
	int clipByRect(Vector3 *hs);
	bool findEdgesIntersection(CollisionBox *box, CollisionRect *rect, Vector3 *vectorToBox, int axisIdx1, int axisIdx2, Contact *contact);
	bool testMainAxis(CollisionBox *box, CollisionRect *rect, Vector3 *axis, int axisIndex, Vector3 *vectorToBox);
	bool testDerivedAxis(CollisionBox *box, CollisionRect *rect, Vector3 *axis1, Vector3 *axis2, int axisIndex, Vector3 *vectorToBox);
};

#endif // BOXRECTCOLLIDER_H
