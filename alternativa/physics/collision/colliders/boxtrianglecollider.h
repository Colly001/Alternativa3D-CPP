#ifndef BOXTRIANGLECOLLIDER_H
#define BOXTRIANGLECOLLIDER_H

#include "boxcollider.h"
#include "alternativa/math/vector3.h"
#include <qglobal.h>

class CollisionBox;
class CollisionTriangle;

class BoxTriangleCollider : public BoxCollider
{
	double minOverlap;
	int bestAxisIndex;
	Vector3 *points1[8];
	Vector3 *points2[8];
	Vector3 toBox;

public:
	double epsilon = 0.001;

	BoxTriangleCollider();
	virtual ~BoxTriangleCollider();
	bool getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact) override;
	bool haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2) override;
	void destroy() override;

private:
	bool testMainAxis(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *axis, int axisIndex, Vector3 *toBox);
	bool testDerivedAxis(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *axis1, Vector3 *axis2, int axisIndex, Vector3 *toBox);
	double overlapOnAxis(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *axis, Vector3 *toBox);
	bool findFaceContactPoints(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *toBox, int faceAxisIndex, Contact *contact);
	bool getBoxToTriContact(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *toBox, Contact *contact);
	bool getTriToBoxContact(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *toBox, int faceAxisIdx, Contact *contact);
	double getPointBoxPenetration(const Vector3 *hs, const Vector3 *p, int faceAxisIdx, bool negativeFace);
	int clipByBox(const Vector3 *hs, int faceAxisIdx);
	int clipByTriangle(const CollisionTriangle *tri);
	int clipByLine(const Vector3 *linePoint, const Vector3 *lineDir, const Vector3 **verticesIn, int vnum, Vector3 **verticesOut);
	bool findEdgesIntersection(const CollisionBox *box, const CollisionTriangle *tri, const Vector3 *toBox, int boxAxisIdx, int triAxisIdx, Contact *contact);
};

#endif // BOXTRIANGLECOLLIDER_H
