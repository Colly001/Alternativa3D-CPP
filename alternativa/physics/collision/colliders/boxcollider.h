#ifndef BOXCOLLIDER_H
#define BOXCOLLIDER_H

#include "../ICollider.h"
#include <qglobal.h>

class Vector3;
class CollisionPrimitive;


class BoxCollider : public ICollider
{
public:
	BoxCollider();
	virtual ~BoxCollider();

	virtual bool getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact) override;
	virtual bool haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2) override;

protected:
	void getFaceVertsByAxis(const Vector3 *hs, int axisIdx, bool negativeFace, Vector3 *result[8]);
	int clipLowX(double x, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon);
	int clipHighX(double x, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon);
	int clipLowY(double y, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon);
	int clipHighY(double y, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon);
	int clipLowZ(double z, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon);
	int clipHighZ(double z, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon);
	virtual void destroy();
};

#endif // BOXCOLLIDER_H
