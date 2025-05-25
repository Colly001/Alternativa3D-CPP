#include "boxtrianglecollider.h"
#include "mymath.h"
#include "alternativa/physics/collision/collisionprimitive.h"
#include "alternativa/physics/contact.h"
#include "alternativa/physics/collision/primitives/collisionbox.h"
#include "alternativa/physics/collision/primitives/collisiontriangle.h"
#include "alternativa/physics/contactpoint.h"
#include "mymath.h"
#include "_global.h"



BoxTriangleCollider::BoxTriangleCollider()
{
	toBox.reset(MyMath::nan(), MyMath::nan(), MyMath::nan());
	bestAxisIndex = 0;
	minOverlap = 0;
	for (int i = 0; i < 8; i++)
	{
		this->points1[i] = new Vector3();
		this->points2[i] = new Vector3();
	}
}

BoxTriangleCollider::~BoxTriangleCollider()
{
	for (int i = 0; i < 8; i++)
	{
		delete this->points1[i];
		delete this->points2[i];
	}
}

bool BoxTriangleCollider::getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact)
{
	if ((!(this->haveCollision(prim1, prim2))))
	{
		return false;
	}

	CollisionBox const * box;
	CollisionTriangle const * tri;

	if (prim1->type == CollisionPrimitive::BOX)
	{
		box = (const CollisionBox*)prim1;
		tri = (const CollisionTriangle*)prim2;
	}
	else
	{
		box = (const CollisionBox*)prim2;
		tri = (const CollisionTriangle*)prim1;
	}


	if (this->bestAxisIndex < 4)
	{
		if ((!(this->findFaceContactPoints(box, tri, &toBox, this->bestAxisIndex, contact))))
		{
			return false;
		}
	}
	else
	{
		this->bestAxisIndex = (this->bestAxisIndex - 4);
		if ((!(this->findEdgesIntersection(box, tri, &toBox, (this->bestAxisIndex % 3), int((this->bestAxisIndex / 3)), contact))))
		{
			return false;
		}
	}
	contact->body1 = box->body;
	contact->body2 = tri->body;
	if (tri->transform.k > 0.99999)
	{
		contact->normal.x = 0;
		contact->normal.y = 0;
		contact->normal.z = 1;
	}
	return true;
}

bool BoxTriangleCollider::haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2)
{
	CollisionBox const * box;
	CollisionTriangle const * tri;

	if (prim1->type == CollisionPrimitive::BOX)
	{
		box = (const CollisionBox*)prim1;
		tri = (const CollisionTriangle*)prim2;
	}
	else
	{
		box = (const CollisionBox*)prim2;
		tri = (const CollisionTriangle*)prim1;
	}



	const Matrix4 &boxTransform = box->transform;
	const Matrix4 &triTransform = tri->transform;



	toBox.x = (boxTransform.d - triTransform.d);
	toBox.y = (boxTransform.h - triTransform.h);
	toBox.z = (boxTransform.l - triTransform.l);
	this->minOverlap = 1E308;

	Vector3 axis;
	axis.x = triTransform.c;
	axis.y = triTransform.g;
	axis.z = triTransform.k;
	if ((!(this->testMainAxis(box, tri, &axis, 0, &toBox))))
	{
		return false;
	}
	Vector3 axis10;
	axis10.x = boxTransform.a;
	axis10.y = boxTransform.e;
	axis10.z = boxTransform.i;
	if ((!(this->testMainAxis(box, tri, &axis10, 1, &toBox))))
	{
		return false;
	}
	Vector3 axis11;
	axis11.x = boxTransform.b;
	axis11.y = boxTransform.f;
	axis11.z = boxTransform.j;
	if ((!(this->testMainAxis(box, tri, &axis11, 2, &toBox))))
	{
		return false;
	}
	Vector3 axis12;
	axis12.x = boxTransform.c;
	axis12.y = boxTransform.g;
	axis12.z = boxTransform.k;
	if ((!(this->testMainAxis(box, tri, &axis12, 3, &toBox))))
	{
		return false;
	}
	const Vector3 &v1 = tri->e0;
	Vector3 axis20;
	axis20.x = (((triTransform.a * v1.x) + (triTransform.b * v1.y)) + (triTransform.c * v1.z));
	axis20.y = (((triTransform.e * v1.x) + (triTransform.f * v1.y)) + (triTransform.g * v1.z));
	axis20.z = (((triTransform.i * v1.x) + (triTransform.j * v1.y)) + (triTransform.k * v1.z));
	if ((!(this->testDerivedAxis(box, tri, &axis10, &axis20, 4, &toBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, tri, &axis11, &axis20, 5, &toBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, tri, &axis12, &axis20, 6, &toBox))))
	{
		return false;
	}
	const Vector3 &v2 = tri->e1;
	Vector3 axis21;
	axis21.x = (((triTransform.a * v2.x) + (triTransform.b * v2.y)) + (triTransform.c * v2.z));
	axis21.y = (((triTransform.e * v2.x) + (triTransform.f * v2.y)) + (triTransform.g * v2.z));
	axis21.z = (((triTransform.i * v2.x) + (triTransform.j * v2.y)) + (triTransform.k * v2.z));
	if ((!(this->testDerivedAxis(box, tri, &axis10, &axis21, 7, &toBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, tri, &axis11, &axis21, 8, &toBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, tri, &axis12, &axis21, 9, &toBox))))
	{
		return false;
	}
	const Vector3 &v3 = tri->e2;
	Vector3 axis22;
	axis22.x = (((triTransform.a * v3.x) + (triTransform.b * v3.y)) + (triTransform.c * v3.z));
	axis22.y = (((triTransform.e * v3.x) + (triTransform.f * v3.y)) + (triTransform.g * v3.z));
	axis22.z = (((triTransform.i * v3.x) + (triTransform.j * v3.y)) + (triTransform.k * v3.z));
	if ((!(this->testDerivedAxis(box, tri, &axis10, &axis22, 10, &toBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, tri, &axis11, &axis22, 11, &toBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, tri, &axis12, &axis22, 12, &toBox))))
	{
		return false;
	}
	return true;
}

void BoxTriangleCollider::destroy()
{
	delete this;
}

bool BoxTriangleCollider::testMainAxis(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *axis, int axisIndex, Vector3 *toBox)
{
	double overlap = this->overlapOnAxis(box, tri, axis, toBox);
	if (overlap < -this->epsilon)
	{
		return false;
	}
	if ((overlap + this->epsilon) < this->minOverlap)
	{
		this->minOverlap = overlap;
		this->bestAxisIndex = axisIndex;
	}
	return true;
}

bool BoxTriangleCollider::testDerivedAxis(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *axis1, Vector3 *axis2, int axisIndex, Vector3 *toBox)
{
	Vector3 axis;
	axis.x = (axis1->y * axis2->z) - (axis1->z * axis2->y);
	axis.y = (axis1->z * axis2->x) - (axis1->x * axis2->z);
	axis.z = (axis1->x * axis2->y) - (axis1->y * axis2->x);
	double lenSqr = (axis.x * axis.x) + (axis.y * axis.y) + (axis.z * axis.z);
	if (lenSqr < 0.0001)
	{
		return true;
	}
	double k = 1. / MyMath::sqrt(lenSqr);
	axis.x = axis.x * k;
	axis.y = axis.y * k;
	axis.z = axis.z * k;
	double overlap = this->overlapOnAxis(box, tri, &axis, toBox);
	if (overlap < -this->epsilon)
	{
		return false;
	}
	if ((overlap + this->epsilon) < this->minOverlap)
	{
		this->minOverlap = overlap;
		this->bestAxisIndex = axisIndex;
	}
	return true;
}

double BoxTriangleCollider::overlapOnAxis(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *axis, Vector3 *toBox)
{
	const Matrix4 &t1 = box->transform;

	double projection = ((((t1.a * axis->x) + (t1.e * axis->y)) + (t1.i * axis->z)) * box->hs.x);
	if (projection < 0)
	{
		projection = -(projection);
	}
	double d = ((((t1.b * axis->x) + (t1.f * axis->y)) + (t1.j * axis->z)) * box->hs.y);
	projection = (projection + ((d < 0) ? -(d) : d));
	d = ((((t1.c * axis->x) + (t1.g * axis->y)) + (t1.k * axis->z)) * box->hs.z);
	projection = (projection + ((d < 0) ? -(d) : d));
	double vectorProjection = (((toBox->x * axis->x) + (toBox->y * axis->y)) + (toBox->z * axis->z));

	const Matrix4 &t2 = tri->transform;
	double ax = (((t2.a * axis->x) + (t2.e * axis->y)) + (t2.i * axis->z));
	double ay = (((t2.b * axis->x) + (t2.f * axis->y)) + (t2.j * axis->z));
	double az = (((t2.c * axis->x) + (t2.g * axis->y)) + (t2.k * axis->z));
	double max = 0;
	if (vectorProjection < 0.)
	{
		vectorProjection = -vectorProjection;
		d = (((tri->v0.x * ax) + (tri->v0.y * ay)) + (tri->v0.z * az));
		if (d < max)
		{
			max = d;
		}
		d = (((tri->v1.x * ax) + (tri->v1.y * ay)) + (tri->v1.z * az));
		if (d < max)
		{
			max = d;
		}
		d = (((tri->v2.x * ax) + (tri->v2.y * ay)) + (tri->v2.z * az));
		if (d < max)
		{
			max = d;
		}
		max = -(max);
	}
	else
	{
		d = (((tri->v0.x * ax) + (tri->v0.y * ay)) + (tri->v0.z * az));
		if (d > max)
		{
			max = d;
		}
		d = (((tri->v1.x * ax) + (tri->v1.y * ay)) + (tri->v1.z * az));
		if (d > max)
		{
			max = d;
		}
		d = (((tri->v2.x * ax) + (tri->v2.y * ay)) + (tri->v2.z * az));
		if (d > max)
		{
			max = d;
		}
	}

	return (projection + max) - vectorProjection;
}

bool BoxTriangleCollider::findFaceContactPoints(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *toBox, int faceAxisIndex, Contact *contact)
{
	if (faceAxisIndex == 0)
	{
		return this->getBoxToTriContact(box, tri, toBox, contact);
	}
	return this->getTriToBoxContact(box, tri, toBox, faceAxisIndex, contact);
}

bool BoxTriangleCollider::getBoxToTriContact(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *toBox, Contact *contact)
{
	const Matrix4 &boxTransform = box->transform;
	const Matrix4 &triTransform = tri->transform;

	Vector3 colNormal;
	colNormal.x = triTransform.c;
	colNormal.y = triTransform.g;
	colNormal.z = triTransform.k;
	bool over = ((((toBox->x * colNormal.x) + (toBox->y * colNormal.y)) + (toBox->z * colNormal.z)) > 0);
	if (!over)
	{
		colNormal.x = -colNormal.x;
		colNormal.y = -colNormal.y;
		colNormal.z = -colNormal.z;
	}
	int incFaceAxisIdx = 0;
	double incAxisDot = 0;
	double maxDot = 0;
	int axisIdx = 0;
	while (axisIdx < 3)
	{
		Vector3 axis;
		boxTransform.getAxis(axisIdx, &axis);
		double dot = (((axis.x * colNormal.x) + (axis.y * colNormal.y)) + (axis.z * colNormal.z));
		double absDot = (dot < 0) ? -(dot) : dot;
		if (absDot > maxDot)
		{
			maxDot = absDot;
			incAxisDot = dot;
			incFaceAxisIdx = axisIdx;
		}
		axisIdx++;
	}
	bool negativeFace = incAxisDot > 0.;
	int code = (1 << (incFaceAxisIdx << 1));
	if (negativeFace)
	{
		code = (code << 1);
	}
	if ((code & box->excludedFaces) != 0)
	{
		return false;
	}
	getFaceVertsByAxis(&box->hs, incFaceAxisIdx, negativeFace, this->points1);
	boxTransform.transformVectorsN((const Vector3**)this->points1, this->points2, 4);
	triTransform.transformVectorsInverseN((const Vector3**)this->points2, this->points1, 4);
	int pnum = this->clipByTriangle(tri);
	contact->pcount = 0;
	int i = 0;
	while (i < pnum)
	{
		Vector3 *v = this->points2[i];
		if ((((over) && (v->z < 0)) || ((!(over)) && (v->z > 0))))
		{
			ContactPoint *cp = contact->points[contact->pcount++];
			Vector3 &cpPos = cp->pos;
			cpPos.x = ((((triTransform.a * v->x) + (triTransform.b * v->y)) + (triTransform.c * v->z)) + triTransform.d);
			cpPos.y = ((((triTransform.e * v->x) + (triTransform.f * v->y)) + (triTransform.g * v->z)) + triTransform.h);
			cpPos.z = ((((triTransform.i * v->x) + (triTransform.j * v->y)) + (triTransform.k * v->z)) + triTransform.l);
			Vector3 *r = &cp->r1;
			r->x = cpPos.x - boxTransform.d;
			r->y = cpPos.y - boxTransform.h;
			r->z = cpPos.z - boxTransform.l;
			r = &cp->r2;
			r->x = cpPos.x - triTransform.d;
			r->y = cpPos.y - triTransform.h;
			r->z = cpPos.z - triTransform.l;
			cp->penetration = over ? -v->z : v->z;
		}
		i++;
	}
	contact->normal.x = colNormal.x;
	contact->normal.y = colNormal.y;
	contact->normal.z = colNormal.z;
	return true;
}

bool BoxTriangleCollider::getTriToBoxContact(const CollisionBox *box, const CollisionTriangle *tri, Vector3 *toBox, int faceAxisIdx, Contact *contact)
{
	faceAxisIdx--;
	const Matrix4 &boxTransform = box->transform;
	const Matrix4 &triTransform = tri->transform;

	Vector3 colNormal;
	boxTransform.getAxis(faceAxisIdx, &colNormal);
	bool negativeFace = ((((toBox->x * colNormal.x) + (toBox->y * colNormal.y)) + (toBox->z * colNormal.z)) > 0);
	int code = (1 << (faceAxisIdx << 1));
	if (negativeFace)
	{
		code = code << 1;
	}
	if ((code & box->excludedFaces) != 0)
	{
		return false;
	}
	if ((!(negativeFace)))
	{
		colNormal.x = -colNormal.x;
		colNormal.y = -colNormal.y;
		colNormal.z = -colNormal.z;
	}
	Vector3 *v = this->points1[0];
	v->x = tri->v0.x;
	v->y = tri->v0.y;
	v->z = tri->v0.z;
	v = this->points1[1];
	v->x = tri->v1.x;
	v->y = tri->v1.y;
	v->z = tri->v1.z;
	v = this->points1[2];
	v->x = tri->v2.x;
	v->y = tri->v2.y;
	v->z = tri->v2.z;
	triTransform.transformVectorsN((const Vector3**)this->points1, this->points2, 3);
	boxTransform.transformVectorsInverseN((const Vector3**)this->points2, this->points1, 3);
	int pnum = this->clipByBox(&box->hs, faceAxisIdx);
	contact->pcount = 0;
	int i = 0;
	while (i < pnum)
	{
		Vector3 *v = this->points1[i];
		double penetration = this->getPointBoxPenetration(&box->hs, v, faceAxisIdx, negativeFace);
		if (penetration > -this->epsilon)
		{
			ContactPoint *cp = contact->points[contact->pcount++];
			Vector3 &cpPos = cp->pos;
			cpPos.x = ((((boxTransform.a * v->x) + (boxTransform.b * v->y)) + (boxTransform.c * v->z)) + boxTransform.d);
			cpPos.y = ((((boxTransform.e * v->x) + (boxTransform.f * v->y)) + (boxTransform.g * v->z)) + boxTransform.h);
			cpPos.z = ((((boxTransform.i * v->x) + (boxTransform.j * v->y)) + (boxTransform.k * v->z)) + boxTransform.l);
			Vector3 *r = &cp->r1;
			r->x = (cpPos.x - boxTransform.d);
			r->y = (cpPos.y - boxTransform.h);
			r->z = (cpPos.z - boxTransform.l);
			r = &cp->r2;
			r->x = (cpPos.x - triTransform.d);
			r->y = (cpPos.y - triTransform.h);
			r->z = (cpPos.z - triTransform.l);
			cp->penetration = penetration;
		}
		i++;
	}
	contact->normal.x = colNormal.x;
	contact->normal.y = colNormal.y;
	contact->normal.z = colNormal.z;
	return true;
}

double BoxTriangleCollider::getPointBoxPenetration(const Vector3 *hs, const Vector3 *p, int faceAxisIdx, bool negativeFace)
{
	switch (faceAxisIdx)
	{
		case 0:
			if (negativeFace)
			{
				return (p->x + hs->x);
			}
			return (hs->x - p->x);
		case 1:
			if (negativeFace)
			{
				return (p->y + hs->y);
			}
			return (hs->y - p->y);
		case 2:
			if (negativeFace)
			{
				return (p->z + hs->z);
			}
			return (hs->z - p->z);
	}
	return 0;
}

int BoxTriangleCollider::clipByBox(const Vector3 *hs, int faceAxisIdx)
{
	int pnum = 3;
	switch (faceAxisIdx)
	{
		case 0:
			if ((pnum = clipLowZ(-(hs->z), pnum, this->points1, this->points2, this->epsilon)) == 0)
			{
				return (0);
			}
			if ((pnum = clipHighZ(hs->z, pnum, this->points2, this->points1, this->epsilon)) == 0)
			{
				return (0);
			}
			if ((pnum = clipLowY(-(hs->y), pnum, this->points1, this->points2, this->epsilon)) == 0)
			{
				return (0);
			}
			return (clipHighY(hs->y, pnum, this->points2, this->points1, this->epsilon));
		case 1:
			if ((pnum = clipLowZ(-(hs->z), pnum, this->points1, this->points2, this->epsilon)) == 0)
			{
				return (0);
			}
			if ((pnum = clipHighZ(hs->z, pnum, this->points2, this->points1, this->epsilon)) == 0)
			{
				return (0);
			}
			if ((pnum = clipLowX(-(hs->x), pnum, this->points1, this->points2, this->epsilon)) == 0)
			{
				return (0);
			}
			return (clipHighX(hs->x, pnum, this->points2, this->points1, this->epsilon));
		case 2:
			if ((pnum = clipLowX(-(hs->x), pnum, this->points1, this->points2, this->epsilon)) == 0)
			{
				return (0);
			}
			if ((pnum = clipHighX(hs->x, pnum, this->points2, this->points1, this->epsilon)) == 0)
			{
				return (0);
			}
			if ((pnum = clipLowY(-(hs->y), pnum, this->points1, this->points2, this->epsilon)) == 0)
			{
				return (0);
			}
			return (clipHighY(hs->y, pnum, this->points2, this->points1, this->epsilon));
	}
	return 0;
}

int BoxTriangleCollider::clipByTriangle(const CollisionTriangle *tri)
{
	int vnum = 4;
	vnum = this->clipByLine(&tri->v0, &tri->e0, (const Vector3 **)this->points1, vnum, this->points2);
	if (vnum == 0)
	{
		return 0;
	}
	vnum = this->clipByLine(&tri->v1, &tri->e1, (const Vector3 **)this->points2, vnum, this->points1);
	if (vnum == 0)
	{
		return 0;
	}
	return this->clipByLine(&tri->v2, &tri->e2, (const Vector3 **)this->points1, vnum, this->points2);
}

int BoxTriangleCollider::clipByLine(const Vector3 *linePoint, const Vector3 *lineDir, const Vector3 **verticesIn, int vnum, Vector3 **verticesOut)
{
	//var t:Number;
	//var v:Vector3;

	double nx = -lineDir->y;
	double ny = lineDir->x;
	double offset = (linePoint->x * nx) + (linePoint->y * ny);
	const Vector3 *v1 = verticesIn[int(vnum - 1)];
	double offset1 = ((v1->x * nx) + (v1->y * ny));
	int num = 0;
	int i = 0;
	while (i < vnum)
	{
		const Vector3 *v2 = verticesIn[i];
		double offset2 = (v2->x * nx) + (v2->y * ny);
		if (offset1 < offset)
		{
			if (offset2 > offset)
			{
				double t = (offset - offset1) / (offset2 - offset1);
				Vector3 *v = verticesOut[num];
				v->x = v1->x + (t * (v2->x - v1->x));
				v->y = v1->y + (t * (v2->y - v1->y));
				v->z = v1->z + (t * (v2->z - v1->z));
				num++;
			}
		}
		else
		{
			Vector3 *v = verticesOut[num];
			v->x = v1->x;
			v->y = v1->y;
			v->z = v1->z;
			num++;
			if (offset2 < offset)
			{
				double t = (offset - offset1) / (offset2 - offset1);
				Vector3 *v = verticesOut[num];
				v->x = v1->x + (t * (v2->x - v1->x));
				v->y = v1->y + (t * (v2->y - v1->y));
				v->z = v1->z + (t * (v2->z - v1->z));
				num++;
			}
		}
		v1 = v2;
		offset1 = offset2;
		i++;
	}
	return num;
}

bool BoxTriangleCollider::findEdgesIntersection(const CollisionBox *box, const CollisionTriangle *tri, const Vector3 *toBox, int boxAxisIdx, int triAxisIdx, Contact *contact)
{
	double tmpx1;
	double tmpy1;
	double tmpz1;
	double tmpx2;
	double tmpy2;
	double tmpz2;
	//double boxHalfLen;


	switch (triAxisIdx)
	{
		case 0:
			tmpx1 = tri->e0.x;
			tmpy1 = tri->e0.y;
			tmpz1 = tri->e0.z;
			tmpx2 = tri->v0.x;
			tmpy2 = tri->v0.y;
			tmpz2 = tri->v0.z;
			break;
		case 1:
			tmpx1 = tri->e1.x;
			tmpy1 = tri->e1.y;
			tmpz1 = tri->e1.z;
			tmpx2 = tri->v1.x;
			tmpy2 = tri->v1.y;
			tmpz2 = tri->v1.z;
			break;
		case 2:
			tmpx1 = tri->e2.x;
			tmpy1 = tri->e2.y;
			tmpz1 = tri->e2.z;
			tmpx2 = tri->v2.x;
			tmpy2 = tri->v2.y;
			tmpz2 = tri->v2.z;
			break;
		default:
			throw 1001;
	}
	const Matrix4 &triTransform = tri->transform;

	Vector3 axis20;
	axis20.x = (((triTransform.a * tmpx1) + (triTransform.b * tmpy1)) + (triTransform.c * tmpz1));
	axis20.y = (((triTransform.e * tmpx1) + (triTransform.f * tmpy1)) + (triTransform.g * tmpz1));
	axis20.z = (((triTransform.i * tmpx1) + (triTransform.j * tmpy1)) + (triTransform.k * tmpz1));
	double x2 = ((((triTransform.a * tmpx2) + (triTransform.b * tmpy2)) + (triTransform.c * tmpz2)) + triTransform.d);
	double y2 = ((((triTransform.e * tmpx2) + (triTransform.f * tmpy2)) + (triTransform.g * tmpz2)) + triTransform.h);
	double z2 = ((((triTransform.i * tmpx2) + (triTransform.j * tmpy2)) + (triTransform.k * tmpz2)) + triTransform.l);
	const Matrix4 &boxTransform = box->transform;
	Vector3 axis10;
	boxTransform.getAxis(boxAxisIdx, &axis10);
	Vector3 *v = &contact->normal;
	v->x = (axis10.y * axis20.z) - (axis10.z * axis20.y);
	v->y = (axis10.z * axis20.x) - (axis10.x * axis20.z);
	v->z = (axis10.x * axis20.y) - (axis10.y * axis20.x);
	double k = 1. / MyMath::sqrt((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
	v->x = v->x * k;
	v->y = v->y * k;
	v->z = v->z * k;
	if ((((v->x * toBox->x) + (v->y * toBox->y)) + (v->z * toBox->z)) < 0)
	{
		v->x = -v->x;
		v->y = -v->y;
		v->z = -v->z;
	}
	tmpx1 = box->hs.x;
	tmpy1 = box->hs.y;
	tmpz1 = box->hs.z;
	if (boxAxisIdx == 0)
	{
		tmpx1 = 0;
		//boxHalfLen = box->hs.x;
	}
	else
	{
		if ((((boxTransform.a * v->x) + (boxTransform.e * v->y)) + (boxTransform.i * v->z)) > 0)
		{
			tmpx1 = -(tmpx1);
			if ((box->excludedFaces & 0x02) != 0)
			{
				return false;
			}
		}
		else
		{
			if ((box->excludedFaces & 0x01) != 0)
			{
				return false;
			}
		}
	}
	if (boxAxisIdx == 1)
	{
		tmpy1 = 0;
		//boxHalfLen = box->hs.y;
	}
	else
	{
		if ((((boxTransform.b * v->x) + (boxTransform.f * v->y)) + (boxTransform.j * v->z)) > 0)
		{
			tmpy1 = -(tmpy1);
			if ((box->excludedFaces & 0x08) != 0)
			{
				return false;
			}
		}
		else
		{
			if ((box->excludedFaces & 0x04) != 0)
			{
				return false;
			}
		}
	}
	if (boxAxisIdx == 2)
	{
		tmpz1 = 0;
		//boxHalfLen = box->hs.z;
	}
	else
	{
		if ((((boxTransform.c * v->x) + (boxTransform.g * v->y)) + (boxTransform.k * v->z)) > 0)
		{
			tmpz1 = -(tmpz1);
			if ((box->excludedFaces & 0x20) != 0)
			{
				return false;
			}
		}
		else
		{
			if ((box->excludedFaces & 0x10) != 0)
			{
				return false;
			}
		}
	}
	double x1 = ((((boxTransform.a * tmpx1) + (boxTransform.b * tmpy1)) + (boxTransform.c * tmpz1)) + boxTransform.d);
	double y1 = ((((boxTransform.e * tmpx1) + (boxTransform.f * tmpy1)) + (boxTransform.g * tmpz1)) + boxTransform.h);
	double z1 = ((((boxTransform.i * tmpx1) + (boxTransform.j * tmpy1)) + (boxTransform.k * tmpz1)) + boxTransform.l);
	k = (((axis10.x * axis20.x) + (axis10.y * axis20.y)) + (axis10.z * axis20.z));
	double det = ((k * k) - 1);
	double vx = (x2 - x1);
	double vy = (y2 - y1);
	double vz = (z2 - z1);
	double c1 = (((axis10.x * vx) + (axis10.y * vy)) + (axis10.z * vz));
	double c2 = (((axis20.x * vx) + (axis20.y * vy)) + (axis20.z * vz));
	double t1 = (((c2 * k) - c1) / det);
	double t2 = ((c2 - (c1 * k)) / det);
	contact->pcount = 1;
	ContactPoint *cp = contact->points[0];
	cp->penetration = this->minOverlap;
	v = &cp->pos;
	v->x = (0.5 * (((x1 + (axis10.x * t1)) + x2) + (axis20.x * t2)));
	v->y = (0.5 * (((y1 + (axis10.y * t1)) + y2) + (axis20.y * t2)));
	v->z = (0.5 * (((z1 + (axis10.z * t1)) + z2) + (axis20.z * t2)));
	Vector3 *r = &cp->r1;
	r->x = (v->x - boxTransform.d);
	r->y = (v->y - boxTransform.h);
	r->z = (v->z - boxTransform.l);
	r = &cp->r2;
	r->x = (v->x - triTransform.d);
	r->y = (v->y - triTransform.h);
	r->z = (v->z - triTransform.l);
	return true;
}
