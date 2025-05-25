#include "boxrectcollider.h"
#include "../collisionprimitive.h"
#include "alternativa/physics/contact.h"
#include "alternativa/physics/collision/primitives/collisionbox.h"
#include "alternativa/physics/collision/primitives/collisionrect.h"
#include "alternativa/physics/contactpoint.h"
#include "mymath.h"


BoxRectCollider::BoxRectCollider()
{
	minOverlap = 0;
	vectorToBox.reset();
	bestAxisIndex = 0;
	epsilon = 0.001;
	for (int i = 0; i < 8; i++)
	{
		this->points1[i] = new Vector3();
		this->points2[i] = new Vector3();
		this->points1[i]->reset();
		this->points2[i]->reset();
	}
}

BoxRectCollider::~BoxRectCollider()
{
	for (int i = 0; i < 8; i++)
	{
		delete this->points1[i];
		delete this->points2[i];
	}
}

bool BoxRectCollider::getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact)
{
	CollisionRect *rect;
	if (!this->haveCollision(prim1, prim2))
	{
		return false;
	}


	CollisionBox *box;
	if (prim1->type != CollisionPrimitive::BOX)
	{
		box = (CollisionBox*)prim2;
		rect = (CollisionRect*)prim1;
	}
	else
	{
		box = (CollisionBox*)prim1;
		rect = (CollisionRect*)prim2;
	}

	if (this->bestAxisIndex < 4)
	{
		if ((!(this->findFaceContactPoints(box, rect, &this->vectorToBox, this->bestAxisIndex, contact))))
		{
			return false;
		}
	}
	else
	{
		this->bestAxisIndex = (this->bestAxisIndex - 4);
		if ((!(this->findEdgesIntersection(box, rect, &this->vectorToBox, int((this->bestAxisIndex / 2)), (this->bestAxisIndex % 2), contact))))
		{
			return false;
		}
	}
	contact->body1 = box->body;
	contact->body2 = rect->body;
	if (rect->transform.k > 0.99999)
	{
		contact->normal.x = 0;
		contact->normal.y = 0;
		contact->normal.z = 1;
	}

	return true;
}

bool BoxRectCollider::haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2)
{
	this->minOverlap = 10000000000;

	CollisionBox *box;
	CollisionRect *rect;

	if (prim1->type != CollisionPrimitive::BOX)
	{
		box = (CollisionBox*)prim2;
		rect = (CollisionRect*)prim1;
	}
	else
	{
		box = (CollisionBox*)prim1;
		rect = (CollisionRect*)prim2;
	}
	Matrix4 &boxTransform = box->transform;
	Matrix4 &rectTransform = rect->transform;
	this->vectorToBox.x = (boxTransform.d - rectTransform.d);
	this->vectorToBox.y = (boxTransform.h - rectTransform.h);
	this->vectorToBox.z = (boxTransform.l - rectTransform.l);
	Vector3 axis22;
	rectTransform.getAxis(2, &axis22);
	if ((!(this->testMainAxis(box, rect, &axis22, 0, &vectorToBox))))
	{
		return false;
	}
	Vector3 axis10;
	boxTransform.getAxis(0, &axis10);
	if ((!(this->testMainAxis(box, rect, &axis10, 1, &vectorToBox))))
	{
		return false;
	}
	Vector3 axis11;
	boxTransform.getAxis(1, &axis11);
	if ((!(this->testMainAxis(box, rect, &axis11, 2, &vectorToBox))))
	{
		return false;
	}
	Vector3 axis12;
	boxTransform.getAxis(2, &axis12);
	if ((!(this->testMainAxis(box, rect, &axis12, 3, &vectorToBox))))
	{
		return false;
	}
	Vector3 axis20;
	Vector3 axis21;

	rectTransform.getAxis(0, &axis20);
	rectTransform.getAxis(1, &axis21);
	if ((!(this->testDerivedAxis(box, rect, &axis10, &axis20, 4, &this->vectorToBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, rect, &axis10, &axis21, 5, &this->vectorToBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, rect, &axis11, &axis20, 6, &this->vectorToBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, rect, &axis11, &axis21, 7, &this->vectorToBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, rect, &axis12, &axis20, 8, &this->vectorToBox))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box, rect, &axis12, &axis21, 9, &this->vectorToBox))))
	{
		return false;
	}

	return true;
}

bool BoxRectCollider::findFaceContactPoints(CollisionBox *box, CollisionRect *rect, Vector3 *vectorToBox, int faceAxisIdx, Contact *contact)
{
	Matrix4 &boxTransform = box->transform;
	Matrix4 &rectTransform = rect->transform;
	Vector3 &colAxis = contact->normal;
	if (faceAxisIdx == 0)
	{
		double incidentAxisDot = 0;
		colAxis.x = rectTransform.c;
		colAxis.y = rectTransform.g;
		colAxis.z = rectTransform.k;
		int incidentAxisIdx = 0;
		double maxDot = 0;
		int axisIdx = 0;
		while (axisIdx < 3)
		{
			Vector3 axis;
			boxTransform.getAxis(axisIdx, &axis);
			double dot = (((axis.x * colAxis.x) + (axis.y * colAxis.y)) + (axis.z * colAxis.z));
			double absDot = ((dot < 0) ? -(dot) : dot);
			if (absDot > maxDot)
			{
				maxDot = absDot;
				incidentAxisIdx = axisIdx;
				incidentAxisDot = dot;
			}
			axisIdx++;
		}
		bool negativeFace = (incidentAxisDot > 0);
		int code = (1 << (incidentAxisIdx << 1));
		if (negativeFace)
		{
			code = (code << 1);
		}
		if ((code & box->excludedFaces) != 0)
		{
			return false;
		}
		Vector3 axis;
		boxTransform.getAxis(incidentAxisIdx, &axis);

		//void BoxCollider::getFaceVertsByAxis(Vector3 *hs, int axisIdx, bool negativeFace, Vector3 *result[8])
		getFaceVertsByAxis(&box->hs, incidentAxisIdx, negativeFace, this->points1);

		boxTransform.transformVectorsN((const Vector3**)this->points1, this->points2, 4);
		rectTransform.transformVectorsInverseN((const Vector3**)this->points2, this->points1, 4);
		int pnum = this->clipByRect(&rect->hs);
		contact->pcount = 0;
		int i = 0;
		while (i < pnum)
		{
			Vector3 *v = this->points1[i];
			if (v->z < this->epsilon)
			{
				ContactPoint *cp = contact->points[contact->pcount++];
				cp->penetration = -(v->z);
				Vector3 &cpPos = cp->pos;
				cpPos.x = ((((rectTransform.a * v->x) + (rectTransform.b * v->y)) + (rectTransform.c * v->z)) + rectTransform.d);
				cpPos.y = ((((rectTransform.e * v->x) + (rectTransform.f * v->y)) + (rectTransform.g * v->z)) + rectTransform.h);
				cpPos.z = ((((rectTransform.i * v->x) + (rectTransform.j * v->y)) + (rectTransform.k * v->z)) + rectTransform.l);
				v = &cp->r1;
				v->x = (cpPos.x - boxTransform.d);
				v->y = (cpPos.y - boxTransform.h);
				v->z = (cpPos.z - boxTransform.l);
				v = &cp->r2;
				v->x = (cpPos.x - rectTransform.d);
				v->y = (cpPos.y - rectTransform.h);
				v->z = (cpPos.z - rectTransform.l);
			}
			i++;
		}
	}
	else
	{
		faceAxisIdx--;
		boxTransform.getAxis(faceAxisIdx, &colAxis);
		bool negativeFace = ((((colAxis.x * vectorToBox->x) + (colAxis.y * vectorToBox->y)) + (colAxis.z * vectorToBox->z)) > 0);
		int code = (1 << (faceAxisIdx << 1));
		if (negativeFace)
		{
			code = (code << 1);
		}
		if ((code & box->excludedFaces) != 0)
		{
			return false;
		}
		if ((!(negativeFace)))
		{
			colAxis.x = -(colAxis.x);
			colAxis.y = -(colAxis.y);
			colAxis.z = -(colAxis.z);
		}
		if ((((rectTransform.c * colAxis.x) + (rectTransform.g * colAxis.y)) + (rectTransform.k * colAxis.z)) < 0)
		{
			return false;
		}
		getFaceVertsByAxis(&rect->hs, 2, false, this->points1);
		rectTransform.transformVectorsN((const Vector3**)this->points1, this->points2, 4);
		boxTransform.transformVectorsInverseN((const Vector3**)this->points2, this->points1, 4);
		int pnum = this->clipByBox(&box->hs, faceAxisIdx);
		contact->pcount = 0;
		int i = 0;
		while (i < pnum)
		{
			Vector3 *v = this->points1[i];
			double pen;
			if ((pen = this->getPointBoxPenetration(&box->hs, v, faceAxisIdx, negativeFace)) > -(this->epsilon))
			{
				ContactPoint *cp = contact->points[contact->pcount++];
				cp->penetration = pen;
				Vector3 &cpPos = cp->pos;
				cpPos.x = ((((boxTransform.a * v->x) + (boxTransform.b * v->y)) + (boxTransform.c * v->z)) + boxTransform.d);
				cpPos.y = ((((boxTransform.e * v->x) + (boxTransform.f * v->y)) + (boxTransform.g * v->z)) + boxTransform.h);
				cpPos.z = ((((boxTransform.i * v->x) + (boxTransform.j * v->y)) + (boxTransform.k * v->z)) + boxTransform.l);
				v = &cp->r1;
				v->x = (cpPos.x - boxTransform.d);
				v->y = (cpPos.y - boxTransform.h);
				v->z = (cpPos.z - boxTransform.l);
				v = &cp->r2;
				v->x = (cpPos.x - rectTransform.d);
				v->y = (cpPos.y - rectTransform.h);
				v->z = (cpPos.z - rectTransform.l);
			}
			i++;
		}
	}
	return true;
}

double BoxRectCollider::getPointBoxPenetration(Vector3 *hs, Vector3 *p, int faceAxisIdx, bool reverse)
{
	switch (faceAxisIdx)
	{
		case 0:
			if (reverse)
			{
				return (p->x + hs->x);
			}
			return (hs->x - p->x);
		case 1:
			if (reverse)
			{
				return (p->y + hs->y);
			}
			return (hs->y - p->y);
		case 2:
			if (reverse)
			{
				return (p->z + hs->z);
			}
			return (hs->z - p->z);
	}
	return 0;
}

int BoxRectCollider::clipByBox(Vector3 *hs, int faceAxisIdx)
{
	int pnum = 4;
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
	return (0);
}

int BoxRectCollider::clipByRect(Vector3 *hs)
{
	int pnum = 4;
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

bool BoxRectCollider::findEdgesIntersection(CollisionBox *box, CollisionRect *rect, Vector3 *vectorToBox, int axisIdx1, int axisIdx2, Contact *contact)
{
	//double halfLen1;
	//double halfLen2;
	Matrix4 &boxTransform = box->transform;
	Matrix4 &rectTransform = rect->transform;


	Vector3 axis10;
	Vector3 axis20;

	boxTransform.getAxis(axisIdx1, &axis10);
	rectTransform.getAxis(axisIdx2, &axis20);
	Vector3 &colAxis = contact->normal;
	colAxis.x = ((axis10.y * axis20.z) - (axis10.z * axis20.y));
	colAxis.y = ((axis10.z * axis20.x) - (axis10.x * axis20.z));
	colAxis.z = ((axis10.x * axis20.y) - (axis10.y * axis20.x));
	double k = (1. / MyMath::sqrt((((colAxis.x * colAxis.x) + (colAxis.y * colAxis.y)) + (colAxis.z * colAxis.z))));
	colAxis.x = (colAxis.x * k);
	colAxis.y = (colAxis.y * k);
	colAxis.z = (colAxis.z * k);
	if ((((colAxis.x * vectorToBox->x) + (colAxis.y * vectorToBox->y)) + (colAxis.z * vectorToBox->z)) < 0)
	{
		colAxis.x = -(colAxis.x);
		colAxis.y = -(colAxis.y);
		colAxis.z = -(colAxis.z);
	}
	double vx = box->hs.x;
	double vy = box->hs.y;
	double vz = box->hs.z;
	double x2 = rect->hs.x;
	double y2 = rect->hs.y;
	double z2 = rect->hs.z;
	if (axisIdx1 == 0)
	{
		vx = 0;
		//halfLen1 = box->hs.x;
	}
	else
	{
		if ((((boxTransform.a * colAxis.x) + (boxTransform.e * colAxis.y)) + (boxTransform.i * colAxis.z)) > 0)
		{
			vx = -vx;
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
	if (axisIdx2 == 0)
	{
		x2 = 0;
		//halfLen2 = rect->hs.x;
	}
	else
	{
		if ((((rectTransform.a * colAxis.x) + (rectTransform.e * colAxis.y)) + (rectTransform.i * colAxis.z)) < 0)
		{
			x2 = -x2;
		}
	}
	if (axisIdx1 == 1)
	{
		vy = 0;
		//halfLen1 = box->hs.y;
	}
	else
	{
		if ((((boxTransform.b * colAxis.x) + (boxTransform.f * colAxis.y)) + (boxTransform.j * colAxis.z)) > 0)
		{
			vy = -vy;
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
	if (axisIdx2 == 1)
	{
		y2 = 0;
		//halfLen2 = rect->hs.y;
	}
	else
	{
		if ((((rectTransform.b * colAxis.x) + (rectTransform.f * colAxis.y)) + (rectTransform.j * colAxis.z)) < 0)
		{
			y2 = -y2;
		}
	}
	if (axisIdx1 == 2)
	{
		vz = 0;
		//halfLen1 = box->hs.z;
	}
	else
	{
		if ((((boxTransform.c * colAxis.x) + (boxTransform.g * colAxis.y)) + (boxTransform.k * colAxis.z)) > 0)
		{
			vz = -vz;
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
	double x1 = ((((boxTransform.a * vx) + (boxTransform.b * vy)) + (boxTransform.c * vz)) + boxTransform.d);
	double y1 = ((((boxTransform.e * vx) + (boxTransform.f * vy)) + (boxTransform.g * vz)) + boxTransform.h);
	double z1 = ((((boxTransform.i * vx) + (boxTransform.j * vy)) + (boxTransform.k * vz)) + boxTransform.l);
	vx = x2;
	vy = y2;
	vz = z2;
	x2 = ((((rectTransform.a * vx) + (rectTransform.b * vy)) + (rectTransform.c * vz)) + rectTransform.d);
	y2 = ((((rectTransform.e * vx) + (rectTransform.f * vy)) + (rectTransform.g * vz)) + rectTransform.h);
	z2 = ((((rectTransform.i * vx) + (rectTransform.j * vy)) + (rectTransform.k * vz)) + rectTransform.l);
	k = (((axis10.x * axis20.x) + (axis10.y * axis20.y)) + (axis10.z * axis20.z));
	double det = ((k * k) - 1);
	vx = (x2 - x1);
	vy = (y2 - y1);
	vz = (z2 - z1);
	double c1 = (((axis10.x * vx) + (axis10.y * vy)) + (axis10.z * vz));
	double c2 = (((axis20.x * vx) + (axis20.y * vy)) + (axis20.z * vz));
	double t1 = (((c2 * k) - c1) / det);
	double t2 = ((c2 - (c1 * k)) / det);
	contact->pcount = 1;
	ContactPoint *cp = contact->points[0];
	cp->penetration = this->minOverlap;
	Vector3 &cpPos = cp->pos;
	cpPos.x = 0.5 * (((x1 + (axis10.x * t1)) + x2) + (axis20.x * t2));
	cpPos.y = 0.5 * (((y1 + (axis10.y * t1)) + y2) + (axis20.y * t2));
	cpPos.z = 0.5 * (((z1 + (axis10.z * t1)) + z2) + (axis20.z * t2));
	Vector3 *v = &cp->r1;
	v->x = cpPos.x - boxTransform.d;
	v->y = cpPos.y - boxTransform.h;
	v->z = cpPos.z - boxTransform.l;
	v = &cp->r2;
	v->x = cpPos.x - rectTransform.d;
	v->y = cpPos.y - rectTransform.h;
	v->z = cpPos.z - rectTransform.l;
	return true;
}



bool BoxRectCollider::testMainAxis(CollisionBox *box, CollisionRect *rect, Vector3 *axis, int axisIndex, Vector3 *vectorToBox)
{
	double overlap = this->overlapOnAxis(box, rect, axis, vectorToBox);
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

bool BoxRectCollider::testDerivedAxis(CollisionBox *box, CollisionRect *rect, Vector3 *axis1, Vector3 *axis2, int axisIndex, Vector3 *vectorToBox)
{
	Vector3 axis;
	axis.x = ((axis1->y * axis2->z) - (axis1->z * axis2->y));
	axis.y = ((axis1->z * axis2->x) - (axis1->x * axis2->z));
	axis.z = ((axis1->x * axis2->y) - (axis1->y * axis2->x));
	double lenSqr = (((axis.x * axis.x) + (axis.y * axis.y)) + (axis.z * axis.z));
	if (lenSqr < 0.0001)
	{
		return true;
	}
	double k = 1. / MyMath::sqrt(lenSqr);
	axis.x = (axis.x * k);
	axis.y = (axis.y * k);
	axis.z = (axis.z * k);
	double overlap = this->overlapOnAxis(box, rect, &axis, vectorToBox);
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

double BoxRectCollider::overlapOnAxis(CollisionBox *box, CollisionRect *rect, const Vector3 *axis, const Vector3 *vectorToBox)
{
	Matrix4 &m1 = box->transform;
	double d = ((((m1.a * axis->x) + (m1.e * axis->y)) + (m1.i * axis->z)) * box->hs.x);
	if (d < 0.)
	{
		d = -(d);
	}
	double projection = d;
	d = ((((m1.b * axis->x) + (m1.f * axis->y)) + (m1.j * axis->z)) * box->hs.y);
	if (d < 0.)
	{
		d = -(d);
	}
	projection = projection + d;
	d = ((((m1.c * axis->x) + (m1.g * axis->y)) + (m1.k * axis->z)) * box->hs.z);
	if (d < 0.)
	{
		d = -(d);
	}
	projection = projection + d;

	Matrix4 &m2 = rect->transform;
	d = ((((m2.a * axis->x) + (m2.e * axis->y)) + (m2.i * axis->z)) * rect->hs.x);
	if (d < 0.)
	{
		d = -(d);
	}
	projection = projection + d;
	d = ((((m2.b * axis->x) + (m2.f * axis->y)) + (m2.j * axis->z)) * rect->hs.y);
	if (d < 0.)
	{
		d = -(d);
	}
	projection = projection + d;
	d = ((vectorToBox->x * axis->x) + (vectorToBox->y * axis->y)) + (vectorToBox->z * axis->z);
	if (d < 0.)
	{
		d = -(d);
	}
	return projection - d;
}

void BoxRectCollider::destroy()
{
	delete this;
}
