#include "boxboxcollider.h"
#include "alternativa/physics/contactpoint.h"
#include "alternativa/math/vector3.h"
#include "../collisionprimitive.h"
#include "alternativa/physics/collision/primitives/collisionbox.h"
#include "alternativa/physics/contact.h"
#include "alternativa/math/matrix4.h"
#include "mymath.h"
#include "_global.h"



BoxBoxCollider::BoxBoxCollider()
{
	_function_name("BoxBoxCollider::BoxBoxCollider");

	vectorToBox1.reset();

	pcount = 0;
	minOverlap = 0;
	this->bestAxisIndex = 0;
	this->epsilon = 0.001;
	for (int i = 0; i < 8; i++)
	{
		this->tmpPoints[i] = new ContactPoint();
		this->points1[i] = new Vector3();
		this->points2[i] = new Vector3();
		this->points1[i]->reset();
		this->points2[i]->reset();
	}
}

BoxBoxCollider::~BoxBoxCollider()
{
	for (int i = 0; i < 8; i++) {
		delete this->points1[i];
		delete this->points2[i];
		delete this->tmpPoints[i];
	}
}

bool BoxBoxCollider::getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact)
{
	_function_name("BoxBoxCollider::getContact");

	CollisionBox *box1;
	CollisionBox *box2;
	if ((!(this->haveCollision(prim1, prim2))))
	{
		return false;
	}

	if (prim1->body != nullptr)
	{
		box1 = (CollisionBox*)prim1;
		box2 = (CollisionBox*)prim2;
	}
	else
	{
		box1 = (CollisionBox*)prim2;
		box2 = (CollisionBox*)prim1;
	}


	if (this->bestAxisIndex < 6)
	{
		if ((!(this->findFaceContactPoints(box1, box2, &vectorToBox1, this->bestAxisIndex, contact))))
		{
			return false;
		}
	}
	else
	{
		this->bestAxisIndex = this->bestAxisIndex - 6;
		this->findEdgesIntersection(box1, box2, &vectorToBox1, int((double(this->bestAxisIndex) / 3)), (this->bestAxisIndex % 3), contact);
	}
	contact->body1 = box1->body;
	contact->body2 = box2->body;

	return true;
}

bool BoxBoxCollider::haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2)
{
	_function_name("BoxBoxCollider::haveCollision");

	CollisionBox *box1;
	CollisionBox *box2;

	this->minOverlap = 10000000000.;
	if (prim1->body != nullptr)
	{
		box1 = (CollisionBox*)prim1;
		box2 = (CollisionBox*)prim2;
	}
	else
	{
		box1 = (CollisionBox*)prim2;
		box2 = (CollisionBox*)prim1;
	}

	const Matrix4 &transform1 = box1->transform;
	const Matrix4 &transform2 = box2->transform;
	vectorToBox1.x = (transform1.d - transform2.d);
	vectorToBox1.y = (transform1.h - transform2.h);
	vectorToBox1.z = (transform1.l - transform2.l);
	Vector3 axis10;
	axis10.x = transform1.a;
	axis10.y = transform1.e;
	axis10.z = transform1.i;

	if ((!(this->testMainAxis(box1, box2, &axis10, 0, &vectorToBox1))))
	{
		return false;
	}


	Vector3 axis11;
	axis11.x = transform1.b;
	axis11.y = transform1.f;
	axis11.z = transform1.j;


	//error 6.5358979307623784391e-07 -0.99999999999978639309 0
	//box_tank
	//ok    -0.19747276672016175758 -0.98028690340377766521 -0.0064879441210758254099


	//box_2
	//-0.99999999999978639309 6.5358979307623784391e-07 0 4999.9979999999995925 -6.5358979307623784391e-07 -0.99999999999978639309 0 5000 0 0 1

	if ((!(this->testMainAxis(box1, box2, &axis11, 1, &vectorToBox1))))
	{
		return false;
	}
	Vector3 axis12;
	axis12.x = transform1.c;
	axis12.y = transform1.g;
	axis12.z = transform1.k;
	if ((!(this->testMainAxis(box1, box2, &axis12, 2, &vectorToBox1))))
	{
		return false;
	}
	Vector3 axis20;
	axis20.x = transform2.a;
	axis20.y = transform2.e;
	axis20.z = transform2.i;
	if ((!(this->testMainAxis(box1, box2, &axis20, 3, &vectorToBox1))))
	{
		return false;
	}
	Vector3 axis21;
	axis21.x = transform2.b;
	axis21.y = transform2.f;
	axis21.z = transform2.j;
	if ((!(this->testMainAxis(box1, box2, &axis21, 4, &vectorToBox1))))
	{
		return false;
	}
	Vector3 axis22;
	axis22.x = transform2.c;
	axis22.y = transform2.g;
	axis22.z = transform2.k;
	if ((!(this->testMainAxis(box1, box2, &axis22, 5, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis10, &axis20, 6, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis10, &axis21, 7, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis10, &axis22, 8, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis11, &axis20, 9, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis11, &axis21, 10, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis11, &axis22, 11, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis12, &axis20, 12, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis12, &axis21, 13, &vectorToBox1))))
	{
		return false;
	}
	if ((!(this->testDerivedAxis(box1, box2, &axis12, &axis22, 14, &vectorToBox1))))
	{
		return false;
	}

	return true;
}

bool BoxBoxCollider::findFaceContactPoints(const CollisionBox *_box1, const CollisionBox *_box2, Vector3 *vectorToBox1, int faceAxisIdx, Contact *contact)
{
	_function_name("BoxBoxCollider::findFaceContactPoints");

	CollisionBox const * box1 = _box1;
	CollisionBox const * box2 = _box2;


	bool swapNormal = false;
	if (faceAxisIdx > 2)
	{
		CollisionBox const * tmpBox = box1;
		box1 = box2;
		box2 = tmpBox;
		vectorToBox1->x = -(vectorToBox1->x);
		vectorToBox1->y = -(vectorToBox1->y);
		vectorToBox1->z = -(vectorToBox1->z);
		faceAxisIdx = faceAxisIdx - 3;
		swapNormal = true;
	}
	const Matrix4 &transform1 = box1->transform;
	const Matrix4 &transform2 = box2->transform;
	Vector3 *colAxis = &contact->normal;

	transform1.getAxis(faceAxisIdx, colAxis);
	bool negativeFace = ((((colAxis->x * vectorToBox1->x) + (colAxis->y * vectorToBox1->y)) + (colAxis->z * vectorToBox1->z)) > 0);
	uint32_t code = (1 << (faceAxisIdx << 1));
	if (negativeFace)
	{
		code = code << 1;
	}
	if ((code & box1->excludedFaces) != 0)
	{
		return false;
	}
	if ((!(negativeFace)))
	{
		colAxis->x = -(colAxis->x);
		colAxis->y = -(colAxis->y);
		colAxis->z = -(colAxis->z);
	}
	double incidentAxisDot = 0;
	int incidentAxisIdx = 0;
	double maxDot = 0.;
	int axisIdx = 0;
	Vector3 axis;
	while (axisIdx < 3)
	{

		transform2.getAxis(axisIdx, &axis);
		double dot = (((axis.x * colAxis->x) + (axis.y * colAxis->y)) + (axis.z * colAxis->z));
		double absDot = ((dot < 0) ? -(dot) : dot);
		if (absDot > maxDot)
		{
			maxDot = absDot;
			incidentAxisDot = dot;
			incidentAxisIdx = axisIdx;
		}
		axisIdx++;
	}

	transform2.getAxis(incidentAxisIdx, &axis);
	getFaceVertsByAxis(&box2->hs, incidentAxisIdx, incidentAxisDot < 0, this->points1);

	transform2.transformVectorsN((const Vector3**)this->points1, this->points2, 4);

	transform1.transformVectorsInverseN((const Vector3**)this->points2, this->points1, 4);
	int pnum = this->clip(&box1->hs, faceAxisIdx);

	this->pcount = 0;
	int i = 0;
	while (i < pnum)
	{
		double pen;
		Vector3 *v = this->points1[i];


		pen = this->getPointBoxPenetration(&box1->hs, v, faceAxisIdx, negativeFace);

		if ((pen) > -(this->epsilon))
		{
			ContactPoint *cp = this->tmpPoints[this->pcount++];
			Vector3 &cpPos = cp->pos;
			cpPos.x = ((((transform1.a * v->x) + (transform1.b * v->y)) + (transform1.c * v->z)) + transform1.d);
			cpPos.y = ((((transform1.e * v->x) + (transform1.f * v->y)) + (transform1.g * v->z)) + transform1.h);
			cpPos.z = ((((transform1.i * v->x) + (transform1.j * v->y)) + (transform1.k * v->z)) + transform1.l);
			Vector3 &r1 = cp->r1;
			r1.x = (cpPos.x - transform1.d);
			r1.y = (cpPos.y - transform1.h);
			r1.z = (cpPos.z - transform1.l);
			Vector3 &r2 = cp->r2;
			r2.x = (cpPos.x - transform2.d);
			r2.y = (cpPos.y - transform2.h);
			r2.z = (cpPos.z - transform2.l);
			cp->penetration = pen;
		}
		i++;
	}
	if (swapNormal)
	{
		colAxis->x = -(colAxis->x);
		colAxis->y = -(colAxis->y);
		colAxis->z = -(colAxis->z);
	}
	if (this->pcount > 4)
	{
		this->reducePoints();
	}
	i = 0;
	while (i < this->pcount)
	{
		ContactPoint *cp = contact->points[i];
		cp->copyFrom(this->tmpPoints[i]);
		i++;
	}
	contact->pcount = this->pcount;

	return true;
}

void BoxBoxCollider::reducePoints()
{
	_function_name("BoxBoxCollider::reducePoints");

	//var i:int;
	int minIdx = 0;
	//var cp1:ContactPoint;
	//var cp2:ContactPoint;
	//var p1:Vector3;
	//var p2:Vector3;
	//var ii:int;
	//var dx:Number;
	//var dy:Number;
	//var dz:Number;
	//var len:Number;
	while (this->pcount > 4)
	{
		double minLen = 10000000000.;
		//Vector3 *p1 = (this->tmpPoints[int((this->pcount - 1))] as ContactPoint).pos;
		Vector3 *p1 = &this->tmpPoints[this->pcount - 1]->pos;

		int i = 0;
		while (i < this->pcount)
		{
			//Vector3 *p2 = (this->tmpPoints[i] as ContactPoint).pos;
			Vector3 *p2 = &this->tmpPoints[i]->pos;
			double dx = (p2->x - p1->x);
			double dy = (p2->y - p1->y);
			double dz = (p2->z - p1->z);
			double len = (((dx * dx) + (dy * dy)) + (dz * dz));
			if (len < minLen)
			{
				minLen = len;
				minIdx = i;
			}
			p1 = p2;
			i++;
		}
		int ii = ((minIdx == 0) ? (this->pcount - 1) : (minIdx - 1));
		ContactPoint *cp1 = this->tmpPoints[ii];
		ContactPoint *cp2 = this->tmpPoints[minIdx];
		p1 = &cp1->pos;
		Vector3 &p2 = cp2->pos;
		p2.x = (0.5 * (p1->x + p2.x));
		p2.y = (0.5 * (p1->y + p2.y));
		p2.z = (0.5 * (p1->z + p2.z));
		cp2->penetration = (0.5 * (cp1->penetration + cp2->penetration));
		if (minIdx > 0)
		{
			i = minIdx;
			while (i < this->pcount)
			{
				this->tmpPoints[int((i - 1))] = this->tmpPoints[i];
				i++;
			}
			this->tmpPoints[int((this->pcount - 1))] = cp1;
		}
		this->pcount--;
	}
}

double BoxBoxCollider::getPointBoxPenetration(const Vector3 *hs, const Vector3 *p, int faceAxisIdx, bool reverse)
{
	_function_name("BoxBoxCollider::getPointBoxPenetration");

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

int BoxBoxCollider::clip(const Vector3 *hs, int faceAxisIdx)
{
	_function_name("BoxBoxCollider::clip");


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

	return 0;
}

void BoxBoxCollider::findEdgesIntersection(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *vectorToBox1, int axisIdx1, int axisIdx2, Contact *contact)
{
	_function_name("BoxBoxCollider::findEdgesIntersection");

	//double halfLen1 = 0;
	//double halfLen2 = 0;
	//var halfLen1:Number;
	//var halfLen2:Number;
	const Matrix4 &transform1 = box1->transform;
	const Matrix4 &transform2 = box2->transform;
	Vector3 axis10;
	Vector3 axis20;
	transform1.getAxis(axisIdx1, &axis10);
	transform2.getAxis(axisIdx2, &axis20);
	Vector3 &colAxis = contact->normal;
	colAxis.x = ((axis10.y * axis20.z) - (axis10.z * axis20.y));
	colAxis.y = ((axis10.z * axis20.x) - (axis10.x * axis20.z));
	colAxis.z = ((axis10.x * axis20.y) - (axis10.y * axis20.x));
	double k = (1. / MyMath::sqrt((((colAxis.x * colAxis.x) + (colAxis.y * colAxis.y)) + (colAxis.z * colAxis.z))));
	colAxis.x = (colAxis.x * k);
	colAxis.y = (colAxis.y * k);
	colAxis.z = (colAxis.z * k);
	if ((((colAxis.x * vectorToBox1->x) + (colAxis.y * vectorToBox1->y)) + (colAxis.z * vectorToBox1->z)) < 0)
	{
		colAxis.x = -(colAxis.x);
		colAxis.y = -(colAxis.y);
		colAxis.z = -(colAxis.z);
	}
	double tx = box1->hs.x;
	double ty = box1->hs.y;
	double tz = box1->hs.z;
	double x2 = box2->hs.x;
	double y2 = box2->hs.y;
	double z2 = box2->hs.z;
	if (axisIdx1 == 0)
	{
		tx = 0;
		//halfLen1 = box1->hs.x;
	}
	else
	{
		if ((((colAxis.x * transform1.a) + (colAxis.y * transform1.e)) + (colAxis.z * transform1.i)) > 0)
		{
			tx = -(tx);
		}
	}
	if (axisIdx2 == 0)
	{
		x2 = 0;
		//halfLen2 = box2->hs.x;
	}
	else
	{
		if ((((colAxis.x * transform2.a) + (colAxis.y * transform2.e)) + (colAxis.z * transform2.i)) < 0)
		{
			x2 = -(x2);
		}
	}
	if (axisIdx1 == 1)
	{
		ty = 0;
		//halfLen1 = box1->hs.y;
	}
	else
	{
		if ((((colAxis.x * transform1.b) + (colAxis.y * transform1.f)) + (colAxis.z * transform1.j)) > 0)
		{
			ty = -(ty);
		}
	}
	if (axisIdx2 == 1)
	{
		y2 = 0;
		//halfLen2 = box2->hs.y;
	}
	else
	{
		if ((((colAxis.x * transform2.b) + (colAxis.y * transform2.f)) + (colAxis.z * transform2.j)) < 0)
		{
			y2 = -(y2);
		}
	}
	if (axisIdx1 == 2)
	{
		tz = 0;
		//halfLen1 = box1->hs.z;
	}
	else
	{
		if ((((colAxis.x * transform1.c) + (colAxis.y * transform1.g)) + (colAxis.z * transform1.k)) > 0)
		{
			tz = -(tz);
		}
	}
	if (axisIdx2 == 2)
	{
		z2 = 0;
		//halfLen2 = box2->hs.z;
	}
	else
	{
		if ((((colAxis.x * transform2.c) + (colAxis.y * transform2.g)) + (colAxis.z * transform2.k)) < 0)
		{
			z2 = -(z2);
		}
	}
	double x1 = ((((transform1.a * tx) + (transform1.b * ty)) + (transform1.c * tz)) + transform1.d);
	double y1 = ((((transform1.e * tx) + (transform1.f * ty)) + (transform1.g * tz)) + transform1.h);
	double z1 = ((((transform1.i * tx) + (transform1.j * ty)) + (transform1.k * tz)) + transform1.l);
	tx = x2;
	ty = y2;
	tz = z2;
	x2 = ((((transform2.a * tx) + (transform2.b * ty)) + (transform2.c * tz)) + transform2.d);
	y2 = ((((transform2.e * tx) + (transform2.f * ty)) + (transform2.g * tz)) + transform2.h);
	z2 = ((((transform2.i * tx) + (transform2.j * ty)) + (transform2.k * tz)) + transform2.l);
	k = (((axis10.x * axis20.x) + (axis10.y * axis20.y)) + (axis10.z * axis20.z));
	double det = ((k * k) - 1);
	tx = (x2 - x1);
	ty = (y2 - y1);
	tz = (z2 - z1);
	double c1 = (((axis10.x * tx) + (axis10.y * ty)) + (axis10.z * tz));
	double c2 = (((axis20.x * tx) + (axis20.y * ty)) + (axis20.z * tz));
	double t1 = (((c2 * k) - c1) / det);
	double t2 = ((c2 - (c1 * k)) / det);
	contact->pcount = 1;
	ContactPoint *cp = contact->points[0];
	Vector3 &cpPos = cp->pos;
	cp->pos.x = (0.5 * (((x1 + (axis10.x * t1)) + x2) + (axis20.x * t2)));
	cp->pos.y = (0.5 * (((y1 + (axis10.y * t1)) + y2) + (axis20.y * t2)));
	cp->pos.z = (0.5 * (((z1 + (axis10.z * t1)) + z2) + (axis20.z * t2)));
	Vector3 *r = &cp->r1;
	r->x = (cpPos.x - transform1.d);
	r->y = (cpPos.y - transform1.h);
	r->z = (cpPos.z - transform1.l);
	r = &cp->r2;
	r->x = (cpPos.x - transform2.d);
	r->y = (cpPos.y - transform2.h);
	r->z = (cpPos.z - transform2.l);
	cp->penetration = this->minOverlap;
}

bool BoxBoxCollider::testMainAxis(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *axis, int axisIndex, const Vector3 *toBox1)
{
	_function_name("BoxBoxCollider::testMainAxis");

	double overlap = this->overlapOnAxis(box1, box2, axis, toBox1);
	if (overlap < -(this->epsilon))
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

bool BoxBoxCollider::testDerivedAxis(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *axis1, const Vector3 *axis2, int axisIndex, const Vector3 *toBox1)
{
	_function_name("BoxBoxCollider::testDerivedAxis");

	Vector3 axis;
	axis.x = ((axis1->y * axis2->z) - (axis1->z * axis2->y));
	axis.y = ((axis1->z * axis2->x) - (axis1->x * axis2->z));
	axis.z = ((axis1->x * axis2->y) - (axis1->y * axis2->x));
	double lenSqr = (((axis.x * axis.x) + (axis.y * axis.y)) + (axis.z * axis.z));
	if (lenSqr < 0.0001)
	{
		return true;
	}
	double k = (1. / MyMath::sqrt(lenSqr));
	axis.x = (axis.x * k);
	axis.y = (axis.y * k);
	axis.z = (axis.z * k);
	double overlap = this->overlapOnAxis(box1, box2, &axis, toBox1);
	if (overlap < -(this->epsilon))
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

double BoxBoxCollider::overlapOnAxis(const CollisionBox *box1, const CollisionBox *box2, const Vector3 *axis, const Vector3 *vectorToBox1)
{
	_function_name("BoxBoxCollider::overlapOnAxis");

	double projection, d;
	{
		const Matrix4 &m = box1->transform;
		d = ((((m.a * axis->x) + (m.e * axis->y)) + (m.i * axis->z)) * box1->hs.x);
		if (d < 0)
		{
			d = -(d);
		}
		projection = d;
		d = ((((m.b * axis->x) + (m.f * axis->y)) + (m.j * axis->z)) * box1->hs.y);
		if (d < 0)
		{
			d = -(d);
		}
		projection = (projection + d);
		d = ((((m.c * axis->x) + (m.g * axis->y)) + (m.k * axis->z)) * box1->hs.z);
		if (d < 0)
		{
			d = -(d);
		}
		projection = (projection + d);
	}
	{
		const Matrix4 &m = box2->transform;
		d = ((((m.a * axis->x) + (m.e * axis->y)) + (m.i * axis->z)) * box2->hs.x);
		if (d < 0)
		{
			d = -(d);
		}
		projection = (projection + d);
		d = ((((m.b * axis->x) + (m.f * axis->y)) + (m.j * axis->z)) * box2->hs.y);
		if (d < 0)
		{
			d = -(d);
		}
		projection = (projection + d);
		d = ((((m.c * axis->x) + (m.g * axis->y)) + (m.k * axis->z)) * box2->hs.z);
		if (d < 0)
		{
			d = -(d);
		}
		projection = (projection + d);
		d = (((vectorToBox1->x * axis->x) + (vectorToBox1->y * axis->y)) + (vectorToBox1->z * axis->z));
		if (d < 0)
		{
			d = -(d);
		}
	}
	return projection - d;
}

void BoxBoxCollider::destroy()
{
	_function_name("BoxBoxCollider::destroy");
	delete this;
}
