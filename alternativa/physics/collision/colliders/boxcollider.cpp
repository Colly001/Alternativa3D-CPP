#include "boxcollider.h"
#include "alternativa/math/vector3.h"
#include "_global.h"


BoxCollider::BoxCollider()
{
	_function_name("BoxCollider::BoxCollider");
}

BoxCollider::~BoxCollider()
{

}

bool BoxCollider::getContact(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2, Contact *contact)
{
	_function_name("BoxCollider::getContact");
	(void)prim1;
	(void)prim2;
	(void)contact;
	return false;
}

bool BoxCollider::haveCollision(const CollisionPrimitive *prim1, const CollisionPrimitive *prim2)
{
	_function_name("BoxCollider::haveCollision");
	(void)prim1;
	(void)prim2;
	return false;
}

void BoxCollider::getFaceVertsByAxis(const Vector3 *hs, int axisIdx, bool negativeFace, Vector3 *result[8])
{
	_function_name("BoxCollider::getFaceVertsByAxis");

	Vector3 *v;
	switch (axisIdx)
	{
		case 0:
			if (negativeFace)
			{
				v = result[0];
				v->x = -(hs->x);
				v->y = hs->y;
				v->z = -(hs->z);
				v = result[1];
				v->x = -(hs->x);
				v->y = -(hs->y);
				v->z = -(hs->z);
				v = result[2];
				v->x = -(hs->x);
				v->y = -(hs->y);
				v->z = hs->z;
				v = result[3];
				v->x = -(hs->x);
				v->y = hs->y;
				v->z = hs->z;
			}
			else
			{
				v = result[0];
				v->x = hs->x;
				v->y = -(hs->y);
				v->z = -(hs->z);
				v = result[1];
				v->x = hs->x;
				v->y = hs->y;
				v->z = -(hs->z);
				v = result[2];
				v->x = hs->x;
				v->y = hs->y;
				v->z = hs->z;
				v = result[3];
				v->x = hs->x;
				v->y = -(hs->y);
				v->z = hs->z;
			}
			break;
		case 1:
			if (negativeFace)
			{
				v = result[0];
				v->x = -(hs->x);
				v->y = -(hs->y);
				v->z = -(hs->z);
				v = result[1];
				v->x = hs->x;
				v->y = -(hs->y);
				v->z = -(hs->z);
				v = result[2];
				v->x = hs->x;
				v->y = -(hs->y);
				v->z = hs->z;
				v = result[3];
				v->x = -(hs->x);
				v->y = -(hs->y);
				v->z = hs->z;
			}
			else
			{
				v = result[0];
				v->x = hs->x;
				v->y = hs->y;
				v->z = -(hs->z);
				v = result[1];
				v->x = -(hs->x);
				v->y = hs->y;
				v->z = -(hs->z);
				v = result[2];
				v->x = -(hs->x);
				v->y = hs->y;
				v->z = hs->z;
				v = result[3];
				v->x = hs->x;
				v->y = hs->y;
				v->z = hs->z;
			}
			break;
		case 2:
			if (negativeFace)
			{
				v = result[0];
				v->x = -(hs->x);
				v->y = hs->y;
				v->z = -(hs->z);
				v = result[1];
				v->x = hs->x;
				v->y = hs->y;
				v->z = -(hs->z);
				v = result[2];
				v->x = hs->x;
				v->y = -(hs->y);
				v->z = -(hs->z);
				v = result[3];
				v->x = -(hs->x);
				v->y = -(hs->y);
				v->z = -(hs->z);
			}
			else
			{
				v = result[0];
				v->x = -(hs->x);
				v->y = -(hs->y);
				v->z = hs->z;
				v = result[1];
				v->x = hs->x;
				v->y = -(hs->y);
				v->z = hs->z;
				v = result[2];
				v->x = hs->x;
				v->y = hs->y;
				v->z = hs->z;
				v = result[3];
				v->x = -(hs->x);
				v->y = hs->y;
				v->z = hs->z;
			}
			break;
	}
}

int BoxCollider::clipLowX(double x, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon)
{
	_function_name("BoxCollider::clipLowX");

	double x1 = (x - epsilon);
	int num = 0;
	Vector3 *p1 = points[int((pnum - 1))];
	int i = 0;
	while (i < pnum)
	{
		Vector3 *p2 = points[i];
		if (p1->x > x1)
		{
			Vector3 *v = result[num];
			num++;
			v->x = p1->x;
			v->y = p1->y;
			v->z = p1->z;
			if (p2->x < x1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((x - p1->x) / dx);
				v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		else
		{
			if (p2->x > x1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((x - p1->x) / dx);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		p1 = p2;
		i++;
	}
	return (num);
}

int BoxCollider::clipHighX(double x, int pnum, Vector3 *points[8],  Vector3 *result[8], double epsilon)
{
	_function_name("BoxCollider::clipHighX");

	double x1 = (x + epsilon);
	int num = 0;
	Vector3 *p1 = points[int((pnum - 1))];
	int i = 0;
	while (i < pnum)
	{
		Vector3 *p2 = points[i];
		if (p1->x < x1)
		{
			Vector3 *v = result[num];
			num++;
			v->x = p1->x;
			v->y = p1->y;
			v->z = p1->z;
			if (p2->x > x1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((x - p1->x) / dx);
				v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		else
		{
			if (p2->x < x1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((x - p1->x) / dx);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		p1 = p2;
		i++;
	}
	return (num);
}

int BoxCollider::clipLowY(double y, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon)
{
	_function_name("BoxCollider::clipLowY");

	double y1 = (y - epsilon);
	int num = 0;
	Vector3 *p1 = points[int((pnum - 1))];
	int i = 0;
	while (i < pnum)
	{
		Vector3 *p2 = points[i];
		if (p1->y > y1)
		{
			Vector3 *v = result[num];
			num++;
			v->x = p1->x;
			v->y = p1->y;
			v->z = p1->z;
			if (p2->y < y1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((y - p1->y) / dy);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		else
		{
			if (p2->y > y1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((y - p1->y) / dy);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		p1 = p2;
		i++;
	}
	return (num);
}

int BoxCollider::clipHighY(double y, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon)
{
	_function_name("BoxCollider::clipHighY");

	double y1 = (y + epsilon);
	int num = 0;
	Vector3 *p1 = points[int((pnum - 1))];
	int i = 0;
	while (i < pnum)
	{
		Vector3 *p2 = points[i];
		if (p1->y < y1)
		{
			Vector3 *v = result[num];
			num++;
			v->x = p1->x;
			v->y = p1->y;
			v->z = p1->z;
			if (p2->y > y1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((y - p1->y) / dy);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		else
		{
			if (p2->y < y1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((y - p1->y) / dy);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		p1 = p2;
		i++;
	}
	return (num);
}

int BoxCollider::clipLowZ(double z, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon)
{
	_function_name("BoxCollider::clipLowZ");

	double z1 = (z - epsilon);
	int num = 0;
	Vector3 *p1 = points[int((pnum - 1))];
	int i = 0;
	while (i < pnum)
	{
		Vector3 *p2 = points[i];
		if (p1->z > z1)
		{
			Vector3 *v = result[num];
			num++;
			v->x = p1->x;
			v->y = p1->y;
			v->z = p1->z;
			if (p2->z < z1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((z - p1->z) / dz);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		else
		{
			if (p2->z > z1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((z - p1->z) / dz);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		p1 = p2;
		i++;
	}
	return (num);
}

int BoxCollider::clipHighZ(double z, int pnum, Vector3 *points[8], Vector3 *result[8], double epsilon)
{
	_function_name("BoxCollider::clipHighZ");

	double z1 = (z + epsilon);
	int num = 0;
	Vector3 *p1 = points[int((pnum - 1))];
	int i = 0;
	while (i < pnum)
	{
		Vector3 *p2 = points[i];
		if (p1->z < z1)
		{
			Vector3 *v = result[num];
			num++;
			v->x = p1->x;
			v->y = p1->y;
			v->z = p1->z;
			if (p2->z > z1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((z - p1->z) / dz);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		else
		{
			if (p2->z < z1)
			{
				double dx = (p2->x - p1->x);
				double dy = (p2->y - p1->y);
				double dz = (p2->z - p1->z);
				double t = ((z - p1->z) / dz);
				Vector3 *v = result[num];
				num++;
				v->x = (p1->x + (t * dx));
				v->y = (p1->y + (t * dy));
				v->z = (p1->z + (t * dz));
			}
		}
		p1 = p2;
		i++;
	}
	return (num);
}

void BoxCollider::destroy()
{
	_function_name("BoxCollider::destroy");
	delete this;
}
