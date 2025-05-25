#include "matrix4.h"
#include "vector3.h"
#include "mymath.h"
#include "matrix3.h"
#include <QString>
#include "_global.h"



//Matrix3x4

void Matrix4::toIdentity()
{
	_function_name("Matrix4::toIdentity");

	this->m00 = this->m11 = this->m22 = 1;
	this->m01 = this->m02 = this->m10 = this->m12 = this->m20 = this->m21 = this->m03 = this->m13 = this->m23 = 0;
}

void Matrix4::transformVector(const Vector3 *param1, Vector3 *param2) const
{
	_function_name("Matrix4::transformVector");

	param2->x = this->m00 * param1->x + this->m01 * param1->y + this->m02 * param1->z + this->m03;
	param2->y = this->m10 * param1->x + this->m11 * param1->y + this->m12 * param1->z + this->m13;
	param2->z = this->m20 * param1->x + this->m21 * param1->y + this->m22 * param1->z + this->m23;
}


void Matrix4::setPosition(const Vector3 *v)
{
	_function_name("Matrix4::setPosition");

	this->x_w = v->x;
	this->y_w = v->y;
	this->z_w = v->z;
}

void Matrix4::getPosition(Vector3 *v) const
{
	_function_name("Matrix4::getPosition");

	v->x = this->x_w;
	v->y = this->y_w;
	v->z = this->z_w;
}



void Matrix4::fromAxisAngle(const Vector3 *vector, double v)
{
	_function_name("Matrix4::fromAxisAngle");

	double _loc3_ = MyMath::cos(v);
	double _loc4_ = MyMath::sin(v);
	double _loc5_ = 1 - _loc3_;
	double _loc6_ = vector->x;
	double _loc7_ = vector->y;
	double _loc8_ = vector->z;
	this->m00 = _loc5_ * _loc6_ * _loc6_ + _loc3_;
	this->m01 = _loc5_ * _loc6_ * _loc7_ - _loc8_ * _loc4_;
	this->m02 = _loc5_ * _loc6_ * _loc8_ + _loc7_ * _loc4_;
	this->m10 = _loc5_ * _loc6_ * _loc7_ + _loc8_ * _loc4_;
	this->m11 = _loc5_ * _loc7_ * _loc7_ + _loc3_;
	this->m12 = _loc5_ * _loc7_ * _loc8_ - _loc6_ * _loc4_;
	this->m20 = _loc5_ * _loc6_ * _loc8_ - _loc7_ * _loc4_;
	this->m21 = _loc5_ * _loc7_ * _loc8_ + _loc6_ * _loc4_;
	this->m22 = _loc5_ * _loc8_ * _loc8_ + _loc3_;
}


void Matrix4::getAxis(int param1, Vector3 *param2) const
{
	_function_name("Matrix4::getAxis");

	switch(param1)
	{
	case 0:
		param2->x = this->x_x;
		param2->y = this->y_x;
		param2->z = this->z_x;
		return;
	case 1:
		param2->x = this->x_y;
		param2->y = this->y_y;
		param2->z = this->z_y;
		return;
	case 2:
		param2->x = this->x_z;
		param2->y = this->y_z;
		param2->z = this->z_z;
		return;
	case 3:
		param2->x = this->x_w;
		param2->y = this->y_w;
		param2->z = this->z_w;
		return;
	default:
		return;
	}
}

void Matrix4::transformVectorInverse(Vector3 *param1, Vector3 *param2) const
{
	_function_name("Matrix4::transformVectorInverse");

	double _loc3_ = param1->x - this->m03;
	double _loc4_ = param1->y - this->m13;
	double _loc5_ = param1->z - this->m23;
	param2->x = this->m00 * _loc3_ + this->m10 * _loc4_ + this->m20 * _loc5_;
	param2->y = this->m01 * _loc3_ + this->m11 * _loc4_ + this->m21 * _loc5_;
	param2->z = this->m02 * _loc3_ + this->m12 * _loc4_ + this->m22 * _loc5_;
}


void Matrix4::setMatrix(double x, double y, double z, double rotationX, double rotationY, double rotationZ)
{
	_function_name("Matrix4::setMatrix");

	double _loc7_ = MyMath::cos(rotationX);
	double _loc8_ = MyMath::sin(rotationX);
	double _loc9_ = MyMath::cos(rotationY);
	double _loc10_ = MyMath::sin(rotationY);
	double _loc11_ = MyMath::cos(rotationZ);
	double _loc12_ = MyMath::sin(rotationZ);

	double _loc13_ = _loc11_ * _loc10_;
	double _loc14_ = _loc12_ * _loc10_;
	this->m00 = _loc11_ * _loc9_;
	this->m01 = _loc13_ * _loc8_ - _loc12_ * _loc7_;
	this->m02 = _loc13_ * _loc7_ + _loc12_ * _loc8_;
	this->m03 = x;
	this->m10 = _loc12_ * _loc9_;
	this->m11 = _loc14_ * _loc8_ + _loc11_ * _loc7_;
	this->m12 = _loc14_ * _loc7_ - _loc11_ * _loc8_;
	this->m13 = y;
	this->m20 = -_loc10_;
	this->m21 = _loc9_ * _loc8_;
	this->m22 = _loc9_ * _loc7_;
	this->m23 = z;
}

void Matrix4::setMatrix(const Vector3 *position, const Vector3 *rotation)
{
	_function_name("Matrix4::setMatrix");

	double _loc7_ = MyMath::cos(rotation->x);
	double _loc8_ = MyMath::sin(rotation->x);
	double _loc9_ = MyMath::cos(rotation->y);
	double _loc10_ = MyMath::sin(rotation->y);
	double _loc11_ = MyMath::cos(rotation->z);
	double _loc12_ = MyMath::sin(rotation->z);

	double _loc13_ = _loc11_ * _loc10_;
	double _loc14_ = _loc12_ * _loc10_;
	this->m00 = _loc11_ * _loc9_;
	this->m01 = _loc13_ * _loc8_ - _loc12_ * _loc7_;
	this->m02 = _loc13_ * _loc7_ + _loc12_ * _loc8_;
	this->m03 = position->x;
	this->m10 = _loc12_ * _loc9_;
	this->m11 = _loc14_ * _loc8_ + _loc11_ * _loc7_;
	this->m12 = _loc14_ * _loc7_ - _loc11_ * _loc8_;
	this->m13 = position->y;
	this->m20 = -_loc10_;
	this->m21 = _loc9_ * _loc8_;
	this->m22 = _loc9_ * _loc7_;
	this->m23 = position->z;
}

void Matrix4::prepend(Matrix4 *param1)
{
	_function_name("Matrix4::prepend");

	double _local_2 = this->m00;
	double _local_3 = this->m01;
	double _local_4 = this->m02;
	double _local_5 = this->m03;
	double _local_6 = this->m10;
	double _local_7 = this->m11;
	double _local_8 = this->m12;
	double _local_9 = this->m13;
	double _local_10 = this->m20;
	double _local_11 = this->m21;
	double _local_12 = this->m22;
	double _local_13 = this->m23;
	this->m00 = (((_local_2 * param1->m00) + (_local_3 * param1->m10)) + (_local_4 * param1->m20));
	this->m01 = (((_local_2 * param1->m01) + (_local_3 * param1->m11)) + (_local_4 * param1->m21));
	this->m02 = (((_local_2 * param1->m02) + (_local_3 * param1->m12)) + (_local_4 * param1->m22));
	this->m03 = ((((_local_2 * param1->m03) + (_local_3 * param1->m13)) + (_local_4 * param1->m23)) + _local_5);
	this->m10 = (((_local_6 * param1->m00) + (_local_7 * param1->m10)) + (_local_8 * param1->m20));
	this->m11 = (((_local_6 * param1->m01) + (_local_7 * param1->m11)) + (_local_8 * param1->m21));
	this->m12 = (((_local_6 * param1->m02) + (_local_7 * param1->m12)) + (_local_8 * param1->m22));
	this->m13 = ((((_local_6 * param1->m03) + (_local_7 * param1->m13)) + (_local_8 * param1->m23)) + _local_9);
	this->m20 = (((_local_10 * param1->m00) + (_local_11 * param1->m10)) + (_local_12 * param1->m20));
	this->m21 = (((_local_10 * param1->m01) + (_local_11 * param1->m11)) + (_local_12 * param1->m21));
	this->m22 = (((_local_10 * param1->m02) + (_local_11 * param1->m12)) + (_local_12 * param1->m22));
	this->m23 = ((((_local_10 * param1->m03) + (_local_11 * param1->m13)) + (_local_12 * param1->m23)) + _local_13);
}

void Matrix4::append(const Matrix4 *param1)
{
	_function_name("Matrix4::append");

	double _loc2_ = this->m00;
	double _loc3_ = this->m01;
	double _loc4_ = this->m02;
	double _loc5_ = this->m03;
	double _loc6_ = this->m10;
	double _loc7_ = this->m11;
	double _loc8_ = this->m12;
	double _loc9_ = this->m13;
	double _loc10_ = this->m20;
	double _loc11_ = this->m21;
	double _loc12_ = this->m22;
	double _loc13_ = this->m23;
	this->m00 = param1->m00 * _loc2_ + param1->m01 * _loc6_ + param1->m02 * _loc10_;
	this->m01 = param1->m00 * _loc3_ + param1->m01 * _loc7_ + param1->m02 * _loc11_;
	this->m02 = param1->m00 * _loc4_ + param1->m01 * _loc8_ + param1->m02 * _loc12_;
	this->m03 = param1->m00 * _loc5_ + param1->m01 * _loc9_ + param1->m02 * _loc13_ + param1->m03;
	this->m10 = param1->m10 * _loc2_ + param1->m11 * _loc6_ + param1->m12 * _loc10_;
	this->m11 = param1->m10 * _loc3_ + param1->m11 * _loc7_ + param1->m12 * _loc11_;
	this->m12 = param1->m10 * _loc4_ + param1->m11 * _loc8_ + param1->m12 * _loc12_;
	this->m13 = param1->m10 * _loc5_ + param1->m11 * _loc9_ + param1->m12 * _loc13_ + param1->m13;
	this->m20 = param1->m20 * _loc2_ + param1->m21 * _loc6_ + param1->m22 * _loc10_;
	this->m21 = param1->m20 * _loc3_ + param1->m21 * _loc7_ + param1->m22 * _loc11_;
	this->m22 = param1->m20 * _loc4_ + param1->m21 * _loc8_ + param1->m22 * _loc12_;
	this->m23 = param1->m20 * _loc5_ + param1->m21 * _loc9_ + param1->m22 * _loc13_ + param1->m23;
}

void Matrix4::getEulerAngles(Vector3 *v) const
{
	_function_name("Matrix4::getEulerAngles");

	if (-1. < this->m20 && this->m20 < 1.)
	{
		v->x = MyMath::atan2(this->m21, this->m22);
		v->y = -MyMath::asin(this->m20);
		v->z = MyMath::atan2(this->m10, this->m00);
	}
	else
	{
		v->x = 0.;
		v->y = (this->m20 <= -1 ? MyMath::PI : -MyMath::PI) * 0.5;
		v->z = MyMath::atan2(-this->m01, this->m11);
	}
}

void Matrix4::transformVectorsN(const Vector3 **arrin, Vector3 **arrout, int len) const
{
	_function_name("Matrix4::transformVectorsN");

	int idx = 0;
	while (idx < len)
	{
		const Vector3 *vin = arrin[idx];
		Vector3 *vout = arrout[idx];
		vout->x = ((((this->a * vin->x) + (this->b * vin->y)) + (this->c * vin->z)) + this->d);
		vout->y = ((((this->e * vin->x) + (this->f * vin->y)) + (this->g * vin->z)) + this->h);
		vout->z = ((((this->i * vin->x) + (this->j * vin->y)) + (this->k * vin->z)) + this->l);
		idx++;
	}
}

void Matrix4::transformVectorsInverseN(const Vector3 **arrin, Vector3 **arrout, int len) const
{
	_function_name("Matrix4::transformVectorsInverseN");

	int idx = 0;
	while (idx < len)
	{
		const Vector3 *vin = arrin[idx];
		Vector3 *vout = arrout[idx];
		double xx = (vin->x - this->d);
		double yy = (vin->y - this->h);
		double zz = (vin->z - this->l);
		vout->x = (((this->a * xx) + (this->e * yy)) + (this->i * zz));
		vout->y = (((this->b * xx) + (this->f * yy)) + (this->j * zz));
		vout->z = (((this->c * xx) + (this->g * yy)) + (this->k * zz));
		idx++;
	}
}


void Matrix4::invert()
{
	_function_name("Matrix4::invert");

	double _loc1_ = this->m00;
	double _loc2_ = this->m01;
	double _loc3_ = this->m02;
	double _loc4_ = this->m03;
	double _loc5_ = this->m10;
	double _loc6_ = this->m11;
	double _loc7_ = this->m12;
	double _loc8_ = this->m13;
	double _loc9_ = this->m20;
	double _loc10_ = this->m21;
	double _loc11_ = this->m22;
	double _loc12_ = this->m23;
	double _loc13_ = -_loc3_ * _loc6_ * _loc9_ + _loc2_ * _loc7_ * _loc9_ + _loc3_ * _loc5_ * _loc10_ - _loc1_ * _loc7_ * _loc10_ - _loc2_ * _loc5_ * _loc11_ + _loc1_ * _loc6_ * _loc11_;
	this->m00 = (-_loc7_ * _loc10_ + _loc6_ * _loc11_) / _loc13_;
	this->m01 = (_loc3_ * _loc10_ - _loc2_ * _loc11_) / _loc13_;
	this->m02 = (-_loc3_ * _loc6_ + _loc2_ * _loc7_) / _loc13_;
	this->m03 = (_loc4_ * _loc7_ * _loc10_ - _loc3_ * _loc8_ * _loc10_ - _loc4_ * _loc6_ * _loc11_ + _loc2_ * _loc8_ * _loc11_ + _loc3_ * _loc6_ * _loc12_ - _loc2_ * _loc7_ * _loc12_) / _loc13_;
	this->m10 = (_loc7_ * _loc9_ - _loc5_ * _loc11_) / _loc13_;
	this->m11 = (-_loc3_ * _loc9_ + _loc1_ * _loc11_) / _loc13_;
	this->m12 = (_loc3_ * _loc5_ - _loc1_ * _loc7_) / _loc13_;
	this->m13 = (_loc3_ * _loc8_ * _loc9_ - _loc4_ * _loc7_ * _loc9_ + _loc4_ * _loc5_ * _loc11_ - _loc1_ * _loc8_ * _loc11_ - _loc3_ * _loc5_ * _loc12_ + _loc1_ * _loc7_ * _loc12_) / _loc13_;
	this->m20 = (-_loc6_ * _loc9_ + _loc5_ * _loc10_) / _loc13_;
	this->m21 = (_loc2_ * _loc9_ - _loc1_ * _loc10_) / _loc13_;
	this->m22 = (-_loc2_ * _loc5_ + _loc1_ * _loc6_) / _loc13_;
	this->m23 = (_loc4_ * _loc6_ * _loc9_ - _loc2_ * _loc8_ * _loc9_ - _loc4_ * _loc5_ * _loc10_ + _loc1_ * _loc8_ * _loc10_ + _loc2_ * _loc5_ * _loc12_ - _loc1_ * _loc6_ * _loc12_) / _loc13_;
}



void Matrix4::setFromMatrix3(const Matrix3 *param1, const Vector3 *param2)
{
	_function_name("Matrix4::setFromMatrix3");

	this->m00 = param1->m00;
	this->m01 = param1->m01;
	this->m02 = param1->m02;
	this->m03 = param2->x;
	this->m10 = param1->m10;
	this->m11 = param1->m11;
	this->m12 = param1->m12;
	this->m13 = param2->y;
	this->m20 = param1->m20;
	this->m21 = param1->m21;
	this->m22 = param1->m22;
	this->m23 = param2->z;
}


void Matrix4::copy(const Matrix4 *m)
{
	_function_name("Matrix4::copy");

	this->m00 = m->m00;
	this->m01 = m->m01;
	this->m02 = m->m02;
	this->m03 = m->m03;
	this->m10 = m->m10;
	this->m11 = m->m11;
	this->m12 = m->m12;
	this->m13 = m->m13;
	this->m20 = m->m20;
	this->m21 = m->m21;
	this->m22 = m->m22;
	this->m23 = m->m23;
}



void Matrix4::setRotationMatrix(double param1, double param2, double param3)
{
	_function_name("Matrix4::setRotationMatrix");

	double _loc4_ = MyMath::cos(param1);
	double _loc5_ = MyMath::sin(param1);
	double _loc6_ = MyMath::cos(param2);
	double _loc7_ = MyMath::sin(param2);
	double _loc8_ = MyMath::cos(param3);
	double _loc9_ = MyMath::sin(param3);
	double _loc10_ = _loc8_ * _loc7_;
	double _loc11_ = _loc9_ * _loc7_;
	this->m00 = _loc8_ * _loc6_;
	this->m01 = _loc10_ * _loc5_ - _loc9_ * _loc4_;
	this->m02 = _loc10_ * _loc4_ + _loc9_ * _loc5_;
	this->m10 = _loc9_ * _loc6_;
	this->m11 = _loc11_ * _loc5_ + _loc8_ * _loc4_;
	this->m12 = _loc11_ * _loc4_ - _loc8_ * _loc5_;
	this->m20 = -_loc7_;
	this->m21 = _loc6_ * _loc5_;
	this->m22 = _loc6_ * _loc4_;
}



QString Matrix4::toString() const
{
	_function_name("Matrix4::toString");

	return QString::number(m00, 'g', 20) + " " +
			QString::number(m01, 'g', 20) + " " +
			QString::number(m02, 'g', 20) + " " +
			QString::number(m03, 'g', 20) + " " +

			QString::number(m10, 'g', 20) + " " +
			QString::number(m11, 'g', 20) + " " +
			QString::number(m12, 'g', 20) + " " +
			QString::number(m13, 'g', 20) + " " +

			QString::number(m20, 'g', 20) + " " +
			QString::number(m21, 'g', 20) + " " +
			QString::number(m22, 'g', 20) + " " +
			QString::number(m23, 'g', 20);
}



void Matrix4::init(double v1,
				   double v2,
				   double v3,
				   double v4,
				   double v5,
				   double v6,
				   double v7,
				   double v8,
				   double v9,
				   double v10,
				   double v11,
				   double v12)
{
	_function_name("Matrix4::init");

	this->m00 = v1;
	this->m01 = v2;
	this->m02 = v3;
	this->m03 = v4;
	this->m10 = v5;
	this->m11 = v6;
	this->m12 = v7;
	this->m13 = v8;
	this->m20 = v9;
	this->m21 = v10;
	this->m22 = v11;
	this->m23 = v12;
}
