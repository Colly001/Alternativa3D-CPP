#include "quaternion.h"
#include "vector3.h"
#include "matrix3.h"
#include "mymath.h"
#include "_global.h"
#include "matrix4.h"
#include <QString>
#include "_global.h"


void Quaternion::getZAxis(Vector3 *v) const
{
	_function_name("Quaternion::getZAxis");

	double _loc2_ = 2 * this->x * this->x;
	double _loc3_ = 2 * this->y * this->y;
	double _loc4_ = 2 * this->y * this->z;
	double _loc5_ = 2 * this->z * this->x;
	double _loc6_ = 2 * this->w * this->x;
	double _loc7_ = 2 * this->w * this->y;
	v->x = _loc5_ + _loc7_;
	v->y = _loc4_ - _loc6_;
	v->z = 1. - _loc2_ - _loc3_;
}

void Quaternion::getYAxis(Vector3 *v) const
{
	_function_name("Quaternion::getYAxis");

	double _local_2 = ((2 * this->x) * this->x);
	double _local_3 = ((2 * this->z) * this->z);
	double _local_4 = ((2 * this->x) * this->y);
	double _local_5 = ((2 * this->y) * this->z);
	double _local_6 = ((2 * this->w) * this->x);
	double _local_7 = ((2 * this->w) * this->z);
	v->x = _local_4 - _local_7;
	v->y = (1. - _local_2) - _local_3;
	v->z = _local_5 + _local_6;
}


void Quaternion::toMatrix3(Matrix3 *m) const
{
	_function_name("Quaternion::toMatrix3");

	double _loc2_ = 2 * this->x * this->x;
	double _loc3_ = 2 * this->y * this->y;
	double _loc4_ = 2 * this->z * this->z;
	double _loc5_ = 2 * this->x * this->y;
	double _loc6_ = 2 * this->y * this->z;
	double _loc7_ = 2 * this->z * this->x;
	double _loc8_ = 2 * this->w * this->x;
	double _loc9_ = 2 * this->w * this->y;
	double _loc10_ = 2 * this->w * this->z;
	m->m00 = 1 - _loc3_ - _loc4_;
	m->m01 = _loc5_ - _loc10_;
	m->m02 = _loc7_ + _loc9_;
	m->m10 = _loc5_ + _loc10_;
	m->m11 = 1 - _loc2_ - _loc4_;
	m->m12 = _loc6_ - _loc8_;
	m->m20 = _loc7_ - _loc9_;
	m->m21 = _loc6_ + _loc8_;
	m->m22 = 1 - _loc2_ - _loc3_;
}




void Quaternion::addScaledVector(const Vector3 *v, double scale)
{
	_function_name("Quaternion::addScaledVector");

	double _loc3_ = v->x * scale;
	double _loc4_ = v->y * scale;
	double _loc5_ = v->z * scale;
	double _loc6_ = -this->x * _loc3_ - this->y * _loc4_ - this->z * _loc5_;
	double _loc7_ = _loc3_ * this->w + _loc4_ * this->z - _loc5_ * this->y;
	double _loc8_ = _loc4_ * this->w + _loc5_ * this->x - _loc3_ * this->z;
	double _loc9_ = _loc5_ * this->w + _loc3_ * this->y - _loc4_ * this->x;
	this->w = this->w + 0.5 * _loc6_;
	this->x = this->x + 0.5 * _loc7_;
	this->y = this->y + 0.5 * _loc8_;
	this->z = this->z + 0.5 * _loc9_;
	double _loc10_ = this->w * this->w + this->x * this->x + this->y * this->y + this->z * this->z;
	if (_loc10_ == 0)
	{
		this->w = 1;
	}
	else
	{
		_loc10_ = 1. / MyMath::sqrt(_loc10_);
		this->w = this->w * _loc10_;
		this->x = this->x * _loc10_;
		this->y = this->y * _loc10_;
		this->z = this->z * _loc10_;
	}
}


void Quaternion::copy(const Quaternion *q)
{
	_function_name("Quaternion::copy");

	this->w = q->w;
	this->x = q->x;
	this->y = q->y;
	this->z = q->z;
}


bool Quaternion::isFiniteQuaternion() const
{
	_function_name("Quaternion::isFiniteQuaternion");

	return MyMath::isFinite(this->w) && MyMath::isFinite(this->x) && MyMath::isFinite(this->y) && MyMath::isFinite(this->z);
}


void Quaternion::setFromEulerAnglesXYZ(double x, double y, double z)
{
	_function_name("Quaternion::setFromEulerAnglesXYZ");

	this->setFromAxisAngleComponents(1, 0, 0, x);
	Quaternion _q;
	_q.setFromAxisAngleComponents(0, 1, 0, y);
	this->append(&_q);
	this->normalize();
	_q.setFromAxisAngleComponents(0, 0, 1, z);
	this->append(&_q);
	this->normalize();
}


void Quaternion::setFromEulerAngles(const Vector3 *v)
{
	_function_name("Quaternion::setFromEulerAngles");

	this->setFromEulerAnglesXYZ(v->x, v->y, v->z);
}


void Quaternion::normalize()
{
	_function_name("Quaternion::normalize");

	double v = this->w * this->w + this->x * this->x + this->y * this->y + this->z * this->z;
	if (v == 0)
	{
		this->w = 1;
	}
	else
	{
		v = 1. / MyMath::sqrt(v);
		this->w = this->w * v;
		this->x = this->x * v;
		this->y = this->y * v;
		this->z = this->z * v;
	}
}


void Quaternion::toMatrix4(Matrix4 *m) const
{
	_function_name("Quaternion::toMatrix4");

	double _loc2_ = (2 * this->x) * this->x;
	double _loc3_ = (2 * this->y) * this->y;
	double _loc4_ = (2 * this->z) * this->z;
	double _loc5_ = (2 * this->x) * this->y;
	double _loc6_ = (2 * this->y) * this->z;
	double _loc7_ = (2 * this->z) * this->x;
	double _loc8_ = (2 * this->w) * this->x;
	double _loc9_ = (2 * this->w) * this->y;
	double _loc10_ = (2 * this->w) * this->z;
	m->m00 = (1 - _loc3_) - _loc4_;
	m->m01 = _loc5_ - _loc10_;
	m->m02 = _loc7_ + _loc9_;
	m->m10 = _loc5_ + _loc10_;
	m->m11 = (1 - _loc2_) - _loc4_;
	m->m12 = _loc6_ - _loc8_;
	m->m20 = _loc7_ - _loc9_;
	m->m21 = _loc6_ + _loc8_;
	m->m22 = (1 - _loc2_) - _loc3_;
}


void Quaternion::getEulerAngles(Vector3 *v) const
{
	_function_name("Quaternion::getEulerAngles");

	double _loc2_ = 2 * this->x * this->x;
	double _loc3_ = 2 * this->y * this->y;
	double _loc4_ = 2 * this->z * this->z;
	double _loc5_ = 2 * this->x * this->y;
	double _loc6_ = 2 * this->y * this->z;
	double _loc7_ = 2 * this->z * this->x;
	double _loc8_ = 2 * this->w * this->x;
	double _loc9_ = 2 * this->w * this->y;
	double _loc10_ = 2 * this->w * this->z;
	double _loc11_ = 1 - _loc3_ - _loc4_;
	double _loc12_ = _loc5_ - _loc10_;
	double _loc13_ = _loc5_ + _loc10_;
	double _loc14_ = 1 - _loc2_ - _loc4_;
	double _loc15_ = _loc7_ - _loc9_;
	double _loc16_ = _loc6_ + _loc8_;
	double _loc17_ = 1 - _loc2_ - _loc3_;
	if (-1 < _loc15_ && _loc15_ < 1)
	{
		v->x = MyMath::atan2(_loc16_, _loc17_);
		v->y = -MyMath::asin(_loc15_);
		v->z = MyMath::atan2(_loc13_, _loc11_);
	}
	else
	{
		v->x = 0.;
		double y = _loc15_ <= -1 ? MyMath::PI : -MyMath::PI;
		v->y = y * 0.5;
		v->z = MyMath::atan2(-_loc12_, _loc14_);
	}
}




void Quaternion::setFromAxisAngleComponents(double x, double y, double z, double angle)
{
	_function_name("Quaternion::setFromAxisAngleComponents");
	double a = 0.5 * angle;
	this->w = MyMath::cos(a);
	double k = MyMath::sin(a) / MyMath::sqrt((x * x) + (y * y) + (z * z));
	this->x = x * k;
	this->y = y * k;
	this->z = z * k;
}

void Quaternion::append(const Quaternion *q)
{
	_function_name("Quaternion::append");

	double _loc2_ = q->w * this->w - q->x * this->x - q->y * this->y - q->z * this->z;
	double _loc3_ = q->w * this->x + q->x * this->w + q->y * this->z - q->z * this->y;
	double _loc4_ = q->w * this->y + q->y * this->w + q->z * this->x - q->x * this->z;
	double _loc5_ = q->w * this->z + q->z * this->w + q->x * this->y - q->y * this->x;
	this->w = _loc2_;
	this->x = _loc3_;
	this->y = _loc4_;
	this->z = _loc5_;
}


void Quaternion::slerp(const Quaternion *a, const Quaternion *b, double t)
{
	_function_name("Quaternion::slerp");

	double flip;

	double cosine = (a->w * b->w) + (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
	if (cosine < 0.)
	{
		cosine = -cosine;
		flip = -1.;
	}
	else
	{
		flip = 1.;
	}

	if ((1. - cosine) < 0.001)
	{
		double k1 = 1. - t;
		double k2 = t * flip;
		this->w = (a->w * k1) + (b->w * k2);
		this->x = (a->x * k1) + (b->x * k2);
		this->y = (a->y * k1) + (b->y * k2);
		this->z = (a->z * k1) + (b->z * k2);
		this->normalize();
	}
	else
	{
		double theta = MyMath::acos(cosine);
		double sine = MyMath::sin(theta);
		double beta = MyMath::sin((1 - t) * theta) / sine;
		double alpha = MyMath::sin(t * theta) / sine * flip;
		this->w = (a->w * beta) + (b->w * alpha);
		this->x = (a->x * beta) + (b->x * alpha);
		this->y = (a->y * beta) + (b->y * alpha);
		this->z = (a->z * beta) + (b->z * alpha);
	}

	/*
	double _local_4 = 1;
	double _local_5 = ((((q->w * _arg_2->w) + (q->x * _arg_2->x)) + (q->y * _arg_2->y)) + (q->z * _arg_2->z));
	if (_local_5 < 0)
	{
		_local_5 = -(_local_5);
		_local_4 = -1;
	}
	if ((1 - _local_5) < 0.001)
	{
		double _local_6 = (1 - _arg_3);
		double _local_7 = (_arg_3 * _local_4);
		this->w = ((q->w * _local_6) + (_arg_2->w * _local_7));
		this->x = ((q->x * _local_6) + (_arg_2->x * _local_7));
		this->y = ((q->y * _local_6) + (_arg_2->y * _local_7));
		this->z = ((q->z * _local_6) + (_arg_2->z * _local_7));
		this->normalize();
	}
	else
	{
		double _local_8 = MyMath::acos(_local_5);
		double _local_9 = MyMath::sin(_local_8);
		double _local_10 = (MyMath::sin(((1 - _arg_3) * _local_8)) / _local_9);
		double _local_11 = ((MyMath::sin((_arg_3 * _local_8)) / _local_9) * _local_4);
		this->w = ((q->w * _local_10) + (_arg_2->w * _local_11));
		this->x = ((q->x * _local_10) + (_arg_2->x * _local_11));
		this->y = ((q->y * _local_10) + (_arg_2->y * _local_11));
		this->z = ((q->z * _local_10) + (_arg_2->z * _local_11));
	}
	*/
}


QString Quaternion::toString() const
{
	_function_name("Quaternion::toString");

	return QString::number(x, 'g', 20) + " " +
			QString::number(y, 'g', 20) + " " +
			QString::number(z, 'g', 20) + " " +
			QString::number(w, 'g', 20);
}




void Quaternion::setFromAxisAngle(const Vector3 *axis, double angle)
{
	_function_name("Quaternion::setFromAxisAngle");

	this->w = MyMath::cos(0.5 * angle);
	double k = MyMath::sin(0.5 * angle) / MyMath::sqrt((axis->x * axis->x) + (axis->y * axis->y) + (axis->z * axis->z));
	this->x = axis->x * k;
	this->y = axis->y * k;
	this->z = axis->z * k;
}
