#include "matrix3.h"
#include "vector3.h"
#include "mymath.h"
#include "quaternion.h"
#include <QString>
#include "_global.h"

//3x3

const Matrix3 Matrix3::ZERO(0, 0, 0, 0, 0, 0, 0, 0, 0);
const Matrix3 Matrix3::IDENTITY(1, 0, 0, 0, 1, 0, 0, 0, 1);

Matrix3::Matrix3(double m00,
				 double m01,
				 double m02,
				 double m10,
				 double m11,
				 double m12,
				 double m20,
				 double m21,
				 double m22)
{
	_function_name("Matrix3::Matrix3");

	this->m00 = m00;
	this->m01 = m01;
	this->m02 = m02;
	this->m10 = m10;
	this->m11 = m11;
	this->m12 = m12;
	this->m20 = m20;
	this->m21 = m21;
	this->m22 = m22;
}

void Matrix3::setDefault()
{
	_function_name("Matrix3::setDefault");

	this->m00 = 1;
	this->m01 = 0;
	this->m02 = 0;
	this->m10 = 0;
	this->m11 = 1;
	this->m12 = 0;
	this->m20 = 0;
	this->m21 = 0;
	this->m22 = 1;
}

void Matrix3::copy(const Matrix3 *param1)
{
	_function_name("Matrix3::copy");

	this->m00 = param1->m00;
	this->m01 = param1->m01;
	this->m02 = param1->m02;
	this->m10 = param1->m10;
	this->m11 = param1->m11;
	this->m12 = param1->m12;
	this->m20 = param1->m20;
	this->m21 = param1->m21;
	this->m22 = param1->m22;
}


void Matrix3::append(const Matrix3 *param1)
{
	_function_name("Matrix3::append");

	double _loc2_ = this->m00;
	double _loc3_ = this->m01;
	double _loc4_ = this->m02;
	double _loc5_ = this->m10;
	double _loc6_ = this->m11;
	double _loc7_ = this->m12;
	double _loc8_ = this->m20;
	double _loc9_ = this->m21;
	double _loc10_ = this->m22;
	this->m00 = param1->m00 * _loc2_ + param1->m01 * _loc5_ + param1->m02 * _loc8_;
	this->m01 = param1->m00 * _loc3_ + param1->m01 * _loc6_ + param1->m02 * _loc9_;
	this->m02 = param1->m00 * _loc4_ + param1->m01 * _loc7_ + param1->m02 * _loc10_;
	this->m10 = param1->m10 * _loc2_ + param1->m11 * _loc5_ + param1->m12 * _loc8_;
	this->m11 = param1->m10 * _loc3_ + param1->m11 * _loc6_ + param1->m12 * _loc9_;
	this->m12 = param1->m10 * _loc4_ + param1->m11 * _loc7_ + param1->m12 * _loc10_;
	this->m20 = param1->m20 * _loc2_ + param1->m21 * _loc5_ + param1->m22 * _loc8_;
	this->m21 = param1->m20 * _loc3_ + param1->m21 * _loc6_ + param1->m22 * _loc9_;
	this->m22 = param1->m20 * _loc4_ + param1->m21 * _loc7_ + param1->m22 * _loc10_;
}


void Matrix3::prependTransposed(const Matrix3 *param1)
{
	_function_name("Matrix3::prependTransposed");

	double _loc2_ = this->m00;
	double _loc3_ = this->m01;
	double _loc4_ = this->m02;
	double _loc5_ = this->m10;
	double _loc6_ = this->m11;
	double _loc7_ = this->m12;
	double _loc8_ = this->m20;
	double _loc9_ = this->m21;
	double _loc10_ = this->m22;
	this->m00 = _loc2_ * param1->m00 + _loc3_ * param1->m01 + _loc4_ * param1->m02;
	this->m01 = _loc2_ * param1->m10 + _loc3_ * param1->m11 + _loc4_ * param1->m12;
	this->m02 = _loc2_ * param1->m20 + _loc3_ * param1->m21 + _loc4_ * param1->m22;
	this->m10 = _loc5_ * param1->m00 + _loc6_ * param1->m01 + _loc7_ * param1->m02;
	this->m11 = _loc5_ * param1->m10 + _loc6_ * param1->m11 + _loc7_ * param1->m12;
	this->m12 = _loc5_ * param1->m20 + _loc6_ * param1->m21 + _loc7_ * param1->m22;
	this->m20 = _loc8_ * param1->m00 + _loc9_ * param1->m01 + _loc10_ * param1->m02;
	this->m21 = _loc8_ * param1->m10 + _loc9_ * param1->m11 + _loc10_ * param1->m12;
	this->m22 = _loc8_ * param1->m20 + _loc9_ * param1->m21 + _loc10_ * param1->m22;
}



void Matrix3::transformVector(const Vector3 *param1, Vector3 *param2) const
{
	_function_name("Matrix3::transformVector");

	param2->x = this->m00 * param1->x + this->m01 * param1->y + this->m02 * param1->z;
	param2->y = this->m10 * param1->x + this->m11 * param1->y + this->m12 * param1->z;
	param2->z = this->m20 * param1->x + this->m21 * param1->y + this->m22 * param1->z;
}

void Matrix3::setRotationMatrix(double x, double y, double z)
{
	_function_name("Matrix3::setRotationMatrix");

	double _loc4_ = MyMath::cos(x);
	double _loc5_ = MyMath::sin(x);
	double _loc6_ = MyMath::cos(y);
	double _loc7_ = MyMath::sin(y);
	double _loc8_ = MyMath::cos(z);
	double _loc9_ = MyMath::sin(z);
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



void Matrix3::fromAxisAngle(const Vector3 *param1, double param2)
{
	_function_name("Matrix3::fromAxisAngle");

	double _loc3_ = MyMath::cos(param2);
	double _loc4_ = MyMath::sin(param2);
	double _loc5_ = 1 - _loc3_;
	double _loc6_ = param1->x;
	double _loc7_ = param1->y;
	double _loc8_ = param1->z;
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



void Matrix3::rotationMatrixToQuaternion(Quaternion *_arg_1) const
{
	_function_name("Matrix3::rotationMatrixToQuaternion");

	double _local_2 = ((this->m00 + this->m11) + this->m22);
	if (_local_2 > 0)
	{
		double _local_3 = (MyMath::sqrt((_local_2 + 1)) * 2);
		_arg_1->w = (0.25 * _local_3);
		_arg_1->x = ((this->m21 - this->m12) / _local_3);
		_arg_1->y = ((this->m02 - this->m20) / _local_3);
		_arg_1->z = ((this->m10 - this->m01) / _local_3);
	}
	else
	{
		if (((this->m00 > this->m11) && (this->m00 > this->m22)))
		{
			double _local_3 = (MyMath::sqrt((((1 + this->m00) - this->m11) - this->m22)) * 2);
			_arg_1->w = ((this->m21 - this->m12) / _local_3);
			_arg_1->x = (0.25 * _local_3);
			_arg_1->y = ((this->m01 + this->m10) / _local_3);
			_arg_1->z = ((this->m02 + this->m20) / _local_3);
		}
		else
		{
			if (this->m11 > this->m22)
			{
				double _local_3 = (MyMath::sqrt((((1 + this->m11) - this->m00) - this->m22)) * 2);
				_arg_1->w = ((this->m02 - this->m20) / _local_3);
				_arg_1->x = ((this->m01 + this->m10) / _local_3);
				_arg_1->y = (0.25 * _local_3);
				_arg_1->z = ((this->m12 + this->m21) / _local_3);
			}
			else
			{
				double _local_3 = (MyMath::sqrt((((1 + this->m22) - this->m00) - this->m11)) * 2);
				_arg_1->w = ((this->m10 - this->m01) / _local_3);
				_arg_1->x = ((this->m02 + this->m20) / _local_3);
				_arg_1->y = ((this->m12 + this->m21) / _local_3);
				_arg_1->z = (0.25 * _local_3);
			}
		}
	}
}


void Matrix3::setDirectionVector(const Vector3 *_arg_1)
{
	_function_name("Matrix3::setDirectionVector");

	Vector3 xAxis, yAxis, zAxis;
	yAxis.copy(_arg_1);
	yAxis.normalize();
	if (yAxis.dot(&Vector3::X_AXIS) < 0.9)
	{
		zAxis.cross2(&Vector3::X_AXIS, &yAxis);
	}
	else
	{
		zAxis.cross2(&yAxis, &Vector3::Y_AXIS);
	}
	zAxis.normalize();
	xAxis.cross2(&yAxis, &zAxis);
	xAxis.normalize();
	this->setAxis(&xAxis, &yAxis, &zAxis);
}

void Matrix3::setAxis(const Vector3 *_arg_1, const Vector3 *_arg_2, const Vector3 *_arg_3)
{
	_function_name("Matrix3::setAxis");

	this->m00 = _arg_1->x;
	this->m01 = _arg_2->x;
	this->m02 = _arg_3->x;
	this->m10 = _arg_1->y;
	this->m11 = _arg_2->y;
	this->m12 = _arg_3->y;
	this->m20 = _arg_1->z;
	this->m21 = _arg_2->z;
	this->m22 = _arg_3->z;
}




QString Matrix3::toString() const
{
	_function_name("Matrix3::toString");

	return QString::number(m00, 'g', 20) + " " +
			QString::number(m01, 'g', 20) + " " +
			QString::number(m02, 'g', 20) + " " +

			QString::number(m10, 'g', 20) + " " +
			QString::number(m11, 'g', 20) + " " +
			QString::number(m12, 'g', 20) + " " +

			QString::number(m20, 'g', 20) + " " +
			QString::number(m21, 'g', 20) + " " +
			QString::number(m22, 'g', 20);
}


void Matrix3::transformVectorInverse(const Vector3 *vin, Vector3 *vout) const
{
	vout->x = (this->a * vin->x) + (this->e * vin->y) + (this->i * vin->z);
	vout->y = (this->b * vin->x) + (this->f * vin->y) + (this->j * vin->z);
	vout->z = (this->c * vin->x) + (this->g * vin->y) + (this->k * vin->z);
}


void Matrix3::getEulerAngles(Vector3 *angles) const
{
	if ((-1. < this->i) && (this->i < 1.))
	{
		angles->x = MyMath::atan2(this->j, this->k);
		angles->y = -MyMath::asin(this->i);
		angles->z = MyMath::atan2(this->e, this->a);
	}
	else
	{
		angles->x = 0.;
		angles->y = (this->i <= -1) ? MyMath::PI : -MyMath::PI;
		angles->y = angles->y * 0.5;
		angles->z = MyMath::atan2(-this->b, this->f);
	}
}
