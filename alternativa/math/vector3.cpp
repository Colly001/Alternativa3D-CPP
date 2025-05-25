#include "vector3.h"
#include "mymath.h"
#include "_global.h"
#include "matrix4.h"
#include "matrix3.h"
#include <QString>
#include "_global.h"

const Vector3 Vector3::ZERO(0,0,0);
const Vector3 Vector3::X_AXIS(1,0,0);
const Vector3 Vector3::Y_AXIS(0,1,0);
const Vector3 Vector3::Z_AXIS(0,0,1);
const Vector3 Vector3::DOWN(0,0,-1);


Vector3::Vector3(double x, double y, double z)
{
	_function_name("Vector3::Vector3");
	this->x = x;
	this->y = y;
	this->z = z;
}

void Vector3::reverse()
{
	_function_name("Vector3::reverse");
	this->x = -this->x;
	this->y = -this->y;
	this->z = -this->z;
}

double Vector3::dot(const Vector3 *v) const
{
	_function_name("Vector3::dot");
	return (this->x * v->x) + (this->y * v->y) + (this->z * v->z);
}

void Vector3::reset(double x, double y, double z)
{
	_function_name("Vector3::reset");
	this->x = x;
	this->y = y;
	this->z = z;
}

double Vector3::lengthSqr() const
{
	_function_name("Vector3::lengthSqr");
	return (this->x * this->x) + (this->y * this->y) + (this->z * this->z);
}

double Vector3::length() const
{
	_function_name("Vector3::length");
	return MyMath::sqrt((this->x * this->x) + (this->y * this->y) + (this->z * this->z));
}


void Vector3::setLength(double l)
{
	_function_name("Vector3::setLength");
	double v = (this->x * this->x) + (this->y * this->y) + (this->z * this->z);
	if (v == 0.) {
		this->x = l;
		this->y = 0.;
		this->z = 0.;
	} else {
		double v = l / MyMath::sqrt((this->x * this->x) + (this->y * this->y) + (this->z * this->z));
		this->x *= v;
		this->y *= v;
		this->z *= v;
	}
}


void Vector3::copy(const Vector3 *v)
{
	_function_name("Vector3::copy");
	this->x = v->x;
	this->y = v->y;
	this->z = v->z;
}

void Vector3::cross2(const Vector3 *v1, const Vector3 *v2)
{
	_function_name("Vector3::cross2");
	this->x = (v1->y * v2->z) - (v1->z * v2->y);
	this->y = (v1->z * v2->x) - (v1->x * v2->z);
	this->z = (v1->x * v2->y) - (v1->y * v2->x);
}

void Vector3::normalize()
{
	_function_name("Vector3::normalize");

	double v = (this->x * this->x) + (this->y * this->y) + (this->z * this->z);
	if (v == 0) {
		this->x = 0.9;
		this->y = 0.;
		this->z = 0.;
	} else {
		v = 1. / MyMath::sqrt(v);
		this->x *= v;
		this->y *= v;
		this->z *= v;
	}
}


bool Vector3::isFiniteVector() const
{
	_function_name("Vector3::isFiniteVector");
	return MyMath::isFinite(this->x) && MyMath::isFinite(this->y) && MyMath::isFinite(this->z);
}

void Vector3::scale(double value)
{
	_function_name("Vector3::scale");
	this->x *= value;
	this->y *= value;
	this->z *= value;
}

void Vector3::setLengthAlongDirection(const Vector3 *vector, double value)
{
	_function_name("Vector3::setLengthAlongDirection");
	double a = (this->x * vector->x) + (this->y * vector->y) + (this->z * vector->z);
	double b = value - a;
	this->x += b * vector->x;
	this->y += b * vector->y;
	this->z += b * vector->z;
}


void Vector3::add(const Vector3 *vector3)
{
	_function_name("Vector3::add");
	this->x += vector3->x;
	this->y += vector3->y;
	this->z += vector3->z;
}

void Vector3::remove(const Vector3 *vector3)
{
	_function_name("Vector3::remove");
	this->x -= vector3->x;
	this->y -= vector3->y;
	this->z -= vector3->z;
}

void Vector3::cross(const Vector3 *vector3)
{
	_function_name("Vector3::cross");
	double a = (this->y * vector3->z) - (this->z * vector3->y);
	double b = (this->z * vector3->x) - (this->x * vector3->z);
	double c = (this->x * vector3->y) - (this->y * vector3->x);
	this->x = a;
	this->y = b;
	this->z = c;
}




void Vector3::transform4(const Matrix4 *matrix4)
{
	_function_name("Vector3::transform4");
	double x = this->x;
	double y = this->y;
	double z = this->z;
	this->x = (matrix4->m00 * x) + (matrix4->m01 * y) + (matrix4->m02 * z) + matrix4->m03;
	this->y = (matrix4->m10 * x) + (matrix4->m11 * y) + (matrix4->m12 * z) + matrix4->m13;
	this->z = (matrix4->m20 * x) + (matrix4->m21 * y) + (matrix4->m22 * z) + matrix4->m23;
}


void Vector3::diff(const Vector3 *a, const Vector3 *b)
{
	_function_name("Vector3::diff");
	this->x = a->x - b->x;
	this->y = a->y - b->y;
	this->z = a->z - b->z;
}



void Vector3::subtract(const Vector3 *v)
{
	_function_name("Vector3::subtract");
	this->x = this->x - v->x;
	this->y = this->y - v->y;
	this->z = this->z - v->z;
}

double Vector3::distanceTo(const Vector3 *v) const
{
	double x = this->x - v->x;
	double y = this->y - v->y;
	double z = this->z - v->z;
	return MyMath::sqrt((x * x) + (y * y) + (z * z));
}

double Vector3::distanceToSquared(const Vector3 *v) const
{
	double x = this->x - v->x;
	double y = this->y - v->y;
	double z = this->z - v->z;
	return (x * x) + (y * y) + (z * z);
}

void Vector3::transformTransposed3(const Matrix3 *matrix)
{
	_function_name("Vector3::transformTransposed3");
	double x = this->x;
	double y = this->y;
	double z = this->z;
	this->x = (matrix->m00 * x) + (matrix->m10) * y + (matrix->m20 * z);
	this->y = (matrix->m01 * x) + (matrix->m11) * y + (matrix->m21 * z);
	this->z = (matrix->m02 * x) + (matrix->m12) * y + (matrix->m22 * z);
}


void Vector3::addScaled(double v, const Vector3 *vector3)
{
	_function_name("Vector3::addScaled");
	this->x = this->x + v * vector3->x;
	this->y = this->y + v * vector3->y;
	this->z = this->z + v * vector3->z;
}


void Vector3::deltaTransform4(const Matrix4 *m)
{
	_function_name("Vector3::deltaTransform4");
	double x = this->x;
	double y = this->y;
	double z = this->z;
	this->x = m->m00 * x + m->m01 * y + m->m02 * z;
	this->y = m->m10 * x + m->m11 * y + m->m12 * z;
	this->z = m->m20 * x + m->m21 * y + m->m22 * z;
}


void Vector3::sum(const Vector3 *a, const Vector3 *b)
{
	_function_name("Vector3::sum");
	this->x = a->x + b->x;
	this->y = a->y + b->y;
	this->z = a->z + b->z;
}


void Vector3::transform3(const Matrix3 *m)
{
	_function_name("Vector3::transform3");
	double x = this->x;
	double y = this->y;
	double z = this->z;
	this->x = (m->m00 * x) + (m->m01 * y) + (m->m02 * z);
	this->y = (m->m10 * x) + (m->m11 * y) + (m->m12 * z);
	this->z = (m->m20 * x) + (m->m21 * y) + (m->m22 * z);
}

void Vector3::transformBy3(const Matrix3 *m)
{
	_function_name("Vector3::transformBy3");
	double xx = this->x;
	double yy = this->y;
	double zz = this->z;
	this->x = (m->a * xx) + (m->b * yy) + (m->c * zz);
	this->y = (m->e * xx) + (m->f * yy) + (m->g * zz);
	this->z = (m->i * xx) + (m->j * yy) + (m->k * zz);
}

void Vector3::transformBy3Tr(const Matrix3 *m)
{
	double xx = this->x;
	double yy = this->y;
	double zz = this->z;
	this->x = (m->a * xx) + (m->e * yy) + (m->i * zz);
	this->y = (m->b * xx) + (m->f * yy) + (m->j * zz);
	this->z = (m->c * xx) + (m->g * yy) + (m->k * zz);
}

void Vector3::transformBy4(const Matrix4 *m)
{
	_function_name("Vector3::transformBy4");
	double xx = this->x;
	double yy = this->y;
	double zz = this->z;
	this->x = (m->a * xx) + (m->b * yy) + (m->c * zz) + m->d;
	this->y = (m->e * xx) + (m->f * yy) + (m->g * zz) + m->h;
	this->z = (m->i * xx) + (m->j * yy) + (m->k * zz) + m->l;
}

void Vector3::interpolate(double param1, const Vector3 *param2, const Vector3 *param3)
{
	this->x = param2->x + (param1 * (param3->x - param2->x));
	this->y = param2->y + (param1 * (param3->y - param2->y));
	this->z = param2->z + (param1 * (param3->z - param2->z));
}

double Vector3::cosAngle(const Vector3 *v) const
{
	double x = this->x;
	double y = this->y;
	double z = this->z;
	return ((x * v->x) + (y * v->y) + (z * v->z)) / (MyMath::sqrt((((x * x) + (y * y)) + (z * z))) *
													   MyMath::sqrt((((v->x * v->x) + (v->y * v->y)) + (v->z * v->z))));
}


QString Vector3::toString() const
{
	_function_name("Vector3::toString");
	return QString::number(x, 'g', 20) + " " +
			QString::number(y, 'g', 20) + " " +
			QString::number(z, 'g', 20);
}

double Vector3::distanceBetween(const Vector3 *a, const Vector3 *b)
{
	double x = a->x - b->x;
	double y = a->y - b->y;
	double z = a->z - b->z;
	return MyMath::sqrt((x * x) + (y * y) + (z * z));
}
