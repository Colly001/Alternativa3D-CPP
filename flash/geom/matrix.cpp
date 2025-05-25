#include "matrix.h"
#include "mymath.h"
#include "point.h"
#include <QString>


void Matrix::identity()
{
	this->a = 1;
	this->b = 0;
	this->c = 0;
	this->d = 1;
	this->tx = 0;
	this->ty = 0;
}

void Matrix::translate(double dx, double dy)
{
	_dx += dx;
	_dy += dy;
	//_dx += (dx * _m11) + (dy * _m21);
	//_dy += (dy * _m22) + (dx * _m12);
}

//radian
void Matrix::rotate(double a)
{
	double sina = MyMath::sin(a);
	double cosa = MyMath::cos(a);
	double tm11 = cosa*_m11 + sina*_m21;
	double tm12 = cosa*_m12 + sina*_m22;
	double tm21 = -sina*_m11 + cosa*_m21;
	double tm22 = -sina*_m12 + cosa*_m22;
	_m11 = tm11;
	_m12 = tm12;
	_m21 = tm21;
	_m22 = tm22;
}

void Matrix::scale(double sx, double sy)
{
	_m11 *= sx;
	_m12 *= sx;
	_m21 *= sy;
	_m22 *= sy;
}

void Matrix::transformPoint(const Point *in, Point *out) const
{
	out->x = this->_dx + (in->x*_m11 + in->y*_m21);
	out->y = this->_dy + (in->y*_m22 + in->x*_m12);
}

QString Matrix::toString() const
{
	return QString::number(a, 'g', 20) + " " +
			QString::number(b, 'g', 20) + " " +
			QString::number(c, 'g', 20) + " " +
			QString::number(d, 'g', 20) + " " +
			QString::number(tx, 'g', 20) + " " +
			QString::number(ty, 'g', 20);
}
