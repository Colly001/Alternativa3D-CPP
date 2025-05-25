#ifndef MATRIX_H
#define MATRIX_H

class QString;
class Point;

class Matrix
{
public:
	Matrix(double a = 1, double b = 0, double c = 0, double d = 1, double tx = 0, double ty = 0)
	{
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
		this->tx = tx;
		this->ty = ty;
	}

	void identity();

	void translate(double dx, double dy);
	void rotate(double angle);
	void scale(double sx, double sy);

	void setTo(double aa, double ba, double ca, double da, double txa, double tya)
	{
		this->a = aa;
		this->b = ba;
		this->c = ca;
		this->d = da;
		this->tx = txa;
		this->ty = tya;
	}

	void transformPoint(const Point *in, Point *out) const;


	union {
		struct { double a, b, c, d; };
		struct { double _m11, _m12, _m21, _m22; };
	};
	union {
		struct { double tx, ty; };
		struct { double _dx, _dy; };
	};


	QString toString() const;
};

#endif // MATRIX_H
