#ifndef POINT_H
#define POINT_H


class Point
{
public:
	Point(double x = 0., double y = 0.)
	{
		this->x = x;
		this->y = y;
	}

	double length() const;

	double x;
	double y;

	void copy(const Point *other)
	{
		this->x = other->x;
		this->y = other->y;
	}
};

#endif // POINT_H
