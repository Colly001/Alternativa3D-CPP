#ifndef GEOMRECTANGLE_H
#define GEOMRECTANGLE_H

#include "point.h"

class GeomRectangle
{
public:

	GeomRectangle(double x = 0, double y = 0, double width = 0, double height = 0)
	{
		this->x = x;
		this->y = y;
		this->width = width;
		this->height = height;
	}

	//Point bottomRight;
	//Point size;
	//Point topLeft;

	//double bottom;
	double height;
	//double left;
	//double right;
	//double top;
	double width;
	double x;
	double y;

	void setTo(double xa, double ya, double widtha, double heighta)
	{
		this->x = xa;
		this->y = ya;
		this->width = widtha;
		this->height = heighta;
	}
};

#endif // GEOMRECTANGLE_H
