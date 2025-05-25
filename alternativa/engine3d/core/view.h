#ifndef VIEW_H
#define VIEW_H

#include "flash/geom/point.h"
#include "flash/geom/geomrectangle.h"

class Device;
class Point;
class Logo;
class Stage;


class View
{
	Logo *logo;
	bool presented;
	bool cleared;
	//Stage *_stage;

	double _x;
	double _y;

public:

	void setPosition(double x, double y);

	double x() { return _x; }
	double y() { return _y; }



	Device *device;
	Point *globalCoords;


	GeomRectangle rect;

	double _width;
	double _height;

	int zBufferPrecision;

	bool constrained;
	bool quality;
	bool configured;
	bool correction;
	bool antiAliasEnabled;

	View(double x, double y, double width, double height, bool constrainedMode = false);
	double getWidth();
	double getHeight();
	void setSize(double width, double height);
	void configure();

	void clearArea();

	void localToGlobal(const Point *in, Point *out);

	void hideLogo();
	void present();

	void addToStage(Stage *stage);

	void dispose();

	void clear();
private:
	void onRemoveFromStage();
};

#endif // VIEW_H
