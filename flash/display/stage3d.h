#ifndef STAGE3D_H
#define STAGE3D_H

#include "flash/display3D/Context3DRenderMode.h"
#include "flash/display3D/Context3DProfile.h"

class Context3D;
class Stage3DPrivate;
class Stage;
class IDirect3DDevice9Ex;
class ImageNodes;

class Stage3D
{
	Stage3DPrivate *d;
	Stage *stage;
	double _x;
	double _y;
public:
	Stage3D(Stage *stage, ImageNodes *imageNodes);
	~Stage3D();

	void setHWND(void *hwnd);

	bool createContext3D();


	Context3D* context3D();

	void setVisible(bool);
	bool getVisible();

	void setPosition(double x, double y);

	double x();
	double y();

	void setSize(int width, int height);

	//Отправьте запрос на создание объекта Context3D для этого экземпляра Stage3D.
	void requestContext3D(Context3DRenderMode context3DRenderMode = Context3DRenderMode::AUTO, Context3DProfile profile = Context3DProfile::BASELINE);

	//void requestContext3DMatchingProfiles(profiles:Vector.<String>);
	//Отправьте запрос на создание объекта Context3D для этого экземпляра Stage3D.


	IDirect3DDevice9Ex *getIDirect3DDevice();
};

#endif // STAGE3D_H
