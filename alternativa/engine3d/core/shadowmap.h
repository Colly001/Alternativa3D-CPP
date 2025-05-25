#ifndef SHADOWMAP_H
#define SHADOWMAP_H


#include <QVector>
#include <memory>
#include "alternativa/physics/collision/types/boundbox.h"

class Camera3D;
class Object3D;
class RenderTargetTextureResource;
class BitmapTextureResource;
class DirectionalLight;
class Vertex;
class VertexBufferResource;
class IndexBufferResource;
class GeomRectangle;


class ShadowMap
{
public:
	ShadowMap(int param1, double param2, double param3, double param4 = 0, double additionalSpace = 0);

	static const int numSamples = 6;

	RenderTargetTextureResource *map;
	std::shared_ptr<BitmapTextureResource> noise;


	//private var programs:Array = new Array();
	VertexBufferResource *spriteVertexBuffer;
	IndexBufferResource *spriteIndexBuffer;

	//alternativa3d var params:Vector.<Number> = Vector.<Number>([(-255 * bigValue), -(bigValue), bigValue, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]);

	double coords[36];



	double fragment[4];
	double alphaTestConst[4];

	GeomRectangle* scissor;
	//alternativa3d var map:RenderTargetTextureResource;
	//alternativa3d var noise:BitmapTextureResource;
	int noiseSize = 64;
	double noiseAngle = 1.0471975511966;
	double noiseRadius = 1.3;
	double noiseRandom = 0.3;
	int mapSize;
	double nearDistance;
	double farDistance;
	double bias = 0;
	double biasMultiplier = 30;
	double additionalSpace = 0;
	double alphaThreshold = 0.1;
	std::shared_ptr<DirectionalLight> defaultLight;
	std::shared_ptr<Vertex> boundVertexList;
	DirectionalLight *light;
	double dirZ;
	double planeX;
	double planeY;
	double planeSize;
	double pixel;

	BoundBox bound;


	double transform[16];
	double params[20];

	void calculateBounds(Camera3D *param1);
	void render(Camera3D *param1, QVector<Object3D*> *param2, int param3);
	void dispose();
};

#endif // SHADOWMAP_H
