#ifndef DEPTHRENDERER_H
#define DEPTHRENDERER_H

#include <QVector>
#include <QHash>
#include <memory>

class Object3D;
class Camera3D;
class RenderTargetTextureResource;
class VertexBufferResource;
class IndexBufferResource;
class BitmapTextureResource;
class GeomRectangle;
class ProgramResource;

class DepthRenderer
{
	//depthPrograms:Array = new Array();

	double correction[12];
	double depthFragment[8];
	double alphaTestConst[4];

	//alphaTestConst:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	//ssaoProgram:ProgramResource;
	VertexBufferResource *ssaoVertexBuffer;
	IndexBufferResource *ssaoIndexBuffer;

	double ssaoVertex[12];
	double ssaoFragment[28];
	//blurProgram:ProgramResource;

	double blurFragment[8];
	//omniProgram:ProgramResource;
	//spotProgram:ProgramResource;
	//tubeProgram:ProgramResource;
	//lightConst:Vector.<Number> = new Vector.<Number>();
	VertexBufferResource *lightVertexBuffer;
	IndexBufferResource *lightIndexBuffer;

	RenderTargetTextureResource *temporaryBuffer;
	GeomRectangle *scissor;

	std::shared_ptr<BitmapTextureResource> table;
	std::shared_ptr<BitmapTextureResource> noise;





public:
	DepthRenderer();



	RenderTargetTextureResource *depthBuffer;
	RenderTargetTextureResource *lightBuffer;

	double correctionX;
	double correctionY;

	void render(Camera3D *param1,
				double param2,
				double param3,
				double param4,
				bool param5,
				bool param6,
				double param7,
				QVector<Object3D*> param8,
				int param9);

	void resetResources();
};

#endif // DEPTHRENDERER_H
