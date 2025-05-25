#ifndef RENDERSTATE_H
#define RENDERSTATE_H

#include "flash/geom/geomrectangle.h"
#include "flash/display3D/Context3DTriangleFace.h"
#include "flash/display3D/Context3DBlendFactor.h"
#include "flash/display3D/Context3DCompareMode.h"
#include "flash/display3D/Context3DStencilAction.h"
#include "flash/display3D/Context3DVertexBufferFormat.h"
#include <stdint.h>



class ProgramResource;
class TextureResource;
class VertexBufferResource;


class RenderState
{
public:

	enum class STATE
	{
		one,
		zero,
		none,
		less,
		frontAndBack,
		always,
		keep,
	};

	Context3DBlendFactor blendSourceFactor;
	Context3DBlendFactor blendDestinationFactor;
	bool colorMaskRed;
	bool colorMaskGreen;
	bool colorMaskBlue;
	bool colorMaskAlpha;
	Context3DTriangleFace culling;
	bool depthTestMask;
	Context3DCompareMode depthTestPassCompareMode;
	ProgramResource *program;
	TextureResource *renderTarget;
	bool renderTargetEnableDepthAndStencil;
	int renderTargetAntiAlias;
	int renderTargetSurfaceSelector;
	bool scissor;
	GeomRectangle scissorRectangle;
	Context3DTriangleFace stencilActionTriangleFace;
	Context3DCompareMode stencilActionCompareMode;
	Context3DStencilAction stencilActionOnBothPass;
	Context3DStencilAction stencilActionOnDepthFail;
	Context3DStencilAction stencilActionOnDepthPassStencilFail;
	uint32_t stencilReferenceValue;
	uint32_t stencilReadMask;
	uint32_t stencilWriteMask;

	TextureResource *textures[8];
	VertexBufferResource *vertexBuffers[8];
	int vertexBuffersOffsets[8];
	Context3DVertexBufferFormat vertexBuffersFormats[8];

	//QVector<TextureResource> textures; //8 true
	//QVector<VertexBufferResource> vertexBuffers; //8 true
	//QVector<int> vertexBuffersOffsets; //8 true
	//QVector<String> vertexBuffersFormats; //8 true

	//QVector<double> vertexConstants; //128*4 true
	//QVector<double> fragmentConstants; //28*4 true

	double *vertexConstants;
	double *fragmentConstants;


	RenderState();
	~RenderState();
};

#endif // RENDERSTATE_H
