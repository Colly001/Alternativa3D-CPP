#include "renderstate.h"
#include <string.h>


RenderState::RenderState()
{
	blendSourceFactor = Context3DBlendFactor::ONE;
	blendDestinationFactor = Context3DBlendFactor::ZERO;
	colorMaskRed = true;
	colorMaskGreen = true;
	colorMaskBlue = true;
	colorMaskAlpha = true;
	culling = Context3DTriangleFace::NONE;
	depthTestMask = true;

	depthTestPassCompareMode = Context3DCompareMode::LESS;
	program = nullptr;
	renderTarget = nullptr;
	renderTargetEnableDepthAndStencil = false;
	renderTargetAntiAlias = 0;
	renderTargetSurfaceSelector = 0;
	scissor = false;
	stencilActionTriangleFace = Context3DTriangleFace::FRONT_AND_BACK;
	stencilActionCompareMode = Context3DCompareMode::ALWAYS;
	stencilActionOnBothPass = Context3DStencilAction::KEEP;
	stencilActionOnDepthFail = Context3DStencilAction::KEEP;
	stencilActionOnDepthPassStencilFail = Context3DStencilAction::KEEP;
	stencilReferenceValue = 0;
	stencilReadMask = 0xFF;
	stencilWriteMask = 0xFF;

	this->vertexConstants = new double[128*4];
	this->fragmentConstants = new double[128*4];

	memset(this->vertexConstants, 0, 128 * 4 * sizeof(double));
	memset(this->fragmentConstants, 0, 128 * 4 * sizeof(double));
	memset(this->textures, 0, 8 * sizeof(void*));
	memset(this->vertexBuffers, 0, 8 * sizeof(void*));
	memset(this->vertexBuffersFormats, 255, 8 * sizeof(Context3DVertexBufferFormat));
	memset(this->vertexBuffersOffsets, 0, 8 * sizeof(int));
}

RenderState::~RenderState()
{
	delete []this->vertexConstants;
	delete []this->fragmentConstants;
}
