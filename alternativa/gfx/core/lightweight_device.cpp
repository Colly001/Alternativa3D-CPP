#include "device.h"


void Device::setAntiAlias(int)
{

}

void Device::setEnableErrorChecking(bool)
{

}

bool Device::getReady()
{
	return false;
}

Device::Device(Stage *stage, Context3DRenderMode renderMode, DeviceProfile profile)
{	
	(void)stage;
	(void)renderMode;
	(void)profile;

	/*
	this->_stage = stage;
	this->_renderMode = renderMode;
	this->_profile = profile;
	this->_stage3D = this->_stage->stage3D();
	this->_x = _stage3D->x();;
	this->_y = _stage3D->y();;
	this->_width = stage->stageWidth();
	this->_height = stage->stageHeight();
	this->_antiAlias = 0;
	this->_enableDepthAndStencil = true;
	this->_enableErrorChecking = false;
	*/
}

void Device::setCulling(Context3DTriangleFace)
{

}

void Device::setBlendFactors(Context3DBlendFactor, Context3DBlendFactor)
{

}

void Device::setStencilActions(Context3DTriangleFace,
							   Context3DCompareMode,
							   Context3DStencilAction,
							   Context3DStencilAction,
							   Context3DStencilAction)
{

}

void Device::setStencilReferenceValue(uint32_t, uint32_t, uint32_t)
{

}

void Device::setProgram(ProgramResource*)
{

}

void Device::setDepthTest(bool, Context3DCompareMode)
{

}

void Device::setProgramConstantsFromVector(Context3DProgramType, int, const double*, int, bool)
{

}

void Device::setProgramConstantsFromVector(Context3DProgramType, int, QVector<double>, int, bool)
{

}

void Device::setTextureAt(int, TextureResource*)
{

}

void Device::setVertexBufferAt(int, VertexBufferResource*, int, Context3DVertexBufferFormat)
{

}

void Device::clear(double, double, double, double, double, double, uint32_t)
{

}

void Device::setRenderToBackBuffer()
{

}

void Device::uploadResource(Resource*)
{

}

void Device::drawTriangles(IndexBufferResource*, int, int)
{

}

void Device::onContext3DCreate()
{

}

void Device::prepareResource(Context3D*, Resource*)
{

}

void Device::setScissorRectangle(GeomRectangle*)
{

}

void Device::present()
{

}

void Device::dispose()
{

}

bool Device::ready()
{
	return false;
}
