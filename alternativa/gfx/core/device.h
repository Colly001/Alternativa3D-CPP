#ifndef DEVICE_H
#define DEVICE_H

#include "flash/display3D/Context3DRenderMode.h"
#include "flash/display3D/Context3DCompareMode.h"
#include "flash/display3D/Context3DStencilAction.h"
#include "flash/display3D/Context3DProgramType.h"
#include "flash/display3D/Context3DVertexBufferFormat.h"
#include "renderstate.h"

#include <QVector>


enum class DeviceProfile
{
	BASELINE,
	BASELINECONSTRAINED,
};

class Stage;
class Stage3D;
class BattlefieldModel;
class VertexBufferResource;
class TextureBase;
class Resource;
class IndexBufferResource;
class Context3D;



class Device
{
public:
	Stage *_stage;
	Context3DRenderMode _renderMode;
	DeviceProfile _profile;
	int _x;
	int _y;
	int _width;
	int _height;
	int _antiAlias;
	bool _enableDepthAndStencil;
	bool _enableErrorChecking;
	Stage3D *_stage3D;
	bool _available = true;
	RenderState _renderState;
	bool configured = false;
	int backBufferWidth = -1;
	int backBufferHeight = -1;
	int backBufferAntiAlias = -1;
	bool backBufferEnableDepthAndStencil = false;
	//Dictionary *resourcesToUpload = new Dictionary();
	//BattlefieldModel *bfModel;


	Stage* stage() { return _stage; }


	int getX() { return this->_x; }
	void setX(int x) { this->_x = x; }

	int getY() { return this->_y; }
	void setY(int y) { this->_y = y; }

	int width() { return this->_width; }
	void setWidth(int width) { this->_width = width; }

	int height() { return this->_height; }
	void setHeight(int height) { this->_height = height; }

	int getAntiAlias() { return this->_antiAlias; }
	void setAntiAlias(int antiAlias);

	bool getEnableDepthAndStencil() { return this->_enableDepthAndStencil; }
	void setEnableDepthAndStencil(bool enable) { this->_enableDepthAndStencil = enable; }


	bool getEnableErrorChecking() { return this->_enableErrorChecking; }
	void setEnableErrorChecking(bool enable);

	bool getReady();


public:
	Device(Stage *stage, const  Context3DRenderMode renderMode = Context3DRenderMode::AUTO, DeviceProfile profile = DeviceProfile::BASELINE);



	void setCulling(Context3DTriangleFace triangleFaceToCull);
	void setBlendFactors(Context3DBlendFactor sourceFactor, Context3DBlendFactor destinationFactor);

	void setStencilActions(Context3DTriangleFace triangleFace = Context3DTriangleFace::FRONT_AND_BACK,
						   Context3DCompareMode compareMode = Context3DCompareMode::ALWAYS,
						   Context3DStencilAction actionOnBothPass = Context3DStencilAction::KEEP,
						   Context3DStencilAction actionOnDepthFail = Context3DStencilAction::KEEP,
						   Context3DStencilAction actionOnDepthPassStencilFail = Context3DStencilAction::KEEP);



	void setStencilReferenceValue(uint32_t referenceValue, uint32_t readMask = 255, uint32_t writeMask = 255);


	void setProgram(ProgramResource *program);


	void setDepthTest(bool depthMask, Context3DCompareMode passCompareMode);



	void setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, const double *data, int numRegisters, bool param5 = true);

	void setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, QVector<double> data, int numRegisters, bool param5 = true);


	void setTextureAt(int sampler, TextureResource *texture);


	void setVertexBufferAt(int index, VertexBufferResource *buffer, int bufferOffset = 0, Context3DVertexBufferFormat format = Context3DVertexBufferFormat::FLOAT_4);


	void clear(double red = 0.0, double green = 0.0, double blue = 0.0, double alpha = 1.0, double depth = 1.0, double stencil = 0, uint32_t mask = 0xffffffff);


	void setRenderToBackBuffer();

	void uploadResource(Resource*);



	void drawTriangles(IndexBufferResource *indexBuffer, int firstIndex = 0, int numTriangles = -1);


	void setScissorRectangle(GeomRectangle *rectangle);


	void present();

	void setRenderToTexture(TextureResource *renderTarget,
							bool renderTargetEnableDepthAndStencil = false,
							int renderTargetAntiAlias = 0,
							int renderTargetSurfaceSelector = 0);

	void dispose();

	bool ready();

	void draw3DLines(void *data, int count);

private:
	void onContext3DCreate();
	void prepareResource(Context3D *context, Resource *resource);
};

#endif // DEVICE_H
