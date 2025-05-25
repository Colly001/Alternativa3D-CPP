#include "shadowatlas.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "alternativa/gfx/core/programresource.h"
#include "camera3d.h"
#include "alternativa/gfx/core/device.h"
#include "shadow.h"
#include "mymath.h"
#include "alternativa/gfx/core/rendertargettextureresource.h"
#include "utils/double_array.h"
#include "alternativa/engine3d/core/shadowatlasvertexshaderhlsl.h"
#include "alternativa/engine3d/core/shadowatlasfragmentshaderhlsl.h"



//Теневой Атлас

static QHash<int, ProgramResource*> blurPrograms;





//IndexBufferResource *blurIndexBuffer = new IndexBufferResource(Vector.<uint>([0, 1, 3, 2, 3, 1]));


static const int sizeLimit = 0x0400;


static ProgramResource* getBlurProgram(int param1, int param2)
{
	int key = (param1 << 16) + param2;
	ProgramResource *programResource = blurPrograms[key];
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = ShadowAtlasVertexShaderHLSL();
		QByteArray fragment = ShadowAtlasFragmentShaderHLSL(samplers, param2, param1 == 1);
		programResource = new ProgramResource(0, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		blurPrograms[key] = programResource;
	}
	return programResource;
}



ShadowAtlas::ShadowAtlas(int mapSize, int blur)
{
	this->shadowsCount = 0;
	this->mapSize = mapSize;
	this->blur = blur;
	this->map1 = nullptr;

	double_array_set_8(blurConst, 0, 0, 0, 1, 0, 0, 0, 1);

	QVector<double> list_vertex = {-1, 1, 0, 0, 0, -1, -1, 0, 0, 1, 1, -1, 0, 1, 1, 1, 1, 0, 1, 0};
	QVector<uint32_t> list_index = {0, 1, 3, 2, 3, 1};

	blurVertexBuffer = new VertexBufferResource(list_vertex, 5);
	blurIndexBuffer = new IndexBufferResource(list_index);
}


void ShadowAtlas::renderCasters(Camera3D *camera)
{
	Device *device = camera->device;
	int _loc3_ = int((sizeLimit / this->mapSize));
	int _loc4_ = int(MyMath::ceil((this->shadowsCount / _loc3_)));
	int _loc5_ = ((this->shadowsCount > _loc3_) ? int(_loc3_) : int(this->shadowsCount));
	_loc4_ = int((1 << int(MyMath::ceil((MyMath::log(_loc4_) / MyMath::LN2)))));
	_loc5_ = int((1 << int(MyMath::ceil((MyMath::log(_loc5_) / MyMath::LN2)))));
	if (_loc4_ > _loc3_)
	{
		_loc4_ = _loc3_;
		this->shadowsCount = (_loc4_ * _loc5_);
	}
	int _loc6_ = (_loc4_ << 8) | _loc5_;
	this->map1 = this->maps[_loc6_];
	int _loc7_ = (1 << 16) | _loc6_;
	this->map2 = this->maps[_loc7_];
	if (this->map1 == nullptr)
	{
		this->map1 = new RenderTargetTextureResource((_loc5_ * this->mapSize), (_loc4_ * this->mapSize));
		this->map2 = new RenderTargetTextureResource((_loc5_ * this->mapSize), (_loc4_ * this->mapSize));
		this->maps[_loc6_] = this->map1;
		this->maps[_loc7_] = this->map2;
	}
	device->setRenderToTexture(this->map1, true);
	device->clear(0, 0, 0, 0, 0);
	int _loc8_ = 0;
	while (_loc8_ < this->shadowsCount)
	{
		Shadow *shadow = this->shadows[_loc8_];
		shadow->texture = this->map1;
		shadow->textureScaleU = 1. / _loc5_;
		shadow->textureScaleV = 1. / _loc4_;
		shadow->textureOffsetU = double(_loc8_ % _loc5_) / _loc5_;
		shadow->textureOffsetV = int((double(_loc8_) / double(_loc5_))) / _loc4_;
		shadow->renderCasters(camera);
		_loc8_++;
	}

}

void ShadowAtlas::renderBlur(Camera3D *param1)
{
	Device *device = param1->device;
	if (this->blur > 0)
	{
		device->setVertexBufferAt(0, blurVertexBuffer, 0, Context3DVertexBufferFormat::FLOAT_3);
		device->setVertexBufferAt(1, blurVertexBuffer, 3, Context3DVertexBufferFormat::FLOAT_2);
		blurConst[0] = (1. / this->map1->getWidth());
		blurConst[1] = (1. / this->map1->getHeight());
		blurConst[3] = ((1. + this->blur) + this->blur);
		blurConst[4] = (double(this->blur) / this->map1->getWidth());
		blurConst[5] = (double(this->blur) / this->map1->getHeight());
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, blurConst, 2);
		device->setRenderToTexture(this->map2, false);
		device->clear(0, 0, 0, 0);
		device->setProgram(getBlurProgram(1, this->blur));
		device->setTextureAt(0, this->map1);
		device->drawTriangles(blurIndexBuffer, 0, 2);
		device->setRenderToTexture(this->map1, false);
		device->clear(0, 0, 0, 0);
		device->setProgram(getBlurProgram(2, this->blur));
		device->setTextureAt(0, this->map2);
		device->drawTriangles(blurIndexBuffer, 0, 2);
	}
}

void ShadowAtlas::clear()
{
	/*
	var _loc2_:Shadow;
	var _loc1_:int;
	while (_loc1_ < this->shadowsCount)
	{
		_loc2_ = this->shadows[_loc1_];
		_loc2_->texture = null;
		_loc1_++;
	}
	this->shadows.length = 0;
	this->shadowsCount = 0;
	*/
}
