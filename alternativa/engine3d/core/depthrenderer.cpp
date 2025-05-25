#include "depthrenderer.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "mymath.h"
#include <QDebug>
#include "alternativa/gfx/core/bitmaptextureresource.h"
#include "randomgenerator.h"
#include "alternativa/gfx/core/rendertargettextureresource.h"
#include "flash/display/bitmapdata.h"
#include "flash/geom/geomrectangle.h"
#include "alternativa/engine3d/lights/omnilight.h"
#include "alternativa/engine3d/lights/spotlight.h"
#include "alternativa/engine3d/lights/tubelight.h"
#include "alternativa/gfx/core/device.h"
#include "alternativa/engine3d/materials/texturematerial.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/engine3d/objects/mesh.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/objects/bsp.h"
#include "alternativa/engine3d/core/view.h"
#include "utils/double_array.h"
#include "alternativa/engine3d/core/depthrendererdepthvertexshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererdepthfragmentshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererlightvertexshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererlightfragmentshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererssaovertexshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererssaofragmentshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererblurvertexshaderhlsl.h"
#include "alternativa/engine3d/core/depthrendererblurfragmentshaderhlsl.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"
#include "alternativa/gfx/core/programresource.h"



static const int limit2const = 62;
static const int limit5const = 24;
static const int samples = 6;
static const double noiseAngle = (MyMath::PI * 2) / samples;
static const int tableSize = 128;
static const int noiseSize = 4;
static const double bias = 0.1;
static const int blurSamples = 16;
static const double intensity = 2.5;
static const double noiseRandom = 0.2;




DepthRenderer::DepthRenderer()
{
	scissor = new GeomRectangle();


	QVector<double> list_1 = {-1, 1, 0, 0, 0, -1, -1, 0, 0, 1, 1, -1, 0, 1, 1, 1, 1, 0, 1, 0};
	ssaoVertexBuffer = new VertexBufferResource(list_1, 5);

	QVector<uint32_t> list_2 = {0, 1, 3, 2, 3, 1};
	ssaoIndexBuffer = new IndexBufferResource(list_2);




	double_array_set_12(correction, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0.5);
	double_array_set_8(depthFragment, 1. / 255., 0, 0, 1, 0.5, 0.5, 0, 1);
	double_array_set_28(ssaoFragment, 0, 0, 0, (MyMath::PI * 2), 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, (MyMath::PI * 2), (MyMath::PI * 2));
	double_array_set_12(ssaoVertex, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1);
	double_array_set_8(blurFragment, 0, 0, 0, 1, 0, 0, 0, 1);

	double_array_set_4(alphaTestConst, 0, 0, 0, 1);



	int _loc3_ = 0;
	int _loc4_ = 0;

	QVector<double> _loc5_;
	QVector<uint> _loc6_;


	_loc5_.resize(2480);
	_loc6_.resize(2232);


	int _loc1_ = 0;
	while (_loc1_ < limit2const)
	{
		int _loc11_ = (4 + (_loc1_ * 2));
		int _loc12_ = (4 + (_loc1_ * 5));
		int _loc13_ = (_loc1_ * 8);
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc14_ = (_loc13_ + 1);
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc15_ = (_loc14_ + 1);
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc16_ = (_loc15_ + 1);
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc17_ = (_loc16_ + 1);
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc18_ = (_loc17_ + 1);
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc19_ = (_loc18_ + 1);
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		int _loc20_ = (_loc19_ + 1);
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = -1;
		_loc3_++;
		_loc5_[_loc3_] = 1;
		_loc3_++;
		_loc5_[_loc3_] = _loc11_;
		_loc3_++;
		_loc5_[_loc3_] = _loc12_;
		_loc3_++;
		_loc6_[_loc4_] = _loc13_;
		_loc4_++;
		_loc6_[_loc4_] = _loc17_;
		_loc4_++;
		_loc6_[_loc4_] = _loc14_;
		_loc4_++;
		_loc6_[_loc4_] = _loc14_;
		_loc4_++;
		_loc6_[_loc4_] = _loc17_;
		_loc4_++;
		_loc6_[_loc4_] = _loc18_;
		_loc4_++;
		_loc6_[_loc4_] = _loc14_;
		_loc4_++;
		_loc6_[_loc4_] = _loc18_;
		_loc4_++;
		_loc6_[_loc4_] = _loc19_;
		_loc4_++;
		_loc6_[_loc4_] = _loc14_;
		_loc4_++;
		_loc6_[_loc4_] = _loc19_;
		_loc4_++;
		_loc6_[_loc4_] = _loc15_;
		_loc4_++;
		_loc6_[_loc4_] = _loc17_;
		_loc4_++;
		_loc6_[_loc4_] = _loc19_;
		_loc4_++;
		_loc6_[_loc4_] = _loc18_;
		_loc4_++;
		_loc6_[_loc4_] = _loc17_;
		_loc4_++;
		_loc6_[_loc4_] = _loc20_;
		_loc4_++;
		_loc6_[_loc4_] = _loc19_;
		_loc4_++;
		_loc6_[_loc4_] = _loc15_;
		_loc4_++;
		_loc6_[_loc4_] = _loc19_;
		_loc4_++;
		_loc6_[_loc4_] = _loc20_;
		_loc4_++;
		_loc6_[_loc4_] = _loc15_;
		_loc4_++;
		_loc6_[_loc4_] = _loc20_;
		_loc4_++;
		_loc6_[_loc4_] = _loc16_;
		_loc4_++;
		_loc6_[_loc4_] = _loc13_;
		_loc4_++;
		_loc6_[_loc4_] = _loc16_;
		_loc4_++;
		_loc6_[_loc4_] = _loc20_;
		_loc4_++;
		_loc6_[_loc4_] = _loc13_;
		_loc4_++;
		_loc6_[_loc4_] = _loc20_;
		_loc4_++;
		_loc6_[_loc4_] = _loc17_;
		_loc4_++;
		_loc6_[_loc4_] = _loc13_;
		_loc4_++;
		_loc6_[_loc4_] = _loc14_;
		_loc4_++;
		_loc6_[_loc4_] = _loc15_;
		_loc4_++;
		_loc6_[_loc4_] = _loc13_;
		_loc4_++;
		_loc6_[_loc4_] = _loc15_;
		_loc4_++;
		_loc6_[_loc4_] = _loc16_;
		_loc4_++;
		_loc1_++;
	}



	this->lightVertexBuffer = new VertexBufferResource(_loc5_, 5);
	this->lightIndexBuffer = new IndexBufferResource(_loc6_);


	double _loc9_ = MyMath::PI * 2;
	int _loc10_ = tableSize - 1;




	std::shared_ptr<BitmapData> bitmapData_table = std::make_shared<BitmapData>(tableSize, tableSize, false, 0);

	for (int y = 0; y < tableSize; y++)
	{
		double v = ((double(y) / _loc10_) - 0.5) * 2;
		for (int x = 0; x < tableSize; x++)
		{
			double _loc22_ = (((double(x) / _loc10_) - 0.5) * 2);
			double _loc23_ = MyMath::atan2(v, _loc22_);
			if (_loc23_ < 0)
				_loc23_ = _loc23_ + _loc9_;

			uint32_t color  = MyMath::round(((0xFF * _loc23_) / _loc9_));
			bitmapData_table->setPixel(x, y, color);
		}
	}


	this->table = std::make_shared<BitmapTextureResource>(bitmapData_table, false);



	std::shared_ptr<BitmapData> bitmapData = std::make_shared<BitmapData>(noiseSize, noiseSize, false, 0);

	for (int y = 0; y < noiseSize; y++)
	{
		for (int x = 0; x < noiseSize; x++)
		{
			double v = RandomGenerator::random() * noiseAngle;
			int red = MyMath::sin(v) * 0xFF;
			int green = MyMath::cos(v) * 0xFF;
			int blue = (noiseRandom + (RandomGenerator::random() * (1 - noiseRandom))) * 0xFF;
			uint32_t color = ((red << 16) | (green << 8)) | blue;
			bitmapData->setPixel(x, y, color);
		}
	}

	this->noise = std::make_shared<BitmapTextureResource>(bitmapData, false);
	//this->noise->bitmapData->setVector(this->noise->bitmapData->rect, _loc7_);
	this->depthBuffer = new RenderTargetTextureResource(1, 1);
	this->temporaryBuffer = new RenderTargetTextureResource(1, 1);
	this->lightBuffer = new RenderTargetTextureResource(1, 1);
}





//DepthProgram
//DepthRendererDepthVertexShader
//DepthRendererDepthFragmentShader

//OmniProgram
//DepthRendererLightVertexShader
//DepthRendererLightFragmentShader

//SpotProgram
//DepthRendererLightVertexShader
//DepthRendererLightFragmentShader

//TubeProgram
//DepthRendererLightVertexShader
//DepthRendererLightFragmentShader

//SSAOProgram
//DepthRendererSSAOVertexShader
//DepthRendererSSAOFragmentShader

//BlurProgram
//DepthRendererBlurVertexShader
//DepthRendererBlurFragmentShader


static QHash<int, ProgramResource*> depthPrograms;
static ProgramResource *omniProgram;
static ProgramResource *spotProgram;
static ProgramResource *tubeProgram;
static ProgramResource *ssaoProgram;
static ProgramResource *blurProgram;



static ProgramResource* getDepthProgram(bool param1,
								 bool param2,
								 bool param3 = false,
								 bool param4 = false,
								 bool param5 = false,
								 bool param6 = false,
								 bool param7 = false)
{
	int key = ((((((int(param1) | (int(param2) << 1)) | (int(param3) << 2)) | (int(param4) << 3)) | (int(param5) << 4)) | (int(param6) << 5)) | (int(param7) << 6));
	ProgramResource *programResource = depthPrograms.value(key);
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = DepthRendererDepthVertexShaderHLSL(param1, param2);
		QByteArray fragment = DepthRendererDepthFragmentShaderHLSL(samplers, param1, param2, param3, param4, param5);
		programResource = new ProgramResource(2, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		depthPrograms.insert(key, programResource);
	}

	return programResource;
}

static ProgramResource* getOmniProgram()
{
	ProgramResource *programResource = omniProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = DepthRendererLightVertexShaderHLSL(0);
		QByteArray fragment = DepthRendererLightFragmentShaderHLSL(0);
		programResource = new ProgramResource(6, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		omniProgram = programResource;
	}
	return programResource;
}


static ProgramResource* getSpotProgram()
{
	ProgramResource *programResource = spotProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = DepthRendererLightVertexShaderHLSL(1);
		QByteArray fragment = DepthRendererLightFragmentShaderHLSL(1);
		programResource = new ProgramResource(5, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		spotProgram = programResource;
	}
	return programResource;
}

static ProgramResource* getTubeProgram()
{
	ProgramResource *programResource = tubeProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex =  DepthRendererLightVertexShaderHLSL(2);
		QByteArray fragment = DepthRendererLightFragmentShaderHLSL(2);
		programResource = new ProgramResource(7, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		tubeProgram = programResource;
	}
	return programResource;
}

static ProgramResource* getSSAOProgram()
{
	ProgramResource *programResource = ssaoProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = DepthRendererSSAOVertexShaderHLSL();
		QByteArray fragment = DepthRendererSSAOFragmentShaderHLSL(samplers, samples);
		programResource = new ProgramResource(3, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		ssaoProgram = programResource;
	}
	return programResource;
}

static ProgramResource* getBlurProgram()
{
	ProgramResource *programResource = blurProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = DepthRendererBlurVertexShaderHLSL();
		QByteArray fragment = DepthRendererBlurFragmentShaderHLSL(samplers);
		programResource = new ProgramResource(4, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		blurProgram = programResource;
	}
	return programResource;
}


void DepthRenderer::render(Camera3D *param1,
						   double param2,
						   double param3,
						   double param4,
						   bool param5,
						   bool param6,
						   double param7,
						   QVector<Object3D*> param8,
						   int param9)
{
	VertexBufferResource *vertexBuffer = nullptr;
	IndexBufferResource *indexBuffer = nullptr;
	TextureMaterial *textureMaterial = nullptr;
	Device *device = param1->device;

	if (param2 > 0x0800)
		param2 = 0x0800;

	if (param3 > 0x0800)
		param3 = 0x0800;

	if (param4 > 1)
		param4 = 1;


	param2 = MyMath::round(param2 * param4);
	param3 = MyMath::round(param3 * param4);

	if (param2 < 1)
		param2 = 1;

	if (param3 < 1)
		param3 = 1;

	this->scissor->width = param2;
	this->scissor->height = param3;
	int width = int((1 << int(MyMath::ceil((MyMath::log(param2) / MyMath::LN2)))));
	int height = int((1 << int(MyMath::ceil((MyMath::log(param3) / MyMath::LN2)))));
	if (((!(width == this->depthBuffer->getWidth())) || (!(height == this->depthBuffer->getHeight()))))
	{
		this->depthBuffer->dispose();
		this->depthBuffer = new RenderTargetTextureResource(width, height);
		this->temporaryBuffer->dispose();
		this->temporaryBuffer = new RenderTargetTextureResource(width, height);
		this->lightBuffer->dispose();
		this->lightBuffer = new RenderTargetTextureResource(width, height);
	}

	if (!param5)
	{
		this->noise.reset();
		this->temporaryBuffer->reset();
		this->ssaoVertexBuffer->reset();
		this->ssaoIndexBuffer->reset();
	}

	if (!param6)
	{
		this->lightBuffer->reset();
		this->lightVertexBuffer->reset();
		this->lightIndexBuffer->reset();
	}

	if (!param5 && !param6)
	{
		this->table->reset();
	}

	this->correctionX = (param2 / this->depthBuffer->getWidth());
	this->correctionY = (param3 / this->depthBuffer->getHeight());
	device->setRenderToTexture(this->depthBuffer, true);
	device->clear(1, 0, 0.25, 1);
	device->setScissorRectangle(this->scissor);
	this->correction[0] = this->correctionX;
	this->correction[1] = this->correctionY;
	this->correction[2] = 255. / param1->farClipping;
	this->correction[4] = 1 - this->correctionX;
	this->correction[5] = 1 - this->correctionY;
	this->correction[8] = param1->correctionX;
	this->correction[9] = param1->correctionY;

	device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 3, param1->projection, 1, false);
	device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 4, this->correction, 3, false);

	if (param5 || param6)
	{
		device->setTextureAt(0, this->table.get());
	}
	int numTriangles = 0;

#if 1
	int _loc10_ = 0;
	while (_loc10_ < param9)
	{
		Object3D *object = param8.at(_loc10_);


		if (object->__type_next == Mesh::TYPE)
		{
			//ssao

			Mesh *mesh = (Mesh*)object;
			vertexBuffer = mesh->vertexBuffer.get();
			indexBuffer = mesh->indexBuffer.get();
			numTriangles = mesh->numTriangles;
			textureMaterial = (TextureMaterial*)mesh->faceList()->material.get();
		}
		else if (object->__type_next == BSP::TYPE)
		{
			_loc10_++;
			continue;

			BSP *bsp = (BSP*)object;
			vertexBuffer = bsp->vertexBuffer.get();
			indexBuffer = bsp->indexBuffer.get();
			numTriangles = bsp->numTriangles;
			textureMaterial = (TextureMaterial*)bsp->faces->first()->material.get();
		}
		else
		{
			_loc10_++;
			qDebug() << "error DepthRenderer::render" << object->__type->getName();
			continue;
		}

		if ((((!(textureMaterial == nullptr)) && (textureMaterial->alphaTestThreshold > 0)) && (textureMaterial->transparent())))
		{
			device->setProgram(getDepthProgram(param5 || param6,
											   true,
											   param1->view->quality,
											   textureMaterial->repeat,
											   textureMaterial->_mipMapping != MipMapping::NONE,
											   false,
											   false));


			device->setVertexBufferAt(2, vertexBuffer, 3, Context3DVertexBufferFormat::FLOAT_2);
			device->setTextureAt(1, textureMaterial->textureResource);
			this->alphaTestConst[0] = textureMaterial->textureResource->correctionU();
			this->alphaTestConst[1] = textureMaterial->textureResource->correctionV();
			this->alphaTestConst[3] = textureMaterial->alphaTestThreshold;
			device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 7, this->alphaTestConst, 1);
		}
		else
		{
			device->setProgram(getDepthProgram(param5 || param6, false));
		}

		device->setVertexBufferAt(0, vertexBuffer, 0, Context3DVertexBufferFormat::FLOAT_3);
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, object->transformConst, 3, false);
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, this->depthFragment, 2);

		if (param5 || param6)
		{
			device->setVertexBufferAt(1, vertexBuffer, 5, Context3DVertexBufferFormat::FLOAT_3);
		}

		device->drawTriangles(indexBuffer, 0, numTriangles);
		device->setTextureAt(1, nullptr);
		device->setVertexBufferAt(2, nullptr);
		_loc10_++;
	}
#endif
	if (param6)
	{
		device->setRenderToTexture(this->lightBuffer, false);
		device->clear(param7, param7, param7, 0);
		device->setBlendFactors(Context3DBlendFactor::ONE, Context3DBlendFactor::ONE);
		device->setTextureAt(0, this->depthBuffer);
		device->setVertexBufferAt(0, this->lightVertexBuffer, 0, Context3DVertexBufferFormat::FLOAT_3);
		device->setVertexBufferAt(1, this->lightVertexBuffer, 3, Context3DVertexBufferFormat::FLOAT_2);
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, param1->projection, 1, false);
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 1, this->correction, 3, false);
		this->ssaoFragment[0] = param1->farClipping;
		this->ssaoFragment[1] = param1->farClipping / 255.;
		this->ssaoFragment[4] = (2. / this->correctionX);
		this->ssaoFragment[5] = (2. / this->correctionY);
		this->ssaoFragment[6] = 0;
		this->ssaoFragment[8] = 1;
		this->ssaoFragment[9] = 1;
		this->ssaoFragment[10] = 0.5;
		this->ssaoFragment[12] = param1->correctionX;
		this->ssaoFragment[13] = param1->correctionY;
		this->ssaoFragment[16] = 0.5;
		this->ssaoFragment[17] = 0.5;
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, this->ssaoFragment, 7, false);
		device->setProgram(getOmniProgram());
		int index = 0;
		int _loc22_ = 0;
		int _loc10_ = 0;

		double lightConst[62];



		while (_loc10_ < param1->omniesCount)
		{
			OmniLight *omniLight = param1->omnies.at(_loc10_);
			lightConst[index] = omniLight->cmd * param1->correctionX;
			index++;
			lightConst[index] = omniLight->cmh * param1->correctionY;
			index++;
			lightConst[index] = omniLight->cml;
			index++;
			lightConst[index] = omniLight->attenuationEnd;
			index++;
			lightConst[index] = ((omniLight->intensity * param1->deferredLightingStrength) * ((omniLight->color >> 16) & 0xFF)) / 255.;
			index++;
			lightConst[index] = ((omniLight->intensity * param1->deferredLightingStrength) * ((omniLight->color >> 8) & 0xFF)) / 255.;
			index++;
			lightConst[index] = ((omniLight->intensity * param1->deferredLightingStrength) * (omniLight->color & 0xFF)) / 255.;
			index++;
			lightConst[index] = 1. / (omniLight->attenuationEnd - omniLight->attenuationBegin);
			index++;
			_loc22_++;
			if (_loc22_ == limit2const || _loc10_ == (param1->omniesCount - 1))
			{
				device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 4, lightConst, (_loc22_ * 2), false);
				device->drawTriangles(this->lightIndexBuffer, 0, ((_loc22_ * 6) * 2));
				_loc22_ = 0;
				index = 0;
			}
			_loc10_++;
		}
		device->setProgram(getSpotProgram());
		index = 0;
		_loc22_ = 0;
		_loc10_ = 0;
		while (_loc10_ < param1->spotsCount)
		{
			SpotLight *_loc24_ = param1->spots[_loc10_];
			double _loc25_ = MyMath::cos(_loc24_->hotspot * 0.5);
			double _loc26_ = MyMath::cos(_loc24_->falloff * 0.5);
			lightConst[index] = _loc24_->cma;
			index++;
			lightConst[index] = _loc24_->cmb;
			index++;
			lightConst[index] = _loc24_->cmc;
			index++;
			lightConst[index] = _loc24_->cmd;
			index++;
			lightConst[index] = _loc24_->cme;
			index++;
			lightConst[index] = _loc24_->cmf;
			index++;
			lightConst[index] = _loc24_->cmg;
			index++;
			lightConst[index] = _loc24_->cmh;
			index++;
			lightConst[index] = _loc24_->cmi;
			index++;
			lightConst[index] = _loc24_->cmj;
			index++;
			lightConst[index] = _loc24_->cmk;
			index++;
			lightConst[index] = _loc24_->cml;
			index++;
			lightConst[index] = _loc24_->attenuationEnd;
			index++;
			lightConst[index] = 1. / (_loc24_->attenuationEnd - _loc24_->attenuationBegin);
			index++;
			lightConst[index] = _loc26_;
			index++;
			lightConst[index] = 1. / (_loc25_ - _loc26_);
			index++;
			lightConst[index] = ((_loc24_->intensity * param1->deferredLightingStrength) * ((_loc24_->color >> 16) & 0xFF)) / 255.;
			index++;
			lightConst[index] = ((_loc24_->intensity * param1->deferredLightingStrength) * ((_loc24_->color >> 8) & 0xFF)) / 255.;
			index++;
			lightConst[index] = ((_loc24_->intensity * param1->deferredLightingStrength) * (_loc24_->color & 0xFF)) / 255.;
			index++;
			lightConst[index] = MyMath::sin(_loc24_->falloff * 0.5) * _loc24_->attenuationEnd;
			index++;
			_loc22_++;
			if (((_loc22_ == limit5const) || (_loc10_ == (param1->spotsCount - 1))))
			{
				device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 4, lightConst, (_loc22_ * 5), false);
				device->drawTriangles(this->lightIndexBuffer, 0, ((_loc22_ * 6) * 2));
				_loc22_ = 0;
				index = 0;
			}
			_loc10_++;
		}
		device->setProgram(getTubeProgram());
		index = 0;
		_loc22_ = 0;
		_loc10_ = 0;
		while (_loc10_ < param1->tubesCount)
		{
			TubeLight *_loc27_ = param1->tubes[_loc10_];
			lightConst[index] = _loc27_->cma;
			index++;
			lightConst[index] = _loc27_->cmb;
			index++;
			lightConst[index] = _loc27_->cmc;
			index++;
			lightConst[index] = _loc27_->cmd;
			index++;
			lightConst[index] = _loc27_->cme;
			index++;
			lightConst[index] = _loc27_->cmf;
			index++;
			lightConst[index] = _loc27_->cmg;
			index++;
			lightConst[index] = _loc27_->cmh;
			index++;
			lightConst[index] = _loc27_->cmi;
			index++;
			lightConst[index] = _loc27_->cmj;
			index++;
			lightConst[index] = _loc27_->cmk;
			index++;
			lightConst[index] = _loc27_->cml;
			index++;
			lightConst[index] = _loc27_->attenuationEnd;
			index++;
			lightConst[index] = (1. / (_loc27_->attenuationEnd - _loc27_->attenuationBegin));
			index++;
			lightConst[index] = ((_loc27_->length * 0.5) + _loc27_->falloff);
			index++;
			lightConst[index] = (1. / _loc27_->falloff);
			index++;
			lightConst[index] = (((_loc27_->intensity * param1->deferredLightingStrength) * ((_loc27_->color >> 16) & 0xFF)) / 255.);
			index++;
			lightConst[index] = (((_loc27_->intensity * param1->deferredLightingStrength) * ((_loc27_->color >> 8) & 0xFF)) / 255.);
			index++;
			lightConst[index] = (((_loc27_->intensity * param1->deferredLightingStrength) * (_loc27_->color & 0xFF)) / 255.);
			index++;
			lightConst[index] = (_loc27_->length * 0.5);
			index++;
			_loc22_++;
			if (((_loc22_ == limit5const) || (_loc10_ == (param1->tubesCount - 1))))
			{
				device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 4, lightConst, (_loc22_ * 5), false);
				device->drawTriangles(this->lightIndexBuffer, 0, ((_loc22_ * 6) * 2));
				_loc22_ = 0;
				index = 0;
			}
			_loc10_++;
		}
		device->setBlendFactors(Context3DBlendFactor::ONE, Context3DBlendFactor::ZERO);
	}

	if (param5)
	{
		device->setRenderToTexture(this->temporaryBuffer, false);
		device->clear(0, 0, 0, 0);
		device->setProgram(getSSAOProgram());
		device->setTextureAt(0, this->depthBuffer);
		device->setTextureAt(1, this->noise.get());
		device->setVertexBufferAt(0, this->ssaoVertexBuffer, 0, Context3DVertexBufferFormat::FLOAT_3);
		device->setVertexBufferAt(1, this->ssaoVertexBuffer, 3, Context3DVertexBufferFormat::FLOAT_2);
		this->ssaoVertex[0] = (double(width) / noiseSize);
		this->ssaoVertex[1] = (double(height) / noiseSize);
		this->ssaoVertex[4] = (2. / this->correctionX);
		this->ssaoVertex[5] = (2. / this->correctionY);
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, this->ssaoVertex, 3, false);
		this->ssaoFragment[0] = param1->farClipping;
		this->ssaoFragment[1] = (param1->farClipping / 255.);
		this->ssaoFragment[4] = (2. / this->correctionX);
		this->ssaoFragment[5] = (2. / this->correctionY);
		this->ssaoFragment[6] = param1->ssaoRadius;
		this->ssaoFragment[8] = 1.;
		this->ssaoFragment[9] = 1.;
		this->ssaoFragment[10] = bias;
		this->ssaoFragment[11] = (intensity * 1.) / samples;
		this->ssaoFragment[12] = param1->correctionX;
		this->ssaoFragment[13] = param1->correctionY;
		this->ssaoFragment[15] = 1. / param1->ssaoRange;
		this->ssaoFragment[16] = MyMath::cos(noiseAngle);
		this->ssaoFragment[17] = MyMath::sin(noiseAngle);
		this->ssaoFragment[20] = -MyMath::sin(noiseAngle);
		this->ssaoFragment[21] = MyMath::cos(noiseAngle);
		this->ssaoFragment[24] = this->correctionX - (1. / width);
		this->ssaoFragment[25] = this->correctionY - (1. / height);
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, this->ssaoFragment, 7, false);
		device->drawTriangles(this->ssaoIndexBuffer, 0, 2);
		device->setTextureAt(1, nullptr);
		device->setRenderToTexture(this->depthBuffer, false);
		device->clear(0, 0, 0, 0);
		device->setProgram(getBlurProgram());
		device->setTextureAt(0, this->temporaryBuffer);
		this->blurFragment[0] = 1. / width;
		this->blurFragment[1] = 1. / height;
		this->blurFragment[3] = 1. / blurSamples;
		this->blurFragment[4] = this->correctionX - (1. / width);
		this->blurFragment[5] = this->correctionY - (1. / height);
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, this->blurFragment, 2, false);
		device->drawTriangles(this->ssaoIndexBuffer, 0, 2);
	}

	device->setVertexBufferAt(1, nullptr);
	device->setTextureAt(0, nullptr);
	device->setScissorRectangle(nullptr);
}

void DepthRenderer::resetResources()
{
	/*
	this->noise.reset();
	this->table.reset();
	this->depthBuffer.reset();
	this->temporaryBuffer.reset();
	this->lightBuffer.reset();
	this->ssaoVertexBuffer.reset();
	this->ssaoIndexBuffer.reset();
	this->lightVertexBuffer.reset();
	this->lightIndexBuffer.reset();
	*/
}
