#include "texturematerial.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "mymath.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/face.h"
#include "flash/display/bitmapdata.h"
#include "textureresourcesregistry.h"
#include "alternativa/gfx/core/bitmaptextureresource.h"
#include "_global.h"
//#include "texturematerialvertexshader.h"
#include "texturematerialvertexshaderhlsl.h"
//#include "texturematerialfragmentshader.h"
#include "texturematerialfragmentshaderhlsl.h"
#include "alternativa/gfx/core/programresource.h"
#include "alternativa/engine3d/objects/skybox.h"
#include "alternativa/engine3d/core/view.h"
#include "alternativa/gfx/core/device.h"
#include "alternativa/gfx/core/compressedtextureresource.h"
#include "alternativa/engine3d/core/shadowmap.h"
#include "alternativa/gfx/core/rendertargettextureresource.h"
#include "alternativa/engine3d/objects/decal.h"
#include "alternativa/engine3d/objects/sprite3d.h"
#include "file.h"
#include "hash.h"
#include "flash/display3D/context3d.h"
#include "flash/display/stage3d.h"
#include "flash/display3D/textures/texture.h"
#include "alternativa/tanks/engine3d/animatedsprite3d.h"
#include "utils/double_array.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"
#include <QVector>
#include <QHash>


const Material::T *TextureMaterial::TYPE = new Material::T("TextureMaterial");
static QHash<int, ProgramResource*> programs;


TextureMaterial::TextureMaterial(const T *type) :
	Material(type)
{

}

TextureMaterial::~TextureMaterial()
{

}

void TextureMaterial::init(bool repeat,
						   bool smooth,
						   MipMapping mipMapping,
						   double resolution)
{
	static int id;
	id++;
	this->_id = id;


	this->numMaps = 0;
	this->correctUV = false;
	this->threshold = 0.01;
	this->bitmap = nullptr;
	this->_hardwareMipMaps = false;
	this->_textureATFAlpha = nullptr;

	this->repeat = repeat;
	this->smooth = smooth;
	this->_mipMapping = mipMapping;
	this->resolution = resolution;
	this->textureATFResource = nullptr;
	this->textureATFAlphaResource = nullptr;


	double_array_set_8(correctionConst,  0, 0, 0, 1,  0, 0, 0, 1);
	double_array_set_8(uvTransformConst,  1, 0, 0, 0,  0, 1, 0, 0);
	double_array_set_4(uvCorrection,  1, 1, 0, 1);
	double_array_set_4(fragmentConst,  0, 0, 0, 1);

	textureResource = nullptr;
}

std::shared_ptr<BitmapData> TextureMaterial::getTexture()
{
	if (this->textureResource != nullptr)
	{
		return this->textureResource->bitmapData();
	}

	return nullptr;
}

BitmapData* TextureMaterial::getTexturePtr()
{
	if (this->textureResource != nullptr)
	{
		return this->textureResource->bitmapDataPtr();
	}

	return nullptr;
}

void TextureMaterial::setTexture(std::shared_ptr<BitmapData> bitmapData)
{
	std::shared_ptr<BitmapData> current = this->getTexture();
	if (bitmapData != current)
	{
		if (current != nullptr)
		{
			//this->textureResource->dispose();
			this->textureResource = nullptr;
			this->_textureResource = nullptr;
		}

		if (bitmapData != nullptr)
		{
			std::shared_ptr<BitmapTextureResource> textureResource = TextureResourcesRegistry::getTextureResource(bitmapData,
																												  this->_mipMapping != MipMapping::NONE,
																												  this->repeat,
																												  this->_hardwareMipMaps);
			setTextureResource(textureResource);
		}
	}
}





void TextureMaterial::disposeResource()
{
	if (this->textureResource != nullptr)
	{
		this->textureResource->dispose();
		this->textureResource = nullptr;
	}
}

bool TextureMaterial::getHardwareMipMaps()
{
	return this->_hardwareMipMaps;
}

void TextureMaterial::setHardwareMipMaps(bool param1)
{
	if (param1 != this->_hardwareMipMaps)
	{
		this->_hardwareMipMaps = param1;
		if (this->getTexture() != nullptr)
		{
			this->textureResource->setCalculateMipMapsUsingGPU(this->_hardwareMipMaps);
		}
	}
}

void TextureMaterial::clonePropertiesFromTextureMaterial(TextureMaterial *src) //override
{
	Material::clonePropertiesFromMaterial(src);
	TextureMaterial *_src = (TextureMaterial*)src;
	this->diffuseMapURL = _src->diffuseMapURL;
	this->opacityMapURL = _src->opacityMapURL;
	this->threshold = _src->threshold;
	this->correctUV = _src->correctUV;
	//this->textureATF = _src->textureATF;
	//this->textureATFAlpha = _src->textureATFAlpha;
	//this->hardwareMipMaps = _src->hardwareMipMaps;
}

bool TextureMaterial::transparent() //override
{
	if (Material::transparent())
		return true;

	if (this->getTexture() != nullptr)
		return this->getTexture()->transparent();

	//if (this->_textureATF != nullptr)
	//return (!(this->_textureATFAlpha == nullptr));
	return false;
}

void TextureMaterial::drawOpaque(Camera3D *camera, VertexBufferResource *vertexBuffer, IndexBufferResource *indexBuffer, int firstIndex, int numTriangles, Object3D *object)
{
	BitmapData *bitmapData = this->getTexturePtr();

	if (bitmapData == nullptr /*&& this->_textureATF == nullptr*/)
	{
		return;
	}

	Device *device = camera->device;


	bool decalType = object->isType(Decal::TYPE);
	bool zEnable = !decalType && this->zOffset;


	bool skyBoxAutoSize = object->isType(SkyBox::TYPE) && ((SkyBox*)object)->autoSize;


	bool fog = (camera->fogAlpha > 0) && (camera->fogStrength > 0);
	bool ssao = (((((!(camera->view->constrained)) && (camera->ssao)) && (camera->ssaoStrength > 0)) && (object->useDepth)) && !skyBoxAutoSize);
	bool light = (((((!(camera->view->constrained)) && (!(camera->directionalLight == nullptr))) && (camera->directionalLightStrength > 0)) && (object->useLight)) && (!(skyBoxAutoSize)));
	bool shadowMap = ((((((!(camera->view->constrained)) && (!(camera->shadowMap == nullptr))) && (camera->shadowMapStrength > 0)) && (object->useLight)) && (object->useShadowMap)) && (!(skyBoxAutoSize)));
	bool lighting = ((((((!(camera->view->constrained)) && (camera->deferredLighting)) && (camera->deferredLightingStrength > 0)) && (object->useDepth)) && (object->useLight)) && (!(skyBoxAutoSize)));
	bool transparent = (alphaTestThreshold > 0) && this->transparent();


	device->setProgram(this->getProgram(!decalType && !transparent,
										skyBoxAutoSize,
										decalType || zEnable,
										false,
										camera->view->quality,
										this->repeat,
										this->_mipMapping != MipMapping::NONE,
										object->concatenatedColorTransform != nullptr,
										decalType && object->concatenatedAlpha < 1,
										fog,
										false,
										ssao,
										light,
										shadowMap,
										bitmapData == nullptr,
										false,
										lighting,
										false,
										camera->view->correction,
										object->concatenatedBlendMode != BlendMode::NORMAL,
										transparent,
										false));


	if (bitmapData != nullptr)
	{
		/*
		if (uploadEveryFrame && drawId != Camera3D::renderId)
		{
			device->uploadResource(this->textureResource);
			drawId = Camera3D::renderId;
		}
		*/
		device->setTextureAt(0, this->textureResource);
		uvCorrection[0] = this->textureResource->correctionU();
		uvCorrection[1] = this->textureResource->correctionV();
	}
	else
	{
		device->setTextureAt(0, this->textureATFResource);
		uvCorrection[0] = 1;
		uvCorrection[1] = 1;
	}

	if (ssao)
	{
		device->setTextureAt(1, camera->depthMap);
	}
	else
	{
		device->setTextureAt(1, nullptr);
	}

	if (shadowMap)
	{
		device->setTextureAt(2, camera->shadowMap->map);
		device->setTextureAt(3, camera->shadowMap->noise.get());
	}
	else
	{
		device->setTextureAt(2, nullptr);
		device->setTextureAt(3, nullptr);
	}

	device->setTextureAt(4, nullptr);
	device->setTextureAt(6, nullptr);

	if (lighting)
	{
		device->setTextureAt(5, camera->lightMap);
	}
	else
	{
		device->setTextureAt(5, nullptr);
	}

	device->setVertexBufferAt(0, vertexBuffer, 0, Context3DVertexBufferFormat::FLOAT_3);
	device->setVertexBufferAt(1, vertexBuffer, 3, Context3DVertexBufferFormat::FLOAT_2);

	if (light)
	{
		device->setVertexBufferAt(2, vertexBuffer, 5, Context3DVertexBufferFormat::FLOAT_3);
		device->setVertexBufferAt(3, nullptr);
	}
	else
	{
		device->setVertexBufferAt(2, nullptr);
	}

	device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, object->transformConst, 3, false);
	device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 4, uvCorrection, 1);


	if (decalType)
	{
		correctionConst[0] = object->matrix.md * camera->correctionX;
		correctionConst[1] = object->matrix.mh * camera->correctionY;
		correctionConst[2] = object->matrix.ml;
		correctionConst[3] = camera->correctionX;
		correctionConst[4] = (object->matrix.mc * camera->correctionX) / ((Decal*)object)->attenuation;
		correctionConst[5] = (object->matrix.mg * camera->correctionY) / ((Decal*)object)->attenuation;
		correctionConst[6] = object->matrix.mk / ((Decal*)object)->attenuation;
		correctionConst[7] = camera->correctionY;
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, correctionConst, 2, false);
	}
	else
	{
		if (zEnable)
		{
			double correctionConst[8];
			correctionConst[0] = 0;
			correctionConst[1] = 0;
			correctionConst[2] = 0;
			correctionConst[3] = camera->correctionX;
			correctionConst[4] = 0;
			correctionConst[5] = 0;
			correctionConst[6] = 0;
			correctionConst[7] = camera->correctionY;
			device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, correctionConst, 2, false);
		}
		else
		{
			if (skyBoxAutoSize)
			{
				device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, ((SkyBox*)object)->reduceConst, 1);
				if (fog)
				{
					double skyFogConst[4];
					skyFogConst[0] = camera->fogFragment[0] * camera->fogFragment[3];
					skyFogConst[1] = camera->fogFragment[1] * camera->fogFragment[3];
					skyFogConst[2] = camera->fogFragment[2] * camera->fogFragment[3];
					skyFogConst[3] = 1 - camera->fogFragment[3];
					device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 13, skyFogConst, 1);
				}
			}
		}
	}

	if (object->concatenatedColorTransform != nullptr)
	{
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, object->colorConst, 2, false);
	}
	else
	{
		if (decalType && (object->concatenatedAlpha < 1))
		{
			device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, object->colorConst, 1);
		}
	}

	if (transparent)
	{
		fragmentConst[3] = alphaTestThreshold;
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 14, fragmentConst, 1, false);
	}


	device->drawTriangles(indexBuffer, firstIndex, numTriangles);
	camera->numDraws++;
	camera->numTriangles = camera->numTriangles + numTriangles;
}


//#include <QDebug>
void TextureMaterial::drawTransparent(Camera3D *param1, VertexBufferResource *param2, IndexBufferResource *param3, int param4, int param5, Object3D *param6, bool param7)
{
	BitmapData *bitmapData = this->getTexturePtr();

	if ((bitmapData == nullptr)/* && (this->_textureATF == nullptr)*/)
	{
		return;
	}

	//if (bitmapData->width() == 256)
		//qDebug() << bitmapData->width() << bitmapData->height();

	Device *device = param1->device;
	bool _loc10_ = zOffset;
	bool _loc11_ = ((param1->fogAlpha > 0) && (param1->fogStrength > 0));
	bool _loc12_ = param6->isType(Sprite3D::TYPE);
	bool _loc13_ = ((((!(param1->view->constrained)) && (param1->softTransparency)) && (param1->softTransparencyStrength > 0)) && (param6->softAttenuation > 0));
	bool _loc14_ = ((((!(param1->view->constrained)) && (param1->ssao)) && (param1->ssaoStrength > 0)) && (param6->useDepth));
	bool _loc15_ = ((((!(param1->view->constrained)) && (!(param1->directionalLight == nullptr))) && (param1->directionalLightStrength > 0)) && (param6->useLight));
	bool _loc16_ = (((((!(param1->view->constrained)) && (!(param1->shadowMap == nullptr))) && (param1->shadowMapStrength > 0)) && (param6->useLight)) && (param6->useShadowMap));
	bool _loc17_ = (((!(param1->view->constrained)) && (param1->deferredLighting)) && (param1->deferredLightingStrength > 0));
	bool _loc18_ = ((((_loc17_) && (param6->useDepth)) && (param6->useLight)) && (!(_loc12_)));
	bool _loc19_ = (((_loc17_) && (_loc12_)) && (param6->useLight));

	device->setProgram(this->getProgram(false,
										false,
										_loc10_,
										_loc12_,
										param1->view->quality,
										this->repeat,
										this->_mipMapping != MipMapping::NONE,
										(!(param6->concatenatedColorTransform == nullptr)),
										(param6->concatenatedAlpha < 1),
										_loc11_,
										_loc13_,
										_loc14_,
										_loc15_,
										_loc16_,
										bitmapData == nullptr,
										(bitmapData == nullptr) && (!(this->_textureATFAlpha == nullptr)),
										_loc18_,
										_loc19_,
										false, //param1->view->correction,
										(!(param6->concatenatedBlendMode == BlendMode::NORMAL)),
										false,
										param7));

	if (bitmapData != nullptr)
	{
		/*
		if (uploadEveryFrame && drawId != Camera3D::renderId)
		{
			device->uploadResource(this->textureResource);
			drawId = Camera3D::renderId;
		}
		*/


		device->setTextureAt(0, this->textureResource);
		uvCorrection[0] = this->textureResource->correctionU();
		uvCorrection[1] = this->textureResource->correctionV();
	}
	else
	{
		device->setTextureAt(0, this->textureATFResource);
		if (this->_textureATFAlpha != nullptr)
		{
			device->setTextureAt(4, this->textureATFAlphaResource);
		}
		else
		{
			device->setTextureAt(4, nullptr);
		}
		uvCorrection[0] = 1;
		uvCorrection[1] = 1;
	}

	if (_loc14_ || _loc13_)
	{
		device->setTextureAt(1, param1->depthMap);
	}
	else
	{
		device->setTextureAt(1, nullptr);
	}

	if (_loc16_)
	{
		device->setTextureAt(2, param1->shadowMap->map);
		device->setTextureAt(3, param1->shadowMap->noise.get());
	}
	else
	{
		device->setTextureAt(2, nullptr);
		device->setTextureAt(3, nullptr);
	}

	device->setTextureAt(4, nullptr);
	device->setTextureAt(6, nullptr);

	if (_loc18_)
	{
		device->setTextureAt(5, param1->lightMap);
	}
	else
	{
		device->setTextureAt(5, nullptr);
	}

	device->setVertexBufferAt(0, param2, 0, Context3DVertexBufferFormat::FLOAT_1);
	device->setVertexBufferAt(1, nullptr);
	device->setVertexBufferAt(2, nullptr);

	if (!_loc12_)
	{
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, param6->transformConst, 3, false);
	}

	device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 4, uvCorrection, 1);

	if (_loc13_)
	{
		fragmentConst[2] = param6->softAttenuation;
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 14, fragmentConst, 1);
	}

	if (_loc10_)
	{
		correctionConst[0] = 0;
		correctionConst[1] = 0;
		correctionConst[2] = 0;
		correctionConst[3] = param1->correctionX;
		correctionConst[4] = 0;
		correctionConst[5] = 0;
		correctionConst[6] = 0;
		correctionConst[7] = param1->correctionY;
		device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, correctionConst, 2, false);
	}
	else
	{
		if (_loc12_)
		{
			if (_loc15_)
			{
				correctionConst[0] = param1->correctionX;
				correctionConst[1] = param1->correctionY;
				correctionConst[2] = 1;
				correctionConst[3] = 0.5;
				device->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, correctionConst, 1, false);
			}

			if (_loc19_)
			{
				device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 13, ((Sprite3D*)param6)->lightConst, 1, false);
			}
		}
	}

	if (param6->concatenatedColorTransform != nullptr)
	{
		device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, param6->colorConst, 2, false);
	}
	else
	{
		if (param6->concatenatedAlpha < 1)
		{
			device->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, param6->colorConst, 1);
		}
	}

	device->drawTriangles(param3, param4, param5);
	param1->numDraws++;
	param1->numTriangles = param1->numTriangles + param5;
}


ProgramResource* TextureMaterial::getProgram(bool param1,
											 bool param2,
											 bool param3,
											 bool param4,
											 bool param5,
											 bool param6,
											 bool param7,
											 bool param8,
											 bool param9,
											 bool param10,
											 bool param11,
											 bool param12,
											 bool param13,
											 bool param14,
											 bool param15,
											 bool param16,
											 bool param17,
											 bool param18,
											 bool param19,
											 bool param20,
											 bool param21,
											 bool param22)
{
	int key = int(param1) | (int(param2) << 1) | (int(param3) << 2) | (int(param4) << 3) | (int(param5) << 4) | (int(param6) << 5) | (int(param7) << 6) | (int(param8) << 7) | (int(param9) << 8) | (int(param10) << 9) |
			(int(param11) << 10) | (int(param12) << 11) | (int(param13) << 12) | (int(param14) << 13) | (int(param15) << 14) | (int(param16) << 15) | (int(param17) << 16) | (int(param18) << 17) | (int(param19) << 18) |
			(int(param20) << 19) | (int(param21) << 20) | (int(param22) << 21);

	ProgramResource *programResource = programs[key];

	if (programResource == nullptr)
	{
		/*
		QByteArray vertexShaderData = TextureMaterialVertexShader(!param22,
																  (((param14 || param11) || param12) || param17),
																  param13,
																  param4,
																  param14,
																  param10,
																  param2,
																  param3,
																  param3,
																  param19);

		ShaderSampler sampler;
		QByteArray fragmentShaderData = TextureMaterialFragmentShader(sampler,
																	  param6,
																	  param5,
																	  param7,
																	  param15,
																	  param16,
																	  param21,
																	  (((!(param1)) && (!(param16))) && (!(param15))),
																	  param8,
																	  param9,
																	  param3,
																	  param13,
																	  param11,
																	  param12,
																	  param17,
																	  param18,
																	  param14,
																	  param10,
																	  param2,
																	  param20);
	*/



		QByteArray vertexShaderData_hlsl = TextureMaterialVertexShaderHLSL(!param22,
																  (((param14 || param11) || param12) || param17),
																  param13,
																  param4,
																  param14,
																  param10,
																  param2,
																  param3,
																  param3,
																  param19);





		ShaderSampler sampler2;
		QByteArray fragmentShaderData_hlsl = TextureMaterialFragmentShaderHLSL(sampler2,
																			   param6,
																			   param5,
																			   param7,
																			   param15,
																			   param16,
																			   param21,
																			   (((!(param1)) && (!(param16))) && (!(param15))),
																			   param8,
																			   param9,
																			   param3,
																			   param13,
																			   param11,
																			   param12,
																			   param17,
																			   param18,
																			   param14,
																			   param10,
																			   param2,
																			   param20);



		//programResource = new ProgramResource(ProgramResource::TYPE::AGAL, vertexShaderData, fragmentShaderData, sampler);
		programResource = new ProgramResource(1, ProgramResource::TYPE::HLSL, vertexShaderData_hlsl, fragmentShaderData_hlsl, sampler2);
		programs[key] = programResource;
	}
	return programResource;
}

void TextureMaterial::dispose()
{
	this->disposeResource();
}

std::shared_ptr<TextureMaterial> new_TextureMaterial()
{
	auto p = std::make_shared<TextureMaterial>(TextureMaterial::TYPE);
	p->ptr = p;
	return p;
}

std::shared_ptr<TextureMaterial> TextureMaterial_clone(std::shared_ptr<TextureMaterial> src)
{
	auto out = new_TextureMaterial();
	out->init(src->repeat, src->smooth, src->_mipMapping, src->resolution);
	out->setTexture(src->getTexture());
	out->clonePropertiesFromTextureMaterial(src.get());
	return out;
}
