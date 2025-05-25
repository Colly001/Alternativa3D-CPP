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
#include "texturematerialvertexshader.h"
#include "texturematerialfragmentshader.h"
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
#include <QVector>
#include <QHash>


const Material::T *TextureMaterial::TYPE = new Material::T("TextureMaterial");



TextureMaterial::TextureMaterial(const T *type) :
	Material(type)
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
	this->resolution = resolution;
	this->textureATFResource = nullptr;
	this->textureATFAlphaResource = nullptr;

	textureResource = nullptr;
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

	textureResource = nullptr;
}


std::shared_ptr<BitmapData> TextureMaterial::getTexture()
{
	return nullptr;
}

BitmapData* TextureMaterial::getTexturePtr()
{
	return nullptr;
}

void TextureMaterial::setTexture(std::shared_ptr<BitmapData>)
{

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

void TextureMaterial::setHardwareMipMaps(bool)
{

}

void TextureMaterial::clonePropertiesFromTextureMaterial(TextureMaterial *src) //override
{
	Material::clonePropertiesFromMaterial(src);
	TextureMaterial *_src = (TextureMaterial*)src;
	this->diffuseMapURL = _src->diffuseMapURL;
	this->opacityMapURL = _src->opacityMapURL;
	this->threshold = _src->threshold;
	this->correctUV = _src->correctUV;
}

bool TextureMaterial::transparent() //override
{
	return false;
}




void TextureMaterial::drawOpaque(Camera3D*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*)
{

}


void TextureMaterial::drawTransparent(Camera3D*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*, bool)
{

}


ProgramResource *TextureMaterial::getProgram(bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool,
											 bool)
{
	return nullptr;
}

void TextureMaterial::dispose()
{
	this->disposeResource();
	//delete this[this];
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
