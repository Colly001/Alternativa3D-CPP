#include "rendertargettextureresource.h"
#include "flash/display3D/Context3DTextureFormat.h"
#include "flash/display3D/context3d.h"


RenderTargetTextureResource::RenderTargetTextureResource(int width, int height)
{
	this->_width = width;
	this->_height = height;
	this->_available = true;
}

void RenderTargetTextureResource::dispose()
{
	TextureResource::dispose();
	this->_available = false;
}

void RenderTargetTextureResource::create(Context3D *context)
{
	TextureResource::create(context);
	std::shared_ptr<Texture> texture = context->createTexture(this->_width, this->_height, Context3DTextureFormat::BGRA, true);
	this->setTexture(texture);
}
