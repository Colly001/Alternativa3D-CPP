#include "textureresource.h"
#include "flash/display3D/textures/texture.h"

TextureResource::TextureResource()
{
	useNullTexture = false;
	_texture = nullptr;
}

void TextureResource::dispose() //override
{
	Resource::dispose();
	this->_texture = nullptr;
}

Texture* TextureResource::getTexture()
{
	if (this->useNullTexture)
	{
		return this->getNullTexture();
	}
	return this->_texture.get();
}

void TextureResource::setTexture(std::shared_ptr<Texture> &texture)
{
	this->_texture = texture;
}

Texture* TextureResource::getNullTexture()
{
	return nullptr;
}
