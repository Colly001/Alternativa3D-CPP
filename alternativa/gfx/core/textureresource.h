#ifndef TEXTURERESOURCE_H
#define TEXTURERESOURCE_H

#include "resource.h"
#include <memory>

class Texture;

class TextureResource : public Resource
{
	std::shared_ptr<Texture> _texture;

public:
	TextureResource();
	void dispose() override;
	Texture* getTexture();
	void setTexture(std::shared_ptr<Texture> &texture);

protected:
	bool useNullTexture;
	Texture* getNullTexture();
};

#endif // TEXTURERESOURCE_H
