#ifndef RENDERTARGETTEXTURERESOURCE_H
#define RENDERTARGETTEXTURERESOURCE_H

#include "textureresource.h"

class RenderTargetTextureResource : public TextureResource
{
	int _width;
	int _height;
	bool _available;

public:
	RenderTargetTextureResource(int width, int height);

	int getWidth() { return this->_width; }
	int getHeight() { return this->_height; }

	void dispose() override;
	bool available() const override { return this->_available; }

	void create(Context3D*) override;
};


#endif // RENDERTARGETTEXTURERESOURCE_H
