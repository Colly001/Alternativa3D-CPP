#ifndef TEXTURECONSTRUCTOR_H
#define TEXTURECONSTRUCTOR_H

#include <memory>

class ITextureConstructorListener;
class BitmapData;
class TextureByteData;
class QByteArray;


class TextureConstructor
{
	std::shared_ptr<BitmapData> _texture;
	ITextureConstructorListener *listener;
	std::shared_ptr<TextureByteData> textureData;
	bool cancelled;

public:
	TextureConstructor()
	{

	}

	std::shared_ptr<BitmapData> texture()
	{
		return this->_texture;
	}

	void cancel();
	void createTexture(std::shared_ptr<TextureByteData> &textureData, ITextureConstructorListener *listener);

private:
	void onDiffuseTextureLoadingComplete();
	void onAlphaTextureLoadingComplete();
	void complete();
};

#endif // TEXTURECONSTRUCTOR_H
