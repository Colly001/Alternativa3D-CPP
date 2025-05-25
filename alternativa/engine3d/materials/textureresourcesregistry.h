#ifndef TEXTURERESOURCESREGISTRY_H
#define TEXTURERESOURCESREGISTRY_H

#include <memory>

class BitmapTextureResource;
class BitmapData;

class TextureResourcesRegistry
{
public:

	static std::shared_ptr<BitmapTextureResource> getTextureResource(std::shared_ptr<BitmapData> bitmapData, bool mipMapping, bool stretchNotPowerOf2Textures, bool calculateMipMapsUsingGPU);
	static void releaseTextureResources();
	static void release(std::shared_ptr<BitmapData> bitmapData);
	static std::shared_ptr<BitmapTextureResource> getTextureResourceNew(std::shared_ptr<BitmapData> bitmapData, bool mipMapping, bool stretchNotPowerOf2Textures, bool calculateMipMapsUsingGPU);

	static void clear();
};

#endif // TEXTURERESOURCESREGISTRY_H
