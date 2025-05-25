#include "textureresourcesregistry.h"
#include "alternativa/gfx/core/bitmaptextureresource.h"
#include "qhash_ptr.h"
#include <QHash>
#include <QHashIterator>


static QHash<std::shared_ptr<BitmapData>, std::shared_ptr<BitmapTextureResource>> texture2Resource;


std::shared_ptr<BitmapTextureResource> TextureResourcesRegistry::getTextureResource(std::shared_ptr<BitmapData> bitmapData,
																					bool mipMapping,
																					bool stretchNotPowerOf2Textures,
																					bool calculateMipMapsUsingGPU)
{
	if (texture2Resource.contains(bitmapData))
	{
		std::shared_ptr<BitmapTextureResource> resource = texture2Resource[bitmapData];
		resource->increaseReferencesCount();
		return resource;
	}

	std::shared_ptr<BitmapTextureResource> resource = std::make_shared<BitmapTextureResource>(bitmapData, mipMapping, stretchNotPowerOf2Textures, calculateMipMapsUsingGPU);
	texture2Resource[bitmapData] = resource;
	return resource;
}

std::shared_ptr<BitmapTextureResource> TextureResourcesRegistry::getTextureResourceNew(std::shared_ptr<BitmapData> bitmapData,
																					   bool mipMapping,
																					   bool stretchNotPowerOf2Textures,
																					   bool calculateMipMapsUsingGPU)
{
	return std::make_shared<BitmapTextureResource>(bitmapData, mipMapping, stretchNotPowerOf2Textures, calculateMipMapsUsingGPU);
}

void TextureResourcesRegistry::clear()
{
	texture2Resource.clear();
}

void TextureResourcesRegistry::releaseTextureResources()
{
	QHashIterator<std::shared_ptr<BitmapData>, std::shared_ptr<BitmapTextureResource>> i(texture2Resource);
	while (i.hasNext()) {
		i.next();
		i.value()->forceDispose();
	}
}

void TextureResourcesRegistry::release(std::shared_ptr<BitmapData> bitmapData)
{
	if (texture2Resource.contains(bitmapData))
	{
		texture2Resource.remove(bitmapData);
	}
}
