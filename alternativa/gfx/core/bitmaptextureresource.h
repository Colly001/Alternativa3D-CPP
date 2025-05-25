#ifndef BITMAPTEXTURERESOURCE_H
#define BITMAPTEXTURERESOURCE_H

class BitmapData;
class Texture;
class Context3D;

#include <stdint.h>
#include <memory>
#include "textureresource.h"


class BitmapTextureResource : public TextureResource
{
	int referencesCount;
	std::shared_ptr<BitmapData> _bitmapData;
	bool _mipMapping;
	bool _stretchNotPowerOf2Textures;
	bool _calculateMipMapsUsingGPU;
	double _correctionU;
	double _correctionV;
	int correctedWidth;
	int correctedHeight;
	bool loaded;

public:

	BitmapTextureResource(std::shared_ptr<BitmapData> bitmapData, bool mipMapping, bool stretchNotPowerOf2Textures = false, bool calculateMipMapsUsingGPU = false);
	virtual ~BitmapTextureResource();

	std::shared_ptr<BitmapData> bitmapData();
	bool mipMapping();
	bool getStretchNotPowerOf2Textures();
	double correctionU() { return this->_correctionU; }
	double correctionV() { return this->_correctionV; }
	bool getCalculateMipMapsUsingGPU();
	void setCalculateMipMapsUsingGPU(bool param1);
	void forceDispose();
	void dispose() override;
	bool available() const override;
	bool isLoaded() override;

protected:
	Texture *getNullTexture();

private:
	void freeMemory();
	void create(Context3D*) override;
	bool upload() override;

protected:
	void uploadTexture(BitmapData *bitmapData, uint32_t mipLevel);

public:
	void increaseReferencesCount();
	void destroy(bool b);
	BitmapData *bitmapDataPtr();
	void updateResource();

	static void clear();
};

#endif // BITMAPTEXTURERESOURCE_H
