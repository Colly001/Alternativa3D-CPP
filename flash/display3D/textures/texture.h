#ifndef TEXTURE_H
#define TEXTURE_H

class BitmapData;
class Context3D;
class IDirect3DTexture9;
class IDirect3DDevice9Ex;

#include "texturebase.h"
#include <stdint.h>

class Texture : public TextureBase
{
	Context3D *context;
	IDirect3DDevice9Ex *device;
	IDirect3DTexture9 *_texture;

public:
	Texture(Context3D *context, IDirect3DTexture9 *texture, IDirect3DSurface9* pSurface, IDirect3DSurface9* pDepthStencilSurface);
	virtual ~Texture();


	//void  uploadCompressedTextureFromByteArray(const QByteArray &data, uint32_t byteArrayOffset, bool async = false)
	//Uploads a compressed texture in Adobe Texture Format (ATF) from a ByteArray object.
	//Texture

	bool uploadFromBitmapData(BitmapData *source, uint32_t miplevel = 0);
	//Uploads a texture from a BitmapData object.
	//Texture

	//void uploadFromBitmapDataAsync(BitmapData *source, uint32_t miplevel = 0)
	//Uploads miplevel 0 for a texture from a BitmapData object asynchronously.
	//Texture

	//void uploadFromByteArray(const QByteArray &data, uint32_t byteArrayOffset, uint32_t miplevel = 0)
	//Uploads a texture from a ByteArray.
	//Texture

	//void uploadFromByteArrayAsync(const QByteArray &data, uint32_t byteArrayOffset, uint32_t miplevel = 0)
	//Uploads miplevel 0 for a texture from a ByteArray asynchronously.
	void clear();
};

#endif // TEXTURE_H
