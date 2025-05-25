#include "texture.h"
#include "../context3d.h"
#include "flash/display/bitmapdata.h"
#include <d3dx9core.h>
#include <QImage>
#include <QDebug>
#include "file.h"
#include "hash.h"


Texture::Texture(Context3D *context, IDirect3DTexture9 *texture, IDirect3DSurface9* pSurface, IDirect3DSurface9* pDepthStencilSurface)
{
	this->context = context;
	this->device = context->getDevice();
	this->_texture = texture;
	this->setTexturBase(texture);
	this->setSurface(pSurface);
	this->setDepthStencilSurface(pDepthStencilSurface);
}

Texture::~Texture()
{
	if (this->_texture != nullptr)
	{
		this->_texture->Release();
		this->_texture = nullptr;
	}

	if (this->pDepthStencilSurface != nullptr)
	{
		this->pDepthStencilSurface->Release();
		this->pDepthStencilSurface = nullptr;
	}

	if (this->pSurface != nullptr)
	{
		this->pSurface->Release();
		this->pSurface = nullptr;
	}
}

bool Texture::uploadFromBitmapData(BitmapData *source, uint32_t miplevel)
{	
	HRESULT r;

	UINT width = source->width();
	UINT height = source->height();

	IDirect3DSurface9* pSurfaceLevel;
	r = _texture->GetSurfaceLevel(miplevel, &pSurfaceLevel);
	if (r != S_OK) {
		qDebug("error GetSurfaceLevel %08lX", r);
		return false;
	}

	IDirect3DSurface9* _pSurface;

	r = device->CreateOffscreenPlainSurface(width, height, D3DFMT_A8R8G8B8, D3DPOOL_SYSTEMMEM, &_pSurface, nullptr);
	if (r != S_OK) {
		qDebug("error CreateOffscreenPlainSurface %08lX", r);
		return false;
	}

	RECT _recv = {0, 0, (LONG)width, (LONG)height};

	D3DLOCKED_RECT locked_rect;

	r = _pSurface->LockRect(&locked_rect, &_recv, 0);

	if (r != S_OK) {
		qDebug("error LockRect %08lX", r);
		return false;
	}

	DWORD *array = (DWORD*)locked_rect.pBits;

	memcpy(array, source->data(), source->width() * source->height() * 4);

	_pSurface->UnlockRect();

	r = device->UpdateSurface(_pSurface, nullptr, pSurfaceLevel, nullptr);

	_pSurface->Release();

	if (r != S_OK) {
		qDebug("error UpdateSurface %08lX", r);
		return false;
	}

	return true;
}
