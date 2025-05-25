#include "texturebase.h"

void TextureBase::setTexturBase(IDirect3DBaseTexture9 *texture_base)
{
	this->texture_base = texture_base;
}

void TextureBase::setSurface(IDirect3DSurface9 *pSurface)
{
	this->pSurface = pSurface;
}

void TextureBase::setDepthStencilSurface(IDirect3DSurface9 *pDepthStencilSurface)
{
	this->pDepthStencilSurface = pDepthStencilSurface;
}

TextureBase::TextureBase()
{
	texture_base = nullptr;
	pSurface = nullptr;
	pDepthStencilSurface = nullptr;
}

void TextureBase::dispose()
{

}
