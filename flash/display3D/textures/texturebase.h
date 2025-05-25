#ifndef TEXTUREBASE_H
#define TEXTUREBASE_H

class IDirect3DBaseTexture9;
class IDirect3DSurface9;

class TextureBase
{
	IDirect3DBaseTexture9 *texture_base;

protected:

	IDirect3DSurface9* pSurface;
	IDirect3DSurface9* pDepthStencilSurface;

	void setTexturBase(IDirect3DBaseTexture9*);
	void setSurface(IDirect3DSurface9* pSurface);
	void setDepthStencilSurface(IDirect3DSurface9* pDepthStencilSurface);

public:

	TextureBase();
	virtual void dispose();

	IDirect3DBaseTexture9* getTexturBase() { return texture_base; }
	IDirect3DSurface9* getSurface() { return pSurface; }
	IDirect3DSurface9* getDepthStencilSurface() { return pDepthStencilSurface; }
};

#endif // TEXTUREBASE_H
