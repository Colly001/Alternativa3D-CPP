#ifndef DIRECTXDATA_H
#define DIRECTXDATA_H

class IDirect3DTexture9;
class IDirect3DSurface9;
class IDirect3DDevice9Ex;
class IDirect3DSwapChain9;
class ImageNodes;

class DirectXData
{
public:

	int width;
	int height;


	IDirect3DTexture9 *texture_1; //D3DUSAGE_DYNAMIC
	IDirect3DTexture9 *texture_2; //D3DUSAGE_RENDERTARGET
	IDirect3DSurface9 *CreateRenderTarget_surface;
	IDirect3DSurface9 *CreateDepthStencilSurface_surface;

	IDirect3DDevice9Ex *device;
	IDirect3DSwapChain9 *pSwapChain;
	IDirect3DSurface9 *pBackBuffer;
	IDirect3DSurface9* pSurfaceLevel;

	ImageNodes *imageNodes;

	DirectXData();
};

#endif // DIRECTXDATA_H
