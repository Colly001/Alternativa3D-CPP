#include "stage3d.h"
#include <d3dx9.h>
#include <d3d9.h>
#include <d3dx9core.h>
#include <d3dx9tex.h>
#include <d3d9types.h>
#include "mymath.h"
#include "../display3D/context3d.h"
#include "stage.h"
#include "../display3D/directxdata.h"
#include <QDebug>


class Stage3DPrivate
{
	Stage3D *d;
	Stage *stage;
public:

	HWND hwnd;

	Context3D *context;

	IDirect3D9Ex *pIDirect3D9Ex;
	DirectXData data;

	Stage3DPrivate(Stage3D *d, Stage *stage, ImageNodes *imageNodes)
	{
		this->d = d;
		this->stage = stage;
		this->data.imageNodes = imageNodes;

		this->hwnd = nullptr;
		this->context = nullptr;
		this->pIDirect3D9Ex = nullptr;
		this->data.device = nullptr;
		this->data.pSwapChain = nullptr;
		this->data.pBackBuffer = nullptr;

		this->data.texture_1 = nullptr;
		this->data.texture_2 = nullptr;
		this->data.CreateRenderTarget_surface = nullptr;
		this->data.CreateDepthStencilSurface_surface = nullptr;
	}

	~Stage3DPrivate()
	{
		if (this->data.pBackBuffer != nullptr)
			this->data.pBackBuffer->Release();

		if (this->data.pSwapChain != nullptr)
			this->data.pSwapChain->Release();

		if (this->data.device != nullptr)
			this->data.device->Release();

		if (this->pIDirect3D9Ex != nullptr)
			this->pIDirect3D9Ex->Release();
	}


	void setHWND(HWND hwnd)
	{
		this->hwnd = hwnd;
	}

	bool createContext3D()
	{
		if (Direct3DCreate9Ex(D3D_SDK_VERSION, &pIDirect3D9Ex) != S_OK) {
			qDebug("error Direct3DCreate9Ex");
			return false;
		}

		D3DPRESENT_PARAMETERS direct3dparametr;

		direct3dparametr.BackBufferWidth = 1;
		direct3dparametr.BackBufferHeight = 1;
		direct3dparametr.BackBufferFormat = D3DFMT_X8R8G8B8;
		direct3dparametr.BackBufferCount = 1;
		direct3dparametr.MultiSampleType = D3DMULTISAMPLE_NONE;
		direct3dparametr.MultiSampleQuality = 0;
		direct3dparametr.SwapEffect = D3DSWAPEFFECT_COPY;
		direct3dparametr.hDeviceWindow = hwnd;
		direct3dparametr.Windowed = TRUE;
		direct3dparametr.EnableAutoDepthStencil = FALSE;
		direct3dparametr.AutoDepthStencilFormat = D3DFMT_UNKNOWN;
		direct3dparametr.Flags = 0;
		direct3dparametr.FullScreen_RefreshRateInHz = 0;
		direct3dparametr.PresentationInterval = 0;


		HRESULT r = pIDirect3D9Ex->CreateDeviceEx(D3DADAPTER_DEFAULT,
												  D3DDEVTYPE_HAL,
												  hwnd,
												  D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE | D3DCREATE_MULTITHREADED,
												  &direct3dparametr,
												  nullptr,
												  &this->data.device);
		if (FAILED(r)) {
			qDebug("error CreateDeviceEx %08lX", r);
			return false;
		}

		this->data.device->Clear(0, nullptr, 1, D3DCOLOR(0xFFFFFFFF), 0.f, 0);

		context = new Context3D(&this->data);

		this->setSize(100, 100);

		return true;
	}


	bool setSize(int width, int height)
	{
		this->data.width = width;
		this->data.height = height;

		if (this->data.device == nullptr)
			return false;

		if (this->data.pSwapChain != nullptr)
			this->data.pSwapChain->Release();
		this->data.pSwapChain = nullptr;

		if (this->data.pBackBuffer != nullptr)
			this->data.pBackBuffer->Release();
		this->data.pBackBuffer = nullptr;

		HRESULT r;

		{
			D3DPRESENT_PARAMETERS direct3dparametr;

			direct3dparametr.BackBufferWidth = width;
			direct3dparametr.BackBufferHeight = height;
			direct3dparametr.BackBufferFormat = D3DFMT_X8R8G8B8; //22
			direct3dparametr.BackBufferCount = 1;
			direct3dparametr.MultiSampleType = D3DMULTISAMPLE_NONE;
			direct3dparametr.MultiSampleQuality = 0;
			direct3dparametr.SwapEffect = D3DSWAPEFFECT_COPY; //3
			direct3dparametr.hDeviceWindow = hwnd;
			direct3dparametr.Windowed = TRUE;
			direct3dparametr.EnableAutoDepthStencil = FALSE;
			direct3dparametr.AutoDepthStencilFormat = D3DFMT_UNKNOWN;
			direct3dparametr.Flags = 0;
			direct3dparametr.FullScreen_RefreshRateInHz = 0;
			direct3dparametr.PresentationInterval = 0;

			r = this->data.device->CreateAdditionalSwapChain(&direct3dparametr, &this->data.pSwapChain);
			if (FAILED(r)) {
				qDebug("error CreateAdditionalSwapChain %08lX", r);
				return false;
			}

		}

		r = this->data.pSwapChain->GetBackBuffer(0, D3DBACKBUFFER_TYPE_MONO, &this->data.pBackBuffer);
		if (FAILED(r)) {
			qDebug("error GetBackBuffer %08lX", r);
			return false;
		}


		if (this->data.CreateDepthStencilSurface_surface != nullptr)
			this->data.CreateDepthStencilSurface_surface->Release();

		if (this->data.texture_1 != nullptr)
			this->data.texture_1->Release();

		if (this->data.texture_2 != nullptr)
			this->data.texture_2->Release();



		//D3DUSAGE_DYNAMIC
		r = this->data.device->CreateTexture(width, height, 1, D3DUSAGE_DYNAMIC, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &this->data.texture_1, NULL);
		if (r != S_OK) {
			qDebug("error CreateTexture");
			return false;
		}

		//StretchRect
		r = this->data.device->CreateTexture(width, height, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &this->data.texture_2, NULL);
		if (r != S_OK) {
			qDebug("error CreateTexture");
			return false;
		}




		this->data.texture_2->GetSurfaceLevel(0, &data.pSurfaceLevel);
		//data.pSurfaceLevel->AddRef();

		if (this->data.CreateRenderTarget_surface != nullptr)
		{
			const RECT _rect = {0, 0, width, height};
			this->data.device->StretchRect(this->data.CreateRenderTarget_surface, &_rect, data.pSurfaceLevel, &_rect, D3DTEXF_POINT);

			//this->data.CreateRenderTarget_surface->Release();
		}


		//CreateRenderTarget 1065 630 21 0 0 1 0x0
		//CreateRenderTarget 274 208 21 0 0 1 0x0
		this->data.device->CreateRenderTarget(width, height, D3DFMT_A8R8G8B8, D3DMULTISAMPLE_NONE, 0, TRUE, &this->data.CreateRenderTarget_surface, nullptr);

		//CreateDepthStencilSurface 1065 630 75 0 0 0 0x0
		this->data.device->CreateDepthStencilSurface(width, height, D3DFMT_D24S8, D3DMULTISAMPLE_NONE, 0, FALSE, &this->data.CreateDepthStencilSurface_surface, nullptr);



		this->data.device->SetRenderTarget(0, this->data.CreateRenderTarget_surface);
		this->data.device->SetDepthStencilSurface(this->data.CreateDepthStencilSurface_surface);


		return true;
	}
};


Stage3D::Stage3D(Stage *stage, ImageNodes *imageNodes)
{
	this->stage = stage;
	this->d = new Stage3DPrivate(this, stage, imageNodes);
	this->_x = 0;
	this->_y = 0;
}

Stage3D::~Stage3D()
{
	delete d;
}

void Stage3D::setHWND(void *hwnd)
{
	d->setHWND((HWND)hwnd);
}

bool Stage3D::createContext3D()
{
	return this->d->createContext3D();
}

Context3D* Stage3D::context3D()
{
	return this->d->context;
}

bool Stage3D::getVisible()
{
	return true;
}

void Stage3D::setPosition(double x, double y)
{
	this->_x = x;
	this->_y = y;
}

double Stage3D::x()
{
	return this->_x;
}

double Stage3D::y()
{
	return this->_y;
}

void Stage3D::setSize(int width, int height)
{
	d->setSize(width, height);
}

void Stage3D::requestContext3D(Context3DRenderMode context3DRenderMode, Context3DProfile profile)
{
	(void)context3DRenderMode;
	(void)profile;
}

IDirect3DDevice9Ex* Stage3D::getIDirect3DDevice()
{
	return this->d->data.device;
}
