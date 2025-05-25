#include "imageitem.h"
#include "flash/display/bitmapdata.h"
#include <d3dx9core.h>
#include <QImage>
#include "flash/display/bitmapdata.h"
#include <QDebug>


ImageItem::ImageItem(BitmapData *data, int level)
{
	this->data = data;
	this->width = this->data->width();
	this->height = this->data->height();
	this->level = level;
	this->texture = nullptr;
}

ImageItem::~ImageItem()
{
	if (this->texture != nullptr)
		this->texture->Release();
}

bool ImageItem::createTexture(IDirect3DDevice9Ex *device)
{
	HRESULT r = device->CreateTexture(this->width, this->height, 1, D3DUSAGE_DYNAMIC, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &this->texture, NULL);
	return r == S_OK;
}


static uint32_t getPixel(uint32_t *d, int w, int x, int y)
{
	return d[(y * w) + x];
}

static void setPixel(uint32_t *d, int w, int x, int y, uint32_t color)
{
	d[(y * w) + x] = color;
}


void ImageItem::upload()
{
	IDirect3DSurface9* pSurfaceLevel;
	texture->GetSurfaceLevel(0, &pSurfaceLevel);

	const RECT rect = {0, 0, this->width, this->height};
	D3DLOCKED_RECT lock_rect;
	HRESULT result = pSurfaceLevel->LockRect(&lock_rect, &rect, 0);
	if (result != S_OK)
		return;

	QImage *image = data->qimage();
	uint32_t *data = (uint32_t*)image->constBits();

	int ww = lock_rect.Pitch / 4;

	for (int _x = 0; _x < this->width; _x++)
	{
		for (int _y = 0; _y < this->height; _y++)
		{
			uint32_t pixel = getPixel(data, this->width, _x, _y);
			setPixel((uint32_t*)lock_rect.pBits, ww, _x, _y, pixel);
		}
	}

	pSurfaceLevel->UnlockRect();
}
