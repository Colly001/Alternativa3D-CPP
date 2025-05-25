#include "indexbuffer3d.h"
#include <d3dx9core.h>
#include <QDebug>


IndexBuffer3D::IndexBuffer3D(IDirect3DDevice9Ex *device, IDirect3DIndexBuffer9 *pIndexBuffer)
{
	this->device = device;
	this->pIndexBuffer = pIndexBuffer;
}

void IndexBuffer3D::dispose()
{
	if (pIndexBuffer != nullptr) {
		pIndexBuffer->Release();
		pIndexBuffer = nullptr;
	}
}

void IndexBuffer3D::uploadFromVector(const QVector<uint32_t> &data, int startOffset, int count)
{
	(void)startOffset;

	uint16_t *_data;
	HRESULT r = pIndexBuffer->Lock(0, count * sizeof(uint16_t), (void**)&_data, 0);
	if (r != S_OK) {
		qDebug() << "error Lock";
		return;
	}

	for (int i = 0; i < count; i++)
		_data[i] = data.at(i);

	//memcpy(_data, data, SizeToLock);
	pIndexBuffer->Unlock();
}

void IndexBuffer3D::uploadFromVector(const uint16_t *data, int startOffset, int count)
{
	(void)startOffset;

	uint16_t *_data;
	HRESULT r = pIndexBuffer->Lock(0, count * sizeof(uint16_t), (void**)&_data, 0);
	if (r != S_OK) {
		qDebug() << "error Lock";
		return;
	}

	for (int i = 0; i < count; i++)
		_data[i] = data[i];

	//memcpy(_data, data, SizeToLock);
	pIndexBuffer->Unlock();
}
