#include "vertexbuffer3d.h"
#include <d3dx9core.h>
#include <QDebug>


VertexBuffer3D::VertexBuffer3D(IDirect3DDevice9Ex *device, IDirect3DVertexBuffer9 *pVertexBuffer, int numVertices, int data32PerVertex)
{
	this->device = device;
	this->pVertexBuffer = pVertexBuffer;
	this->numVertices = numVertices;
	this->data32PerVertex = data32PerVertex;
}

VertexBuffer3D::~VertexBuffer3D()
{
	if (pVertexBuffer != nullptr)
	{
		pVertexBuffer->Release();
		pVertexBuffer = nullptr;
	}
}

bool VertexBuffer3D::uploadFromVector(QVector<double> &data, int startVertex, int numVertices)
{
	UINT SizeToLock = numVertices * data32PerVertex * sizeof(float);
	float *_data;

	HRESULT r = pVertexBuffer->Lock(startVertex, SizeToLock, (void**)&_data, 0);
	if (r != S_OK) {
		qDebug() << "error Lock" << r;
		return false;
	}

	for (int i = 0; i < (numVertices * data32PerVertex); i++)
		_data[i] = data.at(i);

	pVertexBuffer->Unlock();

	return true;
}


bool VertexBuffer3D::uploadFromVector(const float *data, int startVertex, int numVertices)
{
	UINT SizeToLock = numVertices * data32PerVertex * sizeof(float);
	float *_data;
	//(16 * 8) * 4 = 512
	//16 * 4 * 8 = 512
	//4900 MyIDirect3DVertexBuffer9_Lock 0x48eb000 0 512 0x28d0dc 0

	HRESULT r = pVertexBuffer->Lock(startVertex, SizeToLock, (void**)&_data, 0);
	if (r != S_OK) {
		qDebug() << "error Lock" << r;
		return false;
	}

	for (int i = 0; i < (numVertices * data32PerVertex); i++)
		_data[i] = data[i];

	pVertexBuffer->Unlock();

	return true;
}
