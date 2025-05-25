#ifndef VERTEXBUFFER3D_H
#define VERTEXBUFFER3D_H

#include <QVector>

class IDirect3DDevice9Ex;
class IDirect3DVertexBuffer9;

class VertexBuffer3D
{
	IDirect3DDevice9Ex *device;
	IDirect3DVertexBuffer9 *pVertexBuffer;
	int numVertices;
	int data32PerVertex;

public:
	VertexBuffer3D(IDirect3DDevice9Ex *device, IDirect3DVertexBuffer9 *pVertexBuffer, int numVertices, int data32PerVertex);
	~VertexBuffer3D();


	IDirect3DVertexBuffer9* get() { return pVertexBuffer; }

	int getNumVertices() { return numVertices; }
	int getData32PerVertex() { return data32PerVertex; }

	//not used
	//void uploadFromByteArray(ByteArray data, int byteArrayOffset, int startVertex, int numVertices);


	bool uploadFromVector(QVector<double> &data, int startVertex, int numVertices);
	bool uploadFromVector(const float *data, int startVertex, int numVertices);
};

#endif // VERTEXBUFFER3D_H
