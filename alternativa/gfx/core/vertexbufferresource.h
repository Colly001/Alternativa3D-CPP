#ifndef VERTEXBUFFERRESOURCE_H
#define VERTEXBUFFERRESOURCE_H

#include "resource.h"
#include <QVector>
#include <memory>

class VertexBuffer3D;

class VertexBufferResource : public Resource
{
protected:
	QVector<double> _vertices;
	int _numVertices;
	int _data32PerVertex;

public:
	int numVertices() { return _numVertices; }
	int data32PerVertex() { return _data32PerVertex; }
	QVector<double> get() { return _vertices; }


	std::shared_ptr<VertexBuffer3D> _buffer;
	VertexBuffer3D *buffer;

	VertexBufferResource(const QVector<double> &vertices, int data32PerVertex);
	~VertexBufferResource();

	void dispose() override;
	void reset() override;
	bool available() const override;
	void create(Context3D *param1) override;
	bool upload() override;
	void destroy(bool b) override;
};


#endif // VERTEXBUFFERRESOURCE_H
