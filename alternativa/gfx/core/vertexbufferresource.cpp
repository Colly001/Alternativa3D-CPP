#include "vertexbufferresource.h"
#include "flash/display3D/vertexbuffer3d.h"
#include "flash/display3D/context3d.h"


VertexBufferResource::VertexBufferResource(const QVector<double> &vertices, int data32PerVertex)
{
	this->_buffer = nullptr;
	this->_vertices = vertices;
	this->_data32PerVertex = data32PerVertex;
	this->_numVertices = this->_vertices.length() / this->_data32PerVertex;
}

VertexBufferResource::~VertexBufferResource()
{
	this->dispose();
}

void VertexBufferResource::dispose()
{
	Resource::dispose();
	this->buffer = nullptr;
	this->_buffer = nullptr;
	this->_vertices.clear();
}

void VertexBufferResource::reset()
{
	Resource::reset();
	this->buffer = nullptr;
	this->_buffer = nullptr;
}

bool VertexBufferResource::available() const
{
	return !this->_vertices.isEmpty();
}

void VertexBufferResource::create(Context3D *context)
{
	Resource::create(context);
	this->_buffer = context->createVertexBuffer(this->_numVertices, this->_data32PerVertex);
	this->buffer = this->_buffer.get();
	if (this->buffer == nullptr)
	{
		qDebug("error VertexBufferResource::create");
	}
}


bool VertexBufferResource::upload()
{
	Resource::upload();
	return this->buffer->uploadFromVector(this->_vertices, 0, this->_numVertices);;
}

void VertexBufferResource::destroy(bool)
{
	this->reset();
}
