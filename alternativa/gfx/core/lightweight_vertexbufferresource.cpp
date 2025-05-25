#include "vertexbufferresource.h"
#include "flash/display3D/vertexbuffer3d.h"
#include "flash/display3D/context3d.h"


VertexBufferResource::VertexBufferResource(const QVector<double>&, int)
{
	this->buffer = nullptr;
	this->_data32PerVertex = 0;
	this->_numVertices = 0;
}

VertexBufferResource::~VertexBufferResource()
{

}

void VertexBufferResource::dispose()
{
	Resource::dispose();	
}

void VertexBufferResource::reset()
{
	Resource::reset();
}

bool VertexBufferResource::available() const
{
	return false;
}

void VertexBufferResource::create(Context3D *context)
{
	Resource::create(context);
}

bool VertexBufferResource::upload()
{
	Resource::upload();
	return false;
}

void VertexBufferResource::destroy(bool)
{
}
