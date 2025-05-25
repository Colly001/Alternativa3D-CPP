#include "indexbufferresource.h"
#include "flash/display3D/context3d.h"
#include "flash/display3D/indexbuffer3d.h"


IndexBufferResource::IndexBufferResource(const QVector<uint32_t>&)
{
	this->_numIndices = 0;
	this->buffer = nullptr;
}

QVector<uint32_t> IndexBufferResource::getIndices()
{
	return this->_indices;
}

void IndexBufferResource::dispose() //override
{
	Resource::dispose();
}

void IndexBufferResource::reset() //override
{
	Resource::reset();
}

bool IndexBufferResource::available() const
{
	return false;
}

void IndexBufferResource::create(Context3D *context)
{
	Resource::create(context);
}

bool IndexBufferResource::upload()
{
	Resource::upload();
	return false;
}
