#include "indexbufferresource.h"
#include "flash/display3D/context3d.h"
#include "flash/display3D/indexbuffer3d.h"


IndexBufferResource::IndexBufferResource(const QVector<uint32_t> &v)
{
	this->_indices = v;
	this->_numIndices = this->_indices.length();
	this->buffer = nullptr;
}

QVector<uint32_t> IndexBufferResource::getIndices()
{
	return this->_indices;
}

void IndexBufferResource::dispose() //override
{
	Resource::dispose();
	this->buffer = nullptr;
	this->_buffer = nullptr;
	this->_indices.clear();;
}

void IndexBufferResource::reset() //override
{
	Resource::reset();
	this->buffer = nullptr;
	this->_buffer = nullptr;
}

bool IndexBufferResource::available() const
{
	return !this->_indices.isEmpty();
}

void IndexBufferResource::create(Context3D *context)
{
	Resource::create(context);
	this->_buffer = context->createIndexBuffer(this->_numIndices);
	this->buffer = this->_buffer.get();
}

bool IndexBufferResource::upload()
{
	Resource::upload();
	this->buffer->uploadFromVector(this->_indices, 0, this->_numIndices);
	return true;
}
