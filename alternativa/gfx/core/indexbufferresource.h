#ifndef INDEXBUFFERRESOURCE_H
#define INDEXBUFFERRESOURCE_H

#include <QVector>
#include "alternativa/gfx/core/resource.h"
#include <memory>

class IndexBuffer3D;
class Context3D;

class IndexBufferResource : public Resource
{
	QVector<uint32_t> _indices;
	int _numIndices;

public:

	int numIndices() { return _numIndices; }
	QVector<uint32_t> indices() { return _indices; }

	std::shared_ptr<IndexBuffer3D> _buffer;
	IndexBuffer3D *buffer;

	IndexBufferResource(const QVector<uint32_t> &);
	QVector<uint32_t> getIndices();
	void dispose() override;
	void reset() override;
	bool available() const override;
	void create(Context3D *context) override;
	bool upload() override;
};

#endif // INDEXBUFFERRESOURCE_H
