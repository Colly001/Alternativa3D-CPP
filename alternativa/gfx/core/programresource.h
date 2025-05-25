#ifndef PROGRAMRESOURCE_H
#define PROGRAMRESOURCE_H

#include "resource.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"
#include <QByteArray>
#include <memory>

class Program3D;

class ProgramResource : public Resource
{
	QByteArray _vertexProgram;
	QByteArray _fragmentProgram;
	ShaderSampler sampler;
	int id;

public:

	enum class TYPE
	{
		AGAL,
		HLSL,
	};

	TYPE type;

	std::shared_ptr<Program3D> _program;
	Program3D *program;

	ProgramResource(int id, TYPE type, const QByteArray &vertexProgram, const QByteArray &fragmentProgram, const ShaderSampler &sampler);
	~ProgramResource();

	const QByteArray& getVertexProgram() { return this->_vertexProgram; }
	const QByteArray& getFragmentProgram() { return this->_fragmentProgram; }

	void reset() override;
	bool available() const override;
	void create(Context3D*) override;
	bool upload() override;
};

#endif // PROGRAMRESOURCE_H
