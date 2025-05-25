#include "programresource.h"
#include "flash/display3D/program3d.h"
#include "flash/display3D/context3d.h"
#include <QDebug>


ProgramResource::ProgramResource(int id, TYPE type, const QByteArray &vertexProgram, const QByteArray &fragmentProgram,  const ShaderSampler &sampler)
{
	this->id = id;
	this->type = type;
	this->_vertexProgram = vertexProgram;
	this->_fragmentProgram = fragmentProgram;
	this->sampler = sampler;
	this->program = nullptr;
}

ProgramResource::~ProgramResource()
{
	this->_program = nullptr;
}

void ProgramResource::reset()
{
	Resource::reset();
	this->program = nullptr;
	this->_program = nullptr;
}

bool ProgramResource::available() const
{
	return ((!(this->_vertexProgram.isEmpty())) && (!(this->_fragmentProgram.isEmpty())));
}

void ProgramResource::create(Context3D *context)
{
	Resource::create(context);
	this->_program = context->createProgram();
	this->program = this->_program.get();
}

bool ProgramResource::upload()
{
	Resource::upload();

	if (type == TYPE::AGAL)
	{
		/*
		if (!this->program->upload(this->_vertexProgram, this->_fragmentProgram, this->sampler))
		{
			return false;
		}
		*/
	}
	else if (type == TYPE::HLSL)
	{
		if (!this->program->upload_hlsl((const uint32_t*)this->_vertexProgram.constData(),
										(const uint32_t*)this->_fragmentProgram.constData(),
										this->sampler))
		{
			qDebug() << "error ProgramResource::upload" << this->id;
			return false;
		}
	}


	return true;
}
