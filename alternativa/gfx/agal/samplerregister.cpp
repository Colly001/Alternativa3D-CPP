#include "samplerregister.h"
#include "samplerdim.h"
#include "samplertype.h"
#include "samplerspecial.h"
#include "samplerrepeat.h"
#include "samplermipmap.h"
#include "samplerfilter.h"
#include "shaderstreamwrite.h"


SamplerRegister::SamplerRegister(int index)
{
	this->index = index;
	this->samplerbits = 5;
}

SamplerRegister* SamplerRegister::dim(SamplerDim *p)
{
	this->addSamplerOption(p);
	return this;
}

SamplerRegister* SamplerRegister::type(SamplerType *p)
{
	this->addSamplerOption(p);
	return this;
}

SamplerRegister* SamplerRegister::special(SamplerSpecial *p)
{
	this->addSamplerOption(p);
	return this;
}

SamplerRegister* SamplerRegister::repeat(SamplerRepeat *p)
{
	this->addSamplerOption(p);
	return this;
}

SamplerRegister* SamplerRegister::mipmap(SamplerMipMap *p)
{
	this->addSamplerOption(p);
	return this;
}

SamplerRegister* SamplerRegister::filter(SamplerFilter *p)
{
	this->addSamplerOption(p);
	return this;
}

void SamplerRegister::addSamplerOption(SamplerOption *s)
{
	this->samplerbits = s->apply(this->samplerbits);
}

void SamplerRegister::writeSource(ShaderStreamWrite &data)
{
	data.writeUInt16(index);
	data.writeUInt16(0);
	data.writeUInt32(this->samplerbits);

	//param1.writeShort(index);
	//param1.writeShort(0);
	//param1.writeUnsignedInt(this.samplerbits);

	this->samplerbits = 5;
}
