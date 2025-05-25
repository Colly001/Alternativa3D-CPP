#include "samplertype.h"

static const uint32_t SAMPLER_TYPE_SHIFT = 8;

SamplerType *SamplerType::RGBA = new SamplerType(0);
SamplerType *SamplerType::DXT1 = new SamplerType(1);
SamplerType *SamplerType::DXT5 = new SamplerType(2);
SamplerType *SamplerType::VIDEO = new SamplerType(3);



SamplerType::SamplerType(int mask) :
	SamplerOption(mask, SAMPLER_TYPE_SHIFT)
{

}
