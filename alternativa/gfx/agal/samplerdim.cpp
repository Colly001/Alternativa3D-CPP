#include "samplerdim.h"

static const int SAMPLER_TEXTURE_TYPE_SHIFT = 12;

SamplerDim *SamplerDim::D2 = new SamplerDim(0);
SamplerDim *SamplerDim::CUBE = new SamplerDim(1);
SamplerDim *SamplerDim::D3 = new SamplerDim(2);


SamplerDim::SamplerDim(uint32_t mask) :
	SamplerOption(mask, SAMPLER_TEXTURE_TYPE_SHIFT)
{

}
