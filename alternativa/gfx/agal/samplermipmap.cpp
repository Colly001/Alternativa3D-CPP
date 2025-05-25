#include "samplermipmap.h"

static const uint32_t SAMPLER_MIPMAP_SHIFT = 24;

SamplerMipMap *SamplerMipMap::NONE = new SamplerMipMap(0);
SamplerMipMap *SamplerMipMap::NEAREST = new SamplerMipMap(1);
SamplerMipMap *SamplerMipMap::LINEAR = new SamplerMipMap(2);


SamplerMipMap::SamplerMipMap(uint32_t mask) :
	SamplerOption(mask, SAMPLER_MIPMAP_SHIFT)
{

}
