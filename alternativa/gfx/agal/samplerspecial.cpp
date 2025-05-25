#include "samplerspecial.h"

static const uint32_t SAMPLER_SPECIAL_SHIFT = 16;

const SamplerSpecial *CENTROID = new SamplerSpecial(1);
const SamplerSpecial *SINGLE = new SamplerSpecial(2);
const SamplerSpecial *IGNORESAMPLER = new SamplerSpecial(4);


SamplerSpecial::SamplerSpecial(uint32_t mask) :
	SamplerOption(mask, SAMPLER_SPECIAL_SHIFT)
{

}

int SamplerSpecial::apply(int v)
{
	v = v | (uint32_t(mask) << uint32_t(flag));
	return v;
}
