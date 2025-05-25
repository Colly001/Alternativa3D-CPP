#include "samplerfilter.h"

static const uint32_t SAMPLER_FILTER_SHIFT = 28;

SamplerFilter *SamplerFilter::NEAREST = new SamplerFilter(0);
SamplerFilter *SamplerFilter::LINEAR = new SamplerFilter(1);

SamplerFilter::SamplerFilter(uint32_t mask) :
	SamplerOption(mask, SAMPLER_FILTER_SHIFT)
{

}
