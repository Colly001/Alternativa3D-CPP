#include "samplerrepeat.h"

static const uint32_t SAMPLER_REPEAT_SHIFT = 20;

SamplerRepeat *SamplerRepeat::CLAMP = new SamplerRepeat(0);
SamplerRepeat *SamplerRepeat::REPEAT = new SamplerRepeat(1);
SamplerRepeat *SamplerRepeat::WRAP = REPEAT;


SamplerRepeat::SamplerRepeat(uint32_t mask) :
	SamplerOption(mask, SAMPLER_REPEAT_SHIFT)
{

}
