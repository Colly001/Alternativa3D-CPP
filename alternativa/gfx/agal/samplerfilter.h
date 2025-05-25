#ifndef SAMPLERFILTER_H
#define SAMPLERFILTER_H

#include "sampleroption.h"

class SamplerFilter : public SamplerOption
{
public:

	static SamplerFilter *NEAREST;
	static SamplerFilter *LINEAR;

	SamplerFilter(uint32_t mask);
};

#endif // SAMPLERFILTER_H
