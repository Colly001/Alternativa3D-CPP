#ifndef SAMPLERSPECIAL_H
#define SAMPLERSPECIAL_H

#include "sampleroption.h"

class SamplerSpecial : public SamplerOption
{
public:

	static const SamplerSpecial *CENTROID;
	static const SamplerSpecial *SINGLE;
	static const SamplerSpecial *IGNORESAMPLER;

	SamplerSpecial(uint32_t mask);

	int apply(int v) override;
};

#endif // SAMPLERSPECIAL_H
