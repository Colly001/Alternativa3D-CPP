#ifndef SAMPLERDIM_H
#define SAMPLERDIM_H

#include "sampleroption.h"

class SamplerDim : public SamplerOption
{

public:
	static SamplerDim *D2;
	static SamplerDim *CUBE;
	static SamplerDim *D3;
	SamplerDim(uint32_t mask);
};

#endif // SAMPLERDIM_H
