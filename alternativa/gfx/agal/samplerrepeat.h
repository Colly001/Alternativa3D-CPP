#ifndef SAMPLERREPEAT_H
#define SAMPLERREPEAT_H

#include "sampleroption.h"

class SamplerRepeat : public SamplerOption
{
public:

	static SamplerRepeat *CLAMP;
	static SamplerRepeat *REPEAT;
	static SamplerRepeat *WRAP;

	SamplerRepeat(uint32_t mask);
};

#endif // SAMPLERREPEAT_H
