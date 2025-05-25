#ifndef SAMPLERTYPE_H
#define SAMPLERTYPE_H

#include "sampleroption.h"

class SamplerType : public SamplerOption
{
public:

	static SamplerType *RGBA;
	static SamplerType *DXT1;
	static SamplerType *DXT5;
	static SamplerType *VIDEO;

	SamplerType(int mask);
};

#endif // SAMPLERTYPE_H
