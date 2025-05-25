#ifndef SAMPLERMIPMAP_H
#define SAMPLERMIPMAP_H

#include "sampleroption.h"

class SamplerMipMap : public SamplerOption
{
public:

	static SamplerMipMap *NONE;
	static SamplerMipMap *NEAREST;
	static SamplerMipMap *LINEAR;

	SamplerMipMap(uint32_t mask);
};

#endif // SAMPLERMIPMAP_H
