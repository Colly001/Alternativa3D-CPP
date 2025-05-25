#ifndef SAMPLEROPTION_H
#define SAMPLEROPTION_H

#include <stdint.h>

class SamplerOption
{
public:
	uint32_t mask;
	uint32_t flag;

	SamplerOption(uint32_t mask, uint32_t flag);
	virtual int apply(int);
};

#endif // SAMPLEROPTION_H
