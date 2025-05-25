#ifndef SHAREDPARAMSBITS_H
#define SHAREDPARAMSBITS_H

#include <stdint.h>

class ShaderParamsBits
{
public:
	uint32_t k;
	uint8_t offset;

	ShaderParamsBits()
	{
		k = 0;
		offset = 0;
	}

	void add(bool b)
	{
		k |= b << offset;
		offset++;
	}
};

#endif // SHAREDPARAMSBITS_H
