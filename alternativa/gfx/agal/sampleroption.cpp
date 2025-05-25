#include "sampleroption.h"


SamplerOption::SamplerOption(uint32_t mask, uint32_t flag)
{
	this->mask = mask;
	this->flag = flag;
}

int SamplerOption::apply(int v)
{
	v = v & (~(15 << this->flag));
	v = v | (this->mask << this->flag);
	return v;
}
