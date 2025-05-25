#ifndef SAMPLERREGISTER_H
#define SAMPLERREGISTER_H

#include "register.h"

class SamplerDim;
class SamplerType;
class SamplerSpecial;
class SamplerRepeat;
class SamplerMipMap;
class SamplerFilter;
class SamplerOption;


class SamplerRegister : public Register
{
protected:
	int samplerbits;

public:

	SamplerRegister(int index);

	SamplerRegister* dim(SamplerDim *p);
	SamplerRegister* type(SamplerType *p);
	SamplerRegister* special(SamplerSpecial *p);
	SamplerRegister* repeat(SamplerRepeat *p);
	SamplerRegister* mipmap(SamplerMipMap *p);
	SamplerRegister* filter(SamplerFilter *p);

private:
	void addSamplerOption(SamplerOption *p);

public:
	void writeSource(ShaderStreamWrite&) override;
};

#endif // SAMPLERREGISTER_H
