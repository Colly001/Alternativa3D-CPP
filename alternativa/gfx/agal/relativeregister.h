#ifndef RELATIVEREGISTER_H
#define RELATIVEREGISTER_H

#include "commonregister.h"

class RelativeRegister : public CommonRegister
{
public:
	RelativeRegister(int, int);
	CommonRegister* rel(Register *param1, uint32_t param2);
};

#endif // RELATIVEREGISTER_H
