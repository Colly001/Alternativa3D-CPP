#ifndef COMMONREGISTER_H
#define COMMONREGISTER_H

#include "register.h"


class CommonRegister : public Register
{
protected:
	static const int X = 0;
	static const int Y = 1;
	static const int Z = 2;
	static const int W = 3;

public:
	Register *x;
	Register *y;
	Register *z;
	Register *w;

	Register *xyz;
	Register *xy;
	Register *xw;
	Register *xz;
	Register *zw;


	CommonRegister(int index, int emitCode);
};

#endif // COMMONREGISTER_H
