#include "commonregister.h"


CommonRegister::CommonRegister(int index, int emitCode)
{
	this->index = index;
	this->emitCode = emitCode;
	this->x = Register::get(getSwizzle(X, X, X, X), getDestMask(true, false, false, false), this);
	this->y = Register::get(getSwizzle(Y, Y, Y, Y), getDestMask(false, true, false, false), this);
	this->z = Register::get(getSwizzle(Z, Z, Z, Z), getDestMask(false, false, true, false), this);
	this->w = Register::get(getSwizzle(W, W, W, W), getDestMask(false, false, false, true), this);
	this->xyz = Register::get(getSwizzle(X, Y, Z, Z), getDestMask(true, true, true, false), this);
	this->xy = Register::get(getSwizzle(X, Y, Y, Y), getDestMask(true, true, false, false), this);
	this->xz = Register::get(getSwizzle(X, Z, Z, Z), getDestMask(true, false, true, false), this);
	this->xw = Register::get(getSwizzle(X, W, W, W), getDestMask(true, false, false, true), this);
	this->zw = Register::get(getSwizzle(Z, W, W, W), getDestMask(false, false, true, true), this);
}
