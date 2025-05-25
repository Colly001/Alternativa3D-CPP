#ifndef REGISTER_H
#define REGISTER_H

#include <stdint.h>

class ShaderStreamWrite;

class Register
{
protected:
	Register *parent;
	int32_t swizzle;
	int32_t destMask;
	int32_t index;
	int32_t emitCode;
	int32_t relOffset;
	int32_t relType;
	uint32_t relSel;
public:

	Register();

	static Register* get(int swizzle = 228, int destMask = 15, Register *parent = nullptr);

protected:
	static int getSwizzle(int param1 = 0, int param2 = 1, int param3 = 2, int param4 = 3);
	static int getDestMask(bool param1, bool param2, bool param3, bool param4);

public:
	void writeDest(ShaderStreamWrite&) const;
	virtual void writeSource(ShaderStreamWrite&);

protected:
	void relate(Register *r, uint32_t relOffset);
};

#endif // REGISTER_H
