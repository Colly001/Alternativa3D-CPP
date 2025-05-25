#include "register.h"
#include "shaderstreamwrite.h"
#include "mymath.h"


Register::Register()
{
	parent = nullptr;
	swizzle = 228;
	destMask = 15;
	index = 0;
	emitCode = 0;
	relOffset = 0;
	relType = 0;
	relSel = 0;
}

Register *Register::get(int swizzle, int destMask, Register *parent)
{
	Register *out = new Register();
	out->parent = parent;
	if (parent != nullptr)
	{
		out->index = parent->index;
		out->emitCode = parent->emitCode;
	}
	out->swizzle = swizzle;
	out->destMask = destMask;
	return out;
}

int Register::getSwizzle(int p1, int p2, int p3, int p4)
{
	return p1 | (p2 << 2) | (p3 << 4) | (p4 << 6);
}

int Register::getDestMask(bool p1, bool p2, bool p3, bool p4)
{
	int value = 0;

	if (p1)
	{
		value = value | 0x01;
	}

	if (p2)
	{
		value = value | 0x02;
	}

	if (p3)
	{
		value = value | 0x04;
	}

	if (p4)
	{
		value = value | 0x08;
	}

	return value;
}

void Register::writeDest(ShaderStreamWrite &data) const
{
	data.writeUInt16((!(this->parent == nullptr)) ? int(this->parent->index) : int(this->index));
	data.writeUInt8(this->destMask);
	data.writeUInt8(((!(this->parent == nullptr)) ? int(this->parent->emitCode) : int(this->emitCode)));

	//param1.writeShort(((!(this.parent == null)) ? int(this.parent.index) : int(this.index)));
	//param1.writeByte(this.destMask);
	//param1.writeByte(((!(this.parent == null)) ? int(this.parent.emitCode) : int(this.emitCode)));
}

void Register::writeSource(ShaderStreamWrite &data)
{
	//1 2
	data.writeUInt16(this->parent != nullptr ? int(this->parent->index) : int(this->index));

	//3
	data.writeUInt8(this->parent != nullptr ? int(this->parent->relOffset) : int(this->relOffset));

	//4
	data.writeUInt8(this->swizzle);

	//5
	data.writeUInt8(this->parent != nullptr ? int(this->parent->emitCode) : int(this->emitCode));

	//6
	data.writeUInt8(this->parent != nullptr ? int(this->parent->relType) : int(this->relType));

	//7 8
	data.writeUInt16(this->parent != nullptr ? int(this->parent->relSel) : int(this->relSel));

	//param1.writeShort(((!(this.parent == null)) ? int(this.parent.index) : int(this.index)));
	//param1.writeByte(((!(this.parent == null)) ? int(this.parent.relOffset) : int(this.relOffset)));
	//param1.writeByte(this.swizzle);
	//param1.writeByte(((!(this.parent == null)) ? int(this.parent.emitCode) : int(this.emitCode)));
	//param1.writeByte(((!(this.parent == null)) ? int(this.parent.relType) : int(this.relType)));
	//param1.writeShort(((!(this.parent == null)) ? int(this.parent.relSel) : int(this.relSel)));
}

void Register::relate(Register *r, uint32_t relOffset)
{
	this->relType = r->emitCode;
	this->index = r->index;

	if ((r->destMask & (r->destMask - 1)) != 0)
	{
		throw 324134;
		//throw (new Error("Register must has simple swizzle: .x, .y, .z, .w"));
	}

	this->relSel = MyMath::log(r->destMask) / MyMath::LN2;
	this->relSel = this->relSel | (1 << 15);
	this->relOffset = relOffset;
}
