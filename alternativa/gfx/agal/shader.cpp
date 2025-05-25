#include "shader.h"
#include <QByteArray>


CommonRegister *Shader::v0 = new CommonRegister(0, 4);
CommonRegister *Shader::v1 = new CommonRegister(1, 4);
CommonRegister *Shader::v2 = new CommonRegister(2, 4);
CommonRegister *Shader::v3 = new CommonRegister(3, 4);
CommonRegister *Shader::v4 = new CommonRegister(4, 4);
CommonRegister *Shader::v5 = new CommonRegister(5, 4);
CommonRegister *Shader::v6 = new CommonRegister(6, 4);
CommonRegister *Shader::v7 = new CommonRegister(7, 4);

RelativeRegister *Shader::cc = new RelativeRegister(0, 1);





Shader::Shader()
{
	//this.data = new ByteArray();
	//this.data.endian = Endian.LITTLE_ENDIAN;
	this->data.writeUInt8(160);
	this->data.writeUInt32(1);
	this->data.writeUInt8(161);

	//this.data.writeByte(160);
	//this.data.writeUnsignedInt(1);
	//this.data.writeByte(161);
}

void Shader::op2(int o, Register *r1, Register *r2)
{
	this->data.writeUInt32(o);
	r1->writeDest(this->data);
	r2->writeSource(this->data);
	this->data.writeUInt32(0);
	this->data.writeUInt32(0);

	//this.data.writeUnsignedInt(param1);
	//param2.writeDest(this.data);
	//param3.writeSource(this.data);
	//this.data.writeUnsignedInt(0);
	//this.data.writeUnsignedInt(0);
}

void Shader::op3(int o, Register *r1, Register *r2, Register *r3)
{
	this->data.writeUInt32(o);
	r1->writeDest(this->data);
	r2->writeSource(this->data);
	r3->writeSource(this->data);

	//this.data.writeUnsignedInt(param1);
	//param2.writeDest(this.data);
	//param3.writeSource(this.data);
	//param4.writeSource(this.data);
}

QByteArray Shader::getAgalcode()
{
	return this->data.getData();
}
