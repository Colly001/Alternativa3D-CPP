#include "fragmentshader.h"

SamplerRegister *FragmentShader::fs0 = new SamplerRegister(0);
SamplerRegister *FragmentShader::fs1 = new SamplerRegister(1);
SamplerRegister *FragmentShader::fs2 = new SamplerRegister(2);
SamplerRegister *FragmentShader::fs3 = new SamplerRegister(3);
SamplerRegister *FragmentShader::fs4 = new SamplerRegister(4);
SamplerRegister *FragmentShader::fs5 = new SamplerRegister(5);
SamplerRegister *FragmentShader::fs6 = new SamplerRegister(6);
SamplerRegister *FragmentShader::fs7 = new SamplerRegister(7);

CommonRegister *FragmentShader::ft0 = new CommonRegister(0, 2);
CommonRegister *FragmentShader::ft1 = new CommonRegister(1, 2);
CommonRegister *FragmentShader::ft2 = new CommonRegister(2, 2);
CommonRegister *FragmentShader::ft3 = new CommonRegister(3, 2);
CommonRegister *FragmentShader::ft4 = new CommonRegister(4, 2);
CommonRegister *FragmentShader::ft5 = new CommonRegister(5, 2);
CommonRegister *FragmentShader::ft6 = new CommonRegister(6, 2);
CommonRegister *FragmentShader::ft7 = new CommonRegister(7, 2);

CommonRegister *FragmentShader::oc = new CommonRegister(0, 3);


//static QVector<CommonRegister*> getConsts()
static CommonRegister** getConsts()
{
	CommonRegister **array = new CommonRegister*[127];
	//QVector<CommonRegister*> v;
	//v.reserve(127);

	for (int i = 0; i < 127; i++)
		array[i] = new CommonRegister(i, 1);

	return array;
}

CommonRegister **FragmentShader::fc = getConsts();



FragmentShader::FragmentShader()
{
	data.writeUInt8(1);
}

void FragmentShader::kil(Register *r1)
{
	data.writeUInt32(KIL);

	//data.position = (data.position + 4);
	data.writeUInt32(0);

	r1->writeSource(data);

	//data.position = (data.position + 8);
	data.writeUInt32(0);
	data.writeUInt32(0);
}

void FragmentShader::tex(Register *r1, Register *r2, SamplerRegister *r3)
{
	data.writeUInt32(TEX);
	r1->writeDest(data);
	r2->writeSource(data);
	r3->writeSource(data);
}
