#include "vertexshader.h"


CommonRegister *VertexShader::va0 = new CommonRegister(0, 0);
CommonRegister *VertexShader::va1 = new CommonRegister(1, 0);
CommonRegister *VertexShader::va2 = new CommonRegister(2, 0);
CommonRegister *VertexShader::va3 = new CommonRegister(3, 0);
CommonRegister *VertexShader::va4 = new CommonRegister(4, 0);
CommonRegister *VertexShader::va5 = new CommonRegister(5, 0);
CommonRegister *VertexShader::va6 = new CommonRegister(6, 0);
CommonRegister *VertexShader::va7 = new CommonRegister(7, 0);

CommonRegister *VertexShader::vt0 = new CommonRegister(0, 2);
CommonRegister *VertexShader::vt1 = new CommonRegister(1, 2);
CommonRegister *VertexShader::vt2 = new CommonRegister(2, 2);
CommonRegister *VertexShader::vt3 = new CommonRegister(3, 2);
CommonRegister *VertexShader::vt4 = new CommonRegister(4, 2);
CommonRegister *VertexShader::vt5 = new CommonRegister(5, 2);
CommonRegister *VertexShader::vt6 = new CommonRegister(6, 2);
CommonRegister *VertexShader::vt7 = new CommonRegister(7, 2);

CommonRegister *VertexShader::op = new CommonRegister(0, 3);


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

CommonRegister **VertexShader::vc = getConsts();


VertexShader::VertexShader()
{
	 data.writeUInt8(0);
}
