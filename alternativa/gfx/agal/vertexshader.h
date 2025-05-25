#ifndef VERTEXSHADER_H
#define VERTEXSHADER_H

#include "shader.h"

class VertexShader : public Shader
{
public:

	static CommonRegister *va0;
	static CommonRegister *va1;
	static CommonRegister *va2;
	static CommonRegister *va3;
	static CommonRegister *va4;
	static CommonRegister *va5;
	static CommonRegister *va6;
	static CommonRegister *va7;

	static CommonRegister *vt0;
	static CommonRegister *vt1;
	static CommonRegister *vt2;
	static CommonRegister *vt3;
	static CommonRegister *vt4;
	static CommonRegister *vt5;
	static CommonRegister *vt6;
	static CommonRegister *vt7;

	//static QVector<CommonRegister*> vc;
	static CommonRegister **vc;
	static CommonRegister *op;

	VertexShader();
};

#endif // VERTEXSHADER_H
