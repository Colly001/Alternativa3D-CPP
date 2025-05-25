#ifndef FRAGMENTSHADER_H
#define FRAGMENTSHADER_H

#include "shader.h"
#include "vertexshader.h"
#include "samplerregister.h"

class FragmentShader : public Shader
{
public:



	static CommonRegister *ft0;
	static CommonRegister *ft1;
	static CommonRegister *ft2;
	static CommonRegister *ft3;
	static CommonRegister *ft4;
	static CommonRegister *ft5;
	static CommonRegister *ft6;
	static CommonRegister *ft7;

	static SamplerRegister *fs0;
	static SamplerRegister *fs1;
	static SamplerRegister *fs2;
	static SamplerRegister *fs3;
	static SamplerRegister *fs4;
	static SamplerRegister *fs5;
	static SamplerRegister *fs6;
	static SamplerRegister *fs7;

	static CommonRegister *oc;

	//static const fc:Vector.<CommonRegister> = VertexShader.vc;
	static CommonRegister **fc;

	FragmentShader();

	void kil(Register *param1);
	void tex(Register *param1, Register *param2, SamplerRegister *param3);
};

#endif // FRAGMENTSHADER_H
