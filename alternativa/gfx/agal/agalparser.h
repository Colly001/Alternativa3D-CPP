#ifndef AGALPARSER_H
#define AGALPARSER_H

#include <stdint.h>

class QByteArray;

class AgalParser
{
public:

	enum RegType
	{
		Attribute,
		Constant,
		Temporary,
		Output,
		Varying,
		Sampler
	};

	enum ProgramType
	{
		Vertex,
		Fragment
	};



	class SamplerReg
	{
	public:

		ProgramType programType;
		int f; // Filter (0=nearest,1=linear) (4 bits)  SamplerFilter
		int m; // Mipmap (0=disable,1=nearest, 2=linear) SamplerMipMap
		int w; // wrap (0=clamp 1=repeat)   SamplerRepeat
		int s; // special flags bit
		int d; // dimension 0=2d 1=cube SamplerDim?
		RegType type;
		int b; // lod bias
		int n; // number

		SamplerReg *next;
	};


	SamplerReg *samplerReg;


	AgalParser();
	bool parser(const QByteArray &data);
};

#endif // AGALPARSER_H
