#ifndef SHADERSAMPLER_H
#define SHADERSAMPLER_H

#include <vector>

enum class SHADER_SAMPLER_DIM
{
	_2D,
	_CUBE,
	_3D,
};

enum class SHADER_SAMPLER_REPEAT
{
	CLAMP,
	REPEAT,
	WRAP,
};

enum class SHADER_SAMPLER_FILTER
{
	NEAREST,
	LINEAR,
};

enum class SHADER_SAMPLER_MIPMAP
{
	NONE,
	NEAREST,
	LINEAR,
};

enum class SHADER_SAMPLER_TYPE
{
	RGBA,
	DXT1,
	DXT5,
	VIDEO,
};

class ShaderSampler
{
public:
	ShaderSampler();

	struct Item
	{
		int index;
		SHADER_SAMPLER_DIM dim;
		SHADER_SAMPLER_REPEAT repeat;
		SHADER_SAMPLER_FILTER filter;
		SHADER_SAMPLER_MIPMAP mipmap;
		SHADER_SAMPLER_TYPE type;
	};

	std::vector<Item> list;

	void add(int index,
			 SHADER_SAMPLER_DIM d,
			 SHADER_SAMPLER_REPEAT r,
			 SHADER_SAMPLER_FILTER f,
			 SHADER_SAMPLER_MIPMAP m,
			 SHADER_SAMPLER_TYPE t);
};

#endif // SHADERSAMPLER_H
