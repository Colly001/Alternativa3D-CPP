#include "shadersampler.h"

ShaderSampler::ShaderSampler()
{

}

void ShaderSampler::add(int index,
						SHADER_SAMPLER_DIM d,
						SHADER_SAMPLER_REPEAT r,
						SHADER_SAMPLER_FILTER f,
						SHADER_SAMPLER_MIPMAP m,
						SHADER_SAMPLER_TYPE t)
{
	Item item;
	item.index = index;
	item.dim = d;
	item.repeat = r;
	item.filter = f;
	item.mipmap = m;
	item.type = t;
	list.push_back(item);
}
