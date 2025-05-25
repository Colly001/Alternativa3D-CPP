#include "texturematerialfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "materialfragmentshaderhlsl.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray TextureMaterialFragmentShaderHLSL(ShaderSampler &samplers,
											 bool p_a,
											 bool p_b,
											 bool p_c,
											 bool p_d,
											 bool p_e,
											 bool p_f,
											 bool p_g,
											 bool p_h,
											 bool p_i,
											 bool p_j,
											 bool p_k,
											 bool p_l,
											 bool p_m,
											 bool p_n,
											 bool p_o,
											 bool p_p,
											 bool p_q,
											 bool p_r,
											 bool p_s)
{
	(void)p_e;

	using namespace HLSL_ASSEMBLY_PIXEL;


	SHADER_SAMPLER_REPEAT _repeat = p_a ? SHADER_SAMPLER_REPEAT::WRAP : SHADER_SAMPLER_REPEAT::CLAMP;
	SHADER_SAMPLER_FILTER _filter = p_b ? SHADER_SAMPLER_FILTER::LINEAR : SHADER_SAMPLER_FILTER::NEAREST;
	SHADER_SAMPLER_MIPMAP _mipMap = p_c ? (p_b ? SHADER_SAMPLER_MIPMAP::LINEAR : SHADER_SAMPLER_MIPMAP::NEAREST) : SHADER_SAMPLER_MIPMAP::NONE;
	SHADER_SAMPLER_TYPE _type = p_d ? SHADER_SAMPLER_TYPE::DXT1 : SHADER_SAMPLER_TYPE::RGBA;

	HLSL_ASM_PIXEL a;


	//dcl t0
	//dcl t1
	//dcl t2
	//dcl_2d s0
	//dcl_2d s1
	//dcl_2d s2
	//dcl_2d s3
	//def c28, 0, 1, 0, 0

	samplers.add(0, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);
	a.texld(rtemp(0), rtexture(0), rsampler(0)); //texld r8, t0, s0

	//a.texld(rtemp(8), rtexture(0), rsampler(0)); //texld r8, t0, s0
	//a.mov(rtemp(8, "w"), rconst(28, "y")); //mov r8.w, c28.y
	//a.mov(rtemp(0), rtemp(8));

	//texld r8, t0, s0
	//mov r8.w, c28.y
	//mov r0, r8

	//texld r0, t0, s0
	//tex(ft0, v0, fs0->dim(SamplerDim::D2) ->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));



	MaterialFragmentShaderHLSL(a, samplers, p_f, p_g, p_h, p_i, p_j, p_k, p_l, p_m, p_n,
							   p_o, p_p, p_q, p_r, p_s);


	HLSL_ASM_CAPTION caption;

	//texture
	if (a.checkUsedRegister(rtexture(0)))
		caption.dcl(rtexture(0)); //dcl t0

	if (a.checkUsedRegister(rtexture(1)))
		caption.dcl(rtexture(1)); //dcl t1

	if (a.checkUsedRegister(rtexture(2)))
		caption.dcl(rtexture(2)); //dcl t2

	//sampler
	if (a.checkUsedRegister(rsampler(0)))
		caption.dcl_2d(rsampler(0)); //dcl_2d s0

	if (a.checkUsedRegister(rsampler(1)))
		caption.dcl_2d(rsampler(1)); //dcl_2d s1

	if (a.checkUsedRegister(rsampler(2)))
		caption.dcl_2d(rsampler(2)); //dcl_2d s2

	if (a.checkUsedRegister(rsampler(3)))
		caption.dcl_2d(rsampler(3)); //dcl_2d s3

	if (a.checkUsedRegister(rsampler(5)))
		caption.dcl_2d(rsampler(5)); //dcl_2d s5

	//const
	if (a.checkUsedRegister(rconst(28)))
		caption.def(rconst(28), 0, 1, 0, 0); //def c28, 0, 1, 0, 0


	QByteArray data = collect_pixel(caption, a);
	return data;
}
