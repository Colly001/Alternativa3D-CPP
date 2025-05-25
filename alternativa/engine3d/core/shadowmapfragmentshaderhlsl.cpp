#include "shadowmapfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray ShadowMapFragmentShaderHLSL(ShaderSampler &samplers, bool param1)
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;

	HLSL_ASM_CAPTION caption;
	caption.dcl(rtexture(0));


	if (param1)
	{
		caption.dcl(rtexture(1));
		caption.dcl_2d(rsampler(0));
		caption.def(rconst(28), 0, 1, 0, 0);

		samplers.add(0, SHADER_SAMPLER_DIM::_2D,
					 SHADER_SAMPLER_REPEAT::CLAMP,
					 SHADER_SAMPLER_FILTER::LINEAR,
					 SHADER_SAMPLER_MIPMAP::NONE,
					 SHADER_SAMPLER_TYPE::RGBA);

		a.texld(rtemp(0), rtexture(1), rsampler(0)); //texld r0, t1, s0
		a.add(rtemp(0, "w"), rtemp(0), N(rtexture(1))); //add r0.w, r0, -t1
		a.mov(rtemp(8), rtemp(0, "w")); //mov r8, r0.w
		a.texkill(rtemp(8)); //texkill r8
	}

	a.frc(rtemp(0), rtexture(0, "z")); //frc r0, t0.z
	a.add(rtemp(0, "x"), rtexture(0, "z"), N(rtemp(0))); //add r0.x, t0.z, -r0
	a.add(rtemp(0, "x"), rtemp(0), rconst(0)); //mul r0.x, r0, c0
	a.mov(rtemp(0, "zw"), rconst(0)); //mov r0.zw, c0
	a.mov(rcolorout(0), rtemp(0)); //mov oC0, r0

	QByteArray data = collect_pixel(caption, a);
	return data;
}

#if 0
if (param1)
{
tex(ft0, v1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.LINEAR).mipmap(SamplerMipMap.NONE));
sub(ft0.w, ft0, v1);
kil(ft0.w);
}
frc(ft0, v0.z);
sub(ft0.x, v0.z, ft0);
mul(ft0.x, ft0, fc[0]);
mov(ft0.zw, fc[0]);
mov(oc, ft0);

ps_2_x
	  dcl t0
	  dcl t1
	  dcl_2d s0
	  def c28, 0, 1, 0, 0
0:     texld r0, t1, s0
0:     add r0.w, r0, -t1
1:     mov r8, r0.w
2:     texkill r8
3:     frc r0, t0.z
4:     add r0.x, t0.z, -r0
5:     mul r0.x, r0, c0
6:     mov r0.zw, c0
7:     mov oC0, r0

#endif
