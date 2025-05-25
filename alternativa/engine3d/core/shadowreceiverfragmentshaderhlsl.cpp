#include "shadowreceiverfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray ShadowReceiverFragmentShaderHLSL(ShaderSampler &samplers, bool param1, bool param2)
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;

	SHADER_SAMPLER_FILTER filter = param1 ? SHADER_SAMPLER_FILTER::LINEAR : SHADER_SAMPLER_FILTER::NEAREST;
	SHADER_SAMPLER_MIPMAP mipmap = param1 ? SHADER_SAMPLER_MIPMAP::LINEAR : SHADER_SAMPLER_MIPMAP::NEAREST;

	samplers.add(0, SHADER_SAMPLER_DIM::_2D, SHADER_SAMPLER_REPEAT::CLAMP, filter, mipmap, SHADER_SAMPLER_TYPE::RGBA);

	if (param2)
	{
		a.max(rtemp(0), rtexture(0), rconst(16)); //max(ft0, v0, fc[16]);
		a.min(rtemp(0, "x"), rtemp(0), rconst(16, "z")); //min(ft0.x, ft0, fc[16].z);
		a.min(rtemp(0, "y"), rtemp(0), rconst(16, "w")); //min(ft0.y, ft0, fc[16].w);

		//tex(ft0, ft0, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(_loc3_).mipmap(mipmap));
		a.texld(rtemp(0), rtemp(0), rsampler(0));
	}
	else
	{
		//tex(ft0, v0, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(_loc3_).mipmap(mipmap));
		a.texld(rtemp(0), rtexture(0), rsampler(0)); //texld r0, t0, s0
	}


	a.add(rtemp(1), rtexture(0, "z"), N(rconst(14))); //add r1, t0.z, -c14
	a.rcp(rtemp(0, "x"), rconst(13, "x")); //rcp r8.x, c13.x
	a.rcp(rtemp(0, "y"), rconst(13, "y")); //rcp r8.y, c13.y
	a.rcp(rtemp(0, "z"), rconst(13, "z")); //rcp r8.z, c13.z
	a.rcp(rtemp(0, "w"), rconst(13, "w")); //rcp r8.w, c13.w
	a.mul(rtemp(2), rtemp(1), rtemp(8)); //mul r2, r1, r8
	a.max(rtemp(3), rtemp(1), rconst(13, "x")); //max r3, r1, c13.x
	a.mul(rtemp(3), rtemp(3), rconst(13, "y")); //mul r3, r3, c13.y
	a.min(rtemp(3), rtemp(3), rconst(14)); //min r3, r3, c14
	a.add(rtemp(4), rconst(14), N(rtemp(3))); //add r4, c14, -r3
	a.mul(rtemp(2), rtemp(2), rtemp(3)); //mul r2, r2, r3
	a.mul(rtemp(1), rtemp(1), rtemp(4)); //mul r1, r1, r4
	a.add(rtemp(2), rtemp(1), rtemp(2)); //add r2, r1, r2
	a.add(rtemp(2), rconst(14), N(rtemp(2))); //add r2, c14, -r2
	a.mul(rtemp(0), rtemp(0), rtemp(2)); //mul r0, r0, r2
	a.add(rtemp(1), rtexture(0), N(rconst(14))); //add r1, t0, -c14
	a.rcp(rtemp(8), rconst(14, "y")); //rcp r8, c14.y
	a.mul(rtemp(1), rtemp(1), rtemp(8)); //mul r1, r1, r8
	a.mov_sat(rtemp(1), rtemp(1)); //mov_sat r1, r1
	a.mul(rtemp(0), rtemp(0), rtemp(1, "z")); //mul r0, r0, r1.z
	a.mov(rtemp(0, "xyz"), rconst(15)); //mov r0.xyz, c15
	a.mul(rtemp(0, "w"), rtemp(0), rconst(15)); //mul r0.w, r0, c15
	a.mov(rcolorout(0), rtemp(0)); //mov oC0, r0


	HLSL_ASM_CAPTION caption;

	caption.dcl(rtexture(0));
	caption.dcl_2d(rsampler(0));

	QByteArray data = collect_pixel(caption, a);

	return data;
}



#if 0
true false

dcl t0
dcl_2d s0
texld r0, t0, s0
add r1, t0.z, -c14
rcp r8.x, c13.x
rcp r8.y, c13.y
rcp r8.z, c13.z
rcp r8.w, c13.w
mul r2, r1, r8
max r3, r1, c13.x
mul r3, r3, c13.y
min r3, r3, c14
add r4, c14, -r3
mul r2, r2, r3
mul r1, r1, r4
add r2, r1, r2
add r2, c14, -r2
mul r0, r0, r2
add r1, t0, -c14
rcp r8, c14.y
mul r1, r1, r8
mov_sat r1, r1
mul r0, r0, r1.z
mov r0.xyz, c15
mul r0.w, r0, c15
mov oC0, r0


var _loc3_:SamplerFilter = ((!(!(param1))) ? SamplerFilter.LINEAR : SamplerFilter.NEAREST);
var _loc4_:SamplerMipMap = ((!(!(param1))) ? SamplerMipMap.LINEAR : SamplerMipMap.NEAREST);
if (param2)
{
	max(ft0, v0, fc[16]);
	min(ft0.x, ft0, fc[16].z);
	min(ft0.y, ft0, fc[16].w);
	tex(ft0, ft0, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(_loc3_).mipmap(_loc4_));
}
else
{
tex(ft0, v0, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(_loc3_).mipmap(_loc4_));
};
sub(ft1, v0.z, fc[14]);
div(ft2, ft1, fc[13]);
max(ft3, ft1, fc[13].x);
mul(ft3, ft3, fc[13].y);
min(ft3, ft3, fc[14]);
sub(ft4, fc[14], ft3);
mul(ft2, ft2, ft3);
mul(ft1, ft1, ft4);
add(ft2, ft1, ft2);
sub(ft2, fc[14], ft2);
mul(ft0, ft0, ft2);
sub(ft1, v0, fc[14]);
div(ft1, ft1, fc[14].y);
sat(ft1, ft1);
mul(ft0, ft0, ft1.z);
mov(ft0.xyz, fc[15]);
mul(ft0.w, ft0, fc[15]);
mov(oc, ft0);
#endif
