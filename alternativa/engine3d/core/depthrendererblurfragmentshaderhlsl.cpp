#include "depthrendererblurfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"



QByteArray DepthRendererBlurFragmentShaderHLSL(ShaderSampler &samplers)
{
	using namespace HLSL_ASSEMBLY_PIXEL;


	SHADER_SAMPLER_DIM dim = SHADER_SAMPLER_DIM::_2D;
	SHADER_SAMPLER_REPEAT repeat = SHADER_SAMPLER_REPEAT::CLAMP;
	SHADER_SAMPLER_FILTER filter = SHADER_SAMPLER_FILTER::NEAREST;
	SHADER_SAMPLER_MIPMAP mipmap = SHADER_SAMPLER_MIPMAP::NONE;
	SHADER_SAMPLER_TYPE type = SHADER_SAMPLER_TYPE::RGBA;

	for (int i = 0; i < 16; i++)
		samplers.add(0, dim, repeat, filter, mipmap, type);


	HLSL_ASM_PIXEL a;

	a.mov(rtemp(1), rtexture(0)); //mov r1, t0
	a.texld(rtemp(0), rtemp(1), rsampler(0)); //texld r0, r1, s0
	a.add(rtemp(1, "y"), rtemp(1), N(rconst(0))); //add r1.y, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "x"), rtemp(1), N(rconst(0))); //add r1.x, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), rconst(0)); //add r1.y, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), rconst(0)); //add r1.y, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), rconst(0)); //add r1.y, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "x"), rtemp(1), rconst(0)); //add r1.x, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "x"), rtemp(1), rconst(0)); //add r1.x, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "x"), rtemp(1), rconst(0)); //add r1.x, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), N(rconst(0))); //add r1.y, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), N(rconst(0))); //add r1.y, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), N(rconst(0))); //add r1.y, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "x"), rtemp(1), N(rconst(0))); //add r1.x, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), rconst(0)); //add r1.y, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "y"), rtemp(1), rconst(0)); //add r1.y, r1, c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.add(rtemp(1, "x"), rtemp(1), N(rconst(0))); //add r1.x, r1, -c0
	a.min(rtemp(2), rtemp(1), rconst(1)); //min r2, r1, c1
	a.texld(rtemp(2), rtemp(2), rsampler(0)); //texld r2, r2, s0
	a.add(rtemp(0, "w"), rtemp(0), rtemp(2)); //add r0.w, r0, r2
	a.mul(rtemp(0, "w"), rtemp(0), rconst(0)); //mul r0.w, r0, c0
	a.mov(rcolorout(0), rtemp(0)); //mov oC0, r0


	HLSL_ASM_CAPTION caption;

	caption.dcl(rtexture(0));
	caption.dcl_2d(rsampler(0));

	QByteArray data = collect_pixel(caption, a);

	return data;
}


#if 0
mov(ft1, v0);
tex(ft0, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
sub(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
sub(ft1.x, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.x, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.x, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.x, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
sub(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
sub(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
sub(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
sub(ft1.x, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
add(ft1.y, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
sub(ft1.x, ft1, fc[0]);
min(ft2, ft1, fc[1]);
tex(ft2, ft2, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
add(ft0.w, ft0, ft2);
mul(ft0.w, ft0, fc[0]);
mov(oc, ft0);


dcl t0
dcl_2d s0
0:     mov r1, t0

1:     texld r0, r1, s0
1:     add r1.y, r1, -c0

2:     min r2, r1, c1
3:     texld r2, r2, s0
3:     add r0.w, r0, r2
4:     add r1.x, r1, -c0

5:     min r2, r1, c1
6:     texld r2, r2, s0
6:     add r0.w, r0, r2
7:     add r1.y, r1, c0
8:     min r2, r1, c1
9:     texld r2, r2, s0
9:     add r0.w, r0, r2
10:     add r1.y, r1, c0
11:     min r2, r1, c1
12:     texld r2, r2, s0
12:     add r0.w, r0, r2
13:     add r1.y, r1, c0
14:     min r2, r1, c1
15:     texld r2, r2, s0
15:     add r0.w, r0, r2
16:     add r1.x, r1, c0
17:     min r2, r1, c1
18:     texld r2, r2, s0
18:     add r0.w, r0, r2
19:     add r1.x, r1, c0
20:     min r2, r1, c1
21:     texld r2, r2, s0
21:     add r0.w, r0, r2
22:     add r1.x, r1, c0
23:     min r2, r1, c1
24:     texld r2, r2, s0
24:     add r0.w, r0, r2
25:     add r1.y, r1, -c0
26:     min r2, r1, c1
27:     texld r2, r2, s0
27:     add r0.w, r0, r2
28:     add r1.y, r1, -c0
29:     min r2, r1, c1
30:     texld r2, r2, s0
30:     add r0.w, r0, r2
31:     add r1.y, r1, -c0
32:     min r2, r1, c1
33:     texld r2, r2, s0
33:     add r0.w, r0, r2
34:     add r1.x, r1, -c0
35:     min r2, r1, c1
36:     texld r2, r2, s0
36:     add r0.w, r0, r2
37:     add r1.y, r1, c0
38:     min r2, r1, c1
39:     texld r2, r2, s0
39:     add r0.w, r0, r2
40:     add r1.y, r1, c0
41:     min r2, r1, c1
42:     texld r2, r2, s0
42:     add r0.w, r0, r2
43:     add r1.x, r1, -c0
44:     min r2, r1, c1
45:     texld r2, r2, s0
45:     add r0.w, r0, r2
46:     mul r0.w, r0, c0
47:     mov oC0, r0
#endif


