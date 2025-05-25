#include "shadowatlasfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray ShadowAtlasFragmentShaderHLSL(ShaderSampler &samplers, int param1, bool param2)
{
	using namespace HLSL_ASSEMBLY_PIXEL;


	HLSL_ASM_PIXEL a;

	a.mov(rtemp(1), rtexture(0)); //mov r1, t0

	samplers.add(0, SHADER_SAMPLER_DIM::_2D, SHADER_SAMPLER_REPEAT::CLAMP, SHADER_SAMPLER_FILTER::NEAREST, SHADER_SAMPLER_MIPMAP::NONE, SHADER_SAMPLER_TYPE::RGBA);
	a.texld(rtemp(3), rtemp(1), rsampler(0)); //texld r3, r1, s0




	//mov(ft1, v0);
	//tex(ft3, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));

	if (param2)
	{
		a.add(rtemp(1, "x"), rtexture(0), N(rconst(1))); //add r1.x, t0, -c1
		//sub(ft1.x, v0, fc[1]);
	}
	else
	{
		a.add(rtemp(1, "y"), rtexture(0), N(rconst(1))); //add r1.y, t0, -c1
		//sub(ft1.y, v0, fc[1]);
	}

	int i = -param1;
	while (i <= param1)
	{
		if (i != 0)
		{
			//samplers.add(0, SHADER_SAMPLER_DIM::_2D, SHADER_SAMPLER_REPEAT::CLAMP, SHADER_SAMPLER_FILTER::NEAREST, SHADER_SAMPLER_MIPMAP::NONE, SHADER_SAMPLER_TYPE::RGBA);

			a.texld(rtemp(2), rtemp(1), rsampler(0)); //texld r2, r1, s0
			a.add(rtemp(3, "w"), rtemp(3), rtemp(2)); //add r3.w, r3, r2

			//tex(ft2, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
			//add(ft3.w, ft3, ft2);
		}
		if (param2)
		{
			a.add(rtemp(1, "x"), rtemp(1), rconst(0)); //add r1.x, r1, c0
		}
		else
		{
			a.add(rtemp(1, "y"), rtemp(1), rconst(0)); //add r1.y, r1, c0
		}
		i++;
	}

	a.rcp(rtemp(8, "x"), rconst(0, "x")); //rcp r8.x, c0.x
	a.rcp(rtemp(8, "y"), rconst(0, "y")); //rcp r8.y, c0.y
	a.rcp(rtemp(8, "z"), rconst(0, "z")); //rcp r8.z, c0.z
	a.rcp(rtemp(8, "w"), rconst(0, "w")); //rcp r8.w, c0.w
	a.mul(rtemp(8, "w"), rtemp(3), rtemp(8)); //mul r3.w, r3, r8
	a.mov(rcolorout(0), rtemp(3)); //mov oC0, r3

	//div(ft3.w, ft3, fc[0]);
	//mov(oc, ft3);


	HLSL_ASM_CAPTION caption;
	caption.dcl(rtexture(0));
	caption.dcl_2d(rsampler(0));

	QByteArray data = collect_pixel(caption, a);
	return data;
}


#if 0
true false

ps_2_x
dcl t0
dcl_2d s0
0:     mov r1, t0
1:     texld r3, r1, s0
1:     add r1.y, t0, -c1
2:     texld r2, r1, s0
2:     add r3.w, r3, r2
3:     add r1.y, r1, c0
4:     texld r2, r1, s0
4:     add r3.w, r3, r2
5:     add r1.y, r1, c0
6:     texld r2, r1, s0
6:     add r3.w, r3, r2
7:     add r1.y, r1, c0
8:     texld r2, r1, s0
8:     add r3.w, r3, r2
9:     add r1.y, r1, c0
10:     texld r2, r1, s0
10:     add r3.w, r3, r2
11:     add r1.y, r1, c0
12:     texld r2, r1, s0
12:     add r3.w, r3, r2
13:     add r1.y, r1, c0
14:     add r1.y, r1, c0
15:     texld r2, r1, s0
15:     add r3.w, r3, r2
16:     add r1.y, r1, c0
17:     texld r2, r1, s0
17:     add r3.w, r3, r2
18:     add r1.y, r1, c0
19:     texld r2, r1, s0
19:     add r3.w, r3, r2
20:     add r1.y, r1, c0
21:     texld r2, r1, s0
21:     add r3.w, r3, r2
22:     add r1.y, r1, c0
23:     texld r2, r1, s0
23:     add r3.w, r3, r2
24:     add r1.y, r1, c0
25:     texld r2, r1, s0
25:     add r3.w, r3, r2
26:     add r1.y, r1, c0
27:     rcp r8.x, c0.x
28:     rcp r8.y, c0.y
29:     rcp r8.z, c0.z
30:     rcp r8.w, c0.w
31:     mul r3.w, r3, r8
32:     mov oC0, r3


true true

ps_2_x
dcl t0
dcl_2d s0
0:     mov r1, t0
1:     texld r3, r1, s0
1:     add r1.x, t0, -c1
2:     texld r2, r1, s0
2:     add r3.w, r3, r2
3:     add r1.x, r1, c0
4:     texld r2, r1, s0
4:     add r3.w, r3, r2
5:     add r1.x, r1, c0
6:     texld r2, r1, s0
6:     add r3.w, r3, r2
7:     add r1.x, r1, c0
8:     texld r2, r1, s0
8:     add r3.w, r3, r2
9:     add r1.x, r1, c0
10:     texld r2, r1, s0
10:     add r3.w, r3, r2
11:     add r1.x, r1, c0
12:     texld r2, r1, s0
12:     add r3.w, r3, r2
13:     add r1.x, r1, c0
14:     add r1.x, r1, c0
15:     texld r2, r1, s0
15:     add r3.w, r3, r2
16:     add r1.x, r1, c0
17:     texld r2, r1, s0
17:     add r3.w, r3, r2
18:     add r1.x, r1, c0
19:     texld r2, r1, s0
19:     add r3.w, r3, r2
20:     add r1.x, r1, c0
21:     texld r2, r1, s0
21:     add r3.w, r3, r2
22:     add r1.x, r1, c0
23:     texld r2, r1, s0
23:     add r3.w, r3, r2
24:     add r1.x, r1, c0
25:     texld r2, r1, s0
25:     add r3.w, r3, r2
26:     add r1.x, r1, c0
27:     rcp r8.x, c0.x
28:     rcp r8.y, c0.y
29:     rcp r8.z, c0.z
30:     rcp r8.w, c0.w
31:     mul r3.w, r3, r8
32:     mov oC0, r3


	   mov(ft1, v0);
tex(ft3, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
if (param2)
{
	sub(ft1.x, v0, fc[1]);
}
else
{
sub(ft1.y, v0, fc[1]);
}
int _loc3_ = -(param1);
while (_loc3_ <= param1)
{
	if (_loc3_ != 0)
	{
		tex(ft2, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
		add(ft3.w, ft3, ft2);
	}
	if (param2)
	{
		add(ft1.x, ft1, fc[0]);
	}
	else
	{
		add(ft1.y, ft1, fc[0]);
	}
	_loc3_++;
}
div(ft3.w, ft3, fc[0]);
mov(oc, ft3);
#endif
