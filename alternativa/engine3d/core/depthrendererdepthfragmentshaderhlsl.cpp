#include "depthrendererdepthfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray DepthRendererDepthFragmentShaderHLSL(ShaderSampler &samplers, bool param1, bool param2, bool param3, bool param4, bool param5)
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;

	if (param2)
	{
		SHADER_SAMPLER_REPEAT _repeat = param4 ? SHADER_SAMPLER_REPEAT::WRAP : SHADER_SAMPLER_REPEAT::CLAMP;
		SHADER_SAMPLER_FILTER _filter = param3 ? SHADER_SAMPLER_FILTER::LINEAR : SHADER_SAMPLER_FILTER::NEAREST;
		SHADER_SAMPLER_MIPMAP _mipMap = param5 ? (param3 ? SHADER_SAMPLER_MIPMAP::LINEAR : SHADER_SAMPLER_MIPMAP::NEAREST) : SHADER_SAMPLER_MIPMAP::NONE;
		SHADER_SAMPLER_TYPE _type = SHADER_SAMPLER_TYPE::RGBA;
		samplers.add(1, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);
		a.texld(rtemp(0), rtexture(1), rsampler(1)); //tex(ft0, v1, fs1);
		a.add(rtemp(0, "w"), rtemp(0), N(rtexture(1))); //sub(ft0.w, ft0, v1);

		//kil(ft0.w);
		a.mov(rtemp(8), rtemp(0, "w")); //mov r8, r0.w
		a.texkill(rtemp(8)); //texkill r8
	}

	a.frc(rtemp(0), rtexture(0, "z")); //frc r0, t0.z
	a.add(rtemp(0, "x"), rtexture(0, "z"), N(rtemp(0))); //add r0.x, t0.z, -r0
	a.mul(rtemp(0, "x"), rtemp(0), rconst(0)); //mul r0.x, r0, c0

	if (param1)
	{
		a.mov(rtemp(1, "zw"), rconst(0)); //mov r1.zw, c0
		a.mov(rtemp(1, "xy"), rtexture(0)); //mov r1.xy, t0
		//a.nrm(rtemp(1, "xyz"), rtemp(1, "xyz")); //nrm(ft1.xyz, ft1.xyz);


		a.dp3(rtemp(8, "x"), rtemp(1, "xyzz"), rtemp(1, "xyzz")); //dp3 r8.x, r1.xyzz, r1.xyzz
		a.rsq(rtemp(8, "x"), rtemp(8, "x")); //rsq r8.x, r8.x

		a.mul(rtemp(1, "xyz"), rtemp(8, "x"), rtemp(1, "xyzz")); //mul r1.xyz, r8.x, r1.xyzz
		a.mul(rtemp(1, "xy"), rtemp(1), rconst(1)); // mul r1.xy, r1, c1
		a.add(rtemp(1, "xy"), rtemp(1), rconst(1)); //add r1.xy, r1, c1

		samplers.add(0, SHADER_SAMPLER_DIM::_2D,
					 SHADER_SAMPLER_REPEAT::CLAMP,
					 SHADER_SAMPLER_FILTER::NEAREST,
					 SHADER_SAMPLER_MIPMAP::NONE,
					 SHADER_SAMPLER_TYPE::RGBA);

		a.texld(rtemp(2), rtemp(1), rsampler(0));  //texld r2, r1, s0
		a.mov(rtemp(0, "w"), rtemp(2, "z")); //mov(ft0.w, ft2.z);
		a.mul(rtemp(1, "xy"), rtexture(0), rtexture(0)); //mul(ft1.xy, v0, v0);
		a.add(rtemp(1, "x"), rtemp(1), rtemp(1, "y")); //add(ft1.x, ft1, ft1.y);


		//rcp r9.x, r1.x
		//rsq r1.x, r9.x
		a.rcp(rtemp(9, "x"), rtemp(1, "x"));
		a.rsq(rtemp(1, "x"), rtemp(9, "x")); //sqt(ft1.x, ft1);


		a.mov(rtemp(1, "y"), N(rtexture(0, "w"))); //neg(ft1.y, v0.w);
		a.mul(rtemp(1, "xy"), rtemp(1), rconst(1)); //mul(ft1.xy, ft1, fc[1]);
		a.add(rtemp(1, "xy"), rtemp(1), rconst(1)); //add(ft1.xy, ft1, fc[1]);


		samplers.add(0, SHADER_SAMPLER_DIM::_2D,
					 SHADER_SAMPLER_REPEAT::CLAMP,
					 SHADER_SAMPLER_FILTER::NEAREST,
					 SHADER_SAMPLER_MIPMAP::NONE,
					 SHADER_SAMPLER_TYPE::RGBA);

		a.texld(rtemp(2), rtemp(1), rsampler(0)); //tex(ft2, ft1, fs0);

		a.mov(rtemp(0, "z"), rtemp(2, "z")); //mov(ft0.z, ft2.z);
	}

	a.mov(rcolorout(0), rtemp(0)); //mov(oc, ft0);



	HLSL_ASM_CAPTION caption;
	caption.dcl(rtexture(0));
	caption.dcl_2d(rsampler(0));

	if (a.checkUsedRegister(rsampler(1)))
		caption.dcl_2d(rsampler(1));

	if (a.checkUsedRegister(rtexture(1)))
		caption.dcl_2d(rtexture(1));



	QByteArray data = collect_pixel(caption, a);

	return data;



	/*
	var _loc6_:SamplerRepeat;
	var _loc7_:SamplerFilter;
	var _loc8_:SamplerMipMap;
	var _loc9_:SamplerType;
	super();
	if (param2)
	{
		_loc6_ = ((!(!(param4))) ? SamplerRepeat.WRAP : SamplerRepeat.CLAMP);
		_loc7_ = ((!(!(param3))) ? SamplerFilter.LINEAR : SamplerFilter.NEAREST);
		_loc8_ = ((!(!(param5))) ? ((!(!(param3))) ? SamplerMipMap.LINEAR : SamplerMipMap.NEAREST) : SamplerMipMap.NONE);
		_loc9_ = SamplerType.RGBA;
		tex(ft0, v1, fs1.dim(SamplerDim.D2).repeat(_loc6_).filter(_loc7_).mipmap(_loc8_).type(_loc9_));
		sub(ft0.w, ft0, v1);
		kil(ft0.w);
	}
	frc(ft0, v0.z);
	sub(ft0.x, v0.z, ft0);
	mul(ft0.x, ft0, fc[0]);
	if (param1)
	{
		mov(ft1.zw, fc[0]);
		mov(ft1.xy, v0);
		nrm(ft1.xyz, ft1.xyz);
		mul(ft1.xy, ft1, fc[1]);
		add(ft1.xy, ft1, fc[1]);
		tex(ft2, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
		mov(ft0.w, ft2.z);
		mul(ft1.xy, v0, v0);
		add(ft1.x, ft1, ft1.y);
		sqt(ft1.x, ft1);
		neg(ft1.y, v0.w);
		mul(ft1.xy, ft1, fc[1]);
		add(ft1.xy, ft1, fc[1]);
		tex(ft2, ft1, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
		mov(ft0.z, ft2.z);
	}
	mov(oc, ft0);
	*/
}


#if 0
dcl t0
dcl_2d s0
0:     frc r0, t0.z
1:     add r0.x, t0.z, -r0
2:     mul r0.x, r0, c0
3:     mov r1.zw, c0
4:     mov r1.xy, t0
5:     dp3 r8.x, r1.xyzz, r1.xyzz
6:     rsq r8.x, r8.x
7:     mul r1.xyz, r8.x, r1.xyzz
8:     mul r1.xy, r1, c1
9:     add r1.xy, r1, c1
10:     texld r2, r1, s0
10:     mov r0.w, r2.z
11:     mov r8, t0
12:     mul r1.xy, r8, t0
13:     add r1.x, r1, r1.y
14:     rcp r9.x, r1.x
15:     rsq r1.x, r9.x
16:     mov r1.y, -t0.w
17:     mul r1.xy, r1, c1
18:     add r1.xy, r1, c1
19:     texld r2, r1, s0
19:     mov r0.z, r2.z
20:     mov oC0, r0

// approximately 23 instruction slots used (2 texture, 21 arithmetic)
0xFFFF0201,
0x0200001F, 0x80000000, 0xB00F0000,
0x0200001F, 0x90000000, 0xA00F0800,
0 0x02000013, 0x800F0000, 0xB0AA0000, frc
1 0x03000002, 0x80010000, 0xB0AA0000, 0x81E40000, add
2 0x03000005, 0x80010000, 0x80E40000, 0xA0E40000, mul
3 0x02000001, 0x800C0001, 0xA0E40000, mov
4 0x02000001, 0x80030001, 0xB0E40000, mov
5 0x03000008, 0x80010008, 0x80A40001, 0x80A40001, dp3
6 0x02000007, 0x80010008, 0x80000008, rsq
7 0x03000005, 0x80070001, 0x80000008, 0x80A40001, mul
8 0x03000005, 0x80030001, 0x80E40001, 0xA0E40001, mul
9 0x03000002, 0x80030001, 0x80E40001, 0xA0E40001, add
10 0x03000042, 0x800F0002, 0x80E40001, 0xA0E40800, tex
10 0x02000001, 0x80080000, 0x80AA0002, mov
11 0x02000001, 0x800F0008, 0xB0E40000, mov
12 0x03000005, 0x80030001, 0x80E40008, 0xB0E40000, mul
13 0x03000002, 0x80010001, 0x80E40001, 0x80550001, add
14 0x02000006, 0x80010009, 0x80000001, rcp
15 0x02000007, 0x80010001, 0x80000009, rsq
16 0x02000001, 0x80020001, 0xB1FF0000, mov
17 0x03000005, 0x80030001, 0x80E40001, 0xA0E40001,
18 0x03000002, 0x80030001, 0x80E40001, 0xA0E40001, add
19 0x03000042, 0x800F0002, 0x80E40001, 0xA0E40800, tex
19 0x02000001, 0x80040000, 0x80AA0002, mov
20 0x02000001, 0x800F0800, 0x80E40000, mov
0x0000FFFF,
#endif
