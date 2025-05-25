#include "depthrendererlightfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"



QByteArray DepthRendererLightFragmentShaderHLSL(int n)
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;


	a.mov(rtemp(8), rtexture(0)); //mov r8, t0
	a.rcp(rtemp(9), rtexture(0, "z")); //rcp r9, t0.z
	a.mul(rtemp(4), rtemp(8), rtemp(9)); //mul r4, r8, r9
	a.mul(rtemp(4), rtemp(4), rconst(4)); //mul r4, r4, c4
	a.add(rtemp(4), rtemp(4), rconst(4)); //add r4, r4, c4
	a.texld(rtemp(0), rtemp(4), rsampler(0)); //texld r0, r4, s0
	a.mul(rtemp(0, "zw"), rtemp(0), rconst(6)); //mul r0.zw, r0, c6
	a.mov(rtemp(8), rtemp(0)); //mov r8, r0
	a.mad(rtemp(8), rtemp(8), rconst(31, "x"), rconst(31, "y")); //mad r8, r8, c31.x, c31.y
	a.frc(rtemp(8), rtemp(8)); //frc r8, r8
	a.mad(rtemp(8), rtemp(8), rconst(31, "z"), rconst(31, "w")); //mad r8, r8, c31.z, c31.w
	a.sincos(rtemp(9, "x"), rtemp(8, "x"), rconst(29), rconst(30)); //sincos r9.x, r8.x, c29, c30
	a.mov(rtemp(1, "x"), rtemp(9, "x")); //mov r1.x, r9.x
	a.sincos(rtemp(9, "x"), rtemp(8, "y"), rconst(29), rconst(30)); //sincos r9.x, r8.y, c29, c30
	a.mov(rtemp(1, "y"), rtemp(9, "x")); //mov r1.y, r9.x
	a.sincos(rtemp(9, "x"), rtemp(8, "z"), rconst(29), rconst(30)); //sincos r9.x, r8.z, c29, c30
	a.mov(rtemp(1, "z"), rtemp(9, "x")); //mov r1.z, r9.x
	a.sincos(rtemp(9, "x"), rtemp(8, "w"), rconst(29), rconst(30)); //sincos r9.x, r8.w, c29, c30
	a.mov(rtemp(1, "w"), rtemp(9, "x")); //mov r1.w, r9.x
	a.mov(rtemp(8), rtemp(0)); //mov r8, r0
	a.mad(rtemp(8), rtemp(8), rconst(31, "x"), rconst(31, "y")); //mad r8, r8, c31.x, c31.y
	a.frc(rtemp(8), rtemp(8)); //frc r8, r8
	a.mad(rtemp(8), rtemp(8), rconst(31, "z"), rconst(31, "w"));  //mad r8, r8, c31.z, c31.w
	a.sincos(rtemp(9, "y"), rtemp(8, "x"), rconst(29), rconst(30)); //sincos r9.y, r8.x, c29, c30
	a.mov(rtemp(2, "x"), rtemp(9, "y")); //mov r2.x, r9.y
	a.sincos(rtemp(9, "y"), rtemp(8, "y"), rconst(29), rconst(30)); //sincos r9.y, r8.y, c29, c30
	a.mov(rtemp(2, "y"), rtemp(9, "y")); //mov r2.y, r9.y
	a.sincos(rtemp(9, "y"), rtemp(8, "z"), rconst(29), rconst(30)); //sincos r9.y, r8.z, c29, c30
	a.mov(rtemp(2, "z"), rtemp(9, "y")); //mov r2.z, r9.y
	a.sincos(rtemp(9, "y"), rtemp(8, "w"), rconst(29), rconst(30)); //sincos r9.y, r8.w, c29, c30
	a.mov(rtemp(2, "w"), rtemp(9, "y")); //mov r2.w, r9.y
	a.mul(rtemp(1, "x"), rtemp(1, "w"), rtemp(1, "z")); //mul r1.x, r1.w, r1.z
	a.mul(rtemp(1, "y"), rtemp(2, "w"), rtemp(1, "z")); //mul r1.y, r2.w, r1.z
	a.mov(rtemp(1, "z"), N(rtemp(2))); //mov r1.z, -r2
	a.dp3(rtemp(2, "z"), rtemp(0), rconst(0)); //dp3 r2.z, r0, c0
	a.mul(rtemp(2, "xy"), rtemp(4), rconst(1)); //mul r2.xy, r4, c1
	a.add(rtemp(2, "xy"), rtemp(2), N(rconst(2))); //add r2.xy, r2, -c2
	a.mul(rtemp(2, "xy"), rtemp(2), rtemp(2, "z")); //mul r2.xy, r2, r2.z
	a.mul(rtemp(2, "xy"), rtemp(2), rconst(3)); //mul r2.xy, r2, c3



	if (n == 0)
	{
		a.add(rtemp(3), rtexture(1), N(rtemp(2))); //add r3, t1, -r2
		a.dp3(rtemp(3, "w"), rtemp(3), rtemp(3)); //dp3 r3.w, r3, r3
		a.rcp(rtemp(9, "w"), rtemp(3, "w")); //rcp r9.w, r3.w
		a.rsq(rtemp(3, "w"), rtemp(9, "w")); //rsq r3.w, r9.w
		a.rcp(rtemp(8), rtemp(3, "w")); //rcp r8, r3.w
		a.mul(rtemp(3, "xyz"), rtemp(3), rtemp(8)); //mul r3.xyz, r3, r8
		a.add(rtemp(3, "w"), rtexture(1), N(rtemp(3))); //add r3.w, t1, -r3
		a.mul(rtemp(3, "w"), rtemp(3), rtexture(2)); //mul r3.w, r3, t2
		a.mov_sat(rtemp(3, "w"), rtemp(3)); //mov_sat r3.w, r3
		a.mul(rtemp(3, "w"), rtemp(3), rtemp(3)); //mul r3.w, r3, r3
		a.dp3(rtemp(2, "w"), rtemp(3), rtemp(1)); //dp3 r2.w, r3, r1
		a.max(rtemp(2, "w"), rtemp(2), rconst(4)); //max r2.w, r2, c4
		a.mul(rtemp(3, "w"), rtemp(3), rtemp(2)); //mul r3.w, r3, r2
		a.mul(rtemp(0, "xyz"), rtexture(2), rtemp(3, "w")); //mul r0.xyz, t2, r3.w

		//add r3, t1, -r2
		//dp3 r3.w, r3, r3
		//rcp r9.w, r3.w
		//rsq r3.w, r9.w
		//rcp r8, r3.w
		//mul r3.xyz, r3, r8
		//add r3.w, t1, -r3
		//mul r3.w, r3, t2
		//mov_sat r3.w, r3
		//mul r3.w, r3, r3
		//dp3 r2.w, r3, r1
		//max r2.w, r2, c4
		//mul r3.w, r3, r2
		//mul r0.xyz, t2, r3.w
	}
	else
	{
		if (n == 1)
		{
			a.add(rtemp(3), rtexture(1), N(rtemp(2))); //add r3, t1, -r2
			a.dp3(rtemp(3, "w"), rtemp(3), rtemp(3)); //dp3 r3.w, r3, r3
			a.rcp(rtemp(9, "w"), rtemp(3, "w")); //rcp r9.w, r3.w
			a.rsq(rtemp(3, "w"), rtemp(9, "w")); //rsq r3.w, r9.w
			a.rcp(rtemp(8), rtemp(3, "w")); //rcp r8, r3.w
			a.mul(rtemp(3, "xyz"), rtemp(3), rtemp(8)); //mul r3.xyz, r3, r8
			a.add(rtemp(3, "w"), rtexture(3, "x"), N(rtemp(3))); //add r3.w, t3.x, -r3
			a.mul(rtemp(3, "w"), rtemp(3), rtexture(3, "y")); //mul r3.w, r3, t3.y
			a.mov_sat(rtemp(3, "w"), rtemp(3)); //mov_sat r3.w, r3
			a.mul(rtemp(3, "w"), rtemp(3), rtemp(3)); //mul r3.w, r3, r3
			a.dp3(rtemp(4, "w"), rtemp(3), rtemp(1)); //dp3 r4.w, r3, r1
			a.max(rtemp(4, "w"), rtemp(4), rconst(4)); //max r4.w, r4, c4
			a.mul(rtemp(3, "w"), rtemp(3), rtemp(4)); //mul r3.w, r3, r4
			a.dp3(rtemp(4, "w"), rtemp(3), rtexture(2)); //dp3 r4.w, r3, t2
			a.mov(rtemp(4, "w"), N(rtemp(4))); //mov r4.w, -r4
			a.add(rtemp(4, "w"), rtemp(4), N(rtexture(3, "z"))); //add r4.w, r4, -t3.z
			a.mul(rtemp(4, "w"), rtemp(4), rtexture(3)); //mul r4.w, r4, t3
			a.mov_sat(rtemp(4, "w"), rtemp(4)); //mov_sat r4.w, r4
			a.mul(rtemp(4, "w"), rtemp(4), rtemp(4)); //mul r4.w, r4, r4
			a.mul(rtemp(3, "w"), rtemp(3), rtemp(4)); //mul r3.w, r3, r4

			//add r3, t1, -r2
			//dp3 r3.w, r3, r3
			//rcp r9.w, r3.w
			//rsq r3.w, r9.w
			//rcp r8, r3.w
			//mul r3.xyz, r3, r8
			//add r3.w, t3.x, -r3
			//mul r3.w, r3, t3.y
			//mov_sat r3.w, r3
			//mul r3.w, r3, r3
			//dp3 r4.w, r3, r1
			//max r4.w, r4, c4
			//mul r3.w, r3, r4
			//dp3 r4.w, r3, t2
			//mov r4.w, -r4
			//add r4.w, r4, -t3.z
			//mul r4.w, r4, t3
			//mov_sat r4.w, r4
			//mul r4.w, r4, r4
			//mul r3.w, r3, r4
		}
		else
		{
			if (n == 2)
			{
				a.add(rtemp(4), rtemp(2), N(rtexture(1))); //add r4, r2, -t1
				a.dp3(rtemp(4, "w"), rtemp(4), rtexture(2)); //dp3 r4.w, r4, t2
				a.mul(rtemp(4, "xyz"), rtexture(2), rtemp(4, "w")); //mul r4.xyz, t2, r4.w
				a.add(rtemp(4, "xyz"), rtexture(1), rtemp(4)); //add r4.xyz, t1, r4
				a.abs(rtemp(4, "w"), rtemp(4));  //abs r4.w, r4
				a.add(rtemp(4, "w"), rtexture(3, "z"), N(rtemp(4))); //add r4.w, t3.z, -r4
				a.mul(rtemp(4, "w"), rtemp(4), rtexture(3)); //mul r4.w, r4, t3
				a.mov_sat(rtemp(4, "w"), rtemp(4)); //mov_sat r4.w, r4
				a.mul(rtemp(4, "w"), rtemp(4), rtemp(4)); //mul r4.w, r4, r4
				a.add(rtemp(3), rtemp(4), N(rtemp(2))); //add r3, r4, -r2
				a.dp3(rtemp(3, "w"), rtemp(3), rtemp(3)); //dp3 r3.w, r3, r3
				a.rcp(rtemp(9, "w"), rtemp(3, "w")); //rcp r9.w, r3.w
				a.rsq(rtemp(3, "w"), rtemp(9, "w")); //rsq r3.w, r9.w
				a.rcp(rtemp(8), rtemp(3, "w")); //rcp r8, r3.w
				a.mul(rtemp(3, "xyz"), rtemp(3), rtemp(8)); //mul r3.xyz, r3, r8
				a.add(rtemp(3, "w"), rtexture(3, "x"), N(rtemp(3))); //add r3.w, t3.x, -r3
				a.mul(rtemp(3, "w"), rtemp(3), rtexture(3, "y")); //mul r3.w, r3, t3.y
				a.mov_sat(rtemp(3, "w"), rtemp(3)); //mov_sat r3.w, r3
				a.mul(rtemp(3, "w"), rtemp(3), rtemp(3)); //mul r3.w, r3, r3
				a.mul(rtemp(3, "w"), rtemp(3), rtemp(4)); //mul r3.w, r3, r4
				a.dp3(rtemp(4, "w"), rtemp(3), rtemp(1)); //dp3 r4.w, r3, r1
				a.max(rtemp(4, "w"), rtemp(4), rconst(4)); //max r4.w, r4, c4
				a.mul(rtemp(3, "w"), rtemp(3), rtemp(4)); //mul r3.w, r3, r4

				//add r4, r2, -t1
				//dp3 r4.w, r4, t2
				//mul r4.xyz, t2, r4.w
				//add r4.xyz, t1, r4
				//abs r4.w, r4
				//add r4.w, t3.z, -r4
				//mul r4.w, r4, t3
				//mov_sat r4.w, r4
				//mul r4.w, r4, r4
				//add r3, r4, -r2
				//dp3 r3.w, r3, r3
				//rcp r9.w, r3.w
				//rsq r3.w, r9.w
				//rcp r8, r3.w
				//mul r3.xyz, r3, r8
				//add r3.w, t3.x, -r3
				//mul r3.w, r3, t3.y
				//mov_sat r3.w, r3
				//mul r3.w, r3, r3
				//mul r3.w, r3, r4
				//dp3 r4.w, r3, r1
				//max r4.w, r4, c4
				//mul r3.w, r3, r4
			}
		}

		a.mul(rtemp(0, "xyz"), rtexture(4), rtemp(3, "w")); //mul r0.xyz, t4, r3.w
	}


	a.mov(rcolorout(0), rtemp(0)); //mov oC0, r0


	HLSL_ASM_CAPTION caption;

	for (int i = 0; i <= 4; i++)
		if (a.checkUsedRegister(rtexture(i)))
			caption.dcl(rtexture(i));

	caption.dcl_2d(rsampler(0));


	caption.def(rconst(28), 0, 1, 0, 0);
	caption.def(rconst(29), -1.55009923e-006, -2.17013894e-005, 0.00260416674, 0.00026041668);
	caption.def(rconst(30), -0.020833334, -0.125, 1, 0.5);
	caption.def(rconst(31), 0.159154937, 0.5, 6.28318548, -3.14159274);


	QByteArray data = collect_pixel(caption, a);

	return data;



#if 0
	div(ft4, v0, v0.z);
	mul(ft4, ft4, fc[4]);
	add(ft4, ft4, fc[4]);
	tex(ft0, ft4, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
	mul(ft0.zw, ft0, fc[6]);
	cos(ft1, ft0);
	sin(ft2, ft0);
	mul(ft1.x, ft1.w, ft1.z);
	mul(ft1.y, ft2.w, ft1.z);
	neg(ft1.z, ft2); mov r1.z, -r2
	dp3(ft2.z, ft0, fc[0]);
	mul(ft2.xy, ft4, fc[1]);
	sub(ft2.xy, ft2, fc[2]); add r2.xy, r2, -c2
	mul(ft2.xy, ft2, ft2.z);
	mul(ft2.xy, ft2, fc[3]);
	if (param1 == 0)
	{
		sub(ft3, v1, ft2); add r3, t1, -r2
		dp3(ft3.w, ft3, ft3);
		sqt(ft3.w, ft3);
		div(ft3.xyz, ft3, ft3.w);
		sub(ft3.w, v1, ft3); add r3.w, t1, -r3
		mul(ft3.w, ft3, v2);
		sat(ft3.w, ft3);
		mul(ft3.w, ft3, ft3);
		dp3(ft2.w, ft3, ft1);
		max(ft2.w, ft2, fc[4]);
		mul(ft3.w, ft3, ft2);
		mul(ft0.xyz, v2, ft3.w);
	}
	else
	{
		if (param1 == 1)
		{
			sub(ft3, v1, ft2);
			dp3(ft3.w, ft3, ft3);
			sqt(ft3.w, ft3);
			div(ft3.xyz, ft3, ft3.w);
			sub(ft3.w, v3.x, ft3);
			mul(ft3.w, ft3, v3.y);
			sat(ft3.w, ft3);
			mul(ft3.w, ft3, ft3);
			dp3(ft4.w, ft3, ft1);
			max(ft4.w, ft4, fc[4]);
			mul(ft3.w, ft3, ft4);
			dp3(ft4.w, ft3, v2);
			neg(ft4.w, ft4);
			sub(ft4.w, ft4, v3.z);
			mul(ft4.w, ft4, v3);
			sat(ft4.w, ft4);
			mul(ft4.w, ft4, ft4);
			mul(ft3.w, ft3, ft4);
		}
		else
		{
			if (param1 == 2)
			{
				sub(ft4, ft2, v1);
				dp3(ft4.w, ft4, v2);
				mul(ft4.xyz, v2, ft4.w);
				add(ft4.xyz, v1, ft4);
				abs(ft4.w, ft4);
				sub(ft4.w, v3.z, ft4);
				mul(ft4.w, ft4, v3);
				sat(ft4.w, ft4);
				mul(ft4.w, ft4, ft4);
				sub(ft3, ft4, ft2);
				dp3(ft3.w, ft3, ft3);
				sqt(ft3.w, ft3);
				div(ft3.xyz, ft3, ft3.w);
				sub(ft3.w, v3.x, ft3);
				mul(ft3.w, ft3, v3.y);
				sat(ft3.w, ft3);
				mul(ft3.w, ft3, ft3);
				mul(ft3.w, ft3, ft4);
				dp3(ft4.w, ft3, ft1);
				max(ft4.w, ft4, fc[4]);
				mul(ft3.w, ft3, ft4);
			}
		}
		mul(ft0.xyz, v4, ft3.w);
	}
	mov(oc, ft0);
#endif
}


#if 0
	   dcl t0
	   dcl t1
	   dcl t2
	   dcl_2d s0
	   def c28, 0, 1, 0, 0
	   def c29, -1.55009923e-006, -2.17013894e-005, 0.00260416674, 0.00026041668
	   def c30, -0.020833334, -0.125, 1, 0.5
	   def c31, 0.159154937, 0.5, 6.28318548, -3.14159274
0:     mov r8, t0
1:     rcp r9, t0.z
2:     mul r4, r8, r9
3:     mul r4, r4, c4
4:     add r4, r4, c4
5:     texld r0, r4, s0
5:     mul r0.zw, r0, c6
6:     mov r8, r0
7:     mad r8, r8, c31.x, c31.y
8:     frc r8, r8
9:     mad r8, r8, c31.z, c31.w
10:     sincos r9.x, r8.x, c29, c30
18:     mov r1.x, r9.x
19:     sincos r9.x, r8.y, c29, c30
27:     mov r1.y, r9.x
28:     sincos r9.x, r8.z, c29, c30
36:     mov r1.z, r9.x
37:     sincos r9.x, r8.w, c29, c30
45:     mov r1.w, r9.x
46:     mov r8, r0
47:     mad r8, r8, c31.x, c31.y
48:     frc r8, r8
49:     mad r8, r8, c31.z, c31.w
50:     sincos r9.y, r8.x, c29, c30
58:     mov r2.x, r9.y
59:     sincos r9.y, r8.y, c29, c30
67:     mov r2.y, r9.y
68:     sincos r9.y, r8.z, c29, c30
76:     mov r2.z, r9.y
77:     sincos r9.y, r8.w, c29, c30
85:     mov r2.w, r9.y
86:     mul r1.x, r1.w, r1.z
87:     mul r1.y, r2.w, r1.z
88:     mov r1.z, -r2
89:     dp3 r2.z, r0, c0
90:     mul r2.xy, r4, c1
91:     add r2.xy, r2, -c2
92:     mul r2.xy, r2, r2.z
93:     mul r2.xy, r2, c3
if (param1 == 0)
{
94:     add r3, t1, -r2
95:     dp3 r3.w, r3, r3
96:     rcp r9.w, r3.w
97:     rsq r3.w, r9.w
98:     rcp r8, r3.w
99:     mul r3.xyz, r3, r8
100:     add r3.w, t1, -r3
101:     mul r3.w, r3, t2
102:     mov_sat r3.w, r3
103:     mul r3.w, r3, r3
104:     dp3 r2.w, r3, r1
105:     max r2.w, r2, c4
106:     mul r3.w, r3, r2
107:     mul r0.xyz, t2, r3.w
}
108:     mov oC0, r0



0xFFFF0201,
0x0200001F, 0x80000000, 0xB00F0000,
0x0200001F, 0x80000000, 0xB00F0001,
0x0200001F, 0x80000000, 0xB00F0002,
0x0200001F, 0x90000000, 0xA00F0800,

0x05000051, 0xA00F001C, 0x00000000, 0x3F800000, 0x00000000, 0x00000000,
0x05000051, 0xA00F001D, 0xB5D00D01, 0xB7B60B61, 0x3B2AAAAB, 0x39888889,
0x05000051, 0xA00F001E, 0xBCAAAAAB, 0xBE000000, 0x3F800000, 0x3F000000,
0x05000051, 0xA00F001F, 0x3E22F983, 0x3F000000, 0x40C90FDB, 0xC0490FDB,

0x02000001, 0x800F0008, 0xB0E40000,
0x02000006, 0x800F0009, 0xB0AA0000,
0x03000005, 0x800F0004, 0x80E40008, 0x80E40009,
0x03000005, 0x800F0004, 0x80E40004, 0xA0E40004,
0x03000002, 0x800F0004, 0x80E40004, 0xA0E40004,
0x03000042, 0x800F0000, 0x80E40004, 0xA0E40800,
0x03000005, 0x800C0000, 0x80E40000, 0xA0E40006,
0x02000001, 0x800F0008, 0x80E40000,
0x04000004, 0x800F0008, 0x80E40008, 0xA000001F, 0xA055001F, //mad
0x02000013, 0x800F0008, 0x80E40008, //frc
0x04000004, 0x800F0008, 0x80E40008, 0xA0AA001F, 0xA0FF001F, //mad
0x04000025, 0x80010009, 0x80000008, 0xA0E4001D, 0xA0E4001E, //sincos
0x02000001, 0x80010001, 0x80000009,
0x04000025, 0x80010009, 0x80550008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80020001, 0x80000009,
0x04000025, 0x80010009, 0x80AA0008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80040001, 0x80000009,
0x04000025, 0x80010009, 0x80FF0008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80080001, 0x80000009,
0x02000001, 0x800F0008, 0x80E40000,
0x04000004, 0x800F0008, 0x80E40008, 0xA000001F, 0xA055001F,
0x02000013, 0x800F0008, 0x80E40008,
0x04000004, 0x800F0008, 0x80E40008, 0xA0AA001F, 0xA0FF001F,
0x04000025, 0x80020009, 0x80000008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80010002, 0x80550009,
0x04000025, 0x80020009, 0x80550008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80020002, 0x80550009,
0x04000025, 0x80020009, 0x80AA0008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80040002, 0x80550009,
0x04000025, 0x80020009, 0x80FF0008, 0xA0E4001D, 0xA0E4001E,
0x02000001, 0x80080002, 0x80550009,
0x03000005, 0x80010001, 0x80FF0001, 0x80AA0001,
0x03000005, 0x80020001, 0x80FF0002, 0x80AA0001,
0x02000001, 0x80040001, 0x81E40002,
0x03000008, 0x80040002, 0x80E40000, 0xA0E40000,
0x03000005, 0x80030002, 0x80E40004, 0xA0E40001,
0x03000002, 0x80030002, 0x80E40002, 0xA1E40002,
0x03000005, 0x80030002, 0x80E40002, 0x80AA0002,
0x03000005, 0x80030002, 0x80E40002, 0xA0E40003,
0x03000002, 0x800F0003, 0xB0E40001, 0x81E40002,
0x03000008, 0x80080003, 0x80E40003, 0x80E40003,
0x02000006, 0x80080009, 0x80FF0003,
0x02000007, 0x80080003, 0x80FF0009,
0x02000006, 0x800F0008, 0x80FF0003,
0x03000005, 0x80070003, 0x80E40003, 0x80E40008,
0x03000002, 0x80080003, 0xB0E40001, 0x81E40003,
0x03000005, 0x80080003, 0x80E40003, 0xB0E40002,
0x02000001, 0x80180003, 0x80E40003,
0x03000005, 0x80080003, 0x80E40003, 0x80E40003,
0x03000008, 0x80080002, 0x80E40003, 0x80E40001,
0x0300000B, 0x80080002, 0x80E40002, 0xA0E40004,
0x03000005, 0x80080003, 0x80E40003, 0x80E40002,
0x03000005, 0x80070000, 0xB0E40002, 0x80FF0003,
0x02000001, 0x800F0800, 0x80E40000,
0x0000FFFF,
#endif
