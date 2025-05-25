#include "depthrendererdepthvertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"



QByteArray DepthRendererDepthVertexShaderHLSL(bool param1, bool param2)
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.mov(rtemp(0), rconst(0)); //mov r0, c0
	a.dp4(rtemp(0, "x"), rinput(0), rconst(0)); //dp4 r0.x, v0, c0
	a.dp4(rtemp(0, "y"), rinput(0), rconst(1)); //dp4 r0.y, v0, c1
	a.dp4(rtemp(0, "z"), rinput(0), rconst(2)); //dp4 r0.z, v0, c2
	a.mul(rtemp(0, "xy"), rtemp(0), rconst(4)); //mul r0.xy, r0, c4
	a.mul(rtemp(1), rconst(5), rtemp(0, "z")); //mul r1, c5, r0.z
	a.add(rtemp(0, "xy"), rtemp(0), N(rtemp(1))); //add r0.xy, r0, -r1
	a.mul(routput(0), rtemp(0), rconst(4)); //mul oT0, r0, c4

	if (param1)
	{
		a.dp3(rtemp(1), rinput(1), rconst(0)); //dp3 r1, v1, c0
		a.dp3(rtemp(1, "y"), rinput(1), rconst(1)); //dp3 r1.y, v1, c1
		a.mul(routput(0, "xy"), rtemp(1), rconst(6)); //mul oT0.xy, r1, c6
		a.dp3(routput(0, "w"), rinput(1), rconst(2)); //dp3 oT0.w, v1, c2
	}

	a.mov(rtemp(12, "xw"), rtemp(0, "xzzz")); //mov r12.xw, r0.xzzz
	a.mov(rtemp(12, "y"), N(rtemp(0))); //mov r12.y, -r0
	a.mul(rtemp(0, "z"), rtemp(0), rconst(3)); //mul r0.z, r0, c3
	a.add(rtemp(12, "z"), rtemp(0), rconst(3, "w")); //add r12.z, r0, c3.w

	if (param2)
	{
		a.mul(routput(1), rinput(2), rconst(7)); //mul(v1, va2, vc[7]);
	}

	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12


	HLSL_ASM_CAPTION caption;

	caption.dcl_texcoord(rinput(0));

	if (a.checkUsedRegister(rinput(1)))
		caption.dcl_texcoord1(rinput(1));

	if (a.checkUsedRegister(rinput(2)))
		caption.dcl_texcoord2(rinput(2));

	QByteArray data = collect_vertex(caption, a);

	return data;

	/*
	mov(vt0, vc[0]);
	dp4(vt0.x, va0, vc[0]);
	dp4(vt0.y, va0, vc[1]);
	dp4(vt0.z, va0, vc[2]);
	mul(vt0.xy, vt0, vc[4]);
	mul(vt1, vc[5], vt0.z);
	sub(vt0.xy, vt0, vt1);
	mul(v0, vt0, vc[4]);
	if (param1)
	{
		dp3(vt1, va1, vc[0]);
		dp3(vt1.y, va1, vc[1]);
		mul(v0.xy, vt1, vc[6]);
		dp3(v0.w, va1, vc[2]);
	}
	mov(op.xw, vt0.xz);
	neg(op.y, vt0);
	mul(vt0.z, vt0, vc[3]);
	add(op.z, vt0, vc[3].w);
	if (param2)
	{
		mul(v1, va2, vc[7]);
	}
	*/
}

#if 0
dcl_texcoord v0
dcl_texcoord1 v1
0:     mov r0, c0
1:     dp4 r0.x, v0, c0
2:     dp4 r0.y, v0, c1
3:     dp4 r0.z, v0, c2
4:     mul r0.xy, r0, c4
5:     mul r1, c5, r0.z
6:     add r0.xy, r0, -r1
7:     mul oT0, r0, c4
8:     dp3 r1, v1, c0
9:     dp3 r1.y, v1, c1
10:     mul oT0.xy, r1, c6
11:     dp3 oT0.w, v1, c2
12:     mov r12.xw, r0.xzzz
13:     mov r12.y, -r0
14:     mul r0.z, r0, c3
15:     add r12.z, r0, c3.w
16:     mad oPos, r12.w, c254, r12


0xFFFE0201,
0x0200001F, 0x80000005, 0x900F0000,
0x0200001F, 0x80010005, 0x900F0001,
0x02000001, 0x800F0000, 0xA0E40000,
0x03000009, 0x80010000, 0x90E40000, 0xA0E40000,
0x03000009, 0x80020000, 0x90E40000, 0xA0E40001,
0x03000009, 0x80040000, 0x90E40000, 0xA0E40002,
0x03000005, 0x80030000, 0x80E40000, 0xA0E40004,
0x03000005, 0x800F0001, 0xA0E40005, 0x80AA0000,
0x03000002, 0x80030000, 0x80E40000, 0x81E40001,
0x03000005, 0xE00F0000, 0x80E40000, 0xA0E40004,
0x03000008, 0x800F0001, 0x90E40001, 0xA0E40000,
0x03000008, 0x80020001, 0x90E40001, 0xA0E40001,
0x03000005, 0xE0030000, 0x80E40001, 0xA0E40006,
0x03000008, 0xE0080000, 0x90E40001, 0xA0E40002,
0x02000001, 0x8009000C, 0x80A80000,
0x02000001, 0x8002000C, 0x81E40000,
0x03000005, 0x80040000, 0x80E40000, 0xA0E40003,
0x03000002, 0x8004000C, 0x80E40000, 0xA0FF0003,
0x04000004, 0xC00F0000, 0x80FF000C, 0xA0E400FE, 0x80E4000C,
0x0000FFFF,
#endif
