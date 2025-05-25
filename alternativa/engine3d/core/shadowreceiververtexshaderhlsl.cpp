#include "shadowreceiververtexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray ShadowReceiverVertexShaderHLSL(bool param1)
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.dp4(rtemp(0, "x"), rinput(0), rconst(11)); //dp4 r0.x, v0, c11
	a.dp4(rtemp(0, "y"), rinput(0), rconst(12)); //dp4 r0.y, v0, c12
	a.dp4(rtemp(0, "z"), rinput(0), rconst(13)); //dp4 r0.z, v0, c13
	a.mov(rtemp(0, "w"), rinput(0)); //mov r0.w, v0
	a.dp4(rtemp(0), rtemp(0), rconst(15)); //dp4 r1, r0, c15
	a.dp4(rtemp(0, "y"), rtemp(0), rconst(16)); //dp4 r1.y, r0, c16
	a.mul(rtemp(1, "xy"), rtemp(1), rconst(19)); //mul r1.xy, r1, c19
	a.add(routput(0, "xy"), rtemp(1), rconst(20)); //add oT0.xy, r1, c20
	a.dp4(rtemp(0, "zw"), rtemp(0), rconst(17)); //dp4 oT0.zw, r0, c17
	a.rcp(rtemp(8, "x"), rtemp(0, "x")); //rcp r8.x, r0.x
	a.rcp(rtemp(8, "y"), rtemp(0, "y")); //rcp r8.y, r0.y
	a.rcp(rtemp(8, "z"), rtemp(0, "z")); //rcp r8.z, r0.z
	a.rcp(rtemp(8, "w"), rtemp(0, "w")); //rcp r8.w, r0.w
	a.mul(rtemp(1, "z"), rconst(14, "w"), rtemp(8)); //mul r1.z, c14.w, r8
	a.add(rtemp(1, "z"), rtemp(1), rconst(14)); //add r1.z, r1, c14
	a.mul(rtemp(1, "z"), rtemp(1), rconst(14, "x")); //mul r1.z, r1, c14.x
	a.add(rtemp(1, "z"), rtemp(1), N(rconst(14, "y"))); //add r1.z, r1, -c14.y
	a.rcp(rtemp(8), rconst(14, "x")); //rcp r8, c14.x
	a.mul(rtemp(1, "z"), rtemp(1), rtemp(8)); //mul r1.z, r1, r8
	a.add(rtemp(1, "z"), rtemp(1), N(rconst(14))); //add r1.z, r1, -c14
	a.rcp(rtemp(8, "x"), rtemp(1, "x")); //rcp r8.x, r1.x
	a.rcp(rtemp(8, "y"), rtemp(1, "y")); //rcp r8.y, r1.y
	a.rcp(rtemp(8, "z"), rtemp(1, "z")); //rcp r8.z, r1.z
	a.rcp(rtemp(8, "w"), rtemp(1, "w")); //rcp r8.w, r1.w
	a.mul(rtemp(1, "z"), rconst(14, "w"), rtemp(8)); //mul r1.z, c14.w, r8
	a.mov(rtemp(2), rconst(20)); //mov r2, c20
	a.nrm(rtemp(2, "xyz"), rtemp(0, "xyzz")); //nrm r2.xyz, r0.xyzz
	a.add(rtemp(1, "z"), rtemp(0), N(rtemp(1))); //add r1.z, r0, -r1
	a.rcp(rtemp(8, "x"), rtemp(2, "x")); //rcp r8.x, r2.x
	a.rcp(rtemp(8, "y"), rtemp(2, "y")); //rcp r8.y, r2.y
	a.rcp(rtemp(8, "z"), rtemp(2, "z")); //rcp r8.z, r2.z
	a.rcp(rtemp(8, "w"), rtemp(2, "w")); //rcp r8.w, r2.w
	a.mul(rtemp(1, "z"), rtemp(1), rtemp(8)); //mul r1.z, r1, r8
	a.mul(rtemp(2), rtemp(2), rtemp(1, "z")); //mul r2, r2, r1.z
	a.add(rtemp(0), rtemp(0), N(rtemp(2))); //add r0, r0, -r2

	if (param1)
	{
		a.mul(rtemp(0, "xy"), rtemp(0), rconst(18)); //mul(vt0.xy, vt0, vc[18]);
		a.mul(rtemp(1, "xy"), rconst(18, "zw"), rtemp(0, "z")); //mul(vt1.xy, vc[18].zw, vt0.z);
		a.add(rtemp(0, "xy"), rtemp(0), rtemp(1)); //add(vt0.xy, vt0, vt1);
	}

	a.mov(rtemp(12, "xw"), rtemp(0, "xzzz")); //mov r12.xw, r0.xzzz
	a.mov(rtemp(12, "y"), N(rtemp(0))); //mov r12.y, -r0
	a.mul(rtemp(0, "z"), rtemp(0), rconst(14)); //mul r0.z, r0, c14
	a.add(rtemp(12, "z"), rtemp(0), rconst(14, "w")); //add r12.z, r0, c14.w
	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12


	HLSL_ASM_CAPTION caption;

	caption.dcl_texcoord(rinput(0));

	if (a.checkUsedRegister(rinput(1)))
		caption.dcl_texcoord1(rinput(1));

	if (a.checkUsedRegister(rinput(2)))
		caption.dcl_texcoord2(rinput(2));

	QByteArray data = collect_vertex(caption, a);

	return data;
}


#if 0

vs_2_x
	  dcl_texcoord v0
0:     dp4 r0.x, v0, c11
1:     dp4 r0.y, v0, c12
2:     dp4 r0.z, v0, c13
3:     mov r0.w, v0
4:     dp4 r1, r0, c15
5:     dp4 r1.y, r0, c16
6:     mul r1.xy, r1, c19
7:     add oT0.xy, r1, c20
8:     dp4 oT0.zw, r0, c17
9:     rcp r8.x, r0.x
10:     rcp r8.y, r0.y
11:     rcp r8.z, r0.z
12:     rcp r8.w, r0.w
13:     mul r1.z, c14.w, r8
14:     add r1.z, r1, c14
15:     mul r1.z, r1, c14.x
16:     add r1.z, r1, -c14.y
17:     rcp r8, c14.x
18:     mul r1.z, r1, r8
19:     add r1.z, r1, -c14
20:     rcp r8.x, r1.x
21:     rcp r8.y, r1.y
22:     rcp r8.z, r1.z
23:     rcp r8.w, r1.w
24:     mul r1.z, c14.w, r8
25:     mov r2, c20
26:     nrm r2.xyz, r0.xyzz
29:     add r1.z, r0, -r1
30:     rcp r8.x, r2.x
31:     rcp r8.y, r2.y
32:     rcp r8.z, r2.z
33:     rcp r8.w, r2.w
34:     mul r1.z, r1, r8
35:     mul r2, r2, r1.z
36:     add r0, r0, -r2
37:     mov r12.xw, r0.xzzz
38:     mov r12.y, -r0
39:     mul r0.z, r0, c14
40:     add r12.z, r0, c14.w
41:     mad oPos, r12.w, c254, r12

dp4(vt0.x, va0, vc[11]);
dp4(vt0.y, va0, vc[12]);
dp4(vt0.z, va0, vc[13]);
mov(vt0.w, va0);
dp4(vt1, vt0, vc[15]);
dp4(vt1.y, vt0, vc[16]);
mul(vt1.xy, vt1, vc[19]);
add(v0.xy, vt1, vc[20]);
dp4(v0.zw, vt0, vc[17]);
div(vt1.z, vc[14].w, vt0);
add(vt1.z, vt1, vc[14]);
mul(vt1.z, vt1, vc[14].x);
sub(vt1.z, vt1, vc[14].y);
div(vt1.z, vt1, vc[14].x);
sub(vt1.z, vt1, vc[14]);
div(vt1.z, vc[14].w, vt1);
mov(vt2, vc[20]);
nrm(vt2.xyz, vt0.xyz);
sub(vt1.z, vt0, vt1);
div(vt1.z, vt1, vt2);
mul(vt2, vt2, vt1.z);
sub(vt0, vt0, vt2);
if (param1)
{
   mul(vt0.xy, vt0, vc[18]);
   mul(vt1.xy, vc[18].zw, vt0.z);
   add(vt0.xy, vt0, vt1);
}
mov(op.xw, vt0.xz);
neg(op.y, vt0);
mul(vt0.z, vt0, vc[14]);
add(op.z, vt0, vc[14].w);
#endif
