#include "shadowvolumevertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray ShadowVolumeVertexShaderHLSL()
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.mul(rtemp(0), rinput(0), rconst(14)); //mul r1, v0, c14
	a.add(rtemp(1, "xyz"), rtemp(1), rconst(15)); //add r1.xyz, r1, c15
	a.dp4(rtemp(0, "x"), rtemp(1), rconst(11)); //dp4 r0.x, r1, c11
	a.dp4(rtemp(0, "y"), rtemp(1), rconst(12)); //dp4 r0.y, r1, c12
	a.dp4(rtemp(0, "z"), rtemp(1), rconst(13)); //dp4 r0.z, r1, c13
	a.mov(rtemp(0, "w"), rtemp(1)); //mov r0.w, r1
	a.mul(rtemp(0, "xy"), rtemp(0), rconst(17)); //mul r0.xy, r0, c17
	a.mul(rtemp(1, "xy"), rconst(17, "zwww"), rtemp(0, "z")); //mul r1.xy, c17.zwww, r0.z
	a.add(rtemp(0, "xy"), rtemp(0), rtemp(1)); //add r0.xy, r0, r1
	a.mov(rtemp(12, "xw"), rtemp(0, "xzzz")); //mov r12.xw, r0.xzzz
	a.mov(rtemp(12, "y"), N(rtemp(0))); //mov r12.y, -r0
	a.mul(rtemp(0, "z"), rtemp(0), rconst(16)); //mul r0.z, r0, c16
	a.add(rtemp(12, "z"), rtemp(0), rconst(16, "w")); //add r12.z, r0, c16.w
	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12


	HLSL_ASM_CAPTION caption;
	caption.dcl_texcoord(rinput(0));
	QByteArray data = collect_vertex(caption, a);
	return data;
}

#if 0
	vs_2_x
		  dcl_texcoord v0
   0:     mul r1, v0, c14
   1:     add r1.xyz, r1, c15
   2:     dp4 r0.x, r1, c11
   3:     dp4 r0.y, r1, c12
   4:     dp4 r0.z, r1, c13
   5:     mov r0.w, r1
   6:     mul r0.xy, r0, c17
   7:     mul r1.xy, c17.zwww, r0.z
   8:     add r0.xy, r0, r1
   9:     mov r12.xw, r0.xzzz
  10:     mov r12.y, -r0
  11:     mul r0.z, r0, c16
  12:     add r12.z, r0, c16.w
  13:     mad oPos, r12.w, c254, r12

mul(vt1, va0, vc[14]);
add(vt1.xyz, vt1, vc[15]);
dp4(vt0.x, vt1, vc[11]);
dp4(vt0.y, vt1, vc[12]);
dp4(vt0.z, vt1, vc[13]);
mov(vt0.w, vt1);
mul(vt0.xy, vt0, vc[17]);
mul(vt1.xy, vc[17].zw, vt0.z);
add(vt0.xy, vt0, vt1);
mov(op.xw, vt0.xz);
neg(op.y, vt0);
mul(vt0.z, vt0, vc[16]);
add(op.z, vt0, vc[16].w);
#endif
