#include "shadowcastervertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray ShadowCasterVertexShaderHLSL()
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.dp4(rtemp(0, "x"), rinput(0), rconst(0)); //dp4 r0.x, v0, c0
	a.dp4(rtemp(0, "y"), rinput(0), rconst(1)); //dp4 r0.y, v0, c1
	a.dp4(rtemp(0, "zw"), rinput(0), rconst(2)); //dp4 r0.zw, v0, c2
	a.mul(rtemp(0, "xy"), rtemp(0), rconst(3)); //mul r0.xy, r0, c3
	a.add(rtemp(0, "xy"), rtemp(0), rconst(4)); //add r0.xy, r0, c4
	a.mov(rtemp(12, "xyz"), rtemp(0)); //mov r12.xyz, r0
	a.mov(rtemp(12, "w"), rconst(3)); //mov r12.w, c3
	a.mov(routput(0), rtemp(0)); //mov oT0, r0
	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12

	HLSL_ASM_CAPTION caption;
	caption.dcl_texcoord(rinput(0));

	QByteArray data = collect_vertex(caption, a);
	return data;
}

//dp4(vt0.x, va0, vc[0]);
//dp4(vt0.y, va0, vc[1]);
//dp4(vt0.zw, va0, vc[2]);
//mul(vt0.xy, vt0, vc[3]);
//add(vt0.xy, vt0, vc[4]);
//mov(op.xyz, vt0);
//mov(op.w, vc[3]);
//mov(v0, vt0);

/*
vs_2_x
	  dcl_texcoord v0
0:     dp4 r0.x, v0, c0
1:     dp4 r0.y, v0, c1
2:     dp4 r0.zw, v0, c2
3:     mul r0.xy, r0, c3
4:     add r0.xy, r0, c4
5:     mov r12.xyz, r0
6:     mov r12.w, c3
7:     mov oT0, r0
8:     mad oPos, r12.w, c254, r12
*/
