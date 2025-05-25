#include "depthrendererssaovertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray DepthRendererSSAOVertexShaderHLSL()
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.mov(rtemp(12), rinput(0)); //mov r12, v0
	a.mov(routput(0), rinput(1)); //mov oT0, v1
	a.mul(routput(1), rinput(1), rconst(0)); //mul oT1, v1, c0
	a.mul(rtemp(1), rinput(1), rconst(1)); //mul r1, v1, c1
	a.add(routput(2), rtemp(1), N(rconst(2))); //add oT2, r1, -c2
	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12

	HLSL_ASM_CAPTION caption;

	caption.dcl_texcoord(rinput(0));
	caption.dcl_texcoord1(rinput(1));

	QByteArray data = collect_vertex(caption, a);

	return data;
}


//0 mov(op, va0);
//1 mov(v0, va1);
//2 mul(v1, va1, vc[0]);
//3 mul(vt1, va1, vc[1]);
//4 sub(v2, vt1, vc[2]);


//vs_2_x
//	  dcl_texcoord v0
//	  dcl_texcoord1 v1
//0:     mov r12, v0
//1:     mov oT0, v1
//2:     mul oT1, v1, c0
//3:     mul r1, v1, c1
//4:     add oT2, r1, -c2
//5:     mad oPos, r12.w, c254, r12

