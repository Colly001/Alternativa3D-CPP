#include "shadowatlasvertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray ShadowAtlasVertexShaderHLSL()
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.mov(rtemp(12), rinput(0)); //mov r12, v0
	a.mov(routput(0), rinput(1)); //mov oT0, v1
	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12


	HLSL_ASM_CAPTION caption;
	caption.dcl_texcoord(rinput(0));
	caption.dcl_texcoord1(rinput(1));

	QByteArray data = collect_vertex(caption, a);

	return data;
}

//mov(op, va0);
//mov(v0, va1);

//vs_2_x
//	   dcl_texcoord v0
//	   dcl_texcoord1 v1
//0:     mov r12, v0
//1:     mov oT0, v1
//2:     mad oPos, r12.w, c254, r12
