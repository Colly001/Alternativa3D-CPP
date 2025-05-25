#include "paintvertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "materialvertexshaderhlsl.h"
#include <QDebug>


QByteArray PaintVertexShaderHLSL(bool p1,
								 bool p2,
								 bool p3,
								 bool p4,
								 bool p5,
								 bool p6,
								 bool p7,
								 bool p8,
								 bool p9,
								 bool p10)
{

	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	a.mov(rtemp(0), rconst(4)); //mov r0, c4
	//mov(vt0, vc[4]); //mov r0, c4


	if (p1)
	{
		a.dp4(rtemp(0, "x"), rinput(0), rconst(0)); //dp4 r0.x, v0, c0
		a.dp4(rtemp(0, "y"), rinput(0), rconst(1)); //dp4 r0.y, v0, c1
		a.dp4(rtemp(0, "z"), rinput(0), rconst(2)); //dp4 r0.z, v0, c2
		a.mul(routput(0), rinput(1), rconst(4)); //mul oT0, v1, c4
		a.mov(rtemp(1), rconst(4)); //mov r1, c4
		a.dp3(rtemp(1, "x"), rinput(1), rconst(14)); //dp3 r1.x, v1, c14
		a.dp3(rtemp(1, "y"), rinput(1), rconst(15)); //dp3 r1.y, v1, c15
		a.add(rtemp(1, "x"), rtemp(1, "x"), rconst(14, "z")); //add r1.x, r1.x, c14.z
		a.add(rtemp(1, "y"), rtemp(1, "y"), rconst(15, "z")); //add r1.y, r1.y, c15.z
		a.mov(routput(6), rtemp(1)); //mov oT6, r1


		//dp4(vt0->x, va0, vc[0]); //dp4 r0.x, v0, c0
		//dp4(vt0->y, va0, vc[1]); //dp4 r0.y, v0, c1
		//dp4(vt0->z, va0, vc[2]); //dp4 r0.z, v0, c2
		//mul(v0, va1, vc[4]); //mul oT0, v1, c4
		//mov(vt1, vc[4]);  //mov r1, c4
		//dp3(vt1->x, va1, vc[14]); //dp3 r1.x, v1, c14
		//dp3(vt1->y, va1, vc[15]); //dp3 r1.y, v1, c15
		//add(vt1->x, vt1->x, vc[14]->z); //add r1.x, r1.x, c14.z
		//add(vt1->y, vt1->y, vc[15]->z); //add r1.y, r1.y, c15.z
		//mov(v6, vt1); //mov oT6, r1
	}
	else
	{
		a.mova(raddr(0, "x"), rinput(0, "x")); //mova a0.x, v0.x

		//mov(vt0->xyz, cc->rel(va0->x, 0));
		//mov r0.xyz, c0[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(0, "xyz"));
		a.add_src(A(rconst(0)));
		a.add_src(raddr(0, "x"));


		//mov(vt1, cc->rel(va0->x, 1));
		//mov r1, c1[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(1));
		a.add_src(A(rconst(1)));
		a.add_src(raddr(0, "x"));

		a.mul(routput(0), rtemp(1), rconst(4)); //mul oT0, r1, c4

		a.mov(rtemp(2), rconst(4)); //mov(vt2, vc[4]);
		a.dp3(rtemp(2, "x"), rtemp(1), rconst(14)); //dp3(vt2->x, vt1, vc[14]);
		a.dp3(rtemp(2, "y"), rtemp(1), rconst(15)); //dp3(vt2->y, vt1, vc[15]);
		a.add(rtemp(2, "x"), rtemp(2, "x"), rconst(14, "z")); //add(vt2->x, vt2->x, vc[14]->z);
		a.add(rtemp(2, "y"), rtemp(2, "y"), rconst(15, "z")); //add(vt2->y, vt2->y, vc[15]->z);
		a.mov(routput(6), rtemp(2)); //mov oT6, r2
	}

	MaterialVertexShaderHLSL(a, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);

	HLSL_ASM_CAPTION caption;

	if (a.checkUsedRegister(rinput(0))) {
		caption.dcl_texcoord(rinput(0));
	}

	if (a.checkUsedRegister(rinput(1))) {
		caption.dcl_texcoord1(rinput(1));
	}

	if (a.checkUsedRegister(rinput(2))) {
		caption.dcl_texcoord2(rinput(2));
	}

	QByteArray data = collect_vertex(caption, a);

	return data;
}
