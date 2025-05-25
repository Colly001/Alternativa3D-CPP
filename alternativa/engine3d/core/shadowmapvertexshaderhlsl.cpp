#include "shadowmapvertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray ShadowMapVertexShaderHLSL(bool param1, bool param2)
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;

	if (param2)
	{
		//mova a0.x, v0.x
		a.mova(raddr(0, "x"), rinput(0, "x"));

		//mov r0, c0[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(0));
		a.add_src(A(rconst(0)));
		a.add_src(raddr(0, "x"));

		a.mov(routput(0), rtemp(0)); //mov oT0, r0
		a.mul(rtemp(0, "z"), rtemp(0), rconst(8)); //mul r0.z, r0, c8
		a.mov(rtemp(12), rtemp(0)); //mov r12, r0

		if (param1)
		{
			a.mov(rtemp(0), rconst(10)); //mov r0, c10

			//mov r9, c1[a0.x]
			a.add_opcode(OPCODE_MOV, 3);
			a.add_dst(rtemp(9));
			a.add_src(A(rconst(1)));
			a.add_src(raddr(0, "x"));

			a.mul(routput(1), rtemp(9), rtemp(0)); //mul oT1, r9, r0
		}
	}
	else
	{
		a.mov(rtemp(0), rconst(3)); //mov r0, c3
		a.dp4(rtemp(0, "x"), rinput(0), rconst(0)); //dp4 r0.x, v0, c0
		a.dp4(rtemp(0, "y"), rinput(0), rconst(1)); //dp4 r0.y, v0, c1
		a.dp4(rtemp(0, "z"), rinput(0), rconst(2)); //dp4 r0.z, v0, c2
		a.mov(routput(0), rtemp(0)); //mov oT0, r0
		a.mul(rtemp(0, "z"), rtemp(0), rconst(3)); //mul r0.z, r0, c3
		a.mov(rtemp(12), rtemp(0)); //mov r12, r0

		if (param1)
		{
			a.mul(routput(1), rinput(1), rconst(10)); //mul oT1, v1, c10
		}
	}

	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12



	HLSL_ASM_CAPTION caption;

	caption.dcl_texcoord(rinput(0));

	if (a.checkUsedRegister(rinput(1)))
		caption.dcl_texcoord1(rinput(1));

	QByteArray data = collect_vertex(caption, a);

	return data;
}


#if 0
  false false
	  dcl_texcoord v0
0:     mov r0, c3
1:     dp4 r0.x, v0, c0
2:     dp4 r0.y, v0, c1
3:     dp4 r0.z, v0, c2
4:     mov oT0, r0
5:     mul r0.z, r0, c3
6:     mov r12, r0
7:     mad oPos, r12.w, c254, r12

		dcl_texcoord v0
   0:     mova a0.x, v0.x
   1:     mov r0, c0[a0.x]
   2:     mov oT0, r0
   3:     mul r0.z, r0, c8
   4:     mov r12, r0
   5:     mov r0, c10
   6:     mov r9, c1[a0.x]
   7:     mul oT1, r9, r0
   8:     mad oPos, r12.w, c254, r12

		  dcl_texcoord v0
		  dcl_texcoord1 v1
   0:     mov r0, c3
   1:     dp4 r0.x, v0, c0
   2:     dp4 r0.y, v0, c1
   3:     dp4 r0.z, v0, c2
   4:     mov oT0, r0
   5:     mul r0.z, r0, c3
   6:     mov r12, r0
   7:     mul oT1, v1, c10
   8:     mad oPos, r12.w, c254, r12


false false
0xFFFE0201,
0x0200001F, 0x80000005, 0x900F0000,
0x02000001, 0x800F0000, 0xA0E40003,
0x03000009, 0x80010000, 0x90E40000, 0xA0E40000,
0x03000009, 0x80020000, 0x90E40000, 0xA0E40001,
0x03000009, 0x80040000, 0x90E40000, 0xA0E40002,
0x02000001, 0xE00F0000, 0x80E40000,
0x03000005, 0x80040000, 0x80E40000, 0xA0E40003,
0x02000001, 0x800F000C, 0x80E40000,
0x04000004, 0xC00F0000, 0x80FF000C, 0xA0E400FE, 0x80E4000C,
0x0000FFFF,


			if (param2)
			{
				mov(vt0, cc.rel(va0.x, 0));
				mov(v0, vt0);
				mul(vt0.z, vt0, vc[8]);
				mov(op, vt0);
				if (param1)
				{
					mov(vt0, vc[10]);
					mul(v1, cc.rel(va0.x, 1), vt0);
				}
			}
			else
			{
				mov(vt0, vc[3]);
				dp4(vt0.x, va0, vc[0]);
				dp4(vt0.y, va0, vc[1]);
				dp4(vt0.z, va0, vc[2]);
				mov(v0, vt0);
				mul(vt0.z, vt0, vc[3]);
				mov(op, vt0);
				if (param1)
				{
					mul(v1, va1, vc[10]);
				}
			}
#endif
