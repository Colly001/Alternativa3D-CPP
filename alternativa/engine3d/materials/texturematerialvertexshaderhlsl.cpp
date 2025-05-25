#include "materialvertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray TextureMaterialVertexShaderHLSL(bool p1,
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

		//dp4(vt0->x, va0, vc[0]); //dp4 r0.x, v0, c0
		//dp4(vt0->y, va0, vc[1]); //dp4 r0.y, v0, c1
		//dp4(vt0->z, va0, vc[2]); //dp4 r0.z, v0, c2

		//dp4 oPos.x, v0, c0
		//dp4 oPos.y, v0, c1
		//dp4 oPos.z, v0, c2
		//dp4 oPos.w, v0, c3

		//mul(v0, va1, vc[4]); //mul oT0, v1, c4
	}
	else
	{
		//mova a0.x, v0.x	0x0200002E, 0xB0010000, 0x90000000
		a.mova(raddr(0, "x"), rinput(0, "x"));

		//mov r0.xyz, c0[a0.x]	0x03000001, 0x80070000, 0xA0E42000, 0xB0000000
		//mov(vt0->xyz, cc->rel(va0->x, 0));
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(0, "xyz"));
		a.add_src(A(rconst(0)));
		a.add_src(raddr(0, "x"));

		//mov r1, c1[a0.x]	0x03000001, 0x800F0001, 0xA0E42001, 0xB0000000
		//mov(vt1, cc->rel(va0->x, 1));
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(1));
		a.add_src(A(rconst(1)));
		a.add_src(raddr(0, "x"));

		//mul(v0, vt1, vc[4]);
		//mul oT0, r1, c4
		//0x03000005, 0xE00F0000, 0x80E40001, 0xA0E40004
		a.mul(routput(0), rtemp(1), rconst(4));
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



#if 0
	class S : public HLSL_ASM_VERTEX
	{
		bool _dcl_texcoord = false;
		bool _dcl_texcoord1 = false;

		void caption() override
		{
			if (_dcl_texcoord == true)
				dcl_texcoord(rinput(0)); //dcl_texcoord v0
			if (_dcl_texcoord1 == true)
				dcl_texcoord1(rinput(1)); //dcl_texcoord1 v1

			//dcl_texcoord v0
			//0x0200001F, 0x80000005, 0x900F0000

			//dcl_texcoord1 v1
			//0x0200001F, 0x80010005, 0x900F0001
		}
	public:
		S(bool p1,
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
			mov(rtemp(0), rconst(4)); //mov r0, c4
			//mov(vt0, vc[4]); //mov r0, c4

			if (p1)
			{
				//+
				_dcl_texcoord = true;
				_dcl_texcoord1 = true;
				dp4(rtemp(0, "x"), rinput(0), rconst(0)); //dp4 r0.x, v0, c0
				dp4(rtemp(0, "y"), rinput(0), rconst(1)); //dp4 r0.y, v0, c1
				dp4(rtemp(0, "z"), rinput(0), rconst(2)); //dp4 r0.z, v0, c2
				mul(routput(0), rinput(1), rconst(4)); //mul oT0, v1, c4

				//dp4(vt0->x, va0, vc[0]); //dp4 r0.x, v0, c0
				//dp4(vt0->y, va0, vc[1]); //dp4 r0.y, v0, c1
				//dp4(vt0->z, va0, vc[2]); //dp4 r0.z, v0, c2

				//dp4 oPos.x, v0, c0
				//dp4 oPos.y, v0, c1
				//dp4 oPos.z, v0, c2
				//dp4 oPos.w, v0, c3

				//mul(v0, va1, vc[4]); //mul oT0, v1, c4
			}
			else
			{
				_dcl_texcoord = true;
				//mova a0.x, v0.x	0x0200002E, 0xB0010000, 0x90000000
				mova(raddr(0, "x"), rinput(0, "x"));

				//mov r0.xyz, c0[a0.x]	0x03000001, 0x80070000, 0xA0E42000, 0xB0000000
				//mov(vt0->xyz, cc->rel(va0->x, 0));
				add_opcode(OPCODE_MOV, 3);
				add_dst(rtemp(0, "xyz"));
				add_src(A(rconst(0)));
				add_src(raddr(0, "x"));

				//mov r1, c1[a0.x]	0x03000001, 0x800F0001, 0xA0E42001, 0xB0000000
				//mov(vt1, cc->rel(va0->x, 1));
				add_opcode(OPCODE_MOV, 3);
				add_dst(rtemp(1));
				add_src(A(rconst(1)));
				add_src(raddr(0, "x"));

				//mul(v0, vt1, vc[4]);
				//mul oT0, r1, c4
				//0x03000005, 0xE00F0000, 0x80E40001, 0xA0E40004
				mul(routput(0), rtemp(1), rconst(4));
			}
			if (p2 || p3)
			{
				//mov(v1, vt0);
				mov(routput(1), rtemp(0));
			}
			if (p3)
			{
				if (p1)
				{
					if (p4)
					{
						mul(rtemp(1), rinput(2), rconst(11));
						nrm(rtemp(1, "xyz"), rtemp(1, "xyz"));
						rcp(rtemp(8, "x"), rconst(11, "x")); //rcp r8.x, c11.x
						rcp(rtemp(8, "y"), rconst(11, "y")); //rcp r8.y, c11.y
						rcp(rtemp(8, "z"), rconst(11, "z")); //rcp r8.z, c11.z
						rcp(rtemp(8, "w"), rconst(11, "w")); //rcp r8.w, c11.w
						mul(rtemp(1), rtemp(1), rtemp(8)); //mul r1, r1, r8

						//mul(vt1, va2, vc[11]);
						//nrm(vt1->xyz, vt1->xyz);
						//div(vt1, vt1, vc[11]);
					}
					else
					{
						mov(rtemp(1), rconst(4));
						dp3(rtemp(1, "x"), rinput(2), rconst(0));
						dp3(rtemp(1, "y"), rinput(2), rconst(1));
						dp3(rtemp(1, "z"), rinput(2), rconst(2));

						//mov(vt1, vc[4]);
						//dp3(vt1->x, va2, vc[0]);
						//dp3(vt1->y, va2, vc[1]);
						//dp3(vt1->z, va2, vc[2]);
					}
				}
				else
				{
					//mov(vt2, vc[4]);
					mov(rtemp(2), rconst(4));

					//? mova a0.x, v0.x
					mova(raddr(0, "x"), rinput(0, "x"));

					//mov(vt2->x, cc->rel(va0->x, 0)->w);
					add_opcode(OPCODE_MOV, 3);
					add_dst(rtemp(2, "x"));
					add_src(A(rconst(0, "w")));
					add_src(raddr(0, "x"));

					//mov(vt2->y, cc->rel(va0->x, 1)->z);
					add_opcode(OPCODE_MOV, 3);
					add_dst(rtemp(2, "y"));
					add_src(A(rconst(1, "z")));
					add_src(raddr(0, "x"));

					//mov(vt2->z, cc->rel(va0->x, 1)->w);
					add_opcode(OPCODE_MOV, 3);
					add_dst(rtemp(2, "z"));
					add_src(A(rconst(1, "w")));
					add_src(raddr(0, "x"));



					if (p4)
					{
						//mul(vt1, vt2, vc[11]);
						//nrm(vt1->xyz, vt1->xyz);
						//div(vt1, vt1, vc[11]);
						mul(rtemp(1), rtemp(2), rconst(11)); //mul r1, r2, c11
						nrm(rtemp(1, "xyz"), rtemp(1, "xyz")); //nrm r1.xyz, r1.xyz

						//div r1, r1, c11
						rcp(rtemp(8, "x"), rconst(11, "x")); //rcp r8.x, c11.x
						rcp(rtemp(8, "y"), rconst(11, "y")); //rcp r8.y, c11.y
						rcp(rtemp(8, "z"), rconst(11, "z")); //rcp r8.z, c11.z
						rcp(rtemp(8, "w"), rconst(11, "w")); //rcp r8.w, c11.w
						mul(rtemp(1), rtemp(1), rtemp(8)); //mul r1, r1, r8
					}
					else
					{
						//mov(vt1, vc[4]);
						//dp3(vt1->x, vt2, vc[0]);
						//dp3(vt1->y, vt2, vc[1]);
						//dp3(vt1->z, vt2, vc[2]);
						mov(rtemp(1), rconst(4));
						dp3(rtemp(1, "w"), rtemp(2), rconst(0)); //dp3 r1.x, r2, c0
						dp3(rtemp(1, "y"), rtemp(2), rconst(1)); //dp3 r1.y, r2, c1
						dp3(rtemp(1, "z"), rtemp(2), rconst(2)); //dp3 r1.z, r2, c2
					}
				}

				//dp3(vt1->w, vt1, vc[10]);
				dp3(rtemp(1, "w"), rtemp(1), rconst(10));

				if (p4)
				{
					//sub(vt1->w, vc[4], vt1);
					//mul(v1->w, vt1, vc[11]);
					add(rtemp(1, "w"), rconst(4), N(rtemp(1))); //add r1.w, c4, r1
					mul(routput(1, "w"), rtemp(1), rconst(11)); //mul oT1.w, r1, c11
				}
				else
				{
					//sub(v1->w, vc[4], vt1);
					add(routput(1, "w"), rconst(4), N(rtemp(1))); //add oT1.w, c4, -r1
				}
			}
			else
			{
				if (p5 || p6 || p9 || p10)
				{
					//mov(vt1, vc[4]); //mov r1, c4
					mov(rtemp(1), rconst(4)); //mov r1, c4
				}
			}
			if (p5)
			{
				//dp4(v2->x, vt0, vc[6]);
				//dp4(v2->y, vt0, vc[7]);
				//dp4(v2->z, vt0, vc[8]);
				//sub(vt1->w, vt0->z, vc[9]->x);
				//div(vt1->w, vt1, vc[9]->y);
				//sub(v2->w, vc[4], vt1);

				dp4(routput(2, "x"), rtemp(0), rconst(6)); //dp4 oT2.x, r0, c6
				dp4(routput(2, "y"), rtemp(0), rconst(7)); //dp4 oT2.x, r0, c7
				dp4(routput(2, "z"), rtemp(0), rconst(8)); //dp4 oT2.x, r0, c8

				add(rtemp(1, "w"), rtemp(0, "z"), N(rconst(9, "x"))); //add r1.w, r0.z, c9.x

				//div vt1.w, vt1, vc9.y
				rcp(rtemp(8), rconst(9, "y")); //rcp r8, c9.y
				mul(rtemp(1, "w"), rtemp(1), rtemp(8)); //mul r1.w, r1, r8

				add(routput(1, "w"), rtemp(0, "z"), N(rconst(5, "z"))); //add oT2.w, c4, -r1
			}
			if (p6)
			{
				//sub(vt1->w, vt0->z, vc[5]->z);
				//div(v0->w, vt1, vc[5]);
				add(rtemp(1, "w"), rtemp(0, "z"), N(rconst(5, "z"))); //add r.w, r0.z, -c5.z
				rcp(rtemp(8, "x"), rconst(5, "x")); //rcp r8.x, c5.x
				rcp(rtemp(8, "y"), rconst(5, "y")); //rcp r8.y, c5.y
				rcp(rtemp(8, "z"), rconst(5, "z")); //rcp r8.z, c5.z
				rcp(rtemp(8, "w"), rconst(5, "w")); //rcp r8.w, c5.w
				mul(routput(0, "w"), rtemp(1), rtemp(8)); //mul oT.w, r1, r8 ?
			}
			if (p8)
			{
				//mov(vt1, vt0); // mov r1, r0
				//mul(vt1->x, vt1, vc[11]->w); //mul r1.x, r1, c11.w
				//mul(vt1->y, vt1, vc[12]->w); //mul r1.y, r1, c12.w
				//sub(vt1, vt1, vc[11]); //add r1, r1, -c11
				//dp3(v0->z, vt1, vc[12]); //dp3 oT0.z, r1, c12

				mov(rtemp(1), rtemp(0));
				mul(rtemp(1, "x"), rtemp(1), rconst(11, "w")); //mul r1.x, r1, c11.w
				mul(rtemp(1, "y"), rtemp(1), rconst(12, "w"));  //mul r1.y, r1, c12.w
				add(rtemp(1), rtemp(1), N(rconst(11))); //add r1, r1, -c11
				dp3(routput(0, "z"), rtemp(1), rconst(12)); //dp3 oT0.z, r1, c12
			}
			if (p7)
			{
				//+
				//mul(vt0->xyz, vt0, vc[11]);
				mul(rtemp(0, "xyz"), rtemp(0), rconst(11));
			}
			if (p9)
			{
				//vt1.z = vc3.w / vt0.z; // div
				//div(vt1->z, vc[3]->w, vt0);
				//add(vt1->z, vt1, vc[3]);
				//mul(vt1->z, vt1, vc[3]->x);
				//sub(vt1->z, vt1, vc[3]->y);
				//div(vt1->z, vt1, vc[3]->x);
				//sub(vt1->z, vt1, vc[3]);
				//div(vt1->z, vc[3]->w, vt1);
				//mov(vt2, vc[4]); //mov r2, c4
				//nrm(vt2->xyz, vt0->xyz); //nrm r2.xyz, r0.xyzz
				//sub(vt1->z, vt0, vt1); //add r1.z, r0, -r1
				//div(vt1->z, vt1, vt2);
				//mul(vt2, vt2, vt1->z); //mul r2, r2, r1.z
				//sub(vt0, vt0, vt2); //add r0, r0, -r2
				//div vt1.z, vc3.w, vt0
				rcp(rtemp(8, "x"), rtemp(0, "x")); //rcp r8.x, r0.x
				rcp(rtemp(8, "y"), rtemp(0, "y")); //rcp r8.y, r0.y
				rcp(rtemp(8, "z"), rtemp(0, "z")); //rcp r8.z, r0.z
				rcp(rtemp(8, "w"), rtemp(0, "w")); //rcp r8.w, r0.w
				mul(rtemp(1, "z"), rconst(3, "w"), rtemp(8)); //mul r1.z, c3.w, r8

				add(rtemp(1, "z"), rtemp(1), rconst(3));
				mul(rtemp(1, "z"), rtemp(1), rconst(3, "x")); //vt1.z, vt1, vc3.x
				add(rtemp(1, "z"), rtemp(1), N(rconst(3, "y"))); //add vt1.z, vt1, -vc3.y

				//div vt1.z, vt1, vc3.x
				rcp(rtemp(8), rconst(3, "x")); //rcp r8, c3.x
				mul(rtemp(1, "z"), rtemp(1), rtemp(8)); //mul r1.z, r1, r8

				add(rtemp(1, "z"), rtemp(1), N(rconst(3))); //add r1.z, r1, -c3  add vt1.z, vt1, -vc3

				//div vt1.z, vc3.w, vt1
				rcp(rtemp(8, "x"), rtemp(1, "x")); //rcp r8.x, r1.x
				rcp(rtemp(8, "y"), rtemp(1, "y")); //rcp r8.y, r1.y
				rcp(rtemp(8, "z"), rtemp(1, "z")); //rcp r8.z, r1.z
				rcp(rtemp(8, "w"), rtemp(1, "w")); //rcp r8.w, r1.w
				mul(rtemp(1, "z"), rconst(3, "w"), rtemp(8)); //mul r1.z, c3.w, r8  mul vt1.z, vc3.w, r8



				mov(rtemp(2), rconst(4)); //mov r2, c4
				nrm(rtemp(2, "xyz"), rtemp(0, "xyz")); //nrm r2.xyz, r0.xyzz

				add(rtemp(1, "z"), rtemp(0), N(rtemp(1))); //add r1.z, r0, -r1

				//div vt1.z, vt1, vt2
				rcp(rtemp(8, "x"), rtemp(2, "x")); //rcp r8.x, r2.x
				rcp(rtemp(8, "y"), rtemp(2, "y")); //rcp r8.y, r2.y
				rcp(rtemp(8, "z"), rtemp(2, "z")); //rcp r8.z, r2.z
				rcp(rtemp(8, "w"), rtemp(2, "w")); //rcp r8.w, r2.w
				mul(rtemp(1, "z"), rtemp(1), rtemp(8)); //mul r1.z, r1, r8


				mul(rtemp(2), rtemp(2), rtemp(1, "z")); //mul r2, r2, r1.z
				add(rtemp(0), rtemp(0), N(rtemp(2))); //add r0, r0, -r2
			}
			if (p10)
			{
				//mul(vt0->xy, vt0, vc[13]);
				//mul(vt1->xy, vc[13]->zw, vt0->z);
				//add(vt0->xy, vt0, vt1);
				mul(rtemp(0, "xy"), rtemp(0), rconst(13));
				mul(rtemp(1, "xy"), rconst(13, "zw"), rtemp(0, "z"));
				add(rtemp(0, "xy"), rtemp(0), rtemp(1));
			}

			//mov(op->xw, vt0->xz); //mov r12.xw, r0.xzzz
			//neg(op->y, vt0); //mov r12.y, -r0
			//mul(vt0->z, vt0, vc[3]); // mul r0.z, r0, c3
			//add(op->z, vt0, vc[3]->w); //add r12.z, r0, c3.w

			mov(rtemp(12, "xw"), rtemp(0, "xzzz")); //mov r12.xw, r0.xzzz
			mov(rtemp(12, "y"), N(rtemp(0))); //mov r12.y, -r0
			mul(rtemp(0, "z"), rtemp(0), rconst(3)); //mul r0.z, r0, c3
			add(rtemp(12, "z"), rtemp(0), rconst(3, "w")); //add r12.z, r0, c3.w
			mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); // mad oPos, r12.w, c254, r12

			end();
		}


	} s(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);
	QByteArray data = a.data;
#endif



	return data;
}
