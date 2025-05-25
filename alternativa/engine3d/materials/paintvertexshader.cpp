#include "paintvertexshader.h"
#include "sharedparamsbits.h"
#include "alternativa/gfx/agal/vertexshader.h"
#include "alternativa/gfx/agal/agaltest.h"
#include <QDebug>


//#define CHECK_AGAL

#ifdef CHECK_AGAL
namespace
{
	class Test : public AgalTest
	{
	public:

		Test()
		{
			//fe1ae7c7d183777d8ac60ace6981c626
			addData("a0 01 00 00 00 a1 00 00 00 00 00 00 00 0f 02 04 00 00 e4 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 07 02 00 00 00 e4 01 00 00 80 00 00 00 00 00 00 00 00 00 00 00 00 01 00 0f 02 00 00 01 e4 01 00 00 80 00 00 00 00 00 00 00 00 03 00 00 00 00 00 0f 04 01 00 00 e4 02 00 00 00 04 00 00 e4 01 00 00 00 00 00 00 00 02 00 0f 02 04 00 00 e4 01 00 00 00 00 00 00 00 00 00 00 00 12 00 00 00 02 00 01 02 01 00 00 e4 02 00 00 00 0e 00 00 e4 01 00 00 00 12 00 00 00 02 00 02 02 01 00 00 e4 02 00 00 00 0f 00 00 e4 01 00 00 00 01 00 00 00 02 00 01 02 02 00 00 00 02 00 00 00 0e 00 00 aa 01 00 00 00 01 00 00 00 02 00 02 02 02 00 00 55 02 00 00 00 0f 00 00 aa 01 00 00 00 00 00 00 00 06 00 0f 04 02 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 09 03 00 00 00 a8 02 00 00 00 00 00 00 00 00 00 00 00 15 00 00 00 00 00 02 03 00 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 00 00 04 02 00 00 00 e4 02 00 00 00 03 00 00 e4 01 00 00 00 01 00 00 00 00 00 04 03 00 00 00 e4 02 00 00 00 03 00 00 ff 01 00 00 00");
			addKey(0x00000000); //false false false false false false false false false false

			//faaa9a1e99604a96ee32f890ea093a7d
			addData("a0 01 00 00 00 a1 00 00 00 00 00 00 00 0f 02 04 00 00 e4 01 00 00 00 00 00 00 00 00 00 00 00 13 00 00 00 00 00 01 02 00 00 00 e4 00 00 00 00 00 00 00 e4 01 00 00 00 13 00 00 00 00 00 02 02 00 00 00 e4 00 00 00 00 01 00 00 e4 01 00 00 00 13 00 00 00 00 00 04 02 00 00 00 e4 00 00 00 00 02 00 00 e4 01 00 00 00 03 00 00 00 00 00 0f 04 01 00 00 e4 00 00 00 00 04 00 00 e4 01 00 00 00 00 00 00 00 01 00 0f 02 04 00 00 e4 01 00 00 00 00 00 00 00 00 00 00 00 12 00 00 00 01 00 01 02 01 00 00 e4 00 00 00 00 0e 00 00 e4 01 00 00 00 12 00 00 00 01 00 02 02 01 00 00 e4 00 00 00 00 0f 00 00 e4 01 00 00 00 01 00 00 00 01 00 01 02 01 00 00 00 02 00 00 00 0e 00 00 aa 01 00 00 00 01 00 00 00 01 00 02 02 01 00 00 55 02 00 00 00 0f 00 00 aa 01 00 00 00 00 00 00 00 06 00 0f 04 01 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 09 03 00 00 00 a8 02 00 00 00 00 00 00 00 00 00 00 00 15 00 00 00 00 00 02 03 00 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 00 00 04 02 00 00 00 e4 02 00 00 00 03 00 00 e4 01 00 00 00 01 00 00 00 00 00 04 03 00 00 00 e4 02 00 00 00 03 00 00 ff 01 00 00 00");
			addKey(0x00000001); //true false false false false false false false false false
		}
	} test;
}
#endif


QByteArray PaintVertexShader(bool p1,
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

#ifdef CHECK_AGAL
	ShaderParamsBits b;
	b.add(p1);
	b.add(p2);
	b.add(p3);
	b.add(p4);
	b.add(p5);
	b.add(p6);
	b.add(p7);
	b.add(p8);
	b.add(p9);
	b.add(p10);
#endif

	class S : public VertexShader
	{
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
			mov(vt0, vc[4]); //mov r0, c4

			if (p1)
			{
				dp4(vt0->x, va0, vc[0]); //dp4 r0.x, v0, c0
				dp4(vt0->y, va0, vc[1]); //dp4 r0.y, v0, c1
				dp4(vt0->z, va0, vc[2]); //dp4 r0.z, v0, c2
				mul(v0, va1, vc[4]); //mul oT0, v1, c4

				mov(vt1, vc[4]);  //mov r1, c4
				dp3(vt1->x, va1, vc[14]); //dp3 r1.x, v1, c14
				dp3(vt1->y, va1, vc[15]); //dp3 r1.y, v1, c15
				add(vt1->x, vt1->x, vc[14]->z); //add r1.x, r1.x, c14.z
				add(vt1->y, vt1->y, vc[15]->z); //add r1.y, r1.y, c15.z
				mov(v6, vt1);//mov oT6, r1
			}
			else
			{
				mov(vt0->xyz, cc->rel(va0->x, 0));
				mov(vt1, cc->rel(va0->x, 1));
				mul(v0, vt1, vc[4]);

				mov(vt2, vc[4]);
				dp3(vt2->x, vt1, vc[14]);
				dp3(vt2->y, vt1, vc[15]);
				add(vt2->x, vt2->x, vc[14]->z);
				add(vt2->y, vt2->y, vc[15]->z);
				mov(v6, vt2); //mov oT6, r2
			}

			if (p2 || p3)
			{
				mov(v1, vt0); //mov oT1, r0
			}
			if (p3)
			{
				if (p1)
				{
					if (p4)
					{
						mul(vt1, va2, vc[11]); //mul r1, v2, c11
						//nrm r8.xyz, r1.xyzz
						//mov r1.xyz, r8.xyzz
						nrm(vt1->xyz, vt1->xyz);
						//rcp r8.x, c11.x
						//rcp r8.y, c11.y
						//rcp r8.z, c11.z
						//rcp r8.w, c11.w
						//mul r1, r1, r8
						div(vt1, vt1, vc[11]);
					}
					else
					{
						mov(vt1, vc[4]); //mov r1, c4
						dp3(vt1->x, va2, vc[0]); //dp3 r1.x, v2, c0
						dp3(vt1->y, va2, vc[1]); //dp3 r1.y, v2, c1
						dp3(vt1->z, va2, vc[2]); //dp3 r1.z, v2, c2
					}
				}
				else
				{
					mov(vt2, vc[4]); //mov r2, c4
					mov(vt2->x, cc->rel(va0->x, 0)->w); //mov r2.x, c0[a0.x].w
					mov(vt2->y, cc->rel(va0->x, 1)->z); //mov r2.y, c1[a0.x].z
					mov(vt2->z, cc->rel(va0->x, 1)->w); //mov r2.z, c1[a0.x].w
					if (p4)
					{
						mul(vt1, vt2, vc[11]); //mul r1, r2, c11
						//nrm r8.xyz, r1.xyzz
						//mov r1.xyz, r8.xyzz
						nrm(vt1->xyz, vt1->xyz);

						//rcp r8.x, c11.x
						//rcp r8.y, c11.y
						//rcp r8.z, c11.z
						//rcp r8.w, c11.w
						//mul r1, r1, r8
						div(vt1, vt1, vc[11]);
					}
					else
					{
						mov(vt1, vc[4]); //mov r1, c4
						dp3(vt1->x, vt2, vc[0]); //dp3 r1.x, r2, c0
						dp3(vt1->y, vt2, vc[1]); //dp3 r1.y, r2, c1
						dp3(vt1->z, vt2, vc[2]); //dp3 r1.z, r2, c2
					}
				}
				dp3(vt1->w, vt1, vc[10]); //dp3 r1.w, r1, c10
				if (p4)
				{
					sub(vt1->w, vc[4], vt1); //add r1.w, c4, -r1
					mul(v1->w, vt1, vc[11]); //mul oT1.w, r1, c11
				}
				else
				{
					sub(v1->w, vc[4], vt1); //add oT1.w, c4, -r1
				}
			}
			else
			{
				if (p5 || p6 || p9 || p10)
				{
					mov(vt1, vc[4]); //mov r1, c4
				}
			}
			if (p5)
			{
				dp4(v2->x, vt0, vc[6]); //dp4 oT2.x, r0, c6
				dp4(v2->y, vt0, vc[7]); //dp4 oT2.y, r0, c7
				dp4(v2->z, vt0, vc[8]); //dp4 oT2.z, r0, c8
				sub(vt1->w, vt0->z, vc[9]->x); //add r1.w, r0.z, -c9.x
				//rcp r8, c9.y
				//mul r1.w, r1, r8
				div(vt1->w, vt1, vc[9]->y);
				sub(v2->w, vc[4], vt1); //add oT2.w, c4, -r1
			}
			if (p6)
			{
				sub(vt1->w, vt0->z, vc[5]->z); //add r1.w, r0.z, -c5.z
				//rcp r8.x, c5.x
				//rcp r8.y, c5.y
				//rcp r8.z, c5.z
				//rcp r8.w, c5.w
				//mul oT0.w, r1, r8
				div(v0->w, vt1, vc[5]);
			}
			if (p8)
			{
				mov(vt1, vt0); //mov r1, r0
				mul(vt1->x, vt1, vc[11]->w); //mul r1.x, r1, c11.w
				mul(vt1->y, vt1, vc[12]->w); //mul r1.y, r1, c12.w
				sub(vt1, vt1, vc[11]); //add r1, r1, -c11
				dp3(v0->z, vt1, vc[12]); //dp3 oT0.z, r1, c12
			}
			if (p7)
			{
				mul(vt0->xyz, vt0, vc[11]); //mul r0.xyz, r0, c11
			}
			if (p9)
			{
				//rcp r8.x, r0.x
				//rcp r8.y, r0.y
				//rcp r8.z, r0.z
				//rcp r8.w, r0.w
				//mul r1.z, c3.w, r8
				div(vt1->z, vc[3]->w, vt0);
				add(vt1->z, vt1, vc[3]); //add r1.z, r1, c3
				mul(vt1->z, vt1, vc[3]->x); //mul r1.z, r1, c3.x
				sub(vt1->z, vt1, vc[3]->y); //add r1.z, r1, -c3.y
				//rcp r8, c3.x
				//mul r1.z, r1, r8
				div(vt1->z, vt1, vc[3]->x);
				sub(vt1->z, vt1, vc[3]); //add r1.z, r1, -c3
				//rcp r8.x, r1.x
				//rcp r8.y, r1.y
				//rcp r8.z, r1.z
				//rcp r8.w, r1.w
				//mul r1.z, c3.w, r8
				div(vt1->z, vc[3]->w, vt1);
				mov(vt2, vc[4]); //mov r2, c4
				nrm(vt2->xyz, vt0->xyz); //nrm r2.xyz, r0.xyzz
				sub(vt1->z, vt0, vt1); //add r1.z, r0, -r1
				//rcp r8.x, r2.x
				//rcp r8.y, r2.y
				//rcp r8.w, r2.w
				//mul r1.z, r1, r8
				div(vt1->z, vt1, vt2);
				mul(vt2, vt2, vt1->z); //mul r2, r2, r1.z
				sub(vt0, vt0, vt2); //add r0, r0, -r2
			}
			if (p10)
			{
				mul(vt0->xy, vt0, vc[13]); //mul r0.xy, r0, c13
				mul(vt1->xy, vc[13]->zw, vt0->z); //mul r1.xy, c13.zwww, r0.z
				add(vt0->xy, vt0, vt1); //add r0.xy, r0, r1
			}
			mov(op->xw, vt0->xz); //mov r12.xw, r0.xzzz
			neg(op->y, vt0); //mov r12.y, -r0
			mul(vt0->z, vt0, vc[3]); //mul r0.z, r0, c3
			add(op->z, vt0, vc[3]->w); //add r12.z, r0, c3.w
			//mad oPos, r12.w, c254, r12
		}
	} s(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10);

	QByteArray data = s.getAgalcode();

#ifdef CHECK_AGAL
	if (!test.check(b.k, data.toHex(' ')))
	{
		qDebug() << "error PaintVertexShader" << b.k;
	}
#endif

	return data;
}
