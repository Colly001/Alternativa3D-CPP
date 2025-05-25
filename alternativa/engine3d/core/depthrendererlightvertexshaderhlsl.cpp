#include "depthrendererlightvertexshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray DepthRendererLightVertexShaderHLSL(int n)
{
	using namespace HLSL_ASSEMBLY_VERTEX;

	HLSL_ASM_VERTEX a;


	if (n == 0)
	{
		//mov(vt1, cc.rel(va1.x, 0));

		//mova a0.x, v1.x
		a.mova(raddr(0, "x"), rinput(1, "x"));

		//mov r1, c0[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(1));
		a.add_src(A(rconst(0)));
		a.add_src(raddr(0, "x"));

		//mov oT1, r1
		a.mov(routput(1), rtemp(1));

		//mov oT2, c1[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(routput(1));
		a.add_src(A(rconst(1)));
		a.add_src(raddr(0, "x"));

		a.mul(rtemp(0), rinput(0), rtemp(1, "w")); //mul r0, v0, r1.w
		a.add(rtemp(0), rtemp(0), rtemp(1)); //add r0, r0, r1

		a.rcp(rtemp(8, "x"), rconst(3, "x")); //rcp r8.x, c3.x
		a.rcp(rtemp(8, "y"), rconst(3, "y")); //rcp r8.y, c3.y
		a.rcp(rtemp(8, "z"), rconst(3, "z")); //rcp r8.z, c3.z
		a.rcp(rtemp(8, "w"), rconst(3, "w")); //rcp r8.w, c3.w

		a.mul(rtemp(0, "xy"), rtemp(0), rtemp(8)); //mul r0.xy, r0, r8
	}
	else
	{
		//mova a0.x, v1.y
		a.mova(raddr(0, "x"), rinput(1, "y"));

		//mov r2, c0[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(2));
		a.add_src(A(rconst(0)));
		a.add_src(raddr(0, "x"));

		//mov r3, c1[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(3));
		a.add_src(A(rconst(1)));
		a.add_src(raddr(0, "x"));

		//mov r4, c2[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(4));
		a.add_src(A(rconst(2)));
		a.add_src(raddr(0, "x"));

		//mov r5, c3[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(5));
		a.add_src(A(rconst(3)));
		a.add_src(raddr(0, "x"));

		//mov r6, c4[a0.x]
		a.add_opcode(OPCODE_MOV, 3);
		a.add_dst(rtemp(6));
		a.add_src(A(rconst(4)));
		a.add_src(raddr(0, "x"));


		if (n == 1)
		{
			a.mul(routput(1, "x"), rtemp(2, "w"), rconst(3)); //mul oT1.x, r2.w, c3
			a.mul(routput(1, "y"), rtemp(3, "w"), rconst(3)); //mul oT1.y, r3.w, c3
			a.mov(routput(1, "zw"), rtemp(4, "w")); //mov oT1.zw, r4.w
			a.mul(routput(2, "x"), rtemp(2, "z"), rconst(3)); //mul oT2.x, r2.z, c3
			a.mul(routput(2, "y"), rtemp(3, "z"), rconst(3)); //mul oT2.y, r3.z, c3
			a.mov(routput(2, "zw"), rtemp(4, "z")); //mov oT2.zw, r4.z
			a.mov(routput(3), rtemp(5)); //mov oT3, r5
			a.mov(routput(4), rtemp(6)); //mov oT4, r6
			a.mul(rtemp(0), rinput(0), rtemp(6, "w")); //mul r0, v0, r6.w
			a.mul(rtemp(0, "z"), rinput(0), rtemp(5, "x")); //mul r0.z, v0, r5.x
			a.add(rtemp(0, "z"), rtemp(0), rtemp(5, "x")); //add r0.z, r0, r5.x
			a.mul(rtemp(0, "z"), rtemp(0), rconst(3, "w")); //mul r0.z, r0, c3.w
		}
		else if (n == 2)
		{
			a.mul(rtemp(0, "x"), rtemp(2, "z"), rconst(3)); //mul r0.x, r2.z, c3
			a.mul(rtemp(0, "y"), rtemp(3, "z"), rconst(3)); //mul r0.y, r3.z, c3
			a.mov(rtemp(0, "zw"), rtemp(4, "z")); //mov r0.zw, r4.z
			a.mov(routput(2), rtemp(0)); //mov oT2, r0
			a.mul(rtemp(1, "x"), rtemp(2, "w"), rconst(3)); //mul r1.x, r2.w, c3
			a.mul(rtemp(1, "y"), rtemp(3, "w"), rconst(3)); //mul r1.y, r3.w, c3
			a.mov(rtemp(1, "zw"), rtemp(4, "w")); //mov r1.zw, r4.w
			a.mul(rtemp(0), rtemp(0), rtemp(6, "w")); //mul r0, r0, r6.w
			a.add(routput(1), rtemp(1), rtemp(0)); //add oT1, r1, r0
			a.mov(routput(3), rtemp(5)); //mov oT3, r5
			a.mov(routput(4), rtemp(6)); //mov oT4, r6
			a.mul(rtemp(0, "xy"), rinput(0), rtemp(5, "x")); //mul r0.xy, v0, r5.x
			a.mul(rtemp(0, "z"), rinput(0), rtemp(5)); //mul r0.z, v0, r5
			a.add(rtemp(0, "z"), rtemp(0), rtemp(6, "w")); //add r0.z, r0, r6.w
		}

		a.mov(rtemp(0, "w"), rconst(1)); //mov r0.w, c1
		a.dp4(rtemp(1, "x"), rtemp(0), rtemp(2)); //dp4 r1.x, r0, r2
		a.dp4(rtemp(1, "y"), rtemp(0), rtemp(3)); //dp4 r1.y, r0, r3
		a.dp4(rtemp(1, "z"), rtemp(0), rtemp(4)); //dp4 r1.z, r0, r4
		a.mov(rtemp(0, "xyz"), rtemp(1, "xyzz")); //mov r0.xyz, r1.xyzz
	}


	a.mul(rtemp(0, "xy"), rtemp(0), rtemp(8)); //mul r0.xy, r0, r8
	a.mul(rtemp(0, "xy"), rtemp(0), rconst(1)); //mul r0.xy, r0, c1
	a.mul(rtemp(1), rconst(2), rtemp(0, "z")); //mul r1, c2, r0.z
	a.add(rtemp(0, "xy"), rtemp(0), N(rtemp(1))); //add r0.xy, r0, -r1
	a.mov(routput(0), rtemp(0)); //mov oT0, r0
	a.mov(rtemp(12, "xw"), rtemp(0, "xzzz")); //mov r12.xw, r0.xzzz
	a.mov(rtemp(12, "y"), N(rtemp(0))); //mov r12.y, -r0
	a.mul(rtemp(0, "z"), rtemp(0), rconst(0)); //mul r0.z, r0, c0
	a.add(rtemp(12, "z"), rtemp(0), rconst(0, "w")); //add r12.z, r0, c0.w
	a.mad(ropos(), rtemp(12, "w"), rconst(254), rtemp(12)); //mad oPos, r12.w, c254, r12




	HLSL_ASM_CAPTION caption;

	caption.dcl_texcoord(rinput(0));
	caption.dcl_texcoord1(rinput(1));

	QByteArray data = collect_vertex(caption, a);

	return data;


	/*
	if (param1 == 0)
	{
		mov(vt1, cc.rel(va1.x, 0));
		mov(v1, vt1);
		mov(v2, cc.rel(va1.x, 1));
		mul(vt0, va0, vt1.w);
		add(vt0, vt0, vt1);
		div(vt0.xy, vt0, vc[3]);
	}
	else
	{
		mov(vt2, cc.rel(va1.y, 0));
		mov(vt3, cc.rel(va1.y, 1));
		mov(vt4, cc.rel(va1.y, 2));
		mov(vt5, cc.rel(va1.y, 3));
		mov(vt6, cc.rel(va1.y, 4));
		if (param1 == 1)
		{
			mul(v1.x, vt2.w, vc[3]);
			mul(v1.y, vt3.w, vc[3]);
			mov(v1.zw, vt4.w);
			mul(v2.x, vt2.z, vc[3]);
			mul(v2.y, vt3.z, vc[3]);
			mov(v2.zw, vt4.z);
			mov(v3, vt5);
			mov(v4, vt6);
			mul(vt0, va0, vt6.w);
			mul(vt0.z, va0, vt5.x);
			add(vt0.z, vt0, vt5.x);
			mul(vt0.z, vt0, vc[3].w);
		}
		else
		{
			if (param1 == 2)
			{
				mul(vt0.x, vt2.z, vc[3]);
				mul(vt0.y, vt3.z, vc[3]);
				mov(vt0.zw, vt4.z);
				mov(v2, vt0);
				mul(vt1.x, vt2.w, vc[3]);
				mul(vt1.y, vt3.w, vc[3]);
				mov(vt1.zw, vt4.w);
				mul(vt0, vt0, vt6.w);
				add(v1, vt1, vt0);
				mov(v3, vt5);
				mov(v4, vt6);
				mul(vt0.xy, va0, vt5.x);
				mul(vt0.z, va0, vt5);
				add(vt0.z, vt0, vt6.w);
			}
		}
		mov(vt0.w, vc[1]);
		dp4(vt1.x, vt0, vt2);
		dp4(vt1.y, vt0, vt3);
		dp4(vt1.z, vt0, vt4);
		mov(vt0.xyz, vt1.xyz);
	}
	mul(vt0.xy, vt0, vc[1]);
	mul(vt1, vc[2], vt0.z);
	sub(vt0.xy, vt0, vt1);
	mov(v0, vt0);
	mov(op.xw, vt0.xz);
	neg(op.y, vt0);
	mul(vt0.z, vt0, vc[0]);
	add(op.z, vt0, vc[0].w);
	*/
}



#if 0
vs_2_x 0
dcl_texcoord v0
dcl_texcoord1 v1
mova a0.x, v1.x
mov r1, c0[a0.x]
mov oT1, r1
mov oT2, c1[a0.x]
mul r0, v0, r1.w
add r0, r0, r1
rcp r8.x, c3.x
rcp r8.y, c3.y
rcp r8.z, c3.z
rcp r8.w, c3.w
mul r0.xy, r0, r8
mul r0.xy, r0, c1
mul r1, c2, r0.z
add r0.xy, r0, -r1
mov oT0, r0
mov r12.xw, r0.xzzz
mov r12.y, -r0
mul r0.z, r0, c0
add r12.z, r0, c0.w
mad oPos, r12.w, c254, r12
#endif

#if 0
vs_2_x 1
dcl_texcoord v0
dcl_texcoord1 v1

mova a0.x, v1.y
mov r2, c0[a0.x]
mov r3, c1[a0.x]
mov r4, c2[a0.x]
mov r5, c3[a0.x]
mov r6, c4[a0.x]

mul oT1.x, r2.w, c3
mul oT1.y, r3.w, c3
mov oT1.zw, r4.w
mul oT2.x, r2.z, c3
mul oT2.y, r3.z, c3
mov oT2.zw, r4.z
mov oT3, r5
mov oT4, r6
mul r0, v0, r6.w
mul r0.z, v0, r5.x
add r0.z, r0, r5.x
mul r0.z, r0, c3.w



mov r0.w, c1
dp4 r1.x, r0, r2
dp4 r1.y, r0, r3
dp4 r1.z, r0, r4
mov r0.xyz, r1.xyzz

mul r0.xy, r0, c1
mul r1, c2, r0.z
add r0.xy, r0, -r1
mov oT0, r0
mov r12.xw, r0.xzzz
mov r12.y, -r0
mul r0.z, r0, c0
add r12.z, r0, c0.w
mad oPos, r12.w, c254, r12
#endif

#if 0
vs_2_x 2
dcl_texcoord v0
dcl_texcoord1 v1

mova a0.x, v1.y
mov r2, c0[a0.x]
mov r3, c1[a0.x]
mov r4, c2[a0.x]
mov r5, c3[a0.x]
mov r6, c4[a0.x]

mul r0.x, r2.z, c3
mul r0.y, r3.z, c3
mov r0.zw, r4.z
mov oT2, r0
mul r1.x, r2.w, c3
mul r1.y, r3.w, c3
mov r1.zw, r4.w
mul r0, r0, r6.w
add oT1, r1, r0
mov oT3, r5
mov oT4, r6
mul r0.xy, v0, r5.x
mul r0.z, v0, r5
add r0.z, r0, r6.w

mov r0.w, c1
dp4 r1.x, r0, r2
dp4 r1.y, r0, r3
dp4 r1.z, r0, r4
mov r0.xyz, r1.xyzz

mul r0.xy, r0, c1
mul r1, c2, r0.z
add r0.xy, r0, -r1
mov oT0, r0
mov r12.xw, r0.xzzz
mov r12.y, -r0
mul r0.z, r0, c0
add r12.z, r0, c0.w
mad oPos, r12.w, c254, r12
#endif
