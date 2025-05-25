#include "hlslasm.h"
#include <windows.h>
#include <d3d9types.h>

/*
va, vc, fc, vt, ft, op, oc, v, fs

Регистр-вывода - op (output position), oc (output color)
Регистр-атрибут - va(8) Context3D.setVertexBufferAt
Регистр-интерполятор - v(8)
Регистр-переменная - vt(8), ft(8)
Регистр-константа - vc (128) и fc (28).
Регистр-семплер - fs Context3D.setTextureAt

AGAL
vc, fc - const
vt, ft - temp
va - input
v - output, texture
oc - colorout
fs - Context3D.setTextureAt

HLSL
c - const
r - temp
v - input
oT - output
oC - colorout
t - texture
*/


namespace HLSL_ASSEMBLY
{
/*
	D3DSPR_TEMP         =  0, r
	D3DSPR_INPUT        =  1, v
	D3DSPR_CONST        =  2, c
	D3DSPR_ADDR         =  3, a
	D3DSPR_TEXTURE      =  3, a
	D3DSPR_RASTOUT      =  4, oX
	D3DSPR_ATTROUT      =  5, oD
	D3DSPR_TEXCRDOUT    =  6, oT
	D3DSPR_OUTPUT       =  6, oT
	D3DSPR_CONSTINT     =  7, i
	D3DSPR_COLOROUT     =  8, oC
	D3DSPR_DEPTHOUT     =  9, oDepth
	D3DSPR_SAMPLER      = 10, s
	D3DSPR_CONST2       = 11, c2058
	D3DSPR_CONST3       = 12, c4106
	D3DSPR_CONST4       = 13, c6154
	D3DSPR_CONSTBOOL    = 14, b
	D3DSPR_LOOP         = 15, aL
	D3DSPR_TEMPFLOAT16  = 16, x
	D3DSPR_MISCTYPE     = 17, m
	D3DSPR_LABEL        = 18, l
	D3DSPR_PREDICATE    = 19, p
*/


	static uint32_t ins_tk(D3DSHADER_INSTRUCTION_OPCODE_TYPE opcode, uint8_t length)
	{
		return 0x00000000 | opcode | ( D3DSI_INSTLENGTH_MASK & (length << D3DSI_INSTLENGTH_SHIFT) );
	}




	uint32_t HLSL_ASM::src_tk(const R &r)
	{
		uint32_t reg_num = r.n;
		uint32_t reg_type = r.type_register;
		const char *swz = r.v;
		D3DSHADER_PARAM_SRCMOD_TYPE modifier = D3DSPSM_NONE;

		if (r.negative == true)
			modifier = D3DSPSM_NEG;

		uint32_t e = 0;
		if (r.array == true)
			e |= 0x2000;

		uint8_t swz_xyzw[4];
		int len = strlen(swz);

		if (len == 0)
		{
			swz_xyzw[0] = 0;
			swz_xyzw[1] = 1;
			swz_xyzw[2] = 2;
			swz_xyzw[3] = 3;
		}
		else
		{
			uint8_t last = 0;

			for (int i = 0; i < 4; i++)
			{
				if (i >= len)
				{
					swz_xyzw[i] = last;
				}
				else
				{
					char c = swz[i];
					uint8_t v;
					if (c == 'x')
						v = 0;
					else if (c == 'y')
						v = 1;
					else if (c == 'z')
						v = 2;
					else if (c == 'w')
						v = 3;
					else
						v = 0;
					last = swz_xyzw[i] = v;
				}
			}
		}


		return 0x80000000 |
				(reg_num & D3DSP_REGNUM_MASK) |
				((reg_type << D3DSP_REGTYPE_SHIFT)  & D3DSP_REGTYPE_MASK) |
				((reg_type << D3DSP_REGTYPE_SHIFT2) & D3DSP_REGTYPE_MASK2) |
				(swz_xyzw[0] << D3DSP_SWIZZLE_SHIFT) |
				(swz_xyzw[1] << (D3DSP_SWIZZLE_SHIFT + 2)) |
				(swz_xyzw[2] << (D3DSP_SWIZZLE_SHIFT + 4)) |
				(swz_xyzw[3] << (D3DSP_SWIZZLE_SHIFT + 6)) |
				(modifier  & D3DSP_SRCMOD_MASK) | e;
	}

	uint32_t HLSL_ASM::dst_tk(const R &r, uint32_t modifier)
	{
		uint32_t reg_num = r.n;
		uint32_t reg_type = r.type_register;
		const char *wm = r.v;

		uint32_t e = 0;
		if (r.array == true)
			e |= 0x2000;

		uint8_t wmx = 0;
		uint8_t wmy = 0;
		uint8_t wmz = 0;
		uint8_t wmw = 0;

		int len = strlen(wm);

		if (len == 0)
		{
			wmx = wmy = wmz = wmw = 1;
		}
		else
		{
			for (int i = 0; i < len; i++)
			{
				char c = wm[i];
				if (c == 'x')
					wmx = 1;
				else if (c == 'y')
					wmy = 1;
				else if (c == 'z')
					wmz = 1;
				else if (c == 'w')
					wmw = 1;
			}
		}

		return 0x80000000 |
				(reg_num & D3DSP_REGNUM_MASK) |
				((reg_type << D3DSP_REGTYPE_SHIFT)  & D3DSP_REGTYPE_MASK) |
				((reg_type << D3DSP_REGTYPE_SHIFT2) & D3DSP_REGTYPE_MASK2) |
				(wmx << 16) |
				(wmy << (16 + 1)) |
				(wmz << (16 + 2)) |
				(wmw << (16 + 3)) |
				(modifier  & D3DSP_DSTMOD_MASK) | e;
	}

	void HLSL_ASM::add_opcode(uint8_t opcode, int count)
	{
		append(ins_tk((D3DSHADER_INSTRUCTION_OPCODE_TYPE)opcode, count));
	}

	void HLSL_ASM::add_dst(const R &r)
	{
		append(dst_tk(r));
	}

	void HLSL_ASM::add_src(const R &r)
	{
		append(src_tk(r));
	}

	void HLSL_ASM::append(uint32_t v)
	{
		data.append((const char*)&v, 4);
	}

	void HLSL_ASM::append(float v)
	{
		data.append((const char*)&v, 4);
	}

	bool HLSL_ASM::checkUsedRegister(const R &r)
	{
		const uint32_t *list = (const uint32_t*)data.constData();
		int size = data.size() / 4;
		for (int i = 0; i < size;)
		{
			uint32_t length = (list[i] & D3DSI_INSTLENGTH_MASK) >> D3DSI_INSTLENGTH_SHIFT;

			for (uint32_t j = 0; j < length; j++)
			{
				uint32_t a = list[i+j+1];
				uint32_t c1 = a & D3DSP_REGTYPE_MASK;
				uint32_t c2 = a & D3DSP_REGTYPE_MASK2;
				uint32_t reg_type = c1 >> D3DSP_REGTYPE_SHIFT | c2 >> D3DSP_REGTYPE_SHIFT2;
				uint32_t reg_num = a & D3DSP_REGNUM_MASK;

				if (r.type_register == reg_type && r.n == reg_num)
					return true;
			}

			i += 1 + length;
		}

		return false;
	}

	void HLSL_ASM_CAPTION::dcl_texcoord(const R &dest)
	{
		append(ins_tk(D3DSIO_DCL, 2));
		append(0x80000005);
		append(dst_tk(dest));
	}

	void HLSL_ASM_CAPTION::dcl_texcoord1(const R &dest)
	{
		append(ins_tk(D3DSIO_DCL, 2));
		append(0x80010005);
		append(dst_tk(dest));
	}

	void HLSL_ASM_CAPTION::dcl_texcoord2(const R &dest)
	{
		append(ins_tk(D3DSIO_DCL, 2));
		append(0x80020005);
		append(dst_tk(dest));
	}

	void HLSL_ASM_CAPTION::def(const R &dest, float x, float y, float z, float w)
	{
		append(ins_tk(D3DSIO_DEF, 5));
		append(dst_tk(dest));
		append(x);
		append(y);
		append(z);
		append(w);
	}

	void HLSL_ASM_CAPTION::dcl(const R &dest)
	{
		append(ins_tk(D3DSIO_DCL, 2));
		append(0x80000000);
		append(dst_tk(dest));
	}

	void HLSL_ASM_CAPTION::dcl_2d(const R &dest)
	{
		append(ins_tk(D3DSIO_DCL, 2));
		append(0x90000000);
		append(dst_tk(dest));
	}

	void HLSL_ASM_CODE::mov(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_MOV, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_CODE::add(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_ADD, 3));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::dp4(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_DP4, 3));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::mad(const R &dest, const R &src1, const R &src2, const R &src3)
	{
		append(ins_tk(D3DSIO_MAD, 4));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
		append(src_tk(src3));
	}

	void HLSL_ASM_CODE::sincos(const R &dest, const R &src1, const R &src2, const R &src3)
	{
		//04000025 80010009 80000008 A0E4001D A0E4001E
		append(ins_tk(D3DSIO_SINCOS, 4));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
		append(src_tk(src3));
	}

	void HLSL_ASM_CODE::mul(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_MUL, 3));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::nrm(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_NRM, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_CODE::rcp(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_RCP, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_CODE::rsq(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_RSQ, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_CODE::dp3(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_DP3, 3));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::mova(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_MOVA, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_PIXEL::texkill(const R &dest)
	{
		append(ins_tk(D3DSIO_TEXKILL, 1)); //0x01000041 0x800F0008,
		append(dst_tk(dest));
	}

	void HLSL_ASM_CODE::abs(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_ABS, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_CODE::mov_sat(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_MOV, 2)); //SAT
		append(dst_tk(dest, MODIFIER_SATURATE));
		append(src_tk(src));
		//mov_sat r1, r1
		//0x02000001, 0x801F0001, 0x80E40001,
	}

	void HLSL_ASM_PIXEL::texld(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_TEX, 3)); //texld 03000042
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::max(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_MAX, 3));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::min(const R &dest, const R &src1, const R &src2)
	{
		append(ins_tk(D3DSIO_MIN, 3));
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::frc(const R &dest, const R &src)
	{
		append(ins_tk(D3DSIO_FRC, 2));
		append(dst_tk(dest));
		append(src_tk(src));
	}

	void HLSL_ASM_CODE::setp_ge(const R &dest, const R &src1, const R &src2)
	{
		//setp_ge p0, r1.x, c23.z
		//0303005E B00F1000 80000001 A0AA0017
		append(ins_tk(D3DSIO_SETP, 3) | 0x00030000);
		append(dst_tk(dest));
		append(src_tk(src1));
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::mov_label1(const R &dest, const R &src2)
	{
		//(!p0) mov r4.x, c28.x
		//0x13000001 0x80010004 0xBDE41000 0xA000001C
		append(ins_tk(D3DSIO_MOV, 3) | 0x10000000);
		append(dst_tk(dest));
		append(0xBDE41000);
		append(src_tk(src2));
	}

	void HLSL_ASM_CODE::mov_label2(const R &dest, const R &src2)
	{
		//(p0) mov r4.x, c28.y
		//13000001 80010004 B0E41000 A055001C
		append(ins_tk(D3DSIO_MOV, 3) | 0x10000000);
		append(dst_tk(dest));
		append(0xB0E41000);
		append(src_tk(src2));
	}

}


QByteArray HLSL_ASSEMBLY_VERTEX::collect_vertex(const HLSL_ASM_CAPTION &caption, const HLSL_ASM_VERTEX &code)
{
	QByteArray data;
	data.append("\x01\x02\xFE\xFF", 4);
	data.append(caption.data);
	data.append(code.data);
	data.append("\xFF\xFF\x00\x00", 4);
	return data;
}


QByteArray HLSL_ASSEMBLY_PIXEL::collect_pixel(const HLSL_ASM_CAPTION &caption, const HLSL_ASM_PIXEL &code)
{
	QByteArray data;
	data.append("\x01\x02\xFF\xFF", 4);
	data.append(caption.data);
	data.append(code.data);
	data.append("\xFF\xFF\x00\x00", 4);
	return data;
}

