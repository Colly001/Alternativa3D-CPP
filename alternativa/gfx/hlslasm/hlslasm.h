#ifndef HLSLASM_H
#define HLSLASM_H

#include <QByteArray>

namespace HLSL_ASSEMBLY
{
	enum
	{
		R_TEMP = 0, //r
		R_INPUT = 1, //v
		R_CONST = 2, //c
		R_ADDR_TEXTURE = 3, //vertex(a) pixel(t)
		R_RASTOUT = 4, //oPos oFog oPts oX3
		R_ATTROUT = 5, //oD
		R_OUTPUT = 6, //oT
		R_CONSTINT = 7, //i
		R_COLOROUT = 8, //oC
		R_DEPTHOUT = 9, //oDepth
		R_SAMPLER = 10, //s
		R_CONSTBOOL = 14, //b
		R_LOOP = 15, //aL
		R_TEMPFLOAT16 = 16, //x
		R_MISCTYPE = 17, //vPos vFace m2
		R_LABEL = 18, //l
		R_PREDICATE = 19, //p
	};

	enum
	{
		R_D3DSRO_POSITION     = 0,
		R_D3DSRO_FOG          = 1,
		R_D3DSRO_POINT_SIZE   = 2,
	};

	enum
	{
		R_D3DSMO_POSITION  = 0,
		R_D3DSMO_FACE      = 1
	};

	enum
	{
		OPCODE_NOP          =  0,
		OPCODE_MOV          =  1,
		OPCODE_ADD          =  2,
		OPCODE_SUB          =  3,
		OPCODE_MAD          =  4,
		OPCODE_MUL          =  5,
		OPCODE_RCP          =  6,
		OPCODE_RSQ          =  7,
		OPCODE_DP3          =  8,
		OPCODE_DP4          =  9,
		OPCODE_MIN          = 10,
	};

	enum
	{
		MODIFIER_SATURATE = 0x00100000,
		MODIFIER_PARTIALPRECISION = 0x00200000,
		MODIFIER_MSAMPCENTROID = 0x00400000,
	};



	struct R
	{
		uint8_t type_register;
		uint16_t n;
		const char *v;
		bool negative;
		bool array;
		R() {}
		R(uint8_t _type_register, uint16_t _n, const char *_v = "") : type_register(_type_register), n(_n), v(_v), negative(false), array(false) {}
		void copy(const R &r)
		{
			this->type_register = r.type_register;
			this->n = r.n;
			this->v = r.v;
			this->negative = r.negative;
			this->array = r.array;
		}
	};

	struct rtemp : R { rtemp(int _n, const char *_v = "") : R(R_TEMP, _n, _v) {} };
	struct rconst : R { rconst(int _n, const char *_v = "") : R(R_CONST, _n, _v) {} };

	//struct rofog : R { rofog() : R(R_RASTOUT, R_D3DSRO_FOG, "") {} };
	//struct ropts : R { ropts() : R(R_RASTOUT, R_D3DSRO_POINT_SIZE, "") {} };
	//struct rvpos : R { rvpos() : R(R_MISCTYPE, R_D3DSMO_POSITION, "") {} };
	//struct rvface : R { rvface() : R(R_MISCTYPE, R_D3DSMO_FACE, "") {} };


	struct N : R { N(const R &r) : R() { this->copy(r); this->negative = true; } };
	struct A : R { A(const R &r) : R() { this->copy(r); this->array = true; } };


	class HLSL_ASM
	{
	public:
		QByteArray data;

		uint32_t src_tk(const R &r);
		uint32_t dst_tk(const R &r,  uint32_t modifier = 0);

		void add_opcode(uint8_t opcode, int count);
		void add_dst(const R &r);
		void add_src(const R &r);

		void append(uint32_t v);
		void append(float v);

		bool checkUsedRegister(const R &r);
	};

	class HLSL_ASM_CODE : public HLSL_ASM
	{
	public:
		void mov(const R &dest, const R &src);
		void mov_sat(const R &dest, const R &src);
		void add(const R &dest, const R &src1, const R &src2);
		void dp3(const R &dest, const R &src1, const R &src2);
		void dp4(const R &dest, const R &src1, const R &src2);
		void mad(const R &dest, const R &src1, const R &src2, const R &src3);
		void sincos(const R &dest, const R &src1, const R &src2, const R &src3);
		void mul(const R &dest, const R &src1, const R &src2);
		void nrm(const R &dest, const R &src);
		void rcp(const R &dest, const R &src);
		void rsq(const R &dest, const R &src);
		void mova(const R &dest, const R &src);
		void abs(const R &dest, const R &src);
		void max(const R &dest, const R &src1, const R &src2);
		void min(const R &dest, const R &src1, const R &src2);
		void frc(const R &dest, const R &src);
		void setp_ge(const R &dest, const R &src1, const R &src2);
		void mov_label1(const R &dest, const R &src2);
		void mov_label2(const R &dest, const R &src2);
	};


	class HLSL_ASM_CAPTION : public HLSL_ASM
	{
	public:
		void dcl(const R &dest);
		void dcl_2d(const R &dest);
		void dcl_texcoord(const R &dest);
		void dcl_texcoord1(const R &dest);
		void dcl_texcoord2(const R &dest);
		void def(const R &dest, float x, float y, float z,float w);
	};


	class HLSL_ASM_VERTEX : public HLSL_ASM_CODE
	{
	public:

	};

	class HLSL_ASM_PIXEL : public HLSL_ASM_CODE
	{
	public:
		void texld(const R &dest, const R &src1, const R &src2);
		void texkill(const R &dest);
	};
}


namespace HLSL_ASSEMBLY_VERTEX
{
	using namespace HLSL_ASSEMBLY;
	QByteArray collect_vertex(const HLSL_ASM_CAPTION&, const HLSL_ASM_VERTEX&);
	struct routput : R { routput(int _n, const char *_v = "") : R(R_OUTPUT, _n, _v) {} };
	struct rinput : R { rinput(int _n, const char *_v = "") : R(R_INPUT, _n, _v) {} };
	struct ropos : R { ropos() : R(R_RASTOUT, R_D3DSRO_POSITION, "") {} };
	struct raddr : R { raddr(int _n, const char *_v = "") : R(R_ADDR_TEXTURE, _n, _v) {} };
}

namespace HLSL_ASSEMBLY_PIXEL
{
	using namespace HLSL_ASSEMBLY;
	QByteArray collect_pixel(const HLSL_ASM_CAPTION&, const HLSL_ASM_PIXEL&);
	struct rtexture : R { rtexture(int _n, const char *_v = "") : R(R_ADDR_TEXTURE, _n, _v) {} };
	struct rcolorout : R { rcolorout(int _n, const char *_v = "") : R(R_COLOROUT, _n, _v) {} };
	struct rsampler : R { rsampler(int _n, const char *_v = "") : R(R_SAMPLER, _n, _v) {} };
	struct rpredicate : R { rpredicate(int _n, const char *_v = "") : R(R_PREDICATE, _n, _v) {} };
}


#endif // HLSLASM_H
