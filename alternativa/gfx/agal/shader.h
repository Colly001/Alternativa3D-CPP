#ifndef SHADER_H
#define SHADER_H

#include "commonregister.h"
#include "relativeregister.h"
#include "shaderstreamwrite.h"


class Shader
{	
protected:

	static const int MOV = 0;
	static const int ADD = 1;
	static const int SUB = 2;
	static const int MUL = 3;
	static const int DIV = 4;
	static const int RCP = 5;
	static const int MIN = 6;
	static const int MAX = 7;
	static const int FRC = 8;
	static const int SQT = 9;
	static const int RSQ = 10;
	static const int POW = 11;
	static const int LOG = 12;
	static const int EXP = 13;
	static const int NRM = 14;
	static const int SIN = 15;
	static const int COS = 16;
	static const int CRS = 17;
	static const int DP3 = 18;
	static const int DP4 = 19;
	static const int ABS = 20;
	static const int NEG = 21;
	static const int SAT = 22;
	static const int M33 = 23;
	static const int M44 = 24;
	static const int M34 = 25;
	static const int DDX = 26;
	static const int DDY = 27;
	static const int IFE = 28;
	static const int INE = 29;
	static const int IFG = 30;
	static const int IFL = 31;
	static const int ELS = 32;
	static const int EIF = 33;
	static const int TED = 38;
	static const int KIL = 39;
	static const int TEX = 40;
	static const int SGE = 41;
	static const int SLT = 42;
	static const int SGN = 43;
	static const int SEQ = 44;
	static const int SNE = 45;

	ShaderStreamWrite data;



public:

	static CommonRegister *v0;
	static CommonRegister *v1;
	static CommonRegister *v2;
	static CommonRegister *v3;
	static CommonRegister *v4;
	static CommonRegister *v5;
	static CommonRegister *v6;
	static CommonRegister *v7;
	static RelativeRegister *cc;




	Shader();

	inline void mov(Register *r1, Register *r2)
	{
		this->op2(MOV, r1, r2);
	}

	inline void add(Register *r1, Register *r2, Register *r3)
	{
		this->op3(ADD, r1, r2, r3);
	}

	inline void sub(Register *r1, Register *r2, Register *r3)
	{
		this->op3(SUB, r1, r2, r3);
	}

	inline void mul(Register *r1, Register *r2, Register *r3)
	{
		this->op3(MUL, r1, r2, r3);
	}

	inline void div(Register *r1, Register *r2, Register *r3)
	{
		this->op3(DIV, r1, r2, r3);
	}

	inline void rcp(Register *r1, Register *r2)
	{
		this->op2(RCP, r1, r2);
	}

	inline void min(Register *r1, Register *r2, Register *r3)
	{
		this->op3(MIN, r1, r2, r3);
	}

	inline void max(Register *r1, Register *r2, Register *r3)
	{
		this->op3(MAX, r1, r2, r3);
	}

	inline void frc(Register *r1, Register *r2)
	{
		this->op2(FRC, r1, r2);
	}

	inline void sqt(Register *r1, Register *r2)
	{
		this->op2(SQT, r1, r2);
	}

	inline void rsq(Register *r1, Register *r2)
	{
		this->op2(RSQ, r1, r2);
	}

	inline void pow(Register *r1, Register *r2, Register *r3)
	{
		this->op3(POW, r1, r2, r3);
	}

	inline void log(Register *r1, Register *r2)
	{
		this->op2(LOG, r1, r2);
	}

	inline void exp(Register *r1, Register *r2)
	{
		this->op2(EXP, r1, r2);
	}

	inline void nrm(Register *r1, Register *r2)
	{
		this->op2(NRM, r1, r2);
	}

	inline void sin(Register *r1, Register *r2)
	{
		this->op2(SIN, r1, r2);
	}

	inline void cos(Register *r1, Register *r2)
	{
		this->op2(COS, r1, r2);
	}

	inline void crs(Register *r1, Register *r2, Register *r3)
	{
		this->op3(CRS, r1, r2, r3);
	}

	inline void dp3(Register *r1, Register *r2, Register *r3)
	{
		this->op3(DP3, r1, r2, r3);
	}

	inline void dp4(Register *r1, Register *r2, Register *r3)
	{
		this->op3(DP4, r1, r2, r3);
	}

	inline void abs(Register *r1, Register *r2)
	{
		this->op2(ABS, r1, r2);
	}

	inline void neg(Register *r1, Register *r2)
	{
		this->op2(NEG, r1, r2);
	}

	inline void sat(Register *r1, Register *r2)
	{
		this->op2(SAT, r1, r2);
	}

	inline void m33(Register *r1, Register *r2, Register *r3)
	{
		this->op3(M33, r1, r2, r3);
	}

	inline void m44(Register *r1, Register *r2, Register *r3)
	{
		this->op3(M44, r1, r2, r3);
	}

	inline void m34(Register *r1, Register *r2, Register *r3)
	{
		this->op3(M34, r1, r2, r3);
	}

	inline void sge(Register *r1, Register *r2, Register *r3)
	{
		this->op3(SGE, r1, r2, r3);
	}

	inline void slt(Register *r1, Register *r2, Register *r3)
	{
		this->op3(SLT, r1, r2, r3);
	}

	inline void sgn(Register *r1, Register *r2, Register *r3)
	{
		this->op3(SGN, r1, r2, r3);
	}

	inline void seq(Register *r1, Register *r2, Register *r3)
	{
		this->op3(SEQ, r1, r2, r3);
	}

	inline void sne(Register *r1, Register *r2, Register *r3)
	{
		this->op3(SNE, r1, r2, r3);
	}

protected:
	void op2(int, Register *r1, Register *r2);
	void op3(int, Register *r1, Register *r2, Register *r3);

public:
	QByteArray getAgalcode();
};

#endif // SHADER_H
