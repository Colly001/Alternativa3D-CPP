#include "animatedpaintfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "materialfragmentshaderhlsl.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray AnimatedPaintFragmentShaderHLSL(ShaderSampler &samplers,
										   bool p_a,
										   bool p_b,
										   bool p_c,
										   bool p_d,
										   bool p_f,
										   bool p_g,
										   bool p_h,
										   bool p_i,
										   bool p_j,
										   bool p_k,
										   bool p_l,
										   bool p_m,
										   bool p_n,
										   bool p_o,
										   bool p_p,
										   bool p_q,
										   bool p_r,
										   bool p_s)
{
	using namespace HLSL_ASSEMBLY_PIXEL;


	HLSL_ASM_PIXEL a;


	SHADER_SAMPLER_REPEAT _repeat = p_a ? SHADER_SAMPLER_REPEAT::WRAP : SHADER_SAMPLER_REPEAT::CLAMP;
	SHADER_SAMPLER_FILTER _filter = p_b ? SHADER_SAMPLER_FILTER::LINEAR : SHADER_SAMPLER_FILTER::NEAREST;
	SHADER_SAMPLER_MIPMAP _mipMap = p_c ? (p_b ? SHADER_SAMPLER_MIPMAP::LINEAR : SHADER_SAMPLER_MIPMAP::NEAREST) : SHADER_SAMPLER_MIPMAP::NONE;
	SHADER_SAMPLER_TYPE _type = p_d ? SHADER_SAMPLER_TYPE::DXT1 : SHADER_SAMPLER_TYPE::RGBA;

	//dcl t0
	//dcl t6
	//dcl_2d s0
	//dcl_2d s4
	//dcl_2d s6
	//def c28, 0, 1, 0, 0


	a.rcp(rtemp(8, "x"), rconst(24, "x")); //rcp r8.x, c24.x
	a.rcp(rtemp(8, "y"), rconst(24, "y")); //rcp r8.y, c24.y
	a.rcp(rtemp(8, "z"), rconst(24, "z")); //rcp r8.z, c24.z
	a.rcp(rtemp(8, "w"), rconst(24, "w")); //rcp r8.w, c24.w
	a.mul(rtemp(4), rtexture(6), rtemp(8)); //mul r4, t6, r8

	a.frc(rtemp(4), rtemp(4)); //frc r4, r4

	a.mul(rtemp(4), rtemp(4), rconst(24)); //mul r4, r4, c24
	a.add(rtemp(4, "x"), rtemp(4, "x"), rconst(24, "z")); //add r4.x, r4.x, c24.z
	a.add(rtemp(4, "y"), rtemp(4, "y"), rconst(24, "w")); //add r4.y, r4.y, c24.w


	samplers.add(4, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);

	//a.texld(rtemp(8), rtemp(4), rsampler(4)); //texld r8, r4, s4
	//a.mov(rtemp(8, "w"), rconst(28, "y")); //mov r8.w, c28.y
	//a.mov(rtemp(0), rtemp(8)); //mov r0, r8

	a.texld(rtemp(0), rtemp(4), rsampler(4)); //texld r8, r4, s4




	samplers.add(6, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);

	//a.texld(rtemp(8), rtexture(0), rsampler(6)); //texld r8, t0, s6
	//a.mov(rtemp(8, "w"), rconst(28, "y")); //mov r8.w, c28.y
	//a.mov(rtemp(1), rtemp(8)); //mov r1, r8

	a.texld(rtemp(1), rtexture(0), rsampler(6)); //texld r1, t0, s6


	a.mul(rtemp(2), rtemp(0), rtemp(1)); //mul r2, r0, r1
	a.mul(rtemp(2), rconst(23, "w"), rtemp(2)); //mul r2, c23.w, r2
	a.add(rtemp(3), rconst(23, "z"), N(rtemp(0))); //add r3, c23.z, -r0
	a.add(rtemp(4), rconst(23, "z"), N(rtemp(1))); //add r4, c23.z, -r1
	a.mul(rtemp(3), rconst(23, "w"), rtemp(3)); //mul r3, c23.w, r3
	a.mul(rtemp(3), rtemp(3), rtemp(4)); //mul r3, r3, r4
	a.add(rtemp(3), rconst(23, "z"), N(rtemp(3))); //add r3, c23.z, -r3

	a.setp_ge(rpredicate(0), rtemp(1, "x"), rconst(23, "z"));
	a.mov_label1(rtemp(4, "x"), rconst(28, "x")); //(!p0) mov r4.x, c28.x
	a.mov_label2(rtemp(4, "x"), rconst(28, "y")); //(p0) mov r4.x, c28.y


	a.add(rtemp(4, "y"), rconst(23, "z"), N(rtemp(4, "x"))); //add r4.y, c23.z, -r4.x
	a.mul(rtemp(5), rtemp(4, "x"), rtemp(3)); //mul r5, r4.x, r3
	a.mul(rtemp(6), rtemp(4, "y"), rtemp(2)); //mul r6, r4.y, r2
	a.add(rtemp(0), rtemp(5), rtemp(6)); //add r0, r5, r6
	a.mov(rtemp(0, "w"), rtemp(1, "w")); //mov r0.w, r1.w


	samplers.add(0, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);

	a.texld(rtemp(1), rtexture(0), rsampler(0)); //texld r1, t0, s0

	a.add(rtemp(2, "w"), rconst(23, "z"), N(rtemp(1, "w"))); //add r2.w, c23.z, -r1.w
	a.mul(rtemp(0, "x"), rtemp(0, "x"), rtemp(2, "w")); //mul r0.x, r0.x, r2.w
	a.mul(rtemp(0, "y"), rtemp(0, "y"), rtemp(2, "w")); //mul r0.y, r0.y, r2.w
	a.mul(rtemp(0, "z"), rtemp(0, "z"), rtemp(2, "w")); //mul r0.z, r0.z, r2.w
	a.add(rtemp(0, "xyz"), rtemp(0, "xyzz"), rtemp(1, "xyzz")); //add r0.xyz, r0.xyzz, r1.xyzz
	a.mov(rtemp(1), rconst(23, "x")); //mov r1, c23.x
	a.mov(rtemp(2), rconst(23, "x")); //mov r2, c23.x
	a.mov(rtemp(3), rconst(23, "x")); //mov r3, c23.x
	a.mov(rtemp(4), rconst(23, "x")); //mov r4, c23.x
	a.mov(rtemp(5), rconst(23, "x")); //mov r5, c23.x
	a.mov(rtemp(6), rconst(23, "x")); //mov r6, c23.x



	//a.setp_ge(); //setp_ge p0, r1.x, c23.z
	//0x0303005E, 0xB00F1000, 0x80000001, 0xA0AA0017,
	//(!p0) mov r4.x, c28.x
	//0x13000001, 0x80010004, 0xBDE41000, 0xA000001C,
	//(p0) mov r4.x, c28.y
	//0x13000001, 0x80010004, 0xB0E41000, 0xA055001C,

	//rcp r8.x, c24.x
	//rcp r8.y, c24.y
	//rcp r8.z, c24.z
	//rcp r8.w, c24.w
	//mul r4, t6, r8
	//div(ft4, v6, fc[24]);


	//frc(ft4, ft4); //frc r4, r4
	//mul(ft4, ft4, fc[24]); //mul r4, r4, c24
	//add(ft4->x, ft4->x, fc[24]->z); //add r4.x, r4.x, c24.z
	//add(ft4->y, ft4->y, fc[24]->w); //add r4.y, r4.y, c24.w


	//texld r8, r4, s4
	//mov r8.w, c28.y
	//mov r0, r8

	//texld r0, r4, s4
	//tex(ft0, ft4, fs4->dim(SamplerDim::D2)->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));

	//texld r8, t0, s6
	//mov r8.w, c28.y
	//mov r1, r8

	//texld r1, t0, s6
	//tex(ft1, v0, fs6->dim(SamplerDim::D2)->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));


	//mul(ft2, ft0, ft1); //mul r2, r0, r1
	//mul(ft2, fc[23]->w, ft2); //mul r2, c23.w, r2
	//sub(ft3, fc[23]->z, ft0); //add r3, c23.z, -r0
	//sub(ft4, fc[23]->z, ft1); //add r4, c23.z, -r1
	//mul(ft3, fc[23]->w, ft3); //mul r3, c23.w, r3
	//mul(ft3, ft3, ft4); //mul r3, r3, r4
	//sub(ft3, fc[23]->z, ft3); //add r3, c23.z, -r3

	//setp_ge p0, r1.x, c23.z
	//(!p0) mov r4.x, c28.x
	//(p0) mov r4.x, c28.y
	//sge(ft4->x, ft1->x, fc[23]->z);



	//sub(ft4->y, fc[23]->z, ft4->x); //add r4.y, c23.z, -r4.x
	//mul(ft5, ft4->x, ft3); //mul r5, r4.x, r3
	//mul(ft6, ft4->y, ft2); //mul r6, r4.y, r2
	//add(ft0, ft5, ft6); //add r0, r5, r6
	//mov(ft0->w, ft1->w); //mov r0.w, r1.w

	//texld r8, t0, s0
	//mov r8.w, c28.y
	//mov r1, r8

	//texld r1, t0, s0
	//tex(ft1, v0, fs0->dim(SamplerDim::D2)->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));


	//sub(ft2->w, fc[23]->z, ft1->w); //add r2.w, c23.z, -r1.w
	//mul(ft0->x, ft0->x, ft2->w); //mul r0.x, r0.x, r2.w
	//mul(ft0->y, ft0->y, ft2->w); //mul r0.y, r0.y, r2.w
	//mul(ft0->z, ft0->z, ft2->w); //mul r0.z, r0.z, r2.w
	//add(ft0->xyz, ft0->xyz, ft1->xyz); //add r0.xyz, r0.xyzz, r1.xyzz
	//mov(ft1, fc[23]->x); //mov r1, c23.x
	//mov(ft2, fc[23]->x); //mov r2, c23.x
	//mov(ft3, fc[23]->x); //mov r3, c23.x
	//mov(ft4, fc[23]->x); //mov r4, c23.x
	//mov(ft5, fc[23]->x); //mov r5, c23.x
	//mov(ft6, fc[23]->x); //mov r6, c23.x




	MaterialFragmentShaderHLSL(a, samplers, p_f, p_g, p_h, p_i, p_j, p_k, p_l, p_m, p_n,
							   p_o, p_p, p_q, p_r, p_s);


	HLSL_ASM_CAPTION caption;

	//texture
	if (a.checkUsedRegister(rtexture(0)))
		caption.dcl(rtexture(0)); //dcl t0

	if (a.checkUsedRegister(rtexture(1)))
		caption.dcl(rtexture(1)); //dcl t1

	if (a.checkUsedRegister(rtexture(2)))
		caption.dcl(rtexture(2)); //dcl t2

	if (a.checkUsedRegister(rtexture(6)))
		caption.dcl(rtexture(6)); //dcl t6

	//sampler
	if (a.checkUsedRegister(rsampler(0)))
		caption.dcl_2d(rsampler(0)); //dcl_2d s0

	if (a.checkUsedRegister(rsampler(1)))
		caption.dcl_2d(rsampler(1)); //dcl_2d s1

	if (a.checkUsedRegister(rsampler(2)))
		caption.dcl_2d(rsampler(2)); //dcl_2d s2

	if (a.checkUsedRegister(rsampler(3)))
		caption.dcl_2d(rsampler(3)); //dcl_2d s3

	if (a.checkUsedRegister(rsampler(4)))
		caption.dcl_2d(rsampler(4)); //dcl_2d s4

	if (a.checkUsedRegister(rsampler(5)))
		caption.dcl_2d(rsampler(5)); //dcl_2d s5

	if (a.checkUsedRegister(rsampler(6)))
		caption.dcl_2d(rsampler(6)); //dcl_2d s6

	//const
	if (a.checkUsedRegister(rconst(28)))
		caption.def(rconst(28), 0, 1, 0, 0); //def c28, 0, 1, 0, 0


	QByteArray data = collect_pixel(caption, a);
	return data;
}
