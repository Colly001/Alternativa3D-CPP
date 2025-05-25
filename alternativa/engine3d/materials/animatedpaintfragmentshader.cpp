#include "animatedpaintfragmentshader.h"
#include "sharedparamsbits.h"
#include "alternativa/gfx/agal/fragmentshader.h"
#include "alternativa/gfx/agal/agaltest.h"
#include "alternativa/gfx/agal/samplerfilter.h"
#include "alternativa/gfx/agal/samplerrepeat.h"
#include "alternativa/gfx/agal/samplermipmap.h"
#include "alternativa/gfx/agal/samplertype.h"
#include "alternativa/gfx/agal/samplerdim.h"
#include "alternativa/engine3d/core/shadowmap.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"
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
			//1d16d6cb5e404177da711c9459381be7
			addData("a0 01 00 00 00 a1 01 04 00 00 00 04 00 0f 02 06 00 00 e4 04 00 00 00 18 00 00 e4 01 00 00 00 08 00 00 00 04 00 0f 02 04 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 04 00 0f 02 04 00 00 e4 02 00 00 00 18 00 00 e4 01 00 00 00 01 00 00 00 04 00 01 02 04 00 00 00 02 00 00 00 18 00 00 aa 01 00 00 00 01 00 00 00 04 00 02 02 04 00 00 55 02 00 00 00 18 00 00 ff 01 00 00 00 28 00 00 00 00 00 0f 02 04 00 00 e4 02 00 00 00 04 00 00 00 05 00 10 12 28 00 00 00 01 00 0f 02 00 00 00 e4 04 00 00 00 06 00 00 00 05 00 10 12 03 00 00 00 02 00 0f 02 00 00 00 e4 02 00 00 00 01 00 00 e4 02 00 00 00 03 00 00 00 02 00 0f 02 17 00 00 ff 01 00 00 00 02 00 00 e4 02 00 00 00 02 00 00 00 03 00 0f 02 17 00 00 aa 01 00 00 00 00 00 00 e4 02 00 00 00 02 00 00 00 04 00 0f 02 17 00 00 aa 01 00 00 00 01 00 00 e4 02 00 00 00 03 00 00 00 03 00 0f 02 17 00 00 ff 01 00 00 00 03 00 00 e4 02 00 00 00 03 00 00 00 03 00 0f 02 03 00 00 e4 02 00 00 00 04 00 00 e4 02 00 00 00 02 00 00 00 03 00 0f 02 17 00 00 aa 01 00 00 00 03 00 00 e4 02 00 00 00 29 00 00 00 04 00 01 02 01 00 00 00 02 00 00 00 17 00 00 aa 01 00 00 00 02 00 00 00 04 00 02 02 17 00 00 aa 01 00 00 00 04 00 00 00 02 00 00 00 03 00 00 00 05 00 0f 02 04 00 00 00 02 00 00 00 03 00 00 e4 02 00 00 00 03 00 00 00 06 00 0f 02 04 00 00 55 02 00 00 00 02 00 00 e4 02 00 00 00 01 00 00 00 00 00 0f 02 05 00 00 e4 02 00 00 00 06 00 00 e4 02 00 00 00 00 00 00 00 00 00 08 02 01 00 00 ff 02 00 00 00 00 00 00 00 00 00 00 00 28 00 00 00 01 00 0f 02 00 00 00 e4 04 00 00 00 00 00 00 00 05 00 10 12 02 00 00 00 02 00 08 02 17 00 00 aa 01 00 00 00 01 00 00 ff 02 00 00 00 03 00 00 00 00 00 01 02 00 00 00 00 02 00 00 00 02 00 00 ff 02 00 00 00 03 00 00 00 00 00 02 02 00 00 00 55 02 00 00 00 02 00 00 ff 02 00 00 00 03 00 00 00 00 00 04 02 00 00 00 aa 02 00 00 00 02 00 00 ff 02 00 00 00 01 00 00 00 00 00 07 02 00 00 00 a4 02 00 00 00 01 00 00 a4 02 00 00 00 00 00 00 00 01 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 03 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 05 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 06 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0f 03 00 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00");
			addKey(0x00000007); //true true true false false false false false false false false false false false false false false false

			//31abf15878b758df428e625275afa66b
			addData("a0 01 00 00 00 a1 01 04 00 00 00 04 00 0f 02 06 00 00 e4 04 00 00 00 18 00 00 e4 01 00 00 00 08 00 00 00 04 00 0f 02 04 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 04 00 0f 02 04 00 00 e4 02 00 00 00 18 00 00 e4 01 00 00 00 01 00 00 00 04 00 01 02 04 00 00 00 02 00 00 00 18 00 00 aa 01 00 00 00 01 00 00 00 04 00 02 02 04 00 00 55 02 00 00 00 18 00 00 ff 01 00 00 00 28 00 00 00 00 00 0f 02 04 00 00 e4 02 00 00 00 04 00 00 00 05 00 10 12 28 00 00 00 01 00 0f 02 00 00 00 e4 04 00 00 00 06 00 00 00 05 00 10 12 03 00 00 00 02 00 0f 02 00 00 00 e4 02 00 00 00 01 00 00 e4 02 00 00 00 03 00 00 00 02 00 0f 02 17 00 00 ff 01 00 00 00 02 00 00 e4 02 00 00 00 02 00 00 00 03 00 0f 02 17 00 00 aa 01 00 00 00 00 00 00 e4 02 00 00 00 02 00 00 00 04 00 0f 02 17 00 00 aa 01 00 00 00 01 00 00 e4 02 00 00 00 03 00 00 00 03 00 0f 02 17 00 00 ff 01 00 00 00 03 00 00 e4 02 00 00 00 03 00 00 00 03 00 0f 02 03 00 00 e4 02 00 00 00 04 00 00 e4 02 00 00 00 02 00 00 00 03 00 0f 02 17 00 00 aa 01 00 00 00 03 00 00 e4 02 00 00 00 29 00 00 00 04 00 01 02 01 00 00 00 02 00 00 00 17 00 00 aa 01 00 00 00 02 00 00 00 04 00 02 02 17 00 00 aa 01 00 00 00 04 00 00 00 02 00 00 00 03 00 00 00 05 00 0f 02 04 00 00 00 02 00 00 00 03 00 00 e4 02 00 00 00 03 00 00 00 06 00 0f 02 04 00 00 55 02 00 00 00 02 00 00 e4 02 00 00 00 01 00 00 00 00 00 0f 02 05 00 00 e4 02 00 00 00 06 00 00 e4 02 00 00 00 00 00 00 00 00 00 08 02 01 00 00 ff 02 00 00 00 00 00 00 00 00 00 00 00 28 00 00 00 01 00 0f 02 00 00 00 e4 04 00 00 00 00 00 00 00 05 00 10 12 02 00 00 00 02 00 08 02 17 00 00 aa 01 00 00 00 01 00 00 ff 02 00 00 00 03 00 00 00 00 00 01 02 00 00 00 00 02 00 00 00 02 00 00 ff 02 00 00 00 03 00 00 00 00 00 02 02 00 00 00 55 02 00 00 00 02 00 00 ff 02 00 00 00 03 00 00 00 00 00 04 02 00 00 00 aa 02 00 00 00 02 00 00 ff 02 00 00 00 01 00 00 00 00 00 07 02 00 00 00 a4 02 00 00 00 01 00 00 a4 02 00 00 00 00 00 00 00 01 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 03 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 05 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 06 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 00 00 07 02 00 00 00 e4 02 00 00 00 00 00 00 e4 01 00 00 00 01 00 00 00 00 00 07 02 00 00 00 e4 02 00 00 00 01 00 00 e4 01 00 00 00 00 00 00 00 00 00 0f 03 00 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00");
			addKey(0x00000047); //true true true false false false true false false false false false false false false false false false

			//ae19ef9604af61a0b30ca2174d2eecf1
			addData("a0 01 00 00 00 a1 01 04 00 00 00 04 00 0f 02 06 00 00 e4 04 00 00 00 18 00 00 e4 01 00 00 00 08 00 00 00 04 00 0f 02 04 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 04 00 0f 02 04 00 00 e4 02 00 00 00 18 00 00 e4 01 00 00 00 01 00 00 00 04 00 01 02 04 00 00 00 02 00 00 00 18 00 00 aa 01 00 00 00 01 00 00 00 04 00 02 02 04 00 00 55 02 00 00 00 18 00 00 ff 01 00 00 00 28 00 00 00 00 00 0f 02 04 00 00 e4 02 00 00 00 04 00 00 00 05 00 10 12 28 00 00 00 01 00 0f 02 00 00 00 e4 04 00 00 00 06 00 00 00 05 00 10 12 03 00 00 00 02 00 0f 02 00 00 00 e4 02 00 00 00 01 00 00 e4 02 00 00 00 03 00 00 00 02 00 0f 02 17 00 00 ff 01 00 00 00 02 00 00 e4 02 00 00 00 02 00 00 00 03 00 0f 02 17 00 00 aa 01 00 00 00 00 00 00 e4 02 00 00 00 02 00 00 00 04 00 0f 02 17 00 00 aa 01 00 00 00 01 00 00 e4 02 00 00 00 03 00 00 00 03 00 0f 02 17 00 00 ff 01 00 00 00 03 00 00 e4 02 00 00 00 03 00 00 00 03 00 0f 02 03 00 00 e4 02 00 00 00 04 00 00 e4 02 00 00 00 02 00 00 00 03 00 0f 02 17 00 00 aa 01 00 00 00 03 00 00 e4 02 00 00 00 29 00 00 00 04 00 01 02 01 00 00 00 02 00 00 00 17 00 00 aa 01 00 00 00 02 00 00 00 04 00 02 02 17 00 00 aa 01 00 00 00 04 00 00 00 02 00 00 00 03 00 00 00 05 00 0f 02 04 00 00 00 02 00 00 00 03 00 00 e4 02 00 00 00 03 00 00 00 06 00 0f 02 04 00 00 55 02 00 00 00 02 00 00 e4 02 00 00 00 01 00 00 00 00 00 0f 02 05 00 00 e4 02 00 00 00 06 00 00 e4 02 00 00 00 00 00 00 00 00 00 08 02 01 00 00 ff 02 00 00 00 00 00 00 00 00 00 00 00 28 00 00 00 01 00 0f 02 00 00 00 e4 04 00 00 00 00 00 00 00 05 00 10 12 02 00 00 00 02 00 08 02 17 00 00 aa 01 00 00 00 01 00 00 ff 02 00 00 00 03 00 00 00 00 00 01 02 00 00 00 00 02 00 00 00 02 00 00 ff 02 00 00 00 03 00 00 00 00 00 02 02 00 00 00 55 02 00 00 00 02 00 00 ff 02 00 00 00 03 00 00 00 00 00 04 02 00 00 00 aa 02 00 00 00 02 00 00 ff 02 00 00 00 01 00 00 00 00 00 07 02 00 00 00 a4 02 00 00 00 01 00 00 a4 02 00 00 00 00 00 00 00 01 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 03 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 04 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 05 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 06 00 0f 02 17 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 01 00 08 02 00 00 00 e4 02 00 00 00 12 00 00 e4 01 00 00 00 04 00 00 00 00 00 07 02 00 00 00 e4 02 00 00 00 01 00 00 ff 02 00 00 00 03 00 00 00 00 00 07 02 00 00 00 e4 02 00 00 00 00 00 00 e4 01 00 00 00 01 00 00 00 00 00 07 02 00 00 00 e4 02 00 00 00 01 00 00 e4 01 00 00 00 03 00 00 00 00 00 08 02 00 00 00 e4 02 00 00 00 00 00 00 e4 01 00 00 00 00 00 00 00 00 00 0f 03 00 00 00 e4 02 00 00 00 00 00 00 00 00 00 00 00");
			addKey(0x000000E7); //true true true false false true true true false false false false false false false false false false
		}
	} test;
}
#endif


QByteArray AnimatedPaintFragmentShader(ShaderSampler &samplers,
									   bool p1,
									   bool p2,
									   bool p3,
									   bool p4,
									   bool p5,
									   bool p6,
									   bool p7,
									   bool p8,
									   bool p9,
									   bool p10,
									   bool p11,
									   bool p12,
									   bool p13,
									   bool p14,
									   bool p15,
									   bool p16,
									   bool p17,
									   bool p18)
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

	class S : public FragmentShader
	{
	public:
		S(ShaderSampler &samplers,
		  bool p_a, //1
		  bool p_b, //2
		  bool p_c, //3
		  bool p_d, //4
		  bool p_f, //5
		  bool p_g, //6
		  bool p_h, //7
		  bool p_i, //8
		  bool p_j, //9
		  bool p_k, //10
		  bool p_l, //11
		  bool p_m, //12
		  bool p_n, //13
		  bool p_o, //14
		  bool p_p, //15
		  bool p_q, //16
		  bool p_r, //17
		  bool p_s) //18
		{
			SamplerRepeat *repeat = p_a ? SamplerRepeat::WRAP : SamplerRepeat::CLAMP;
			SamplerFilter *filter = p_b ? SamplerFilter::LINEAR : SamplerFilter::NEAREST;
			SamplerMipMap *mipMap = p_c ? (p_b ? SamplerMipMap::LINEAR : SamplerMipMap::NEAREST) : SamplerMipMap::NONE;
			SamplerType *type = p_d ? SamplerType::DXT1 : SamplerType::RGBA;

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


			//rcp r8.x, c24.x
			//rcp r8.y, c24.y
			//rcp r8.z, c24.z
			//rcp r8.w, c24.w
			//mul r4, t6, r8
			div(ft4, v6, fc[24]);

			//frc r4, r4
			frc(ft4, ft4);

			//mul r4, r4, c24
			mul(ft4, ft4, fc[24]);

			//add r4.x, r4.x, c24.z
			add(ft4->x, ft4->x, fc[24]->z);

			//add r4.y, r4.y, c24.w
			add(ft4->y, ft4->y, fc[24]->w);


			//texld r8, r4, s4
			//mov r8.w, c28.y
			//mov r0, r8


			//texld r0, r4, s4
			samplers.add(4, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);
			tex(ft0, ft4, fs4->dim(SamplerDim::D2)->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));

			//texld r8, t0, s6
			//mov r8.w, c28.y
			//mov r1, r8

			//texld r1, t0, s6
			samplers.add(6, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);
			tex(ft1, v0, fs6->dim(SamplerDim::D2)->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));

			//mul r2, r0, r1
			mul(ft2, ft0, ft1);

			//mul r2, c23.w, r2
			mul(ft2, fc[23]->w, ft2);

			//add r3, c23.z, -r0
			sub(ft3, fc[23]->z, ft0);

			//add r4, c23.z, -r1
			sub(ft4, fc[23]->z, ft1);

			//mul r3, c23.w, r3
			mul(ft3, fc[23]->w, ft3);

			//mul r3, r3, r4
			mul(ft3, ft3, ft4);

			//add r3, c23.z, -r3
			sub(ft3, fc[23]->z, ft3);

			//setp_ge p0, r1.x, c23.z
			//(!p0) mov r4.x, c28.x
			//(p0) mov r4.x, c28.y
			sge(ft4->x, ft1->x, fc[23]->z);



			//add r4.y, c23.z, -r4.x
			sub(ft4->y, fc[23]->z, ft4->x);

			//mul r5, r4.x, r3
			mul(ft5, ft4->x, ft3);

			//mul r6, r4.y, r2
			mul(ft6, ft4->y, ft2);

			//add r0, r5, r6
			add(ft0, ft5, ft6);

			//mov r0.w, r1.w
			mov(ft0->w, ft1->w);

			//texld r8, t0, s0
			//mov r8.w, c28.y
			//mov r1, r8

			//texld r1, t0, s0
			samplers.add(0, SHADER_SAMPLER_DIM::_2D, _repeat, _filter, _mipMap, _type);
			tex(ft1, v0, fs0->dim(SamplerDim::D2)->repeat(repeat)->filter(filter)->mipmap(mipMap)->type(type));

			//add r2.w, c23.z, -r1.w
			sub(ft2->w, fc[23]->z, ft1->w);

			//mul r0.x, r0.x, r2.w
			mul(ft0->x, ft0->x, ft2->w);

			//mul r0.y, r0.y, r2.w
			mul(ft0->y, ft0->y, ft2->w);

			//mul r0.z, r0.z, r2.w
			mul(ft0->z, ft0->z, ft2->w);

			//add r0.xyz, r0.xyzz, r1.xyzz
			add(ft0->xyz, ft0->xyz, ft1->xyz);

			//mov r1, c23.x
			mov(ft1, fc[23]->x);

			//mov r2, c23.x
			mov(ft2, fc[23]->x);

			//mov r3, c23.x
			mov(ft3, fc[23]->x);

			//mov r4, c23.x
			mov(ft4, fc[23]->x);

			//mov r5, c23.x
			mov(ft5, fc[23]->x);

			//mov r6, c23.x
			mov(ft6, fc[23]->x);

			if (p_f)
			{
				sub(ft1->w, ft0, fc[14]); //add r1.w, r0, -c14
				//mov r8, r1.w
				//texkill r8
				kil(ft1->w);
			}

			if (p_g)
			{
				add(ft1->w, ft0, fc[18]); //add r1.w, r0, c18
				//rcp r8, r1.w
				//mul r0.xyz, r0, r8
				div(ft0->xyz, ft0, ft1->w);
			}

			if (p_h)
			{	
				mul(ft0->xyz, ft0, fc[0]); //mul r0.xyz, r0, c0
				add(ft0->xyz, ft0, fc[1]); //add r0.xyz, r0, c1
			}

			if (p_i)
			{
				mul(ft0->w, ft0, fc[0]); //mul r0.w, r0, c0
			}

			if (p_j)
			{
				abs(ft1, v0->z); //abs r1, t0.z
				sat(ft1, ft1); //mov_sat r1, r1
				sub(ft1, fc[17], ft1); //add r1, c17, -r1
				mul(ft0->w, ft0, ft1); //mul r0.w, r0, r1
			}

			if (p_l || p_m || p_n || p_p)
			{
				//mov r8, t1
				//rcp r9, t1.z
				//mul r4, r8, r9
				div(ft4, v1, v1->z);
				mul(ft4->xy, ft4, fc[18]); //mul r4.xy, r4, c18
				add(ft4->xy, ft4, fc[18]); //add r4.xy, r4, c18
			}

			if (p_l || p_m || p_n)
			{
				mul(ft3, ft4, fc[4]); //mul r3, r4, c4
			}

			if (p_l || p_m)
			{
				//texld r1, r3, s1
				samplers.add(1, SHADER_SAMPLER_DIM::_2D,
							 SHADER_SAMPLER_REPEAT::CLAMP,
							 SHADER_SAMPLER_FILTER::NEAREST,
							 SHADER_SAMPLER_MIPMAP::NONE,
							 SHADER_SAMPLER_TYPE::RGBA);
				tex(ft1, ft3, fs1->dim(SamplerDim::D2)->repeat(SamplerRepeat::CLAMP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
			}

			if (p_n)
			{
				//texld r6, r3, s5
				samplers.add(5, SHADER_SAMPLER_DIM::_2D,
							 SHADER_SAMPLER_REPEAT::CLAMP,
							 SHADER_SAMPLER_FILTER::NEAREST,
							 SHADER_SAMPLER_MIPMAP::NONE,
							 SHADER_SAMPLER_TYPE::RGBA);
				tex(ft6, ft3, fs5->dim(SamplerDim::D2)->repeat(SamplerRepeat::CLAMP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
			}

			if (p_l)
			{
				dp3(ft2, ft1, fc[17]); //dp3 r2, r1, c17
				sub(ft2, ft2, v1); //add r2, r2, -t1
				abs(ft2, ft2); //abs r2, r2
				//rcp r8.x, c14.x
				//rcp r8.y, c14.y
				//rcp r8.z, c14.z
				//rcp r8.w, c14.w
				//mul r2, r2, r8
				div(ft2, ft2, fc[14]);
				sat(ft2, ft2); //mov_sat r2, r2
				mul(ft0->w, ft0, ft2->z); //mul r0.w, r0, r2.z
			}

			if (p_m)
			{
				mul(ft2, fc[12], ft1->w); //mul r2, c12, r1.w
				sub(ft2, fc[17]->w, ft2); //add r2, c17.w, -r2
				mul(ft0->xyz, ft0, ft2); //mul r0.xyz, r0, r2
			}

			if (p_p)
			{
				mov(ft5, fc[5]); //mov r5, c5
				mul(ft5->z, ft5, v2); //mul r5.z, r5, t2
				mul(ft3, ft4, fc[7]); //mul r3, r4, c7
				//texld r1, r3, s3
				samplers.add(3, SHADER_SAMPLER_DIM::_2D,
							 SHADER_SAMPLER_REPEAT::WRAP,
							 SHADER_SAMPLER_FILTER::NEAREST,
							 SHADER_SAMPLER_MIPMAP::NONE,
							 SHADER_SAMPLER_TYPE::RGBA);
				tex(ft1, ft3, fs3->dim(SamplerDim::D2)->repeat(SamplerRepeat::WRAP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
				mul(ft1->z, ft1, fc[6]); //mul r1.z, r1, c6
				mul(ft2, ft1->z, ft1); //mul r2, r1.z, r1
				for (int i = 0; i < ShadowMap::numSamples; i++)
				{
					if (i == 0)
					{
						add(ft1, ft2, v2); //add r1, r2, t2
						//texld r1, r1, s2
						samplers.add(2, SHADER_SAMPLER_DIM::_2D,
									 SHADER_SAMPLER_REPEAT::CLAMP,
									 SHADER_SAMPLER_FILTER::NEAREST,
									 SHADER_SAMPLER_MIPMAP::NONE,
									 SHADER_SAMPLER_TYPE::RGBA);
						tex(ft1, ft1, fs2->dim(SamplerDim::D2)->
							repeat(SamplerRepeat::CLAMP)->
							filter(SamplerFilter::NEAREST)->
							mipmap(SamplerMipMap::NONE));
						dp3(ft1, ft1, ft5); //dp3 r1, r1, r5
						sat(ft1, ft1); //mov_sat r1, r1
						mov(ft4, ft1); //mov r4, r1
					}
					else
					{
						if ((i % 2) > 0)
						{
							dp3(ft3->x, ft2, fc[8]); //dp3 r3.x, r2, c8
							dp3(ft3->y, ft2, fc[9]); //dp3 r3.y, r2, c9
							add(ft1, ft3, v2); //add r1, r3, t2
						}
						else
						{
							dp3(ft2->x, ft3, fc[8]); //dp3 r2.x, r3, c8
							dp3(ft2->y, ft3, fc[9]); //dp3 r2.y, r3, c9
							add(ft1, ft2, v2); //add r1, r2, t2
						}
						//texld r1, r1, s2
						samplers.add(2, SHADER_SAMPLER_DIM::_2D,
									 SHADER_SAMPLER_REPEAT::CLAMP,
									 SHADER_SAMPLER_FILTER::NEAREST,
									 SHADER_SAMPLER_MIPMAP::NONE,
									 SHADER_SAMPLER_TYPE::RGBA);
						tex(ft1, ft1, fs2->dim(SamplerDim::D2)->
							repeat(SamplerRepeat::CLAMP)->
							filter(SamplerFilter::NEAREST)->
							mipmap(SamplerMipMap::NONE));
						dp3(ft1, ft1, ft5); //dp3 r1, r1, r5
						sat(ft1, ft1); //mov_sat r1, r1
						add(ft4, ft4, ft1); //add r4, r4, r1
					}
				}
				mul(ft2, ft4, fc[6]); //mul r2, r4, c6
				sat(ft1, v2); //mov_sat r1, t2
				mul(ft2, ft2, ft1); //mul r2, r2, r1
				mul(ft2->w, ft2, fc[7]); //mul r2.w, r2, c7
			}
			if (p_k)
			{
				if (p_p)
				{
					sat(ft1, v1); //mov_sat r1, t1
					max(ft2, ft2, ft1); //max r2, r2, r1
				}
				else
				{
					sat(ft2, v1); //mov_sat r2, t1
				}
			}
			if (p_p || p_k)
			{
				sub(ft2, fc[17], ft2); //add r2, c17, -r2
				mul(ft2, fc[10], ft2->w); //mul r2, c10, r2.w
				add(ft2, ft2, fc[11]); //add r2, r2, c11
				if (p_n)
				{
					add(ft6, ft6, ft6); //add r6, r6, r6
					add(ft2, ft2, ft6); //add r2, r2, r6
				}
				else
				{
					if (p_o)
					{
						add(ft2, ft2, fc[13]); //add r2, r2, c13
					}
				}
				mul(ft0->xyz, ft0, ft2); //mul r0.xyz, r0, r2
			}
			else
			{
				if (p_n)
				{
					add(ft2, ft6, ft6); //add r2, r6, r
					mul(ft0->xyz, ft0, ft2); //mul r0.xyz, r0, r2
				}
				else
				{
					if (p_o)
					{
						mul(ft0->xyz, ft0, fc[13]); //mul r0.xyz, r0, c13
					}
				}
			}
			if (p_q)
			{
				if (p_r)
				{
					mul(ft0->xyz, ft0, fc[13]->w); //mul r0.xyz, r0, c13.w
					add(ft0->xyz, ft0, fc[13]); //add r0.xyz, r0, c13
				}
				else
				{
					sat(ft1, v0); //mov_sat r1, t0
					mul(ft1, ft1, fc[2]); //mul r1, r1, c2
					if (p_s)
					{
						sub(ft1, fc[17], ft1); //add r1, c17, -r1
						mul(ft0->w, ft0, ft1); //mul r0.w, r0, r1
					}
					else
					{
						mul(ft1->xyz, fc[2], ft1->w); //mul r1.xyz, c2, r1.w
						sub(ft1->w, fc[17], ft1); //add r1.w, c17, -r1
						mul(ft0->xyz, ft0, ft1->w); //mul r0.xyz, r0, r1.w
						add(ft0->xyz, ft0, ft1); //add r0.xyz, r0, r1
					}
				}
			}
			mov(oc, ft0); //mov oC0, r0
		}
	} s(samplers, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18);

	QByteArray data = s.getAgalcode();

#ifdef CHECK_AGAL
	if (!test.check(b.k, data.toHex(' ')))
	{
		qDebug() << "error AnimatedPaintFragmentShader" << b.k;
	}
#endif

	return data;
}
