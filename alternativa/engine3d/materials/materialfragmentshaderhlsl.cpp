#include "materialfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/engine3d/core/shadowmap.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


void MaterialFragmentShaderHLSL(HLSL_ASSEMBLY::HLSL_ASM_PIXEL &a,
								ShaderSampler &samplers,
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

	if (p_f)
	{
		a.add(rtemp(1, "w"), rtemp(0), N(rconst(14))); //add r1.w, r0, -c14
		a.mov(rtemp(8), rtemp(1, "w")); //mov r8, r1.w
		a.texkill(rtemp(8));

		//sub(ft1->w, ft0, fc[14]); //add r1.w, r0, -c14
		//mov r8, r1.w
		//texkill r8
		//kil(ft1->w);
	}

	if (p_g)
	{
		a.add(rtemp(1, "w"), rtemp(0), rconst(18)); //add r1.w, r0, c18
		a.rcp(rtemp(8), rtemp(1, "w")); //rcp r8, r1.w
		a.mul(rtemp(0, "xyz"), rtemp(0), rtemp(8)); //mul r0.xyz, r0, r8

		//add(ft1->w, ft0, fc[18]); //add r1.w, r0, c18
		//rcp r8, r1.w
		//mul r0.xyz, r0, r8
		//div(ft0->xyz, ft0, ft1->w);
	}

	if (p_h)
	{
		a.mul(rtemp(0, "xyz"), rtemp(0), rconst(0)); //mul r0.xyz, r0, c0
		a.add(rtemp(0, "xyz"), rtemp(0), rconst(1)); //add r0.xyz, r0, c1

		//mul(ft0->xyz, ft0, fc[0]); //mul r0.xyz, r0, c0
		//add(ft0->xyz, ft0, fc[1]); //add r0.xyz, r0, c1
	}

	if (p_i)
	{
		a.mul(rtemp(0, "w"), rtemp(0), rconst(0)); //mul r0.w, r0, c0
		//mul(ft0->w, ft0, fc[0]); //mul r0.w, r0, c0
	}

	if (p_j)
	{
		a.abs(rtemp(1), rtexture(0, "z")); //abs r1, t0.z
		a.mov_sat(rtemp(1), rtemp(1)); //mov_sat r1, r1
		a.add(rtemp(1), rconst(17), N(rtemp(1))); //add r1, c17, -r1
		a.mul(rtemp(0, "w"), rtemp(0), rtemp(1)); //mul r0.w, r0, r1

		//abs(ft1, v0->z); //abs r1, t0.z
		//sat(ft1, ft1); //mov_sat r1, r1
		//sub(ft1, fc[17], ft1); //add r1, c17, -r1
		//mul(ft0->w, ft0, ft1); //mul r0.w, r0, r1
	}

	if (p_l || p_m || p_n || p_p)
	{
		a.mov(rtemp(8), rtexture(1)); //mov r8, t1
		a.rcp(rtemp(9), rtexture(1, "z")); //rcp r9, t1.z
		a.mul(rtemp(4), rtemp(8), rtemp(9)); //mul r4, r8, r9
		a.mul(rtemp(4, "xy"), rtemp(4), rconst(18)); //mul r4.xy, r4, c18
		a.add(rtemp(4, "xy"), rtemp(4), rconst(18)); //add r4.xy, r4, c18

		//mov r8, t1
		//rcp r9, t1.z
		//mul r4, r8, r9
		//div(ft4, v1, v1->z);
		//mul(ft4->xy, ft4, fc[18]); //mul r4.xy, r4, c18
		//add(ft4->xy, ft4, fc[18]); //add r4.xy, r4, c18
	}

	if (p_l || p_m || p_n)
	{
		a.mul(rtemp(3), rtemp(4), rconst(4)); //mul r3, r4, c4
		//mul(ft3, ft4, fc[4]); //mul r3, r4, c4
	}

	if (p_l || p_m)
	{
		samplers.add(1, SHADER_SAMPLER_DIM::_2D,
					 SHADER_SAMPLER_REPEAT::CLAMP,
					 SHADER_SAMPLER_FILTER::NEAREST,
					 SHADER_SAMPLER_MIPMAP::NONE,
					 SHADER_SAMPLER_TYPE::RGBA);
		a.texld(rtemp(1), rtemp(3), rsampler(1)); //texld r1, r3, s1
		//tex(ft1, ft3, fs1->dim(SamplerDim::D2)->repeat(SamplerRepeat::CLAMP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
	}

	if (p_n)
	{
		samplers.add(5, SHADER_SAMPLER_DIM::_2D,
					 SHADER_SAMPLER_REPEAT::CLAMP,
					 SHADER_SAMPLER_FILTER::NEAREST,
					 SHADER_SAMPLER_MIPMAP::NONE,
					 SHADER_SAMPLER_TYPE::RGBA);
		a.texld(rtemp(6), rtemp(3), rsampler(5)); //texld r6, r3, s5
		//tex(ft6, ft3, fs5->dim(SamplerDim::D2)->repeat(SamplerRepeat::CLAMP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
	}

	if (p_l)
	{
		a.dp3(rtemp(2), rtemp(1), rconst(17)); //dp3 r2, r1, c17
		a.add(rtemp(2), rtemp(2), N(rtexture(1))); //add r2, r2, -t1
		a.abs(rtemp(2), rtemp(2)); //abs r2, r2
		a.rcp(rtemp(8, "x"), rconst(14, "x")); //rcp r8.x, c14.x
		a.rcp(rtemp(8, "y"), rconst(14, "y")); //rcp r8.y, c14.y
		a.rcp(rtemp(8, "z"), rconst(14, "z")); //rcp r8.z, c14.z
		a.rcp(rtemp(8, "w"), rconst(14, "w")); //rcp r8.w, c14.w
		a.mul(rtemp(2), rtemp(2), rtemp(8)); //mul r2, r2, r8
		a.mov_sat(rtemp(2), rtemp(2)); //mov_sat r2, r2
		a.mul(rtemp(0, "w"), rtemp(0), rtemp(2, "z")); //mul r0.w, r0, r2.z

		//dp3(ft2, ft1, fc[17]); //dp3 r2, r1, c17
		//sub(ft2, ft2, v1); //add r2, r2, -t1
		//abs(ft2, ft2); //abs r2, r2
		//rcp r8.x, c14.x
		//rcp r8.y, c14.y
		//rcp r8.z, c14.z
		//rcp r8.w, c14.w
		//mul r2, r2, r8
		//div(ft2, ft2, fc[14]);
		//sat(ft2, ft2); //mov_sat r2, r2
		//mul(ft0->w, ft0, ft2->z); //mul r0.w, r0, r2.z
	}

	if (p_m)
	{
		a.mul(rtemp(2), rconst(12), rtemp(1, "w")); //mul r2, c12, r1.w
		a.add(rtemp(2), rconst(17, "w"), N(rtemp(2))); //add r2, c17.w, -r2
		a.mul(rtemp(0, "xyz"), rtemp(0), rtemp(2)); //mul r0.xyz, r0, r2

		//mul(ft2, fc[12], ft1->w); //mul r2, c12, r1.w
		//sub(ft2, fc[17]->w, ft2); //add r2, c17.w, -r2
		//mul(ft0->xyz, ft0, ft2); //mul r0.xyz, r0, r2
	}

	if (p_p)
	{
		a.mov(rtemp(5), rconst(5)); //mov r5, c5
		a.mul(rtemp(5, "z"), rtemp(5), rtexture(2)); //mul r5.z, r5, t2
		a.mul(rtemp(3), rtemp(4), rconst(7)); //mul r3, r4, c7
		samplers.add(3, SHADER_SAMPLER_DIM::_2D,
					 SHADER_SAMPLER_REPEAT::WRAP,
					 SHADER_SAMPLER_FILTER::NEAREST,
					 SHADER_SAMPLER_MIPMAP::NONE,
					 SHADER_SAMPLER_TYPE::RGBA);
		a.texld(rtemp(1), rtemp(3), rsampler(3)); //texld r1, r3, s3
		a.mul(rtemp(1, "z"), rtemp(1), rconst(6)); //mul r1.z, r1, c6
		a.mul(rtemp(2), rtemp(1, "z"), rtemp(1)); //mul r2, r1.z, r1

		//mov(ft5, fc[5]); //mov r5, c5
		//mul(ft5->z, ft5, v2); //mul r5.z, r5, t2
		//mul(ft3, ft4, fc[7]); //mul r3, r4, c7
		//texld r1, r3, s3
		//tex(ft1, ft3, fs3->dim(SamplerDim::D2)->repeat(SamplerRepeat::WRAP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
		//mul(ft1->z, ft1, fc[6]); //mul r1.z, r1, c6
		//mul(ft2, ft1->z, ft1); //mul r2, r1.z, r1
		for (int i = 0; i < ShadowMap::numSamples; i++)
		{
			if (i == 0)
			{
				a.add(rtemp(1), rtemp(2), rtexture(2)); //add r1, r2, t2
				samplers.add(2, SHADER_SAMPLER_DIM::_2D,
							 SHADER_SAMPLER_REPEAT::CLAMP,
							 SHADER_SAMPLER_FILTER::NEAREST,
							 SHADER_SAMPLER_MIPMAP::NONE,
							 SHADER_SAMPLER_TYPE::RGBA);
				a.texld(rtemp(1), rtemp(1), rsampler(2)); //texld r1, r1, s2
				a.dp3(rtemp(1), rtemp(1), rtemp(5)); //dp3 r1, r1, r5
				a.mov_sat(rtemp(1), rtemp(1)); //mov_sat r1, r1
				a.mov(rtemp(4), rtemp(1)); //mov r4, r1

				//add(ft1, ft2, v2); //add r1, r2, t2
				//texld r1, r1, s2
				//tex(ft1, ft1, fs2->dim(SamplerDim::D2)->repeat(SamplerRepeat::CLAMP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
				//dp3(ft1, ft1, ft5); //dp3 r1, r1, r5
				//sat(ft1, ft1); //mov_sat r1, r1
				//mov(ft4, ft1); //mov r4, r1
			}
			else
			{
				if ((i % 2) > 0)
				{
					a.dp3(rtemp(3, "x"), rtemp(2), rconst(8)); //dp3 r3.x, r2, c8
					a.dp3(rtemp(3, "y"), rtemp(2), rconst(9)); //dp3 r3.y, r2, c9
					a.add(rtemp(1), rtemp(3), rtexture(2)); //add r1, r3, t2

					//dp3(ft3->x, ft2, fc[8]); //dp3 r3.x, r2, c8
					//dp3(ft3->y, ft2, fc[9]); //dp3 r3.y, r2, c9
					//add(ft1, ft3, v2); //add r1, r3, t2
				}
				else
				{
					a.dp3(rtemp(2, "x"), rtemp(3), rconst(8)); //dp3 r2.x, r3, c8
					a.dp3(rtemp(2, "y"), rtemp(3), rconst(9)); //dp3 r2.y, r3, c9
					a.add(rtemp(1), rtemp(2), rtexture(2)); //add r1, r2, t2

					//dp3(ft2->x, ft3, fc[8]); //dp3 r2.x, r3, c8
					//dp3(ft2->y, ft3, fc[9]); //dp3 r2.y, r3, c9
					//add(ft1, ft2, v2); //add r1, r2, t2
				}

				samplers.add(2, SHADER_SAMPLER_DIM::_2D,
							 SHADER_SAMPLER_REPEAT::CLAMP,
							 SHADER_SAMPLER_FILTER::NEAREST,
							 SHADER_SAMPLER_MIPMAP::NONE,
							 SHADER_SAMPLER_TYPE::RGBA);

				a.texld(rtemp(1), rtemp(1), rsampler(2)); ////texld r1, r1, s2
				a.dp3(rtemp(1), rtemp(1), rtemp(5)); //dp3 r1, r1, r5
				a.mov_sat(rtemp(1), rtemp(1)); //mov_sat r1, r1
				a.add(rtemp(4), rtemp(4), rtemp(1)); //add r4, r4, r1

				//texld r1, r1, s2
				//tex(ft1, ft1, fs2->dim(SamplerDim::D2)->repeat(SamplerRepeat::CLAMP)->filter(SamplerFilter::NEAREST)->mipmap(SamplerMipMap::NONE));
				//dp3(ft1, ft1, ft5); //dp3 r1, r1, r5
				//sat(ft1, ft1); //mov_sat r1, r1
				//add(ft4, ft4, ft1); //add r4, r4, r1
			}
		}

		a.mul(rtemp(2), rtemp(4), rconst(6)); //mul r2, r4, c6
		a.mov_sat(rtemp(1), rtexture(2)); //mov_sat r1, t2
		a.mul(rtemp(2), rtemp(2), rtemp(1)); //mul r2, r2, r1
		a.mul(rtemp(2, "w"), rtemp(2), rconst(7)); //mul r2.w, r2, c7

		//mul(ft2, ft4, fc[6]); //mul r2, r4, c6
		//sat(ft1, v2); //mov_sat r1, t2
		//mul(ft2, ft2, ft1); //mul r2, r2, r1
		//mul(ft2->w, ft2, fc[7]); //mul r2.w, r2, c7
	}
	if (p_k)
	{
		if (p_p)
		{
			a.mov_sat(rtemp(1), rtexture(1)); //mov_sat r1, t1
			a.max(rtemp(2), rtemp(2), rtemp(1)); //max r2, r2, r1

			//sat(ft1, v1); //mov_sat r1, t1
			//max(ft2, ft2, ft1); //max r2, r2, r1
		}
		else
		{
			a.mov_sat(rtemp(2), rtexture(1)); //mov_sat r2, t1
			//sat(ft2, v1); //mov_sat r2, t1
		}
	}
	if (p_p || p_k)
	{
		a.add(rtemp(2), rconst(17), N(rtemp(2))); //add r2, c17, -r2
		a.mul(rtemp(2), rconst(10), rtemp(2, "w")); //mul r2, c10, r2.w
		a.add(rtemp(2), rtemp(2), rconst(11)); //add r2, r2, c11

		//sub(ft2, fc[17], ft2); //add r2, c17, -r2
		//mul(ft2, fc[10], ft2->w); //mul r2, c10, r2.w
		//add(ft2, ft2, fc[11]); //add r2, r2, c11
		if (p_n)
		{
			a.add(rtemp(6), rtemp(6), rtemp(6)); //add r6, r6, r6
			a.add(rtemp(2), rtemp(2), rtemp(6)); //add r2, r2, r6

			//add(ft6, ft6, ft6); //add r6, r6, r6
			//add(ft2, ft2, ft6); //add r2, r2, r6
		}
		else
		{
			if (p_o)
			{
				a.add(rtemp(2), rtemp(2), rconst(13)); //add r2, r2, c13
				//add(ft2, ft2, fc[13]); //add r2, r2, c13
			}
		}
		a.mul(rtemp(0, "xyz"), rtemp(0), rtemp(2)); //mul r0.xyz, r0, r2
		//mul(ft0->xyz, ft0, ft2); //mul r0.xyz, r0, r2
	}
	else
	{
		if (p_n)
		{
			a.add(rtemp(2), rtemp(6), rtemp(6)); //add r2, r6, r6
			a.mul(rtemp(0, "xyz"), rtemp(0), rtemp(2)); //mul r0.xyz, r0, r2

			//add(ft2, ft6, ft6); //add r2, r6, r6
			//mul(ft0->xyz, ft0, ft2); //mul r0.xyz, r0, r2
		}
		else
		{
			if (p_o)
			{
				a.mul(rtemp(0, "xyz"), rtemp(0), rconst(13)); //mul r0.xyz, r0, c13
				//mul(ft0->xyz, ft0, fc[13]); //mul r0.xyz, r0, c13
			}
		}
	}
	if (p_q)
	{
		if (p_r)
		{
			a.mul(rtemp(0, "xyz"), rtemp(0), rconst(13, "w")); //mul r0.xyz, r0, c13.w
			a.add(rtemp(0, "xyz"), rtemp(0), rconst(13)); //add r0.xyz, r0, c13
			//mul(ft0->xyz, ft0, fc[13]->w); //mul r0.xyz, r0, c13.w
			//add(ft0->xyz, ft0, fc[13]); //add r0.xyz, r0, c13
		}
		else
		{
			a.mov_sat(rtemp(1), rtexture(0)); //mov_sat r1, t0
			a.mul(rtemp(1), rtemp(1), rconst(2)); //mul r1, r1, c2

			//sat(ft1, v0); //mov_sat r1, t0
			//mul(ft1, ft1, fc[2]); //mul r1, r1, c2
			if (p_s)
			{
				a.add(rtemp(1), rconst(17), N(rtemp(1))); //add r1, c17, -r1
				a.mul(rtemp(0, "w"), rtemp(0), rtemp(1)); //mul r0.w, r0, r1
				//sub(ft1, fc[17], ft1); //add r1, c17, -r1
				//mul(ft0->w, ft0, ft1); //mul r0.w, r0, r1
			}
			else
			{
				a.mul(rtemp(1, "xyz"), rconst(2), rtemp(1, "w")); //mul r1.xyz, c2, r1.w
				a.add(rtemp(1, "w"), rconst(17), N(rtemp(1))); //add r1.w, c17, -r1
				a.mul(rtemp(0, "xyz"), rtemp(0), rtemp(1, "w")); //mul r0.xyz, r0, r1.w
				a.add(rtemp(0, "xyz"), rtemp(0), rtemp(1)); //add r0.xyz, r0, r1

				//mul(ft1->xyz, fc[2], ft1->w); //mul r1.xyz, c2, r1.w
				//sub(ft1->w, fc[17], ft1); //add r1.w, c17, -r1
				//mul(ft0->xyz, ft0, ft1->w); //mul r0.xyz, r0, r1.w
				//add(ft0->xyz, ft0, ft1); //add r0.xyz, r0, r1
			}
		}
	}

	a.mov(rcolorout(0), rtemp(0)); //mov oC0, r0
	//mov(oc, ft0); //mov oC0, r0
}
