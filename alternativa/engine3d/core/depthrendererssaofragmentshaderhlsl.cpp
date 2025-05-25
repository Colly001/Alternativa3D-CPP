#include "depthrendererssaofragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"


QByteArray DepthRendererSSAOFragmentShaderHLSL(ShaderSampler &samplers, int count)
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;

	samplers.add(0, SHADER_SAMPLER_DIM::_2D, SHADER_SAMPLER_REPEAT::CLAMP, SHADER_SAMPLER_FILTER::NEAREST, SHADER_SAMPLER_MIPMAP::NONE, SHADER_SAMPLER_TYPE::RGBA);

	a.texld(rtemp(0), rtexture(0), rsampler(0)); //texld r0, t0, s0
	a.mul(rtemp(0, "zw"), rtemp(0), rconst(6)); //mul r0.zw, r0, c6
	a.mov(rtemp(8), rtemp(0)); //mov r8, r0
	a.mad(rtemp(8), rtemp(8), rconst(31, "x"), rconst(31, "y")); //mad r8, r8, c31.x, c31.y
	a.frc(rtemp(8), rtemp(8)); //frc r8, r8
	a.mad(rtemp(8), rtemp(8), rconst(31, "z"), rconst(31, "w")); //mad r8, r8, c31.z, c31.w
	a.sincos(rtemp(9, "x"), rtemp(8, "x"), rconst(29), rconst(30)); //sincos r9.x, r8.x, c29, c30
	a.mov(rtemp(1, "x"), rtemp(9, "x")); //mov r1.x, r9.x
	a.sincos(rtemp(9, "x"), rtemp(8, "y"), rconst(29), rconst(30)); //sincos r9.x, r8.y, c29, c30
	a.mov(rtemp(1, "y"), rtemp(9, "x")); //mov r1.y, r9.x
	a.sincos(rtemp(9, "x"), rtemp(8, "z"), rconst(29), rconst(30)); //sincos r9.x, r8.z, c29, c30
	a.mov(rtemp(1, "z"), rtemp(9, "x")); // mov r1.z, r9.x
	a.sincos(rtemp(9, "x"), rtemp(8, "w"), rconst(29), rconst(30)); //sincos r9.x, r8.w, c29, c30
	a.mov(rtemp(1, "w"), rtemp(9, "x")); //mov r1.w, r9.x
	a.mov(rtemp(8), rtemp(0)); //mov r8, r0
	a.mad(rtemp(8), rtemp(8), rconst(31, "x"), rconst(31, "y")); //mad r8, r8, c31.x, c31.y
	a.frc(rtemp(8), rtemp(8)); //frc r8, r8
	a.mad(rtemp(8), rtemp(8), rconst(31, "z"), rconst(31, "w")); //mad r8, r8, c31.z, c31.w
	a.sincos(rtemp(9, "y"), rtemp(8, "x"), rconst(29), rconst(30)); //sincos r9.y, r8.x, c29, c30
	a.mov(rtemp(2, "x"), rtemp(9, "y")); //mov r2.x, r9.y
	a.sincos(rtemp(9, "y"), rtemp(8, "y"), rconst(29), rconst(30)); //sincos r9.y, r8.y, c29, c30
	a.mov(rtemp(2, "y"), rtemp(9, "y")); //mov r2.y, r9.y
	a.sincos(rtemp(9, "y"), rtemp(8, "z"), rconst(29), rconst(30)); //sincos r9.y, r8.z, c29, c30
	a.mov(rtemp(2, "z"), rtemp(9, "y")); //mov r2.z, r9.y
	a.sincos(rtemp(9, "y"), rtemp(8, "w"), rconst(29), rconst(30)); //sincos r9.y, r8.w, c29, c30
	a.mov(rtemp(2, "w"), rtemp(9, "y")); //mov r2.w, r9.y
	a.mul(rtemp(1, "x"), rtemp(1, "w"), rtemp(1, "z")); //mul r1.x, r1.w, r1.z
	a.mul(rtemp(1, "y"), rtemp(2, "w"), rtemp(1, "z")); //mul r1.y, r2.w, r1.z
	a.mov(rtemp(1, "z"), N(rtemp(2))); //mov r1.z, -r2
	a.dp3(rtemp(2, "z"), rtemp(0), rconst(0)); //dp3 r2.z, r0, c0
	a.mul(rtemp(2, "xy"), rtexture(2), rtemp(2, "z")); //mul r2.xy, t2, r2.z
	samplers.add(1, SHADER_SAMPLER_DIM::_2D, SHADER_SAMPLER_REPEAT::WRAP, SHADER_SAMPLER_FILTER::NEAREST, SHADER_SAMPLER_MIPMAP::NONE, SHADER_SAMPLER_TYPE::RGBA);
	a.texld(rtemp(3), rtexture(1), rsampler(1)); //texld r3, t1, s1
	a.mul(rtemp(3, "z"), rtemp(3), rconst(1)); //mul r3.z, r3, c1
	a.mul(rtemp(3), rtemp(3, "z"), rtemp(3)); //mul r3, r3.z, r3


	for (int i = 0 ; i < count; i++)
	{
		if (i > 0)
		{
			if ((i % 2) > 0)
			{
				a.dp3(rtemp(6), rtemp(3), rconst(4)); //dp3 r6, r3, c4
				a.dp3(rtemp(6, "y"), rtemp(3), rconst(5)); //dp3 r6.y, r3, c5
			}
			else
			{
				a.dp3(rtemp(3, "x"), rtemp(6), rconst(4)); //dp3 r3.x, r6, c4
				a.dp3(rtemp(3, "y"), rtemp(6), rconst(5)); //dp3 r3.y, r6, c5
			}
		}


		a.rcp(rtemp(8, "x"), rconst(3, "x")); //rcp r8.x, c3.x
		a.rcp(rtemp(8, "y"), rconst(3, "y")); //rcp r8.y, c3.y
		a.rcp(rtemp(8, "z"), rconst(3, "z")); //rcp r8.z, c3.z
		a.rcp(rtemp(8, "w"), rconst(3, "w")); //rcp r8.w, c3.w

		if ((i % 2) > 0)
		{
			a.mul(rtemp(4), rtemp(6), rtemp(8));
		}
		else
		{
			a.mul(rtemp(4), rtemp(3), rtemp(8));
		}



		a.rcp(rtemp(8), rtemp(2, "z")); //rcp r8, r2.z
		a.mul(rtemp(4), rtemp(4), rtemp(8)); //mul r4, r4, r8
		a.rcp(rtemp(8, "x"), rconst(1, "x")); //rcp r8.x, c1.x
		a.rcp(rtemp(8, "y"), rconst(1, "y")); //rcp r8.y, c1.y
		a.rcp(rtemp(8, "z"), rconst(1, "z")); //rcp r8.z, c1.z
		a.rcp(rtemp(8, "w"), rconst(1, "w")); //rcp r8.w, c1.w
		a.mul(rtemp(4), rtemp(4), rtemp(8)); //mul r4, r4, r8
		a.add(rtemp(4), rtexture(0), rtemp(4)); //add r4, t0, r4
		a.min(rtemp(5), rtemp(4), rconst(6)); //min r5, r4, c6
		samplers.add(0, SHADER_SAMPLER_DIM::_2D, SHADER_SAMPLER_REPEAT::CLAMP, SHADER_SAMPLER_FILTER::NEAREST, SHADER_SAMPLER_MIPMAP::NONE, SHADER_SAMPLER_TYPE::RGBA);
		a.texld(rtemp(5), rtemp(5), rsampler(0)); //texld r5, r5, s0
		a.dp3(rtemp(5, "z"), rtemp(5), rconst(0)); //dp3 r5.z, r5, c0
		a.mul(rtemp(5, "xy"), rtemp(4), rconst(1)); //mul r5.xy, r4, c1
		a.add(rtemp(5, "xy"), rtemp(5), N(rconst(2))); //add r5.xy, r5, -c2
		a.mul(rtemp(5, "xy"), rtemp(5), rtemp(5, "z")); //mul r5.xy, r5, r5.z
		a.add(rtemp(5), rtemp(5), N(rtemp(2))); //add r5, r5, -r2
		a.mul(rtemp(5), rtemp(5), rconst(3)); //mul r5, r5, c3
		a.dp3(rtemp(5, "w"), rtemp(5), rtemp(5)); //dp3 r5.w, r5, r5
		a.rcp(rtemp(9, "w"), rtemp(5, "w")); //rcp r9.w, r5.w
		a.rsq(rtemp(5, "w"), rtemp(9, "w")); //rsq r5.w, r9.w
		a.rcp(rtemp(8), rtemp(5, "w")); //rcp r8, r5.w
		a.mul(rtemp(5, "xyz"), rtemp(5), rtemp(8)); //mul r5.xyz, r5, r8
		a.mul(rtemp(5, "w"), rtemp(5), rconst(3)); //mul r5.w, r5, c3
		a.add(rtemp(5, "w"), rconst(1), N(rtemp(5))); //add r5.w, c1, -r5
		a.max(rtemp(5, "w"), rtemp(5), rconst(4)); //max r5.w, r5, c4
		a.dp3(rtemp(5, "z"), rtemp(5), rtemp(1)); //dp3 r5.z, r5, r1
		a.add(rtemp(5, "z"), rtemp(5), N(rconst(2))); //add r5.z, r5, -c2
		a.max(rtemp(5, "z"), rtemp(5), rconst(0)); //max r5.z, r5, c0
		a.mul(rtemp(5, "w"), rtemp(5), rtemp(5, "z")); //mul r5.w, r5, r5.z


		if (i == 0)
		{
			a.mov(rtemp(0, "w"), rtemp(5)); //mov r0.w, r5
		}
		else
		{
			a.add(rtemp(0, "w"), rtemp(0), rtemp(5)); //add r0.w, r0, r5
		}
	}

	a.mul(rtemp(0, "w"), rtemp(0), rconst(2)); //mul r0.w, r0, c2
	a.mov(rcolorout(0), rtemp(0)); //mov oC0, r0



	HLSL_ASM_CAPTION caption;


	caption.dcl(rtexture(0));
	caption.dcl(rtexture(1));
	caption.dcl(rtexture(2));

	caption.dcl_2d(rsampler(0));
	caption.dcl_2d(rsampler(1));

	caption.def(rconst(28), 0, 1, 0, 0);
	caption.def(rconst(29), -1.55009923e-006, -2.17013894e-005, 0.00260416674, 0.00026041668);
	caption.def(rconst(30), -0.020833334, -0.125, 1, 0.5);
	caption.def(rconst(31), 0.159154937, 0.5, 6.28318548, -3.14159274);





	QByteArray data = collect_pixel(caption, a);


	//const uint32_t *array = (uint32_t*)data.constData();
	//for (int i = 0; i < (data.size() / 4); i++)
		//qDebug("%08X,", array[i]);

	return data;
}


#if 0
			tex(ft0, v0, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
			mul(ft0.zw, ft0, fc[6]);
			cos(ft1, ft0);
			sin(ft2, ft0);
			mul(ft1.x, ft1.w, ft1.z);
			mul(ft1.y, ft2.w, ft1.z);
			neg(ft1.z, ft2);
			dp3(ft2.z, ft0, fc[0]);
			mul(ft2.xy, v2, ft2.z);
			tex(ft3, v1, fs1.dim(SamplerDim.D2).repeat(SamplerRepeat.WRAP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
			mul(ft3.z, ft3, fc[1]);
			mul(ft3, ft3.z, ft3);
			var _loc2_:int;
			while (_loc2_ < param1)
			{
				if (_loc2_ > 0)
				{
					if ((_loc2_ % 2) > 0)
					{
						dp3(ft6, ft3, fc[4]);
						dp3(ft6.y, ft3, fc[5]);
					}
					else
					{
						dp3(ft3.x, ft6, fc[4]);
						dp3(ft3.y, ft6, fc[5]);
					}
				}
				if ((_loc2_ % 2) > 0)
				{
					div(ft4, ft6, fc[3]);
				}
				else
				{
					div(ft4, ft3, fc[3]);
				}
				div(ft4, ft4, ft2.z);
				div(ft4, ft4, fc[1]);
				add(ft4, v0, ft4);
				min(ft5, ft4, fc[6]);
				tex(ft5, ft5, fs0.dim(SamplerDim.D2).repeat(SamplerRepeat.CLAMP).filter(SamplerFilter.NEAREST).mipmap(SamplerMipMap.NONE));
				dp3(ft5.z, ft5, fc[0]);
				mul(ft5.xy, ft4, fc[1]);
				sub(ft5.xy, ft5, fc[2]);
				mul(ft5.xy, ft5, ft5.z);
				sub(ft5, ft5, ft2);
				mul(ft5, ft5, fc[3]);
				dp3(ft5.w, ft5, ft5);
				sqt(ft5.w, ft5);
				div(ft5.xyz, ft5, ft5.w);
				mul(ft5.w, ft5, fc[3]);
				sub(ft5.w, fc[1], ft5);
				max(ft5.w, ft5, fc[4]);
				dp3(ft5.z, ft5, ft1);
				sub(ft5.z, ft5, fc[2]);
				max(ft5.z, ft5, fc[0]);
				mul(ft5.w, ft5, ft5.z);
				if (_loc2_ == 0)
				{
					mov(ft0.w, ft5);
				}
				else
				{
					add(ft0.w, ft0, ft5);
				}
				_loc2_++;
			}
			mul(ft0.w, ft0, fc[2]);
			mov(oc, ft0);


			ps_2_x
				  dcl t0
				  dcl t1
				  dcl t2
				  dcl_2d s0
				  dcl_2d s1
				  def c28, 0, 1, 0, 0
				  def c29, -1.55009923e-006, -2.17013894e-005, 0.00260416674, 0.00026041668
				  def c30, -0.020833334, -0.125, 1, 0.5
				  def c31, 0.159154937, 0.5, 6.28318548, -3.14159274
		   0:     texld r0, t0, s0
		   0:     mul r0.zw, r0, c6
		   1:     mov r8, r0
		   2:     mad r8, r8, c31.x, c31.y
		   3:     frc r8, r8
		   4:     mad r8, r8, c31.z, c31.w
		   5:     sincos r9.x, r8.x, c29, c30
		  13:     mov r1.x, r9.x
		  14:     sincos r9.x, r8.y, c29, c30
		  22:     mov r1.y, r9.x
		  23:     sincos r9.x, r8.z, c29, c30
		  31:     mov r1.z, r9.x
		  32:     sincos r9.x, r8.w, c29, c30
		  40:     mov r1.w, r9.x
		  41:     mov r8, r0
		  42:     mad r8, r8, c31.x, c31.y
		  43:     frc r8, r8
		  44:     mad r8, r8, c31.z, c31.w
		  45:     sincos r9.y, r8.x, c29, c30
		  53:     mov r2.x, r9.y
		  54:     sincos r9.y, r8.y, c29, c30
		  62:     mov r2.y, r9.y
		  63:     sincos r9.y, r8.z, c29, c30
		  71:     mov r2.z, r9.y
		  72:     sincos r9.y, r8.w, c29, c30
		  80:     mov r2.w, r9.y
		  81:     mul r1.x, r1.w, r1.z
		  82:     mul r1.y, r2.w, r1.z
		  83:     mov r1.z, -r2
		  84:     dp3 r2.z, r0, c0
		  85:     mul r2.xy, t2, r2.z
		  86:     texld r3, t1, s1
		  86:     mul r3.z, r3, c1
		  87:     mul r3, r3.z, r3

		  while 0 {
		  88:     rcp r8.x, c3.x
		  89:     rcp r8.y, c3.y
		  90:     rcp r8.z, c3.z
		  91:     rcp r8.w, c3.w
		  92:     mul r4, r3, r8

		  93:     rcp r8, r2.z
		  94:     mul r4, r4, r8
		  95:     rcp r8.x, c1.x
		  96:     rcp r8.y, c1.y
		  97:     rcp r8.z, c1.z
		  98:     rcp r8.w, c1.w
		  99:     mul r4, r4, r8
		 100:     add r4, t0, r4
		 101:     min r5, r4, c6
		 102:     texld r5, r5, s0
		 102:     dp3 r5.z, r5, c0
		 103:     mul r5.xy, r4, c1
		 104:     add r5.xy, r5, -c2
		 105:     mul r5.xy, r5, r5.z
		 106:     add r5, r5, -r2
		 107:     mul r5, r5, c3
		 108:     dp3 r5.w, r5, r5
		 109:     rcp r9.w, r5.w
		 110:     rsq r5.w, r9.w
		 111:     rcp r8, r5.w
		 112:     mul r5.xyz, r5, r8
		 113:     mul r5.w, r5, c3
		 114:     add r5.w, c1, -r5
		 115:     max r5.w, r5, c4
		 116:     dp3 r5.z, r5, r1
		 117:     add r5.z, r5, -c2
		 118:     max r5.z, r5, c0
		 119:     mul r5.w, r5, r5.z

		 120:     mov r0.w, r5
		 }

		 while 1 {
		 121:     dp3 r6, r3, c4
		 122:     dp3 r6.y, r3, c5
		 123:     rcp r8.x, c3.x
		 124:     rcp r8.y, c3.y
		 125:     rcp r8.z, c3.z
		 126:     rcp r8.w, c3.w
		 127:     mul r4, r6, r8
		 128:     rcp r8, r2.z
		 129:     mul r4, r4, r8
		 130:     rcp r8.x, c1.x
		 131:     rcp r8.y, c1.y
		 132:     rcp r8.z, c1.z
		 133:     rcp r8.w, c1.w
		 134:     mul r4, r4, r8
		 135:     add r4, t0, r4
		 136:     min r5, r4, c6
		 137:     texld r5, r5, s0
		 137:     dp3 r5.z, r5, c0
		 138:     mul r5.xy, r4, c1
		 139:     add r5.xy, r5, -c2
		 140:     mul r5.xy, r5, r5.z
		 141:     add r5, r5, -r2
		 142:     mul r5, r5, c3
		 143:     dp3 r5.w, r5, r5
		 144:     rcp r9.w, r5.w
		 145:     rsq r5.w, r9.w
		 146:     rcp r8, r5.w
		 147:     mul r5.xyz, r5, r8
		 148:     mul r5.w, r5, c3
		 149:     add r5.w, c1, -r5
		 150:     max r5.w, r5, c4
		 151:     dp3 r5.z, r5, r1
		 152:     add r5.z, r5, -c2
		 153:     max r5.z, r5, c0
		 154:     mul r5.w, r5, r5.z
		 155:     add r0.w, r0, r5
		 }

		 while 2 {
		 156:     dp3 r3.x, r6, c4
		 157:     dp3 r3.y, r6, c5
		 158:     rcp r8.x, c3.x
		 159:     rcp r8.y, c3.y
		 160:     rcp r8.z, c3.z
		 161:     rcp r8.w, c3.w
		 162:     mul r4, r3, r8
		 163:     rcp r8, r2.z
		 164:     mul r4, r4, r8
		 165:     rcp r8.x, c1.x
		 166:     rcp r8.y, c1.y
		 167:     rcp r8.z, c1.z
		 168:     rcp r8.w, c1.w
		 169:     mul r4, r4, r8
		 170:     add r4, t0, r4
		 171:     min r5, r4, c6
		 172:     texld r5, r5, s0
		 172:     dp3 r5.z, r5, c0
		 173:     mul r5.xy, r4, c1
		 174:     add r5.xy, r5, -c2
		 175:     mul r5.xy, r5, r5.z
		 176:     add r5, r5, -r2
		 177:     mul r5, r5, c3
		 178:     dp3 r5.w, r5, r5
		 179:     rcp r9.w, r5.w
		 180:     rsq r5.w, r9.w
		 181:     rcp r8, r5.w
		 182:     mul r5.xyz, r5, r8
		 183:     mul r5.w, r5, c3
		 184:     add r5.w, c1, -r5
		 185:     max r5.w, r5, c4
		 186:     dp3 r5.z, r5, r1
		 187:     add r5.z, r5, -c2
		 188:     max r5.z, r5, c0
		 189:     mul r5.w, r5, r5.z
		 190:     add r0.w, r0, r5
		 }

		 while 3 {
		 191:     dp3 r6, r3, c4
		 192:     dp3 r6.y, r3, c5
		 193:     rcp r8.x, c3.x
		 194:     rcp r8.y, c3.y
		 195:     rcp r8.z, c3.z
		 196:     rcp r8.w, c3.w
		 197:     mul r4, r6, r8
		 198:     rcp r8, r2.z
		 199:     mul r4, r4, r8
		 200:     rcp r8.x, c1.x
		 201:     rcp r8.y, c1.y
		 202:     rcp r8.z, c1.z
		 203:     rcp r8.w, c1.w
		 204:     mul r4, r4, r8
		 205:     add r4, t0, r4
		 206:     min r5, r4, c6
		 207:     texld r5, r5, s0
		 207:     dp3 r5.z, r5, c0
		 208:     mul r5.xy, r4, c1
		 209:     add r5.xy, r5, -c2
		 210:     mul r5.xy, r5, r5.z
		 211:     add r5, r5, -r2
		 212:     mul r5, r5, c3
		 213:     dp3 r5.w, r5, r5
		 214:     rcp r9.w, r5.w
		 215:     rsq r5.w, r9.w
		 216:     rcp r8, r5.w
		 217:     mul r5.xyz, r5, r8
		 218:     mul r5.w, r5, c3
		 219:     add r5.w, c1, -r5
		 220:     max r5.w, r5, c4
		 221:     dp3 r5.z, r5, r1
		 222:     add r5.z, r5, -c2
		 223:     max r5.z, r5, c0
		 224:     mul r5.w, r5, r5.z
		 225:     add r0.w, r0, r5
		 }

		 while 4 {
		 226:     dp3 r3.x, r6, c4
		 227:     dp3 r3.y, r6, c5
		 228:     rcp r8.x, c3.x
		 229:     rcp r8.y, c3.y
		 230:     rcp r8.z, c3.z
		 231:     rcp r8.w, c3.w
		 232:     mul r4, r3, r8
		 233:     rcp r8, r2.z
		 234:     mul r4, r4, r8
		 235:     rcp r8.x, c1.x
		 236:     rcp r8.y, c1.y
		 237:     rcp r8.z, c1.z
		 238:     rcp r8.w, c1.w
		 239:     mul r4, r4, r8
		 240:     add r4, t0, r4
		 241:     min r5, r4, c6
		 242:     texld r5, r5, s0
		 242:     dp3 r5.z, r5, c0
		 243:     mul r5.xy, r4, c1
		 244:     add r5.xy, r5, -c2
		 245:     mul r5.xy, r5, r5.z
		 246:     add r5, r5, -r2
		 247:     mul r5, r5, c3
		 248:     dp3 r5.w, r5, r5
		 249:     rcp r9.w, r5.w
		 250:     rsq r5.w, r9.w
		 251:     rcp r8, r5.w
		 252:     mul r5.xyz, r5, r8
		 253:     mul r5.w, r5, c3
		 254:     add r5.w, c1, -r5
		 255:     max r5.w, r5, c4
		 256:     dp3 r5.z, r5, r1
		 257:     add r5.z, r5, -c2
		 258:     max r5.z, r5, c0
		 259:     mul r5.w, r5, r5.z
		 260:     add r0.w, r0, r5
		 }

		 while 5 {
		 261:     dp3 r6, r3, c4
		 262:     dp3 r6.y, r3, c5
		 263:     rcp r8.x, c3.x
		 264:     rcp r8.y, c3.y
		 265:     rcp r8.z, c3.z
		 266:     rcp r8.w, c3.w
		 267:     mul r4, r6, r8
		 268:     rcp r8, r2.z
		 269:     mul r4, r4, r8
		 270:     rcp r8.x, c1.x
		 271:     rcp r8.y, c1.y
		 272:     rcp r8.z, c1.z
		 273:     rcp r8.w, c1.w
		 274:     mul r4, r4, r8
		 275:     add r4, t0, r4
		 276:     min r5, r4, c6
		 277:     texld r5, r5, s0
		 277:     dp3 r5.z, r5, c0
		 278:     mul r5.xy, r4, c1
		 279:     add r5.xy, r5, -c2
		 280:     mul r5.xy, r5, r5.z
		 281:     add r5, r5, -r2
		 282:     mul r5, r5, c3
		 283:     dp3 r5.w, r5, r5
		 284:     rcp r9.w, r5.w
		 285:     rsq r5.w, r9.w
		 286:     rcp r8, r5.w
		 287:     mul r5.xyz, r5, r8
		 288:     mul r5.w, r5, c3
		 289:     add r5.w, c1, -r5
		 290:     max r5.w, r5, c4
		 291:     dp3 r5.z, r5, r1
		 292:     add r5.z, r5, -c2
		 293:     max r5.z, r5, c0
		 294:     mul r5.w, r5, r5.z
		 295:     add r0.w, r0, r5
		 }
		 296:     mul r0.w, r0, c2
		 297:     mov oC0, r0
#endif
