#ifndef MATERIALFRAGMENTSHADERHLSL_H
#define MATERIALFRAGMENTSHADERHLSL_H

namespace HLSL_ASSEMBLY
{
	class HLSL_ASM_PIXEL;
}

class ShaderSampler;

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
								bool p_s);

#endif // MATERIALFRAGMENTSHADERHLSL_H
