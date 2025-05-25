#include "shadowcasterfragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"

QByteArray ShadowCasterFragmentShaderHLSL()
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;

	a.mov(rcolorout(0), rtexture(0)); //mov oC0, t0

	HLSL_ASM_CAPTION caption;
	caption.dcl(rtexture(0));

	QByteArray data = collect_pixel(caption, a);
	return data;
}


//ps_2_x
//dcl t0
//mov oC0, t0
