#include "shadowvolumefragmentshaderhlsl.h"
#include "alternativa/gfx/hlslasm/hlslasm.h"


QByteArray ShadowVolumeFragmentShaderHLSL()
{
	using namespace HLSL_ASSEMBLY_PIXEL;

	HLSL_ASM_PIXEL a;
	a.mov(rcolorout(0), rconst(13)); //mov oC0, c13

	HLSL_ASM_CAPTION caption;
	QByteArray data = collect_pixel(caption, a);
	return data;
}


//ps_2_x
//mov oC0, c13
