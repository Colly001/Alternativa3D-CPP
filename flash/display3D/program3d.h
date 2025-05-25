#ifndef PROGRAM3D_H
#define PROGRAM3D_H

#include <stdint.h>
#include "alternativa/gfx/agal/agalparser.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"

class QByteArray;
class IDirect3DDevice9Ex;
class IDirect3DPixelShader9;
class IDirect3DVertexShader9;


class Program3D
{
	IDirect3DDevice9Ex *device;
	IDirect3DPixelShader9 *pixel;
	IDirect3DVertexShader9 *vertex;

public:
	Program3D(IDirect3DDevice9Ex *device);

	IDirect3DPixelShader9 *getPixel() { return pixel; }
	IDirect3DVertexShader9 *getVertex() { return vertex; }

	//const char *hash_pixel;
	//const char *hash_vertex;

	AgalParser::SamplerReg *pixelSamplerReg;
	AgalParser::SamplerReg *vertexSamplerReg;

	ShaderSampler sampler;



	//Освобождает все ресурсы, связанные с этим объектом.
	void dispose();

	//Загружает пару программ визуализации, выраженных байт-кодом AGAL (Adobe Graphics Assembly Language).
	//bool upload(const QByteArray &vertexProgram, const QByteArray &fragmentProgram, const ShaderSampler &sampler);
	bool upload_hlsl(const uint32_t *vertexCode, const uint32_t *pixelCode, const ShaderSampler &sampler);
};

#endif // PROGRAM3D_H
