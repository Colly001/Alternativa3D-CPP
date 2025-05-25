#include "program3d.h"
//#include "agal_to_hlsl.h"
#include <d3dx9core.h>
#include "hash.h"
//#include "alternativa/gfx/agal/agalparser.h"
#include <QByteArray>
#include <QDebug>

/*
static uint32_t CodeGetLength(const DWORD *byte_code)
{
	for (int i = 0;; i++) {
		if (byte_code[i] == 0x0000FFFF)
			return i+1;
	}
}


static const char * get(const DWORD *code)
{
	QByteArray hash = Hash::md5(QByteArray((const char*)code, CodeGetLength(code) * 4));
	char *str = new char [33];
	memcpy(str, hash.constData(), hash.size());
	str[32] = '\0';
	return str;
}
*/


Program3D::Program3D(IDirect3DDevice9Ex *device)
{
	this->device = device;
	this->pixel = nullptr;
	this->vertex = nullptr;
	this->pixelSamplerReg = nullptr;
	this->vertexSamplerReg = nullptr;
}

void Program3D::dispose()
{
	if (pixel != nullptr) {
		pixel->Release();
		pixel = nullptr;
	}

	if (vertex != nullptr) {
		vertex->Release();
		vertex = nullptr;
	}
}

#if 0
bool Program3D::upload(const QByteArray &vertexProgram, const QByteArray &fragmentProgram, const ShaderSampler &sampler)
{
	dispose();

	HRESULT r;

	const DWORD *pixel_code = AGAL_TO_HLSL(fragmentProgram);
	const DWORD *vertex_code = AGAL_TO_HLSL(vertexProgram);



	if (pixel_code == nullptr) {
		qDebug() << "error Program3D::upload pixel" << fragmentProgram.toHex(' ');
		return false;
	}

	if (vertex_code == nullptr) {
		qDebug() << "error Program3D::upload vertex" << vertexProgram.toHex(' ');
		return false;
	}

	hash_pixel = get(pixel_code);
	hash_vertex = get(vertex_code);


	AgalParser parserPixel, parserVertex;

	if (parserPixel.parser(fragmentProgram)) {
		this->pixelSamplerReg = parserPixel.samplerReg;
	}

	if (parserVertex.parser(vertexProgram)) {
		this->vertexSamplerReg = parserVertex.samplerReg;
	}

	this->sampler = sampler;


	r = device->CreatePixelShader(pixel_code, &pixel);
	if (FAILED(r)) {
		qDebug("error CreatePixelShader %08lX", r);
		return false;
	}

	r = device->CreateVertexShader(vertex_code, &vertex);
	if (FAILED(r)) {
		qDebug("error CreateVertexShader %08lX", r);
		return false;
	}

	return true;
}
#endif

bool Program3D::upload_hlsl(const uint32_t *vertexCode, const uint32_t *pixelCode, const ShaderSampler &sampler)
{
	dispose();

	HRESULT r;

	r = device->CreateVertexShader((const DWORD*)vertexCode, &vertex);
	if (FAILED(r)) {
		qDebug("error CreateVertexShader %08lX", r);
		return false;
	}

	r = device->CreatePixelShader((const DWORD*)pixelCode, &pixel);
	if (FAILED(r)) {
		qDebug("error CreatePixelShader %08lX", r);
		return false;
	}

	this->sampler = sampler;

	return true;
}
