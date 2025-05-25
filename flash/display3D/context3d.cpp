#include "context3d.h"
#include <d3dx9core.h>
#include "program3d.h"
#include "indexbuffer3d.h"
#include "vertexbuffer3d.h"
#include "textures/texturebase.h"
#include "textures/texture.h"
#include "mymath.h"
#include "flash/display/bitmapdata.h"
#include "file.h"
#include "directxdata.h"
#include "alternativa/gfx/agal/agalparser.h"
#include "flash/display/imagenodes.h"
#include "flash/display/imageitem.h"
#include "Context3DClearMask.h"
#include <QImage>
#include <QDebug>

static float temp_float[500];


/*
	+get  enableErrorChecking
	+set  enableErrorChecking
	+configureBackBuffer
	+clear
	+drawTriangles
	+present
	+setProgram
	+setProgramConstantsFromVector
	-setProgramConstantsFromMatrix
	-setProgramConstantsFromByteArray
	+setVertexBufferAt
	+setBlendFactors
	+setColorMask
	+setDepthTest
	+setTextureAt
	-setRenderToTexture
	-setRenderToBackBuffer
	-setRenderToTextureInternal
	+setCulling
	+setStencilActions
	+setStencilReferenceValue
	+setScissorRectangle
	+createVertexBuffer
	+createIndexBuffer
	+createTexture
	-createCubeTexture
	-createRectangleTexture
	+createProgram
	-setSamplerStateAt
	-get  profile
	+setTextureInternal
	-setCubeTextureInternal
	-setRectangleTextureInternal
	-setVideoTextureInternal
	-get  backBufferWidth
	-get  backBufferHeight
	-get  maxBackBufferWidth
	-set  maxBackBufferWidth
	-get  maxBackBufferHeight
	-set  maxBackBufferHeight
	-createVideoTexture
	-drawToBitmapData
*/

/*
static QString toString(const float *array, int count)
{
	QString s;
	s.append("{\n");
	for (UINT i = 0; i < count; i++) {
		s.append(QString::number(array[0]));
		s.append(", ");
		s.append(QString::number(array[1]));
		s.append(", ");
		s.append(QString::number(array[2]));
		s.append(", ");
		s.append(QString::number(array[3]));
		if (count-1 != i)
			s.append(",");
		s.append("\n");
		array += 4;
	}

	s.append("}");
	return s;
}
*/

namespace
{
	DWORD AS3_TO_DIRECTX(Context3DBlendFactor s)
	{
		switch (s)
		{
			case Context3DBlendFactor::DESTINATION_ALPHA:
				return D3DBLEND_DESTALPHA;

			case Context3DBlendFactor::DESTINATION_COLOR:
				return D3DBLEND_DESTCOLOR;

			case Context3DBlendFactor::ONE:
				return D3DBLEND_ONE;

			case Context3DBlendFactor::ONE_MINUS_DESTINATION_ALPHA:
				return D3DBLEND_INVDESTALPHA;

			case Context3DBlendFactor::ONE_MINUS_DESTINATION_COLOR:
				return D3DBLEND_INVDESTCOLOR;

			case Context3DBlendFactor::ONE_MINUS_SOURCE_ALPHA:
				return D3DBLEND_INVSRCALPHA;

			case Context3DBlendFactor::ONE_MINUS_SOURCE_COLOR:
				return D3DBLEND_INVSRCCOLOR;

			case Context3DBlendFactor::SOURCE_ALPHA:
				return D3DBLEND_SRCALPHA;


			case Context3DBlendFactor::SOURCE_COLOR:
				return D3DBLEND_SRCCOLOR;

			case Context3DBlendFactor::ZERO:
				return D3DBLEND_ZERO;

			default:
				return D3DBLEND_ONE;
		}
	}


	DWORD AS3_TO_DIRECTX(Context3DCompareMode s)
	{

		//D3DCMP_NEVER                = 1, NEVER -
		//D3DCMP_LESS                 = 2, LESS +
		//D3DCMP_EQUAL                = 3, EQUAL +
		//D3DCMP_LESSEQUAL            = 4, LESS_EQUAL +
		//D3DCMP_GREATER              = 5, GREATER -
		//D3DCMP_NOTEQUAL             = 6, NOT_EQUAL -
		//D3DCMP_GREATEREQUAL         = 7, GREATER_EQUAL +
		//D3DCMP_ALWAYS               = 8, ALWAYS +

		switch (s)
		{
			case Context3DCompareMode::ALWAYS:
				return D3DCMP_ALWAYS;

			case Context3DCompareMode::EQUAL:
				return D3DCMP_EQUAL;

			case Context3DCompareMode::GREATER:
				return D3DCMP_GREATER;

			case Context3DCompareMode::GREATER_EQUAL:
				return D3DCMP_GREATEREQUAL;

			case Context3DCompareMode::LESS:
				return D3DCMP_LESS;

			case Context3DCompareMode::LESS_EQUAL:
				return D3DCMP_LESSEQUAL;

			case Context3DCompareMode::NEVER:
				return D3DCMP_NEVER;

			case Context3DCompareMode::NOT_EQUAL:
				return D3DCMP_NOTEQUAL;

			default:
				return D3DCMP_NEVER;
		}
	}

	struct VERTEX_ELEMENT
	{
		union
		{
			struct
			{
				uint8_t Offset;
				uint8_t Type;
				uint8_t UsageIndex;
				uint8_t _none;
			};

			uint32_t data;
		};

		VERTEX_ELEMENT()
		{
			this->_none = 0;
		}
	};


	class DeclarationElementItem
	{
	public:
		DeclarationElementItem *next;
		DeclarationElementItem *child;

		IDirect3DVertexDeclaration9 *declaration;

		VERTEX_ELEMENT element;

		DeclarationElementItem()
		{
			this->next = nullptr;
			this->child = nullptr;
			this->declaration = nullptr;
		}

		~DeclarationElementItem()
		{
			free();
		}

		void free()
		{
			if (declaration != nullptr)
			{
				declaration->Release();
				declaration = nullptr;
			}
		}

		bool init(IDirect3DDevice9Ex *device, const VERTEX_ELEMENT *list)
		{
			D3DVERTEXELEMENT9 declaration_list[9];

			int i = 0;
			for (;;)
			{
				const auto &element_item = list[i];
				auto &declaration_item = declaration_list[i];

				if (element_item.Type == D3DDECLTYPE_UNUSED)
				{
					declaration_item = {255, 0, D3DDECLTYPE_UNUSED, 0, 0, 0};
					break;
				}

				declaration_item.Stream = 0;
				declaration_item.Offset = element_item.Offset * 4;
				declaration_item.Type = element_item.Type;
				declaration_item.Method = D3DDECLMETHOD_DEFAULT;
				declaration_item.Usage = D3DDECLUSAGE_TEXCOORD;
				declaration_item.UsageIndex = element_item.UsageIndex;

				//qDebug() << declaration_item.Method << declaration_item.Offset << declaration_item.Type << declaration_item.Method << declaration_item.Usage << declaration_item.UsageIndex;
				i++;
			}

			HRESULT result = device->CreateVertexDeclaration(declaration_list, &this->declaration);
			if (result != S_OK) {
				qDebug("error CreateVertexDeclaration");
				return false;
			}

			return true;
		}
	};



	class DeclarationEngine
	{
		DeclarationElementItem *first;
		IDirect3DDevice9Ex *device;


	public:
		DeclarationEngine(IDirect3DDevice9Ex *device)
		{
			this->device = device;
			first = nullptr;
		}


		void free()
		{
			freeItem(first);
			first = nullptr;
		}



	private:

		void freeItem(DeclarationElementItem *first)
		{
			DeclarationElementItem *item = first;
			while (item != nullptr)
			{
				freeItem(item->child);
				delete item;
				item = item->next;
			}
		}

		IDirect3DVertexDeclaration9* get(const VERTEX_ELEMENT *begin, int level, DeclarationElementItem *first)
		{
			auto item = first;

			const auto element = begin[level];
			const auto element_next = begin[level+1];

			uint32_t data = element.data;

			bool end = element_next.Type == D3DDECLTYPE_UNUSED;

			DeclarationElementItem *last = nullptr;

			while (item != nullptr)
			{
				if (item->element.data == data)
				{
					if (end == true)
					{
						if (item->declaration == nullptr)
						{
							if (item->init(device, begin))
							{
								return item->declaration;
							}
							else
							{
								qDebug("error DeclarationEngine 1");
								return nullptr;
							}
						}
						else
						{
							return item->declaration;
						}
					}
					else
					{
						if (item->child == nullptr)
						{
							item->child = new DeclarationElementItem();
							item->element.data = data;
							return get(begin, level+1, item->child);
						}
						else
						{
							return get(begin, level+1, item->child);
						}
					}
				}

				last = item;
				item = item->next;
			}


			item = new DeclarationElementItem();
			item->element.data = data;

			if (this->first == nullptr)
				this->first = item;

			if (last != nullptr)
				last->next = item;


			if (end == true)
			{
				if (item->init(device, begin))
				{

					return item->declaration;
				}
				else
				{
					qDebug("error DeclarationEngine 2");
					return nullptr;
				}
			}
			else
			{
				item->child = new DeclarationElementItem();
				item->element.data = data;
				return get(begin, level+1, item->child);
			}

		}

	public:
		IDirect3DVertexDeclaration9* get(const VERTEX_ELEMENT *list)
		{
			return get(list, 0, first);
		}
	};
}


class Context3DPrivate
{
	Context3D *d;
public:
	int NumVertices;

	VertexBuffer3D *vertex_buffer;
	int vertex_index;
	int vertex_bufferOffset;

	VERTEX_ELEMENT list_vertex_element[9];

	Program3D *program;
	DeclarationEngine *declarationEngine;


	IDirect3DPixelShader9 *shader_pixel_49ee6e61e4565256ff2ed18f53fac3ba;
	IDirect3DVertexBuffer9 *vertex_5;

	Context3DPrivate(Context3D *d)
	{
		this->d = d;
		this->declarationEngine = new DeclarationEngine(d->getDevice());
		this->program = nullptr;
		this->NumVertices = 0;

		for (int i = 0; i < 9; i++)
		{
			VERTEX_ELEMENT &e = list_vertex_element[i];
			//e.Stream = 255;
			e.Offset = 0;
			e.Type = D3DDECLTYPE_UNUSED;
			e.UsageIndex = 0;
		}
	}

	~Context3DPrivate()
	{
		delete this->declarationEngine;
	}
};



/*
ps_2_0
	  def c13, 1, 1, 1, 1
	  dcl t0.xy
	  dcl_2d s0
0:     texld r0, t0, s0
0:     mov r0.w, c13.w
1:     mov oC0, r0
*/

static const uint32_t data_pixel_shader_49ee6e61e4565256ff2ed18f53fac3ba[] =
{
	0xFFFF0200,
	0x05000051, 0xA00F000D, 0x3F800000, 0x3F800000, 0x3F800000, 0x3F800000,
	0x0200001F, 0x80000000, 0xB0030000,
	0x0200001F, 0x90000000, 0xA00F0800,
	0x03000042, 0x800F0000, 0xB0E40000, 0xA0E40800,
	0x02000001, 0x80080000, 0xA0FF000D,
	0x02000001, 0x800F0800, 0x80E40000,
	0x0000FFFF
};



static void SetSamplerState_1(IDirect3DDevice9Ex *device, int sampler_idx)
{
	device->SetSamplerState(sampler_idx, D3DSAMP_MAXMIPLEVEL, 0); //0 0D3DSAMP_MAXMIPLEVEL
	device->SetSamplerState(sampler_idx, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR); //5 2 D3DSAMP_MAGFILTER +
	device->SetSamplerState(sampler_idx, D3DSAMP_MINFILTER, D3DTEXF_LINEAR); //6 2 D3DSAMP_MINFILTER +
	device->SetSamplerState(sampler_idx, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR); //7 2 D3DSAMP_MIPFILTER
	device->SetSamplerState(sampler_idx, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP); //1 1 D3DSAMP_ADDRESSU
	device->SetSamplerState(sampler_idx, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP); //2 1 D3DSAMP_ADDRESSV
	device->SetSamplerState(sampler_idx, D3DSAMP_MIPMAPLODBIAS, 0); //8 0 D3DSAMP_MIPMAPLODBIAS
	device->SetSamplerState(sampler_idx, D3DSAMP_SRGBTEXTURE, 0); //11 0 D3DSAMP_SRGBTEXTURE
}

static void SetTextureStageState_1(IDirect3DDevice9Ex *device, int stage)
{
	device->SetTextureStageState(stage, D3DTSS_TEXCOORDINDEX, stage); //11 stage
	device->SetTextureStageState(stage, D3DTSS_COLOROP, D3DTOP_DISABLE); //1 1
}

static void SetTextureStageState_2(IDirect3DDevice9Ex *device, int stage)
{
	device->SetTextureStageState(stage, D3DTSS_COLORARG1, 2); //2 2
	device->SetTextureStageState(stage, D3DTSS_COLOROP, 2); //1 2
	device->SetTextureStageState(stage, D3DTSS_ALPHAARG1, 2); //5 2
	device->SetTextureStageState(stage, D3DTSS_ALPHAOP, 2); //4 2
	device->SetTextureStageState(stage, D3DTSS_TEXCOORDINDEX, stage); //11 0
	device->SetTextureStageState(stage, D3DTSS_COLOROP, 1); //1 1
	device->SetTextureStageState(stage, D3DTSS_ALPHAOP, 1); //4 1
}


static void SetViewport(IDirect3DDevice9Ex *device, DWORD X, DWORD Y, DWORD Width, DWORD Height, float MinZ, float MaxZ)
{
	const D3DVIEWPORT9 viewport = {X, Y, Width, Height, MinZ, MaxZ};
	HRESULT r = device->SetViewport(&viewport);
	if (r != S_OK)
	{
		qDebug() << "error SetViewport" << r;
	}
}





Context3D::Context3D(DirectXData *data)
{
	this->directXData = data;
	this->device = data->device;

	d = new Context3DPrivate(this);
	this->_width = 100;
	this->_height = 100;
	_size[0] = _size[1] = _size[2] = _size[3] = 0;

	this->_sourceFactor = AS3_TO_DIRECTX(Context3DBlendFactor::ONE);
	this->_destinationFactor = AS3_TO_DIRECTX(Context3DBlendFactor::ZERO);
	this->_zfunc = AS3_TO_DIRECTX(Context3DCompareMode::LESS);

	this->_triangleFaceToCull = D3DCULL_NONE;
	this->changed_size = true;
	this->_depthMask = false;


	{
		HRESULT r = device->CreatePixelShader((DWORD*)data_pixel_shader_49ee6e61e4565256ff2ed18f53fac3ba, &d->shader_pixel_49ee6e61e4565256ff2ed18f53fac3ba);
		if (FAILED(r)) {
			qDebug("error CreatePixelShader 2 %08lX", r);
			return;
		}
	}

	{
		uint32_t size = 144;
		uint32_t usage = 520;
		uint32_t fvf = 580;
		D3DPOOL pool = D3DPOOL_DEFAULT;

		HRESULT r = device->CreateVertexBuffer(size, usage, fvf, pool, &d->vertex_5, nullptr);
		if (r != S_OK) {
			qDebug() << "error CreateVertexBuffer" << r;
			return;
		}
	}


	{
		const float data [4] = {65536, 1, 1, -1};
		setProgramConstantsVertex(3, data, 1);
	}


	device->BeginScene();

	device->SetRenderTarget(0, directXData->pBackBuffer);
}



void Context3D::clear(double red, double green, double blue, double alpha, double depth, double stencil, uint32_t mask)
{
	//Context3DClearMask::STENCIL

	(void)depth;
	(void)stencil;
	(void)mask;



	uint32_t color = uint8_t(int32_t(blue));
	color |=  (uint8_t(int32_t(green)) * 0xFF) << 8;
	color |=  (uint8_t(int32_t(red)) * 0xFF) << 16;
	color |=  (uint8_t(int32_t(alpha)) * 0xFF) << 24;


	DWORD flags = 0;

	if ((mask & (uint32_t)Context3DClearMask::STENCIL) != 0)
	{
		flags |= D3DCLEAR_STENCIL;
	}

	if ((mask & (uint32_t)Context3DClearMask::COLOR) != 0)
	{
		flags |= D3DCLEAR_TARGET;
	}

	if ((mask & (uint32_t)Context3DClearMask::DEPTH) != 0)
	{
		flags |= D3DCLEAR_ZBUFFER;
	}


	//COLOR = 1, //Очищает только буфер цвета. D3DCLEAR_TARGET
	//DEPTH = 2, //Очищает только буфер глубины. D3DCLEAR_ZBUFFER
	//STENCIL = 4,//Очищает только буфер трафарета. D3DCLEAR_STENCIL
	//ALL = 7, //Очистка всех буферов.

	//1 D3DCLEAR_STENCIL
	//2 D3DCLEAR_TARGET
	//4 D3DCLEAR_ZBUFFER

	//DWORD rect_count;
	//const D3DRECT *rects;
	//DWORD flags;
	//D3DCOLOR colour;
	//float z;
	//DWORD stencil;

	device->Clear(0, nullptr, flags, color, 1, stencil);
}

void Context3D::configureBackBuffer(int width, int height, int antiAlias, bool enableDepthAndStencil, bool wantsBestResolution,
									bool wantsBestResolutionOnBrowserZoom)
{
	//qDebug() << "Context3D::configureBackBuffer" << width << height;
	(void)antiAlias;
	(void)wantsBestResolution;
	(void)wantsBestResolutionOnBrowserZoom;

	this->_depthMask = enableDepthAndStencil; //D3DRS_ZWRITEENABLE
	this->_depthMaskBack = this->_depthMask;

	this->_width = width;
	this->_height = height;
	_size[0] = 1. / this->_width;
	_size[1] = 1. / this->_height;
	this->changed_size = true;
}




std::shared_ptr<Program3D> Context3D::createProgram()
{
	return std::make_shared<Program3D>(this->device);
}


//String bufferUsage = "staticDraw"
std::shared_ptr<IndexBuffer3D> Context3D::createIndexBuffer(int numIndices)
{
	IDirect3DIndexBuffer9 *pIndexBuffer;

	HRESULT r = device->CreateIndexBuffer(numIndices * sizeof(uint16_t), 8, D3DFMT_INDEX16, D3DPOOL_DEFAULT, &pIndexBuffer, nullptr);
	if (r != S_OK) {
		qDebug() << "error CreateIndexBuffer" << r;
		return nullptr;
	}

	return std::make_shared<IndexBuffer3D>(this->device, pIndexBuffer);
}


//String bufferUsage = "staticDraw"
std::shared_ptr<VertexBuffer3D> Context3D::createVertexBuffer(int numVertices, int data32PerVertex)
{
	DWORD usage = D3DUSAGE_WRITEONLY;
	UINT size = numVertices * data32PerVertex * sizeof(float);
	DWORD fvf = 0;
	IDirect3DVertexBuffer9 *pVertexBuffer = nullptr;

	HRESULT r = device->CreateVertexBuffer(size, usage, fvf, D3DPOOL_DEFAULT, &pVertexBuffer, nullptr);
	if (r != S_OK) {
		qDebug() << "error CreateVertexBuffer" << r;
		return nullptr;
	}

	return std::make_shared<VertexBuffer3D>(this->device, pVertexBuffer, numVertices, data32PerVertex);
}

void Context3D::drawTriangles(IndexBuffer3D *indexBuffer, int firstIndex, int numTriangles)
{
	IDirect3DDevice9Ex *device = this->device;

	Program3D *program = d->program;

	if (program != nullptr)
	{
		//qDebug() << "program hash" << program->hash_pixel << program->hash_vertex;
		device->SetPixelShader(program->getPixel());
		device->SetVertexShader(program->getVertex());
	}
	else
	{
		device->SetPixelShader(nullptr);
		device->SetVertexShader(nullptr);
	}

	device->SetVertexShaderConstantF(254, _size, 1);


	//5
	//device->SetRenderState(D3DRS_SRCBLEND, 5);
	//device->SetRenderState(D3DRS_DESTBLEND, 6);

	device->SetRenderState(D3DRS_SRCBLEND, this->_sourceFactor); //19
	device->SetRenderState(D3DRS_DESTBLEND, this->_destinationFactor); //20

	//device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE); //27
	device->SetRenderState(D3DRS_ALPHABLENDENABLE, this->_sourceFactor != D3DBLEND_ONE); //27

	//device->SetRenderState(D3DRS_CULLMODE, _triangleFaceToCull); //22
	//device->SetRenderState(D3DRS_CULLMODE, D3DCULL_CW); //22  D3DCULL_CW 2


	device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE); //7 1
	device->SetRenderState(D3DRS_ZWRITEENABLE, this->_depthMask);

	//device->SetRenderState(D3DRS_STENCILENABLE, FALSE); //52 0
	//device->SetRenderState(D3DRS_STENCILWRITEMASK, 0); //59 0
	//device->SetRenderState(D3DRS_STENCILMASK, 0); //58 0
	//device->SetRenderState(D3DRS_COLORWRITEENABLE, 15); //168 15

	device->SetRenderState(D3DRS_ZFUNC, this->_zfunc); //D3DCMP_LESS
	device->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
	device->SetRenderState(D3DRS_LIGHTING, TRUE);
	device->SetRenderState(D3DRS_SHADEMODE, D3DSHADE_GOURAUD);
	device->SetRenderState(D3DRS_DITHERENABLE, TRUE);
	device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
	device->SetRenderState(D3DRS_CLIPPING, true);



	/*
	for (int i = 0; i < 8; i++)
	{
		device->SetSamplerState(i, D3DSAMP_ADDRESSW, 1);
		device->SetSamplerState(i, D3DSAMP_BORDERCOLOR, 0);

		device->SetSamplerState(i, D3DSAMP_MAGFILTER, 2);
		device->SetSamplerState(i, D3DSAMP_MINFILTER, 2);


		device->SetSamplerState(i, D3DSAMP_MIPFILTER, 2);
		device->SetSamplerState(i, D3DSAMP_MIPMAPLODBIAS, 0);
		device->SetSamplerState(i, D3DSAMP_MAXMIPLEVEL, 0);

		device->SetSamplerState(i, D3DSAMP_MAXANISOTROPY, 1);
		device->SetSamplerState(i, D3DSAMP_SRGBTEXTURE, 0);
		device->SetSamplerState(i, D3DSAMP_ELEMENTINDEX, 0);
		device->SetSamplerState(i, D3DSAMP_DMAPOFFSET, 0);
	}



	device->SetSamplerState(0, D3DSAMP_FORCE_DWORD, 3131951870); //3131951870

	for (int i = 1; i < 8; i++)
	{
		device->SetSamplerState(i, D3DSAMP_FORCE_DWORD, 0);
	}
	*/




	//device->SetSamplerState(0, D3DSAMP_ADDRESSU, 1); //+
	//device->SetSamplerState(0, D3DSAMP_ADDRESSV, 1);


	/*
	device->SetSamplerState(1, D3DSAMP_ADDRESSU, 3);
	device->SetSamplerState(1, D3DSAMP_ADDRESSV, 3);

	device->SetSamplerState(2, D3DSAMP_ADDRESSU, 3);
	device->SetSamplerState(2, D3DSAMP_ADDRESSV, 3);

	device->SetSamplerState(3, D3DSAMP_ADDRESSU, 3);
	device->SetSamplerState(3, D3DSAMP_ADDRESSV, 3);
	*/


	if (program != nullptr)
	{

#if 1

		int size = program->sampler.list.size();
		for (int i = 0; i < size; i++)
		{
			const ShaderSampler::Item &sampler = program->sampler.list.data()[i];

			int n = sampler.index;

			//int m; // Mipmap (0=disable,1=nearest, 2=linear)
			if (sampler.mipmap == SHADER_SAMPLER_MIPMAP::NONE) {
				device->SetSamplerState(n, D3DSAMP_MIPFILTER, D3DTEXF_NONE); //7 0
			} else if (sampler.mipmap == SHADER_SAMPLER_MIPMAP::LINEAR) {
				device->SetSamplerState(n, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR); //7 2
			} else if (sampler.mipmap == SHADER_SAMPLER_MIPMAP::NEAREST) {
				qDebug("sampler->m == 1");
			}

			//int w; // wrap (0=clamp 1=repeat)

			if (sampler.repeat == SHADER_SAMPLER_REPEAT::CLAMP) {
				device->SetSamplerState(n, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP); //1 3
				device->SetSamplerState(n, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP); //2 3
			} else if (sampler.repeat == SHADER_SAMPLER_REPEAT::WRAP) {
				device->SetSamplerState(n, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP); //1 1
				device->SetSamplerState(n, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP); //2 1
			}

			/*
			if (sampler->w == 0)
			{
				device->SetSamplerState(n, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP); //1 3
				device->SetSamplerState(n, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP); //2 3
			}

			if (sampler->w == 1)
			{
				device->SetSamplerState(n, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP); //1 1
				device->SetSamplerState(n, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP); //2 1
			}
			*/

			device->SetSamplerState(n, D3DSAMP_MAXMIPLEVEL, 0); //9 0
			device->SetSamplerState(n, D3DSAMP_MAGFILTER, 2); //5 2
			device->SetSamplerState(n, D3DSAMP_MINFILTER, 2); //6 2
			device->SetSamplerState(n, D3DSAMP_MIPMAPLODBIAS, 0); //8 0
			device->SetSamplerState(n, D3DSAMP_SRGBTEXTURE, 0); //11 0
			device->SetSamplerState(n, D3DSAMP_MAXANISOTROPY, 0); //9 0


			//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 9 0
			//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 5 2
			//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 6 2
			//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 8 0
			//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 11 0
		}

#else
		AgalParser::SamplerReg *sampler;
		//qDebug() << d->NumVertices << numTriangles;

		sampler = program->pixelSamplerReg;
		while (sampler != nullptr)
		{
			//qDebug() << "SamplerPixel" << sampler->f << sampler->m << sampler->w << sampler->s << sampler->d << (int)sampler->type << sampler->b << sampler->n;


			if (sampler->type == AgalParser::Sampler)
			{
				int n = sampler->n;

				//int m; // Mipmap (0=disable,1=nearest, 2=linear)
				if (sampler->m == 0)
				{
					device->SetSamplerState(n, D3DSAMP_MIPFILTER, D3DTEXF_NONE); //7 0
				}

				if (sampler->m == 2)
				{
					device->SetSamplerState(n, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR); //7 2
				}

				if (sampler->m == 1)
				{
					qDebug("sampler->m == 1");
				}

				//int w; // wrap (0=clamp 1=repeat)

				if (sampler->w == 0)
				{
					device->SetSamplerState(n, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP); //1 3
					device->SetSamplerState(n, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP); //2 3
				}

				if (sampler->w == 1)
				{
					device->SetSamplerState(n, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP); //1 1
					device->SetSamplerState(n, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP); //2 1
				}

				device->SetSamplerState(n, D3DSAMP_MAXMIPLEVEL, 0); //9 0
				device->SetSamplerState(n, D3DSAMP_MAGFILTER, 2); //5 2
				device->SetSamplerState(n, D3DSAMP_MINFILTER, 2); //6 2
				device->SetSamplerState(n, D3DSAMP_MIPMAPLODBIAS, 0); //8 0
				device->SetSamplerState(n, D3DSAMP_SRGBTEXTURE, 0); //11 0
				device->SetSamplerState(n, D3DSAMP_MAXANISOTROPY, 0); //9 0


				//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 9 0
				//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 5 2
				//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 6 2
				//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 8 0
				//4528 MyIDirect3DDevice9_SetSamplerState 0x4eee640 0 11 0
			}

			sampler = sampler->next;
		}
#endif

		for (int i = 0; i < 8; i++)
		{
			device->SetSamplerState(i, D3DSAMP_MAXMIPLEVEL, 0); //9 0
			device->SetSamplerState(i, D3DSAMP_MAGFILTER, 2); //5 2
			device->SetSamplerState(i, D3DSAMP_MINFILTER, 2); //6 2
			device->SetSamplerState(i, D3DSAMP_MIPMAPLODBIAS, 0); //8 0
			device->SetSamplerState(i, D3DSAMP_SRGBTEXTURE, 0); //11 0
		}

#if 0

		device->SetTextureStageState(0, D3DTSS_ALPHAOP, 2);
		device->SetTextureStageState(1, D3DTSS_ALPHAOP, 1);
		device->SetTextureStageState(2, D3DTSS_ALPHAOP, 2);
		device->SetTextureStageState(3, D3DTSS_ALPHAOP, 2);
		device->SetTextureStageState(4, D3DTSS_ALPHAOP, 2);
		device->SetTextureStageState(5, D3DTSS_ALPHAOP, 2);
		device->SetTextureStageState(6, D3DTSS_ALPHAOP, 2);
		device->SetTextureStageState(7, D3DTSS_ALPHAOP, 1);


		for (int i = 0; i < 8; i++)
		{
			device->SetTextureStageState(i, D3DTSS_COLOROP, 1);
			device->SetTextureStageState(i, D3DTSS_COLORARG1, 2);
			device->SetTextureStageState(i, D3DTSS_COLORARG2, 1);

			device->SetTextureStageState(i, D3DTSS_ALPHAARG1, 2);
			device->SetTextureStageState(i, D3DTSS_ALPHAARG2, 1);
			device->SetTextureStageState(i, D3DTSS_BUMPENVMAT00, 0);
			device->SetTextureStageState(i, D3DTSS_BUMPENVMAT01, 0);
			device->SetTextureStageState(i, D3DTSS_BUMPENVMAT10, 0);
			device->SetTextureStageState(i, D3DTSS_BUMPENVMAT11, 0);
			device->SetTextureStageState(i, D3DTSS_TEXCOORDINDEX, i);
			device->SetTextureStageState(i, D3DTSS_BUMPENVLSCALE, 0);
			device->SetTextureStageState(i, D3DTSS_BUMPENVLOFFSET, 0);
			device->SetTextureStageState(i, D3DTSS_TEXTURETRANSFORMFLAGS, 0);
			device->SetTextureStageState(i, D3DTSS_COLORARG0, 1);
			device->SetTextureStageState(i, D3DTSS_ALPHAARG0, 1);
			device->SetTextureStageState(i, D3DTSS_RESULTARG, 1);
			device->SetTextureStageState(i, D3DTSS_CONSTANT, 0);
			device->SetTextureStageState(i, D3DTSS_FORCE_DWORD, 0);
		}




		device->SetRenderState(D3DRS_SRCBLEND, 5);
		device->SetRenderState(D3DRS_DESTBLEND, 6);
		device->SetRenderState(D3DRS_ALPHABLENDENABLE, 1);
		device->SetRenderState(D3DRS_CULLMODE, 2);
		device->SetRenderState(D3DRS_ZENABLE, 1);
		device->SetRenderState(D3DRS_ZWRITEENABLE, 0);
		device->SetRenderState(D3DRS_STENCILENABLE, 0);
		device->SetRenderState(D3DRS_STENCILWRITEMASK, 0);
		device->SetRenderState(D3DRS_STENCILMASK, 0);

		device->SetRenderState(D3DRS_COLORWRITEENABLE, 15);
		device->SetRenderState(D3DRS_ZFUNC, 2);
		device->SetRenderState(D3DRS_SCISSORTESTENABLE, 0);
		device->SetRenderState(D3DRS_LIGHTING, 0);
		device->SetRenderState(D3DRS_SHADEMODE, 2);
		device->SetRenderState(D3DRS_DITHERENABLE, 1);
		device->SetRenderState(D3DRS_ALPHATESTENABLE, 0);
		device->SetRenderState(D3DRS_CLIPPING, 1);




		device->SetSamplerState(0, D3DSAMP_FORCE_DWORD, 3131951870);

		for (int i = 1; i < 8; i++)
		{
			device->SetSamplerState(i, D3DSAMP_FORCE_DWORD, 0);
		}





		device->SetSamplerState(0, D3DSAMP_ADDRESSU, 1);
		device->SetSamplerState(0, D3DSAMP_ADDRESSV, 1);

		device->SetSamplerState(1, D3DSAMP_ADDRESSU, 3);
		device->SetSamplerState(1, D3DSAMP_ADDRESSV, 3);

		device->SetSamplerState(2, D3DSAMP_ADDRESSU, 3);
		device->SetSamplerState(2, D3DSAMP_ADDRESSV, 3);

		device->SetSamplerState(3, D3DSAMP_ADDRESSU, 3);
		device->SetSamplerState(3, D3DSAMP_ADDRESSV, 3);

		device->SetSamplerState(4, D3DSAMP_ADDRESSU, 1);
		device->SetSamplerState(4, D3DSAMP_ADDRESSV, 1);

		device->SetSamplerState(5, D3DSAMP_ADDRESSU, 3);
		device->SetSamplerState(5, D3DSAMP_ADDRESSV, 3);

		device->SetSamplerState(6, D3DSAMP_ADDRESSU, 1);
		device->SetSamplerState(6, D3DSAMP_ADDRESSV, 1);

		device->SetSamplerState(7, D3DSAMP_ADDRESSU, 3);
		device->SetSamplerState(7, D3DSAMP_ADDRESSV, 3);

		for (int i = 0; i < 8; i++)
		{
			device->SetSamplerState(i, D3DSAMP_ADDRESSW, 1);
			device->SetSamplerState(i, D3DSAMP_BORDERCOLOR, 0);
			device->SetSamplerState(i, D3DSAMP_MAGFILTER, 2);
			device->SetSamplerState(i, D3DSAMP_MINFILTER, 2);
			device->SetSamplerState(i, D3DSAMP_MIPFILTER, 2);
			device->SetSamplerState(i, D3DSAMP_MIPMAPLODBIAS, 0);
			device->SetSamplerState(i, D3DSAMP_MAXMIPLEVEL, 0);
			device->SetSamplerState(i, D3DSAMP_MAXANISOTROPY, 1);
			device->SetSamplerState(i, D3DSAMP_SRGBTEXTURE, 0);
			device->SetSamplerState(i, D3DSAMP_ELEMENTINDEX, 0);
			device->SetSamplerState(i, D3DSAMP_DMAPOFFSET, 0);
		}

#endif
	}


	//1 0 0 0 0 5 0 0


	//93 60
	//SamplerPixel 1 2 1 0 0 5 0 4
	//SamplerPixel 1 2 1 0 0 5 0 6
	//SamplerPixel 1 2 1 0 0 5 0 0

	//D3DTADDRESS_WRAP            = 1,
	//D3DTADDRESS_MIRROR          = 2,
	//D3DTADDRESS_CLAMP           = 3,
	//D3DTADDRESS_BORDER          = 4,
	//D3DTADDRESS_MIRRORONCE      = 5,
	//D3DTADDRESS_FORCE_DWORD     = 0x7fffffff


	/*
	device->SetSamplerState(4, D3DSAMP_ADDRESSU, 1);
	device->SetSamplerState(4, D3DSAMP_ADDRESSV, 1);

	device->SetSamplerState(5, D3DSAMP_ADDRESSU, 3);
	device->SetSamplerState(5, D3DSAMP_ADDRESSV, 3);

	device->SetSamplerState(6, D3DSAMP_ADDRESSU, 1);
	device->SetSamplerState(6, D3DSAMP_ADDRESSV, 1);

	device->SetSamplerState(7, D3DSAMP_ADDRESSU, 3);
	device->SetSamplerState(7, D3DSAMP_ADDRESSV, 3);
	*/


	//device->SetRenderState((D3DRENDERSTATETYPE)27, 0); //D3DRS_ALPHABLENDENABLE   TRUE, чтобы включить прозрачность с альфа-смешиванием. Значение по умолчанию FALSE.
	//device->SetRenderState((D3DRENDERSTATETYPE)22, 2); //D3DRS_CULLMODE    D3DCULL_CW
	//device->SetRenderState((D3DRENDERSTATETYPE)23, 2); //D3DRS_ZFUNC  D3DCMP_LESS
	//device->SetRenderState((D3DRENDERSTATETYPE)7, 1); //D3DRS_ZENABLE    D3DZBUFFERTYPE  D3DZB_TRUE
	//device->SetRenderState((D3DRENDERSTATETYPE)14, 1); //D3DRS_ZWRITEENABLE   TRUE
	//device->SetRenderState((D3DRENDERSTATETYPE)52, 0); //D3DRS_STENCILENABLE  FALSE
	//device->SetRenderState((D3DRENDERSTATETYPE)59, 0); //D3DRS_STENCILWRITEMASK   Маска записи применяется к значениям, записанным в буфер трафарета. Маска по умолчанию - 0xFFFFFFFF.
	//device->SetRenderState((D3DRENDERSTATETYPE)58, 0); //D3DRS_STENCILMASK   Маска применяется к опорному значению и каждой записи буфера трафарета для определения значимых битов для проверки трафарета. Маска по умолчанию - 0xFFFFFFFF.
	//device->SetRenderState((D3DRENDERSTATETYPE)168, 15); //D3DRS_COLORWRITEENABLE
	/*
	device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //D3DRS_ALPHABLENDENABLE   TRUE, чтобы включить прозрачность с альфа-смешиванием. Значение по умолчанию FALSE.
	device->SetRenderState(D3DRS_CULLMODE, D3DCULL_CW); //D3DRS_CULLMODE    D3DCULL_CW
	device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESS); //D3DRS_ZFUNC  D3DCMP_LESS
	device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE); //D3DRS_ZENABLE    D3DZBUFFERTYPE  D3DZB_TRUE
	device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE); //D3DRS_ZWRITEENABLE   TRUE
	device->SetRenderState(D3DRS_STENCILENABLE, FALSE); //D3DRS_STENCILENABLE  FALSE
	device->SetRenderState(D3DRS_STENCILWRITEMASK, 0x00000000); //D3DRS_STENCILWRITEMASK   Маска записи применяется к значениям, записанным в буфер трафарета. Маска по умолчанию - 0xFFFFFFFF.
	device->SetRenderState(D3DRS_STENCILMASK, 0x00000000); //D3DRS_STENCILMASK   Маска применяется к опорному значению и каждой записи буфера трафарета для определения значимых битов для проверки трафарета. Маска по умолчанию - 0xFFFFFFFF.
	device->SetRenderState(D3DRS_COLORWRITEENABLE, 0xF); //D3DRS_COLORWRITEENABLE
	*/

	/*
		4840 MyIDirect3DDevice9_BeginScene 0x3c07e0
		4840 MyIDirect3DDevice9_SetRenderTarget 0x3c07e0 0 0x30c4660
		4840 MyIDirect3DDevice9_SetRenderTarget 0x3c07e0 1 0x0
		4840 MyIDirect3DDevice9_SetRenderTarget 0x3c07e0 2 0x0
		4840 MyIDirect3DDevice9_SetDepthStencilSurface 0x3c07e0 0x0
		4840 MyIDirect3DDevice9_SetPixelShader 0x3c07e0 0x0
		4840 MyIDirect3DDevice9_SetVertexShader 0x3c07e0 0x0
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 168 15
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 14 0
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 137 0
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 9 2
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 26 1
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 27 0
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 15 0
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 19 2
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 20 6
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 136 0
		4840 MyIDirect3DDevice9_SetRenderState 0x3c07e0 22 1
	*/


	device->SetStreamSource(d->vertex_index, d->vertex_buffer->get(), d->vertex_bufferOffset, d->vertex_buffer->getData32PerVertex() * sizeof(float));
	device->SetVertexDeclaration(d->declarationEngine->get(d->list_vertex_element));
	device->SetIndices(indexBuffer->get());


	/*
	Context3D.drawTriangles 0x917d9d0 0x30f1dd8 0 2

	2720 MyIDirect3DDevice9_SetPixelShader 0x479e640 0x47a5140
	2720 MyIDirect3DDevice9_SetVertexShader 0x479e640 0x47ae7c0

	2720 MyIDirect3DDevice9_SetVertexShaderConstantF 0x479e640 254 0x28b798 1
	new
	{
	-0.003125, 0.005, 0, 0
	}
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 27 0
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 22 1
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 7 0
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 14 0
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 52 0
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 59 0
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 58 0
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 168 15
	2720 MyIDirect3DDevice9_SetStreamSource 0x479e640 0 0x47a5000 0 24
	2720 MyIDirect3DDevice9_CreateVertexDeclaration 0x479e640 0x28b21c
	0 4 2 0 5 0
	0 8 1 0 5 1
	255 0 17 0 0 0
	2720 MyIDirect3DDevice9_AddRef
	declaration 0x47aa420
	hook_IDirect3DVertexDeclaration9 0x714d24c0
	2720 MyIDirect3DDevice9_SetVertexDeclaration 0x479e640 0x47aa420
	2720 MyIDirect3DDevice9_SetIndices 0x479e640 0x4804e60
	2720 MyIDirect3DDevice9_SetRenderState 0x479e640 174 0
	2720 MyIDirect3DDevice9_DrawIndexedPrimitive 0x479e640 4 0 0 5 0 2

	Context3D.drawTriangles end
	*/

	//device->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE); //174  TRUE, чтобы включить ножничное тестирование, и FALSE, чтобы отключить его. Значение по умолчанию FALSE.

	//qDebug() << d->NumVertices << numTriangles;

	//qDebug() << d->NumVertices << firstIndex << numTriangles;

	//if (d->NumVertices == 54) //&& numTriangles == 2
	//return;

	device->DrawIndexedPrimitive(D3DPT_TRIANGLELIST, 0, 0, d->NumVertices, firstIndex, numTriangles);

	//qDebug("Context3D::drawTriangles end");
}


QString Context3D::dumpVertexShaderConstantF(int reg_start, int reg_end)
{
	IDirect3DDevice9Ex *device = this->device;

	QString s;
	s.append("{\n");

	for (int i = reg_start; i < (reg_end+1); i++)
	{
		struct VECTOR4 { float x; float y; float z; float w; };
		VECTOR4 v;
		device->GetVertexShaderConstantF(i, (float*)&v, 1);

		s.append(QString::number(v.x));
		s.append(", ");
		s.append(QString::number(v.y));
		s.append(", ");
		s.append(QString::number(v.z));
		s.append(", ");
		s.append(QString::number(v.w));
		if (reg_end != i)
			s.append(",");
		s.append("\n");
	}

	s.append("}");
	return s;
}


QString Context3D::dumpPixelShaderConstantF(int reg_start, int reg_end)
{
	IDirect3DDevice9Ex *device = this->device;

	QString s;
	s.append("{\n");

	for (int i = reg_start; i < (reg_end+1); i++)
	{
		struct VECTOR4 { float x; float y; float z; float w; };
		VECTOR4 v;
		device->GetPixelShaderConstantF(i, (float*)&v, 1);

		s.append(QString::number(v.x));
		s.append(", ");
		s.append(QString::number(v.y));
		s.append(", ");
		s.append(QString::number(v.z));
		s.append(", ");
		s.append(QString::number(v.w));
		if (reg_end != i)
			s.append(",");
		s.append("\n");
	}

	s.append("}");
	return s;
}

std::shared_ptr<Texture> Context3D::createTexture(int width, int height, Context3DTextureFormat format, bool optimizeForRenderToTexture, int streamingLevels)
{
	(void)optimizeForRenderToTexture;
	(void)streamingLevels;

	if (format != Context3DTextureFormat::BGRA) {
		qDebug("error Context3D::createTexture format");
		return nullptr;
	}

	//STDMETHOD(CreateTexture)(THIS_ UINT width, UINT height, UINT levels, DWORD usage,
	//D3DFORMAT format, D3DPOOL pool, IDirect3DTexture9 **texture, HANDLE *shared_handle) PURE;

	//Context3D.createTexture 0x330dac38 256 256 "bgra" false 0
	//5668 MyIDirect3DDevice9_CreateTexture 256 256 0 0 21 0 0x0

	IDirect3DTexture9 *_texture;

	//5256 MyIDirect3DDevice9_CreateTexture 1065 630 1 512 21 0 0x0
	//5256 MyIDirect3DDevice9_CreateTexture 274 208 1 1 21 0 0x0
	//5256 MyIDirect3DDevice9_CreateTexture 274 208 1 1 21 0 0x0

	//USAGE
	//D3DUSAGE_RENDERTARGET 1
	//D3DUSAGE_WRITEONLY 8
	//D3DUSAGE_DYNAMIC 512 0x200

	DWORD usage = 0;

	if (optimizeForRenderToTexture == true)
		usage = D3DUSAGE_RENDERTARGET;


	HRESULT r = device->CreateTexture(width, height, streamingLevels, usage, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &_texture, nullptr);

	if (r != S_OK) {
		qDebug() << "error CreateTexture" << r;
		return nullptr;
	}

	IDirect3DSurface9* pSurface = nullptr;
	IDirect3DSurface9* pDepthStencilSurface = nullptr;

	if (optimizeForRenderToTexture == true)
	{
		_texture->GetSurfaceLevel(0, &pSurface);
		device->CreateDepthStencilSurface(width, height, D3DFMT_D24S8, D3DMULTISAMPLE_NONE, 0, FALSE, &pDepthStencilSurface, nullptr);
	}

	return std::make_shared<Texture>(this, _texture, pSurface, pDepthStencilSurface);
}

void Context3D::setProgramConstantsVertex(int firstRegister, const float *data, int numRegisters)
{
	//qDebug() << "SetVertexShaderConstantF" << firstRegister << numRegisters;
	//qDebug().noquote() << toString(data, numRegisters);

	device->SetVertexShaderConstantF(firstRegister, data, numRegisters);
}


void Context3D::setProgramConstantsFragment(int firstRegister, const float *data, int numRegisters)
{
	//qDebug() << "SetPixelShaderConstantF" << firstRegister << numRegisters;
	//qDebug().noquote() << toString(data, numRegisters);

	device->SetPixelShaderConstantF(firstRegister, data, numRegisters);
}

void Context3D::setScissorRectangle(GeomRectangle *rectangle)
{
	(void)rectangle;
}


void Context3D::setStencilActions(Context3DTriangleFace triangleFace, Context3DCompareMode compareMode,
								  Context3DStencilAction actionOnBothPass, Context3DStencilAction actionOnDepthFail,
								  Context3DStencilAction actionOnDepthPassStencilFail)
{
	(void)triangleFace;
	(void)compareMode;
	(void)actionOnBothPass;
	(void)actionOnDepthFail;
	(void)actionOnDepthPassStencilFail;
}

void Context3D::setStencilReferenceValue(uint32_t referenceValue, uint32_t readMask, uint32_t writeMask)
{
	(void)referenceValue;
	(void)readMask;
	(void)writeMask;
}

void Context3D::setTextureAt(int sampler, TextureBase *texture)
{
	device->SetTexture(sampler, texture != nullptr ? texture->getTexturBase() : nullptr);
}


void Context3D::setVertexBufferAt(int index,
								  VertexBuffer3D *buffer,
								  int bufferOffset,
								  Context3DVertexBufferFormat format)
{
	if (index > 7)
		return;

	if (index == 0)
	{
		d->NumVertices = buffer->getNumVertices();
		d->vertex_index = index;
		d->vertex_buffer = buffer;
		d->vertex_bufferOffset = bufferOffset;
	}

	VERTEX_ELEMENT &e = d->list_vertex_element[index];

	if (buffer != nullptr)
	{
		e.Offset = bufferOffset; //bufferOffset * 4;
		e.UsageIndex = index;

		switch (format)
		{
			case Context3DVertexBufferFormat::FLOAT_1:
				e.Type = D3DDECLTYPE_FLOAT1;
				break;
			case Context3DVertexBufferFormat::FLOAT_2:
				e.Type = D3DDECLTYPE_FLOAT2;
				break;
			case Context3DVertexBufferFormat::FLOAT_3:
				e.Type = D3DDECLTYPE_FLOAT3;
				break;
			case Context3DVertexBufferFormat::FLOAT_4:
				e.Type = D3DDECLTYPE_FLOAT4;
				break;
			default:
				qDebug("error setVertexBufferAt format %d", (int)format);
				break;
		}
	}
	else
	{
		e.Offset = 0;
		e.Type = D3DDECLTYPE_UNUSED;
		e.UsageIndex = 0;
	}
}


void Context3D::setProgram(Program3D *program)
{
	//qDebug() << "setProgram hash" << program->hash_vertex << program->hash_pixel;
	d->program = program;
}

void Context3D::setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, QVector<double> data, int numRegisters)
{
	for (int i = 0; i < (numRegisters * 4); i++)
		temp_float[i] = data.at(i);


	if (programType == Context3DProgramType::VERTEX) {
		//qDebug() << "SetVertexShaderConstantF" << firstRegister << numRegisters;
		device->SetVertexShaderConstantF(firstRegister, temp_float, numRegisters);
	} else if (programType == Context3DProgramType::FRAGMENT) {
		//qDebug() << "SetPixelShaderConstantF" << firstRegister << numRegisters;
		device->SetPixelShaderConstantF(firstRegister, temp_float, numRegisters);
	}

	//qDebug().noquote() << toString(temp_float, numRegisters);
}



void Context3D::setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, const double *data, int numRegisters)
{
	for (int i = 0; i < (numRegisters * 4); i++)
		temp_float[i] = data[i];

	if (programType == Context3DProgramType::VERTEX) {
		//qDebug() << "SetVertexShaderConstantF" << firstRegister << numRegisters;
		device->SetVertexShaderConstantF(firstRegister, temp_float, numRegisters);
	} else if (programType == Context3DProgramType::FRAGMENT) {
		//qDebug() << "SetPixelShaderConstantF" << firstRegister << numRegisters;
		device->SetPixelShaderConstantF(firstRegister, temp_float, numRegisters);
	}
}


static uint32_t getPixel(uint32_t *d, int w, int x, int y)
{
	return d[(y * w) + x];
}

static void setPixel(uint32_t *d, int w, int x, int y, uint32_t color)
{
	d[(y * w) + x] = color;
}


void Context3D::draw(uint32_t *data, int x, int y, int w, int h)
{
	IDirect3DSurface9* pSurfaceLevel;
	directXData->texture_1->GetSurfaceLevel(0, &pSurfaceLevel);

	const RECT rect = {x, y, x + w, y + h};
	D3DLOCKED_RECT lock_rect;
	HRESULT result = pSurfaceLevel->LockRect(&lock_rect, &rect, 0);
	if (result != S_OK)
		return;


	D3DSURFACE_DESC desc;
	directXData->texture_1->GetLevelDesc(0, &desc);

	int ww = lock_rect.Pitch / 4;

	for (int _x = 0; _x < w; _x++)
	{
		for (int _y = 0; _y < h; _y++)
		{
			uint32_t pixel = getPixel(data, w, _x, _y);

			setPixel((uint32_t*)lock_rect.pBits, ww, _x, _y, pixel);
		}
	}

	pSurfaceLevel->UnlockRect();
}


void Context3D::present()
{
	//qDebug("Context3D::present()");

	IDirect3DDevice9Ex *device = this->device;


	D3DSURFACE_DESC desc;
	directXData->texture_1->GetLevelDesc(0, &desc);

	LONG Width =  desc.Width;
	LONG Height =  desc.Height;

	{
		//CreateRenderTarget_surface -> texture

		IDirect3DSurface9* pSurfaceLevel;
		directXData->texture_2->GetSurfaceLevel(0, &pSurfaceLevel);

		const RECT _rect = {0, 0, Width, Height};
		HRESULT r = device->StretchRect(directXData->CreateRenderTarget_surface, &_rect, pSurfaceLevel, &_rect, D3DTEXF_POINT);
		if (r != S_OK)
			qDebug() << "error StretchRect" << r;
	}


	if (this->changed_size == true)
	{
		this->changed_size = false;

		struct
		{
			float X1, Y1, v3, v4, v5, v6, v7, v8, v9;
			float XW1, Y2, v12, v13, v14, v15, v16, v17, v18;
			float X2, YH1, v21, v22, v23, v24, v25, v26, v27;
			float XW2, YH2, v30, v31, v32, v33, v34, v35, v36;
		} ViewSize = {0, 0, 0, 1, MyMath::f_nan(), 0, 0, 0, 0,
					0, 0, 0, 1, MyMath::f_nan(), 1, 0, 0, 0,
					0, 0, 0, 1, MyMath::f_nan(), 0, 1, 0, 0,
					0, 0, 0, 1, MyMath::f_nan(), 1, 1, 0, 0};


		ViewSize.X1 = ViewSize.X2 = 0;
		ViewSize.Y1 = ViewSize.Y2 = 0;
		ViewSize.XW1 = ViewSize.XW2 = directXData->width;
		ViewSize.YH1 = ViewSize.YH2 = directXData->height;

		void* _data;
		d->vertex_5->Lock(0, 144, &_data, 0);
		memcpy(_data, &ViewSize, 144);
		d->vertex_5->Unlock();
	}

	device->SetRenderTarget(0, directXData->pBackBuffer);
	device->SetRenderTarget(1, nullptr);
	device->SetRenderTarget(2, nullptr);

	device->SetDepthStencilSurface(nullptr);

	device->SetPixelShader(nullptr);
	device->SetVertexShader(nullptr);

	device->SetRenderState(D3DRS_COLORWRITEENABLE, 15); //168 15
	device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE); //14 0
	device->SetRenderState(D3DRS_LIGHTING, FALSE); //137 0
	device->SetRenderState(D3DRS_SHADEMODE, D3DSHADE_GOURAUD); //9 2
	device->SetRenderState(D3DRS_DITHERENABLE, TRUE); //26 1
	//device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //27 0
	//device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE); //15 0

	device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE); //27 0
	device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE); //15 0

	device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_ONE); //19 2
	device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA); //20 6



	device->SetRenderState(D3DRS_CLIPPING, FALSE); //136 0
	device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE); //22 1


	for (int i = 0; i < 8; i++)
	{
		device->SetTexture(i, nullptr);
		SetTextureStageState_2(device, i);
	}


	SetViewport(device, 0, 0, Width, Height, 0, 1);

	device->SetPixelShader(d->shader_pixel_49ee6e61e4565256ff2ed18f53fac3ba);
	device->SetTexture(0, directXData->texture_2);
	device->SetTexture(1, nullptr);

	device->SetFVF(D3DFVF_TEX2 | D3DFVF_DIFFUSE | D3DFVF_XYZRHW); //580 244
	device->SetStreamSource(0, d->vertex_5, 0, 36);
	device->SetIndices(nullptr);

	device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE); //27 0

	//device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //27 0
	device->DrawPrimitive(D3DPT_TRIANGLESTRIP, 0, 2); //5 0 2

	device->SetPixelShader(nullptr);

	const float ww = Width;
	const float hh = Height;

	float _SIZE[24] = {
		-0.5, -0.5, 0x00000000, 1, 0x00000000, 0x00000000, ww,
		-0.5, 0x00000000, 1, 1, 0x00000000, -0.5, hh,
		0x00000000, 1, 0x00000000, 1, ww, hh,
		0x00000000, 1, 1, 1
	};


	/*
	uint32_t array[25] = {
		0xBF000000, 0xBF000000, 0x00000000, 0x3F800000,
		0x00000000,
		0x00000000, 0x44851000, 0xBF000000,

		0x00000000, 0x3F800000, 0x3F800000, 0x00000000, 0xBF000000, 0x441D6000,
		0x00000000, 0x3F800000, 0x00000000, 0x3F800000, 0x44851000, 0x441D6000,
		0x00000000, 0x3F800000, 0x3F800000, 0x3F800000, 0xA9799214
	};
	*/


#if 1
#if 0
	device->SetRenderState((D3DRENDERSTATETYPE)27, 0);
	device->DrawPrimitive((D3DPRIMITIVETYPE)5, 0, 2);
	device->SetPixelShader(0x0);
	device->SetRenderState((D3DRENDERSTATETYPE)19, 2);
	device->SetRenderState((D3DRENDERSTATETYPE)20, 6);
	device->SetRenderState((D3DRENDERSTATETYPE)27, 1);
	device->SetRenderState((D3DRENDERSTATETYPE)15, 1);
	device->SetRenderState((D3DRENDERSTATETYPE)24, 1);
	device->SetRenderState((D3DRENDERSTATETYPE)25, 7);
	device->SetRenderState((D3DRENDERSTATETYPE)52, 0);
	device->SetRenderState((D3DRENDERSTATETYPE)185, 0);
	device->SetRenderState((D3DRENDERSTATETYPE)56, 8);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)2, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)1, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)5, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)4, 2);
	device->SetTextureStageState(1, (D3DTEXTURESTAGESTATETYPE)1, 1);
	device->SetTextureStageState(1, (D3DTEXTURESTAGESTATETYPE)4, 1);
	device->SetTexture(0, directXData->texture_1);
	device->SetFVF(260);
	device->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, cc, 24);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)2, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)1, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)5, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)4, 2);
	device->SetTextureStageState(1, (D3DTEXTURESTAGESTATETYPE)1, 1);
	device->SetTextureStageState(1, (D3DTEXTURESTAGESTATETYPE)4, 1);
	device->SetTexture(0, directXData->texture_1);
	device->SetFVF(260);
	device->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, cc, 24);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)2, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)1, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)5, 2);
	device->SetTextureStageState(0, (D3DTEXTURESTAGESTATETYPE)4, 2);
	device->SetTextureStageState(1, (D3DTEXTURESTAGESTATETYPE)1, 1);
	device->SetTextureStageState(1, (D3DTEXTURESTAGESTATETYPE)4, 1);
	device->SetTexture(0, directXData->texture_1);
	device->SetFVF(D3DFVF_XYZRHW | D3DFVF_TEX1);
	device->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, cc, 24);
#endif
	device->SetTextureStageState(0, D3DTSS_COLORARG1, 2); //0 2 2
	device->SetTextureStageState(0, D3DTSS_COLOROP, 2); //0 1 2
	device->SetTextureStageState(0, D3DTSS_ALPHAARG1, 2); //0 5 2
	device->SetTextureStageState(0, D3DTSS_ALPHAOP, 2); //0 4 2
	device->SetTextureStageState(1, D3DTSS_COLOROP, 1); //1 1 1
	device->SetTextureStageState(1, D3DTSS_ALPHAOP, 1); //1 4 1
	device->SetTexture(0, directXData->texture_1);
	device->SetFVF(260);
	device->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, _SIZE, 24);

#endif

	{
		static float _SIZE[24] = {
			-0.5, -0.5, 0, 1, 0, 0, 55,
			-0.5, 0, 1, 1, 0, -0.5, 55,
			0, 1, 0, 1, 55, 55,
			0, 1, 1, 1
		};


		auto node = directXData->imageNodes->first;
		while (node != nullptr)
		{
			_SIZE[0] = _SIZE[12] = node->x;
			_SIZE[1] = _SIZE[7] = node->y;

			_SIZE[6] = _SIZE[18] = node->x + node->width;
			_SIZE[13] = _SIZE[19] = node->y + node->height;

			device->SetTexture(0, node->texture);
			device->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, _SIZE, 24);

			node = node->next;
		}
	}



	device->EndScene();


	directXData->pSwapChain->Present(nullptr, nullptr, nullptr, nullptr, 0);



	device->BeginScene();


	//1
	device->SetRenderTarget(0, directXData->pBackBuffer);


	device->Clear(0, nullptr, 1, D3DCOLOR(0xFFFFFFFF), 0.f, 0);



	device->SetRenderState(D3DRS_COLORWRITEENABLE, 15); //168 15
	device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE); //14 0
	device->SetRenderState(D3DRS_LIGHTING, FALSE); //137 0
	device->SetRenderState(D3DRS_SHADEMODE, D3DSHADE_GOURAUD); //9 2
	device->SetRenderState(D3DRS_DITHERENABLE, TRUE); //26 1
	device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //27 0
	device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE); //15 0
	device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_ONE); //19 2
	device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA); //20 6
	device->SetRenderState(D3DRS_CLIPPING, FALSE); //136 0
	device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE); //22 1


	device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //27 0
	device->SetRenderState(D3DRS_CULLMODE, D3DCULL_CW); //22 2
	device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESS); //23 2
	device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE); //7 1
	device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE); //14 1
	device->SetRenderState(D3DRS_STENCILENABLE, FALSE); //52 0
	device->SetRenderState(D3DRS_STENCILWRITEMASK, 0x00000000); //59 0
	device->SetRenderState(D3DRS_STENCILMASK, 0x00000000); //58
	device->SetRenderState(D3DRS_COLORWRITEENABLE, 15); //168 15



	for (int i = 0; i < 8; i++)
		SetSamplerState_1(device, i);


	for (int i = 0; i < 8; i++)
	{
		SetTextureStageState_1(device, i);
		device->SetTexture(i, nullptr);
	}

	device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE); //15 0
	device->SetRenderState(D3DRS_CLIPPING, TRUE); //136 1

	//3
	device->SetRenderTarget(0, directXData->CreateRenderTarget_surface);
	device->SetDepthStencilSurface(directXData->CreateDepthStencilSurface_surface);





	//SetViewport(device, 0, 0, this->_width, this->_height, 0, 1);

	device->Clear(0, 0x0, 7, 0xFFF0F0F0, 1, 0);



	/*
	device->SetRenderState(D3DRS_COLORWRITEENABLE, 15); //168 15
	device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE); //14 0
	device->SetRenderState(D3DRS_LIGHTING, FALSE); //137 0
	device->SetRenderState(D3DRS_SHADEMODE, D3DSHADE_GOURAUD); //9 2
	device->SetRenderState(D3DRS_DITHERENABLE, TRUE); //26 1
	device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //27 0
	device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE); //15 0
	device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_ONE); //19 2
	device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA); //20 6
	device->SetRenderState(D3DRS_CLIPPING, FALSE); //136 0
	device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE); //22 1


	device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE); //27 0
	device->SetRenderState(D3DRS_CULLMODE, D3DCULL_CW); //22 2
	device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESS); //23 2
	device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE); //7 1
	device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE); //14 1
	device->SetRenderState(D3DRS_STENCILENABLE, FALSE); //52 0
	device->SetRenderState(D3DRS_STENCILWRITEMASK, 0x00000000); //59 0
	device->SetRenderState(D3DRS_STENCILMASK, 0x00000000); //58
	device->SetRenderState(D3DRS_COLORWRITEENABLE, 15); //168 15
	*/
}



void Context3D::setBlendFactors(Context3DBlendFactor sourceFactor, Context3DBlendFactor destinationFactor)
{
	//qDebug() << "setBlendFactors" << (int)sourceFactor << (int)destinationFactor;
	this->_sourceFactor = AS3_TO_DIRECTX(sourceFactor);
	this->_destinationFactor = AS3_TO_DIRECTX(destinationFactor);
	//qDebug() << "Context3D::setBlendFactors" << this->_sourceFactor << this->_destinationFactor;
}

void Context3D::setColorMask(bool red, bool green, bool blue, bool alpha)
{
	(void)red;
	(void)green;
	(void)blue;
	(void)alpha;
}

void Context3D::setCulling(Context3DTriangleFace triangleFaceToCull)
{	
	//qDebug() << "setCulling" << (int)triangleFaceToCull;

	switch (triangleFaceToCull)
	{
		case Context3DTriangleFace::BACK:
			//qDebug() << "Context3D::setCulling" << "BACK";
			_triangleFaceToCull = D3DCULL_CCW;
			break;
		case Context3DTriangleFace::FRONT:
			//qDebug() << "Context3D::setCulling" << "FRONT";
			_triangleFaceToCull = D3DCULL_CW;
			break;
			//case Context3DTriangleFace::FRONT_AND_BACK:
			//break;
		case Context3DTriangleFace::NONE:
			//qDebug() << "Context3D::setCulling" << "NONE";
			_triangleFaceToCull = D3DCULL_NONE;
			break;
		default:
			qDebug("error Context3D::setCulling");
			break;
	}
}

void Context3D::setDepthTest(bool depthMask, Context3DCompareMode passCompareMode)
{
	this->_depthMask = depthMask;
	this->_zfunc = AS3_TO_DIRECTX(passCompareMode);
}


void Context3D::draw3DLines(void *data, int count)
{
	device->SetTexture(0, NULL);
	device->SetFVF(D3DFVF_XYZRHW | D3DFVF_DIFFUSE);
	device->DrawPrimitiveUP(D3DPT_LINESTRIP, count, data, 5 * 4);
}

void Context3D::setRenderToTexture(TextureBase *renderTarget,
								bool renderTargetEnableDepthAndStencil,
								int renderTargetAntiAlias,
								int renderTargetSurfaceSelector)
{
	(void)renderTargetEnableDepthAndStencil;
	(void)renderTargetAntiAlias;
	(void)renderTargetSurfaceSelector;

	device->SetRenderTarget(0, renderTarget->getSurface());
	if (renderTargetEnableDepthAndStencil == true)
		device->SetDepthStencilSurface(renderTarget->getDepthStencilSurface());
}

void Context3D::setRenderToBackBuffer()
{
	device->SetRenderTarget(0, directXData->CreateRenderTarget_surface);
	device->SetDepthStencilSurface(directXData->CreateDepthStencilSurface_surface);
}
