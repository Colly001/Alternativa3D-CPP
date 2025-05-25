#ifndef CONTEXT3D_H
#define CONTEXT3D_H

#include "Context3DTextureFormat.h"
#include "Context3DProgramType.h"
#include "Context3DBlendFactor.h"
#include "Context3DTriangleFace.h"
#include "Context3DCompareMode.h"
#include "Context3DWrapMode.h"
#include "Context3DTextureFilter.h"
#include "Context3DMipFilter.h"
#include "Context3DStencilAction.h"
#include "Context3DVertexBufferFormat.h"
#include <QVector>
#include <memory>

class VertexBuffer3D;
class IndexBuffer3D;
class Program3D;
class Texture;
class TextureBase;
class GeomRectangle;
class IDirect3DDevice9Ex;
class Context3DPrivate;
class IDirect3DSurface9;
class IDirect3DSwapChain9;
class DirectXData;


class Context3D
{
	IDirect3DDevice9Ex *device;
	DirectXData *directXData;
	Context3DPrivate *d;

	bool _enableErrorChecking;
	bool _depthMask;
	bool _depthMaskBack;
	bool changed_size;

	int _width;
	int _height;

	uint32_t _sourceFactor;
	uint32_t _destinationFactor;
	uint32_t _triangleFaceToCull;
	uint32_t _zfunc;

	float _size[4];

public:
	Context3D(DirectXData*);



	IDirect3DDevice9Ex *getDevice() { return device; }

	DirectXData* getDirectXData() { return directXData; }


	bool getEnableErrorChecking() { return this->_enableErrorChecking; }
	void setEnableErrorChecking(bool enable) { this->_enableErrorChecking = enable; }


	//Очищает буферы цвета, глубины и трафарета, связанные с данным объектом Context3D, и заполняет их указанными значениями.
	void clear(double red = 0.0, double green = 0.0, double blue = 0.0, double alpha = 1.0, double depth = 1.0, double stencil = 0, uint32_t mask = 0xffffffff);


	//Задает размеры области просмотра и другие атрибуты буфера визуализации.
	void configureBackBuffer(int width, int height, int antiAlias, bool enableDepthAndStencil = true, bool wantsBestResolution = false,
							 bool wantsBestResolutionOnBrowserZoom = false);


	//Создается объект IndexBuffer3D.
	std::shared_ptr<IndexBuffer3D> createIndexBuffer(int numIndices);


	//Создается объект Program3D.
	std::shared_ptr<Program3D> createProgram();


	//Создается объект Texture.
	std::shared_ptr<Texture> createTexture(int width, int height, Context3DTextureFormat format, bool optimizeForRenderToTexture, int streamingLevels = 0);


	//Создается объект VertexBuffer3D.
	std::shared_ptr<VertexBuffer3D> createVertexBuffer(int numVertices, int data32PerVertex);


	//Освобождает все ресурсы и внутреннее хранилище, связанные с данным объектом Context3D.
	void dispose(bool recreate = true);


	//Указанные треугольники визуализируются с использованием текущих буферов и состояния данного объекта Context3D.
	void drawTriangles(IndexBuffer3D *indexBuffer, int firstIndex = 0, int numTriangles = -1);


	//Отображает задний буфер визуализации.
	void present();


	//Указывает коэффициенты, используемые для наложения цвета вывода операции рисования на существующий цвет.
	void setBlendFactors(Context3DBlendFactor sourceFactor, Context3DBlendFactor destinationFactor);


	//Устанавливает маску, используемую при записи цветов в буфер визуализации.
	void setColorMask(bool red, bool green, bool blue, bool alpha);


	//Задает режим отбора треугольников.
	void setCulling(Context3DTriangleFace triangleFaceToCull);


	//Задает тип сравнения, используемый для тестирования глубины.
	void setDepthTest(bool depthMask, Context3DCompareMode passCompareMode);


	//Задает вершинные и фрагментные шейдерные программы для использования в ходе последующей визуализации.
	void setProgram(Program3D *program);


	//Задает входные данные констант для шейдерных программ.
	void setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, QVector<double> data, int numRegisters);
	void setProgramConstantsFromVector(Context3DProgramType programType, int firstRegister, const double *data, int numRegisters);

	void setProgramConstantsVertex(int firstRegister, const float *data, int numRegisters);

	void setProgramConstantsFragment(int firstRegister, const float *data, int numRegisters);


	//Ручная перезапись состояния образца текстуры.
	//void setSamplerStateAt(int sampler, Context3DWrapMode wrap, Context3DTextureFilter filter, Context3DMipFilter mipfilter);


	//Задает прямоугольник вырезания, который является типом маски рисования.
	void setScissorRectangle(GeomRectangle *rectangle);


	//Задает режим трафаретов и операцию.
	void setStencilActions(Context3DTriangleFace triangleFace = Context3DTriangleFace::FRONT_AND_BACK,
						   Context3DCompareMode compareMode = Context3DCompareMode::ALWAYS,
						   Context3DStencilAction actionOnBothPass = Context3DStencilAction::KEEP,
						   Context3DStencilAction actionOnDepthFail = Context3DStencilAction::KEEP,
						   Context3DStencilAction actionOnDepthPassStencilFail = Context3DStencilAction::KEEP);


	//Задает значение сравнения трафаретов, используемое для проверок трафаретов.
	void setStencilReferenceValue(uint32_t referenceValue, uint32_t readMask = 255, uint32_t writeMask = 255);


	//Задает текстуру, которая будет использоваться в качестве регистра входной текстуры фрагментной программы.
	void setTextureAt(int sampler, TextureBase *texture);


	//Указывает, какие компоненты данных вершин соответствуют одному вводу данных в шейдерную вершинную программу.
	void setVertexBufferAt(int index, VertexBuffer3D *buffer, int bufferOffset = 0, Context3DVertexBufferFormat format = Context3DVertexBufferFormat::FLOAT_4);

	QString dumpVertexShaderConstantF(int reg_start, int reg_end);
	QString dumpPixelShaderConstantF(int reg_start, int reg_end);

	void draw(uint32_t *data, int x, int y, int w, int h);
	void draw3DLines(void *data, int count);

	void setRenderToTexture(TextureBase *renderTarget, bool renderTargetEnableDepthAndStencil, int renderTargetAntiAlias, int renderTargetSurfaceSelector);

	void setRenderToBackBuffer();
};

#endif // CONTEXT3D_H
