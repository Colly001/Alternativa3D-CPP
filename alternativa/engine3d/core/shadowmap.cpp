#include "shadowmap.h"
#include "alternativa/engine3d/lights/directionallight.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "mymath.h"
#include "randomgenerator.h"
#include "alternativa/gfx/core/bitmaptextureresource.h"
#include "flash/display/bitmapdata.h"
#include "alternativa/gfx/core/rendertargettextureresource.h"
#include "alternativa/engine3d/objects/sprite3d.h"
#include "alternativa/engine3d/materials/texturematerial.h"
#include "alternativa/gfx/core/device.h"
#include "alternativa/engine3d/objects/mesh.h"
#include "alternativa/engine3d/objects/bsp.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "utils/double_array.h"
#include "alternativa/engine3d/core/view.h"
#include "shadowmapvertexshaderhlsl.h"
#include "shadowmapfragmentshaderhlsl.h"
#include "alternativa/gfx/shadersampler/shadersampler.h"
#include "alternativa/gfx/core/programresource.h"


//Карта теней

static const int sizeLimit = 0x0800;
static const double bigValue = 0x0800;


static QHash<int, ProgramResource*> programs;

static ProgramResource* getProgram(bool param1, bool param2)
{
	int key = int(param1) | (int(param2) << 1);
	ProgramResource *programResource = programs.value(key);
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = ShadowMapVertexShaderHLSL(param1, param2);
		QByteArray fragment = ShadowMapFragmentShaderHLSL(samplers, param1);
		programResource = new ProgramResource(10, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		programs.insert(key, programResource);
	}
	return programResource;
}




ShadowMap::ShadowMap(int param1, double param2, double param3, double param4, double additionalSpace)
{
	this->map = nullptr;
	this->noise = nullptr;

	this->pixel = 0;

	this->defaultLight = new_DirectionalLight(0x7F7F7F);
	this->mapSize = 0;
	this->planeSize = 0;

	this->dirZ = MyMath::nan();

	this->boundVertexList = Vertex::createList(8);

	{
		QVector<double> list = {0, 2, 4, 6};
		spriteVertexBuffer = new VertexBufferResource(list, 1);
	}
	{
		QVector<uint32_t> list = {0, 1, 3, 1, 2, 3};
		spriteIndexBuffer = new IndexBufferResource(list);
	}

	scissor = new GeomRectangle();

	//private var coords:Vector.<Number> = Vector.<Number>([0, 0, 0, 1, 0, 0, 0, 1, 0, 0,   0, 1, 0, 0, 0, 1, 0, 0, 0, 1,   0, 0, 0, 1, 0, 0, 0, 1, 0, 0,   0, 1, 0, 0, (1 / 0xFF), 1]);
	//36

	double_array_set_36(coords,
						0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
						0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
						0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
						0, 1, 0, 0, 1. / 255., 1);


	double_array_set_4(alphaTestConst, 0, 0, 0, 1);
	double_array_set_4(fragment, 1. / 255., 0, 1, 1);


	//QVector<Number> transform; = Vector.<Number>([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
	this->transform[0] = 0;
	this->transform[1] = 0;
	this->transform[2] = 0;
	this->transform[3] = 0;
	this->transform[4] = 0;
	this->transform[5] = 0;
	this->transform[6] = 0;
	this->transform[7] = 0;
	this->transform[8] = 0;
	this->transform[9] = 0;
	this->transform[10] = 0;
	this->transform[11] = 0;
	this->transform[12] = 0;
	this->transform[13] = 0;
	this->transform[14] = 0;
	this->transform[15] = 1;


	//Vector.<Number>([(-255 * bigValue), -(bigValue), bigValue, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]);
	this->params[0] = -255 * bigValue;
	this->params[1] = -bigValue;
	this->params[2] = bigValue;
	this->params[3] = 1;
	this->params[4] = 0;
	this->params[5] = 0;
	this->params[6] = 0;
	this->params[7] = 1;
	this->params[8] = 0;
	this->params[9] = 0;
	this->params[10] = 0;
	this->params[11] = 1;
	this->params[12] = 0;
	this->params[13] = 0;
	this->params[14] = 0;
	this->params[15] = 1;
	this->params[16] = 0;
	this->params[17] = 0;
	this->params[18] = 0;
	this->params[19] = 1;


	if (param1 > sizeLimit)
	{
		//throw (new Error("Value of mapSize too big."));
		qDebug("error Value of mapSize too big.");
		return;
	}

	double _loc6_ = MyMath::log(param1) / MyMath::LN2;
	if (_loc6_ != int(_loc6_))
	{
		//throw (new Error("Value of mapSize must be power of 2."));
		qDebug("error Value of mapSize must be power of 2.");
		return;
	}

	this->mapSize = param1;
	this->nearDistance = param2;
	this->farDistance = param3;
	this->bias = param4;
	this->additionalSpace = additionalSpace;
	this->defaultLight->rotation.x = MyMath::PI;
	this->map = new RenderTargetTextureResource(param1, param1);

	std::shared_ptr<BitmapData> bitmap = std::make_shared<BitmapData>(this->noiseSize, this->noiseSize, false, 0);

	for (int y = 0; y < this->noiseSize; y++)
	{
		for (int x = 0; x < this->noiseSize; x++)
		{
			double v = RandomGenerator::random() * this->noiseAngle;
			int r = MyMath::sin(v) * 0xFF;
			int g = MyMath::cos(v) * 0xFF;
			int b = (this->noiseRandom + (RandomGenerator::random() * (1 - this->noiseRandom))) * 0xFF;
			uint32_t color = ((r << 16) | (g << 8)) | b;
			bitmap->setPixel(x, y, color);
		}
	}

	this->noise = std::make_shared<BitmapTextureResource>(bitmap, false);
	//this->noise->bitmapData->setVector(this->noise.bitmapData.rect, _loc7_);

}



void ShadowMap::calculateBounds(Camera3D *camera)
{
	if (camera->directionalLight != nullptr)
	{
		this->light = camera->directionalLight;
	}
	else
	{
		this->light = this->defaultLight.get();
	}

	this->light->composeMatrix();
	this->dirZ = this->light->matrix.mk;
	this->light->calculateInverseMatrix();
	double _loc2_ = this->light->matrix_i.ma;
	double _loc3_ = this->light->matrix_i.mb;
	double _loc4_ = this->light->matrix_i.mc;
	double _loc5_ = this->light->matrix_i.md;
	double _loc6_ = this->light->matrix_i.me;
	double _loc7_ = this->light->matrix_i.mf;
	double _loc8_ = this->light->matrix_i.mg;
	double _loc9_ = this->light->matrix_i.mh;
	double _loc10_ = this->light->matrix_i.mi;
	double _loc11_ = this->light->matrix_i.mj;
	double _loc12_ = this->light->matrix_i.mk;
	double _loc13_ = this->light->matrix_i.ml;
	this->light->matrix_i.ma = (((_loc2_ * camera->matrix_g.ma) + (_loc3_ * camera->matrix_g.me)) + (_loc4_ * camera->matrix_g.mi));
	this->light->matrix_i.mb = (((_loc2_ * camera->matrix_g.mb) + (_loc3_ * camera->matrix_g.mf)) + (_loc4_ * camera->matrix_g.mj));
	this->light->matrix_i.mc = (((_loc2_ * camera->matrix_g.mc) + (_loc3_ * camera->matrix_g.mg)) + (_loc4_ * camera->matrix_g.mk));
	this->light->matrix_i.md = ((((_loc2_ * camera->matrix_g.md) + (_loc3_ * camera->matrix_g.mh)) + (_loc4_ * camera->matrix_g.ml)) + _loc5_);
	this->light->matrix_i.me = (((_loc6_ * camera->matrix_g.ma) + (_loc7_ * camera->matrix_g.me)) + (_loc8_ * camera->matrix_g.mi));
	this->light->matrix_i.mf = (((_loc6_ * camera->matrix_g.mb) + (_loc7_ * camera->matrix_g.mf)) + (_loc8_ * camera->matrix_g.mj));
	this->light->matrix_i.mg = (((_loc6_ * camera->matrix_g.mc) + (_loc7_ * camera->matrix_g.mg)) + (_loc8_ * camera->matrix_g.mk));
	this->light->matrix_i.mh = ((((_loc6_ * camera->matrix_g.md) + (_loc7_ * camera->matrix_g.mh)) + (_loc8_ * camera->matrix_g.ml)) + _loc9_);
	this->light->matrix_i.mi = (((_loc10_ * camera->matrix_g.ma) + (_loc11_ * camera->matrix_g.me)) + (_loc12_ * camera->matrix_g.mi));
	this->light->matrix_i.mj = (((_loc10_ * camera->matrix_g.mb) + (_loc11_ * camera->matrix_g.mf)) + (_loc12_ * camera->matrix_g.mj));
	this->light->matrix_i.mk = (((_loc10_ * camera->matrix_g.mc) + (_loc11_ * camera->matrix_g.mg)) + (_loc12_ * camera->matrix_g.mk));
	this->light->matrix_i.ml = ((((_loc10_ * camera->matrix_g.md) + (_loc11_ * camera->matrix_g.mh)) + (_loc12_ * camera->matrix_g.ml)) + _loc13_);
	Vertex *_loc14_ = this->boundVertexList.get();
	_loc14_->x = -(camera->nearClipping);
	_loc14_->y = -(camera->nearClipping);
	_loc14_->z = camera->nearClipping;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = -(camera->nearClipping);
	_loc14_->y = camera->nearClipping;
	_loc14_->z = camera->nearClipping;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = camera->nearClipping;
	_loc14_->y = camera->nearClipping;
	_loc14_->z = camera->nearClipping;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = camera->nearClipping;
	_loc14_->y = -(camera->nearClipping);
	_loc14_->z = camera->nearClipping;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = -(this->farDistance);
	_loc14_->y = -(this->farDistance);
	_loc14_->z = this->farDistance;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = -(this->farDistance);
	_loc14_->y = this->farDistance;
	_loc14_->z = this->farDistance;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->farDistance;
	_loc14_->y = this->farDistance;
	_loc14_->z = this->farDistance;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->farDistance;
	_loc14_->y = -(this->farDistance);
	_loc14_->z = this->farDistance;
	this->light->bound.MinX = 1E22;
	this->light->bound.MinY = 1E22;
	this->light->bound.MinZ = 1E22;
	this->light->bound.MaxX = -1E22;
	this->light->bound.MaxY = -1E22;
	this->light->bound.MaxZ = -1E22;
	_loc14_ = this->boundVertexList.get();
	while (_loc14_ != nullptr)
	{
		_loc14_->cameraX = ((((this->light->matrix_i.ma * _loc14_->x) + (this->light->matrix_i.mb * _loc14_->y)) + (this->light->matrix_i.mc * _loc14_->z)) + this->light->matrix_i.md);
		_loc14_->cameraY = ((((this->light->matrix_i.me * _loc14_->x) + (this->light->matrix_i.mf * _loc14_->y)) + (this->light->matrix_i.mg * _loc14_->z)) + this->light->matrix_i.mh);
		_loc14_->cameraZ = ((((this->light->matrix_i.mi * _loc14_->x) + (this->light->matrix_i.mj * _loc14_->y)) + (this->light->matrix_i.mk * _loc14_->z)) + this->light->matrix_i.ml);
		if (_loc14_->cameraX < this->light->bound.MinX)
		{
			this->light->bound.MinX = _loc14_->cameraX;
		}
		if (_loc14_->cameraX > this->light->bound.MaxX)
		{
			this->light->bound.MaxX = _loc14_->cameraX;
		}
		if (_loc14_->cameraY < this->light->bound.MinY)
		{
			this->light->bound.MinY = _loc14_->cameraY;
		}
		if (_loc14_->cameraY > this->light->bound.MaxY)
		{
			this->light->bound.MaxY = _loc14_->cameraY;
		}
		if (_loc14_->cameraZ < this->light->bound.MinZ)
		{
			this->light->bound.MinZ = _loc14_->cameraZ;
		}
		if (_loc14_->cameraZ > this->light->bound.MaxZ)
		{
			this->light->bound.MaxZ = _loc14_->cameraZ;
		}
		_loc14_ = _loc14_->next.get();
	}
	Vertex *_loc15_ = this->boundVertexList.get();
	Vertex *_loc16_ = this->boundVertexList->next->next->next->next->next->next.get();
	Vertex *_loc17_ = this->boundVertexList->next->next->next->next.get();
	double _loc18_ = (_loc16_->cameraX - _loc15_->cameraX);
	double _loc19_ = (_loc16_->cameraY - _loc15_->cameraY);
	double _loc20_ = (_loc16_->cameraZ - _loc15_->cameraZ);
	double _loc21_ = (_loc17_->cameraX - _loc16_->cameraX);
	double _loc22_ = (_loc17_->cameraY - _loc16_->cameraY);
	double _loc23_ = (_loc17_->cameraZ - _loc16_->cameraZ);
	double _loc24_ = (((_loc18_ * _loc18_) + (_loc19_ * _loc19_)) + (_loc20_ * _loc20_));
	double _loc25_ = (((_loc21_ * _loc21_) + (_loc22_ * _loc22_)) + (_loc23_ * _loc23_));
	int _loc26_ = MyMath::ceil(this->noiseRadius);
	this->planeSize = ((_loc24_ > _loc25_) ? (MyMath::sqrt(_loc24_)) : (MyMath::sqrt(_loc25_)));
	this->pixel = (this->planeSize / ((this->mapSize - 1) - this->noiseRadius));
	this->planeSize = (this->planeSize + ((_loc26_ * this->pixel) * 2));
	this->light->bound.MinX = (this->light->bound.MinX - (_loc26_ * this->pixel));
	this->light->bound.MaxX = (this->light->bound.MaxX + (_loc26_ * this->pixel));
	this->light->bound.MinY = (this->light->bound.MinY - (_loc26_ * this->pixel));
	this->light->bound.MaxY = (this->light->bound.MaxY + (_loc26_ * this->pixel));
	this->light->bound.MinZ = (this->light->bound.MinZ - this->additionalSpace);
	_loc14_ = this->boundVertexList.get();
	_loc14_->x = this->light->bound.MinX;
	_loc14_->y = this->light->bound.MinY;
	_loc14_->z = this->light->bound.MinZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MaxX;
	_loc14_->y = this->light->bound.MinY;
	_loc14_->z = this->light->bound.MinZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MinX;
	_loc14_->y = this->light->bound.MaxY;
	_loc14_->z = this->light->bound.MinZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MaxX;
	_loc14_->y = this->light->bound.MaxY;
	_loc14_->z = this->light->bound.MinZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MinX;
	_loc14_->y = this->light->bound.MinY;
	_loc14_->z = this->light->bound.MaxZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MaxX;
	_loc14_->y = this->light->bound.MinY;
	_loc14_->z = this->light->bound.MaxZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MinX;
	_loc14_->y = this->light->bound.MaxY;
	_loc14_->z = this->light->bound.MaxZ;
	_loc14_ = _loc14_->next.get();
	_loc14_->x = this->light->bound.MaxX;
	_loc14_->y = this->light->bound.MaxY;
	_loc14_->z = this->light->bound.MaxZ;
	this->bound.MinX = 1E22;
	this->bound.MinY = 1E22;
	this->bound.MinZ = 1E22;
	this->bound.MaxX = -1E22;
	this->bound.MaxY = -1E22;
	this->bound.MaxZ = -1E22;
	_loc14_ = this->boundVertexList.get();
	while (_loc14_ != nullptr)
	{
		_loc14_->cameraX = ((((this->light->matrix.ma * _loc14_->x) + (this->light->matrix.mb * _loc14_->y)) + (this->light->matrix.mc * _loc14_->z)) + this->light->matrix.md);
		_loc14_->cameraY = ((((this->light->matrix.me * _loc14_->x) + (this->light->matrix.mf * _loc14_->y)) + (this->light->matrix.mg * _loc14_->z)) + this->light->matrix.mh);
		_loc14_->cameraZ = ((((this->light->matrix.mi * _loc14_->x) + (this->light->matrix.mj * _loc14_->y)) + (this->light->matrix.mk * _loc14_->z)) + this->light->matrix.ml);
		if (_loc14_->cameraX < this->bound.MinX)
		{
			this->bound.MinX = _loc14_->cameraX;
		}
		if (_loc14_->cameraX > this->bound.MaxX)
		{
			this->bound.MaxX = _loc14_->cameraX;
		}
		if (_loc14_->cameraY < this->bound.MinY)
		{
			this->bound.MinY = _loc14_->cameraY;
		}
		if (_loc14_->cameraY > this->bound.MaxY)
		{
			this->bound.MaxY = _loc14_->cameraY;
		}
		if (_loc14_->cameraZ < this->bound.MinZ)
		{
			this->bound.MinZ = _loc14_->cameraZ;
		}
		if (_loc14_->cameraZ > this->bound.MaxZ)
		{
			this->bound.MaxZ = _loc14_->cameraZ;
		}
		_loc14_ = _loc14_->next.get();
	}
}

void ShadowMap::render(Camera3D *camera, QVector<Object3D*> *param2, int param3)
{
	int _loc15_ = 0;
	bool _loc16_ = false;

	TextureMaterial *_loc17_ = nullptr;


	Device *_loc4_ = camera->device;
	this->planeX = (MyMath::floor((this->light->bound.MinX / this->pixel)) * this->pixel);
	this->planeY = (MyMath::floor((this->light->bound.MinY / this->pixel)) * this->pixel);
	this->scissor->width = (MyMath::ceil((this->light->bound.MaxX / this->pixel)) - (this->planeX / this->pixel));
	this->scissor->height = (MyMath::ceil((this->light->bound.MaxY / this->pixel)) - (this->planeY / this->pixel));
	double _loc5_ = (2. / this->planeSize);
	double _loc6_ = (-2. / this->planeSize);
	double _loc7_ = (255. / (this->light->bound.MaxZ - this->light->bound.MinZ));
	double _loc8_ = (-(this->planeX + (this->planeSize * 0.5)) * _loc5_);
	double _loc9_ = (-(this->planeY + (this->planeSize * 0.5)) * _loc6_);
	double _loc10_ = (-(this->light->bound.MinZ) * _loc7_);
	if (this->mapSize != this->map->getWidth())
	{
		this->map->dispose();
		this->map = new RenderTargetTextureResource(this->mapSize, this->mapSize);
	}
	_loc4_->setRenderToTexture(this->map, true);
	_loc4_->clear(1, 0, 0);
	_loc4_->setScissorRectangle(this->scissor);
	this->transform[14] = (1. / 0xFF);
	int _loc11_ = 0;
	while (_loc11_ < param3)
	{
		Object3D *_loc12_ = param2->at(_loc11_);
		VertexBufferResource *_loc13_ = nullptr;
		IndexBufferResource *_loc14_ = nullptr;
		_loc16_ = false;
		if (_loc12_->isType(Sprite3D::TYPE))
		{
			Sprite3D *_loc18_ = (Sprite3D*)_loc12_;
			_loc17_ = (TextureMaterial*)_loc18_->material.get();
			double _loc19_ = _loc18_->width;
			double _loc20_ = _loc18_->height;
			if (_loc18_->autoSize)
			{
				double _loc31_ = (_loc18_->bottomRightU - _loc18_->topLeftU);
				double _loc32_ = (_loc18_->bottomRightV - _loc18_->topLeftV);
				_loc19_ = (_loc17_->getTexturePtr()->width() * _loc31_);
				_loc20_ = (_loc17_->getTexturePtr()->height() * _loc32_);
			}
			double _loc21_ = MyMath::tan(MyMath::asin(-this->dirZ));
			_loc19_ = (_loc19_ * _loc18_->scale.x);
			_loc20_ = (_loc20_ * _loc18_->scale.y);
			double _loc22_ = ((((this->light->matrix_i.ma * _loc12_->matrix.md) + (this->light->matrix_i.mb * _loc12_->matrix.mh)) + (this->light->matrix_i.mc * _loc12_->matrix.ml)) + this->light->matrix_i.md);
			double _loc23_ = ((((this->light->matrix_i.me * _loc12_->matrix.md) + (this->light->matrix_i.mf * _loc12_->matrix.mh)) + (this->light->matrix_i.mg * _loc12_->matrix.ml)) + this->light->matrix_i.mh);
			double _loc24_ = ((((this->light->matrix_i.mi * _loc12_->matrix.md) + (this->light->matrix_i.mj * _loc12_->matrix.mh)) + (this->light->matrix_i.mk * _loc12_->matrix.ml)) + this->light->matrix_i.ml);
			_loc23_ = (_loc23_ + ((MyMath::sin(-(this->dirZ)) * _loc20_) / 4));
			_loc24_ = (_loc24_ - ((MyMath::cos(-(this->dirZ)) * _loc20_) / 4));
			double _loc25_ = (-(_loc19_) * _loc18_->originX);
			double _loc26_ = (-(_loc20_) * _loc18_->originY);
			double _loc27_ = (-(_loc26_) / _loc21_);
			double _loc28_ = (_loc25_ + _loc19_);
			double _loc29_ = (_loc26_ + _loc20_);
			double _loc30_ = (-(_loc29_) / _loc21_);
			_loc25_ = (((_loc25_ + _loc22_) * _loc5_) + _loc8_);
			_loc26_ = (((_loc26_ + _loc23_) * _loc6_) + _loc9_);
			_loc27_ = (((_loc27_ + _loc24_) * _loc7_) + _loc10_);
			_loc28_ = (((_loc28_ + _loc22_) * _loc5_) + _loc8_);
			_loc29_ = (((_loc29_ + _loc23_) * _loc6_) + _loc9_);
			_loc30_ = (((_loc30_ + _loc24_) * _loc7_) + _loc10_);
			_loc27_ = (_loc27_ - (((this->bias * this->biasMultiplier) * _loc7_) / _loc21_));
			_loc30_ = (_loc30_ - (((this->bias * this->biasMultiplier) * _loc7_) / _loc21_));
			this->coords[0] = _loc25_;
			this->coords[1] = _loc26_;
			this->coords[2] = _loc27_;
			this->coords[4] = 0;
			this->coords[5] = 0;
			this->coords[8] = _loc25_;
			this->coords[9] = _loc29_;
			this->coords[10] = _loc30_;
			this->coords[12] = 0;
			this->coords[13] = 1;
			this->coords[16] = _loc28_;
			this->coords[17] = _loc29_;
			this->coords[18] = _loc30_;
			this->coords[20] = 1;
			this->coords[21] = 1;
			this->coords[24] = _loc28_;
			this->coords[25] = _loc26_;
			this->coords[26] = _loc27_;
			this->coords[28] = 1;
			this->coords[29] = 0;
			_loc13_ = this->spriteVertexBuffer;
			_loc14_ = this->spriteIndexBuffer;
			_loc15_ = 2;
			_loc16_ = true;
			_loc4_->setProgram(getProgram(true, true));
			_loc4_->setVertexBufferAt(0, _loc13_, 0, Context3DVertexBufferFormat::FLOAT_1);
			_loc4_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, this->coords, 9, false);
		}
		else
		{
			this->transform[0] = ((((this->light->matrix_i.ma * _loc12_->matrix.ma) + (this->light->matrix_i.mb * _loc12_->matrix.me)) + (this->light->matrix_i.mc * _loc12_->matrix.mi)) * _loc5_);
			this->transform[1] = ((((this->light->matrix_i.ma * _loc12_->matrix.mb) + (this->light->matrix_i.mb * _loc12_->matrix.mf)) + (this->light->matrix_i.mc * _loc12_->matrix.mj)) * _loc5_);
			this->transform[2] = ((((this->light->matrix_i.ma * _loc12_->matrix.mc) + (this->light->matrix_i.mb * _loc12_->matrix.mg)) + (this->light->matrix_i.mc * _loc12_->matrix.mk)) * _loc5_);
			this->transform[3] = ((((((this->light->matrix_i.ma * _loc12_->matrix.md) + (this->light->matrix_i.mb * _loc12_->matrix.mh)) + (this->light->matrix_i.mc * _loc12_->matrix.ml)) + this->light->matrix_i.md) * _loc5_) + _loc8_);
			this->transform[4] = ((((this->light->matrix_i.me * _loc12_->matrix.ma) + (this->light->matrix_i.mf * _loc12_->matrix.me)) + (this->light->matrix_i.mg * _loc12_->matrix.mi)) * _loc6_);
			this->transform[5] = ((((this->light->matrix_i.me * _loc12_->matrix.mb) + (this->light->matrix_i.mf * _loc12_->matrix.mf)) + (this->light->matrix_i.mg * _loc12_->matrix.mj)) * _loc6_);
			this->transform[6] = ((((this->light->matrix_i.me * _loc12_->matrix.mc) + (this->light->matrix_i.mf * _loc12_->matrix.mg)) + (this->light->matrix_i.mg * _loc12_->matrix.mk)) * _loc6_);
			this->transform[7] = ((((((this->light->matrix_i.me * _loc12_->matrix.md) + (this->light->matrix_i.mf * _loc12_->matrix.mh)) + (this->light->matrix_i.mg * _loc12_->matrix.ml)) + this->light->matrix_i.mh) * _loc6_) + _loc9_);
			this->transform[8] = ((((this->light->matrix_i.mi * _loc12_->matrix.ma) + (this->light->matrix_i.mj * _loc12_->matrix.me)) + (this->light->matrix_i.mk * _loc12_->matrix.mi)) * _loc7_);
			this->transform[9] = ((((this->light->matrix_i.mi * _loc12_->matrix.mb) + (this->light->matrix_i.mj * _loc12_->matrix.mf)) + (this->light->matrix_i.mk * _loc12_->matrix.mj)) * _loc7_);
			this->transform[10] = ((((this->light->matrix_i.mi * _loc12_->matrix.mc) + (this->light->matrix_i.mj * _loc12_->matrix.mg)) + (this->light->matrix_i.mk * _loc12_->matrix.mk)) * _loc7_);
			this->transform[11] = ((((((this->light->matrix_i.mi * _loc12_->matrix.md) + (this->light->matrix_i.mj * _loc12_->matrix.mh)) + (this->light->matrix_i.mk * _loc12_->matrix.ml)) + this->light->matrix_i.ml) * _loc7_) + _loc10_);


			if (_loc12_->isType(Mesh::TYPE))
			{
				Mesh *_loc33_ = (Mesh*)_loc12_;
				_loc33_->prepareResources();
				_loc13_ = _loc33_->vertexBuffer.get();
				_loc14_ = _loc33_->indexBuffer.get();
				_loc15_ = _loc33_->numTriangles;
				_loc17_ = (TextureMaterial*)_loc33_->faceList()->material.get();
			}
			else
			{
				if (_loc12_->isType(BSP::TYPE))
				{
					BSP *_loc34_ = (BSP*)_loc12_;
					_loc34_->prepareResources();
					_loc13_ = _loc34_->vertexBuffer.get();
					_loc14_ = _loc34_->indexBuffer.get();
					_loc15_ = _loc34_->numTriangles;
					_loc17_ = (TextureMaterial*)_loc34_->faces->first()->material.get();
				}
				else
				{
					_loc17_ = nullptr;
				}
			}

			if (((!(_loc17_ == nullptr)) && (_loc17_->transparent())))
			{
				_loc16_ = true;
				_loc4_->setProgram(getProgram(true, false));
				_loc4_->setVertexBufferAt(1, _loc13_, 3, Context3DVertexBufferFormat::FLOAT_2);
			}
			else
			{
				_loc4_->setProgram(getProgram(false, false));
			}
			_loc4_->setVertexBufferAt(0, _loc13_, 0, Context3DVertexBufferFormat::FLOAT_3);
			_loc4_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, this->transform, 4, false);
		}

		if (((!(_loc13_ == nullptr)) && (!(_loc14_ == nullptr))))
		{
			_loc4_->setTextureAt(4, nullptr);
			_loc4_->setTextureAt(6, nullptr);
			if (_loc16_)
			{

				_loc4_->setTextureAt(0, _loc17_->textureResource);
				this->alphaTestConst[0] = _loc17_->textureResource->correctionU();
				this->alphaTestConst[1] = _loc17_->textureResource->correctionV();
				this->alphaTestConst[3] = ((_loc12_->isType(Sprite3D::TYPE)) ? 0.99 : this->alphaThreshold);
				_loc4_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 10, this->alphaTestConst, 1);
			}
			_loc4_->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 0, this->fragment, 1);
			_loc4_->drawTriangles(_loc14_, 0, _loc15_);
		}
		if (_loc16_)
		{
			_loc4_->setTextureAt(0, nullptr);
			_loc4_->setVertexBufferAt(1, nullptr);
		}
		_loc11_++;
	}

	_loc4_->setScissorRectangle(nullptr);
	_loc5_ = (1. / this->planeSize);
	_loc6_ = (1. / this->planeSize);
	_loc8_ = (-(this->planeX) * _loc5_);
	_loc9_ = (-(this->planeY) * _loc6_);
	this->transform[0] = (this->light->matrix_i.ma * _loc5_);
	this->transform[1] = (this->light->matrix_i.mb * _loc5_);
	this->transform[2] = (this->light->matrix_i.mc * _loc5_);
	this->transform[3] = ((this->light->matrix_i.md * _loc5_) + _loc8_);
	this->transform[4] = (this->light->matrix_i.me * _loc6_);
	this->transform[5] = (this->light->matrix_i.mf * _loc6_);
	this->transform[6] = (this->light->matrix_i.mg * _loc6_);
	this->transform[7] = ((this->light->matrix_i.mh * _loc6_) + _loc9_);
	this->transform[8] = (this->light->matrix_i.mi * _loc7_);
	this->transform[9] = (this->light->matrix_i.mj * _loc7_);
	this->transform[10] = (this->light->matrix_i.mk * _loc7_);
	this->transform[11] = (((this->light->matrix_i.ml * _loc7_) + _loc10_) - ((this->bias * this->biasMultiplier) * _loc7_));
	this->transform[12] = this->nearDistance;
	this->transform[13] = (this->farDistance - this->nearDistance);
	this->transform[14] = -(_loc7_);
	this->params[4] = 0;
	this->params[5] = 0;
	this->params[6] = (this->noiseRadius / this->mapSize);
	this->params[7] = (1. / numSamples);
	this->params[8] = (camera->view->_width / this->noiseSize);
	this->params[9] = (camera->view->_height / this->noiseSize);
	this->params[11] = ((!(camera->directionalLight == nullptr)) ? ((camera->directionalLightStrength * camera->shadowMapStrength)) : 0);
	this->params[12] = MyMath::cos(this->noiseAngle);
	this->params[13] = MyMath::sin(this->noiseAngle);
	this->params[16] = -(MyMath::sin(this->noiseAngle));
	this->params[17] = MyMath::cos(this->noiseAngle);
}

void ShadowMap::dispose()
{
	//this->map.reset();
	//this->noise.reset();
}
