#include "shadow.h"
#include "mymath.h"
#include "alternativa/gfx/core/programresource.h"
#include <QDebug>
#include "alternativa/gfx/shadersampler/shadersampler.h"
#include "shadowcastervertexshaderhlsl.h"
#include "shadowcasterfragmentshaderhlsl.h"
#include "shadowvolumevertexshaderhlsl.h"
#include "shadowvolumefragmentshaderhlsl.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/gfx/core/device.h"
#include "alternativa/engine3d/lights/directionallight.h"
#include "utils/double_array.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "view.h"
#include "vertex.h"
#include "../objects/mesh.h"
#include "object3dcontainer.h"
#include "shadowreceiververtexshaderhlsl.h"
#include "shadowreceiverfragmentshaderhlsl.h"


static Object3D tA(Object3D::TT(Object3D::TYPE));

//Тень

static ProgramResource *casterProgram;
static ProgramResource *volumeProgram;
static QHash<int, ProgramResource*> receiverPrograms;



static ProgramResource* getVolumeProgram()
{
	ProgramResource *programResource = volumeProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = ShadowVolumeVertexShaderHLSL();
		QByteArray fragment = ShadowVolumeFragmentShaderHLSL();
		programResource = new ProgramResource(12, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		volumeProgram = programResource;
	}
	return programResource;
}

static ProgramResource* getReceiverProgram(bool param1, bool param2, bool param3)
{
	int key = ((int(param1) | (int(param2) << 1)) | (int(param3) << 2));
	ProgramResource *programResource = receiverPrograms.value(key);
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = ShadowReceiverVertexShaderHLSL(param3);
		QByteArray fragment = ShadowReceiverFragmentShaderHLSL(samplers, param1, param2);
		programResource = new ProgramResource(13, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		receiverPrograms.insert(key, programResource);
	}
	return programResource;
}


Shadow::Shadow(int mapSize, int blur, double attenuation, double nearDistance, double farDistance, int color, double alpha)
{
	this->cameraInside = false;
	this->texture = nullptr;
	this->offset = 0;
	this->backFadeRange = 0;
	this->receiversCount = 0;
	this->castersCount = 0;

	double_array_set_4(volumeFragmentConst, 1, 0, 1, 0.5);
	double_array_set_8(uvConst, 0, 0, 0, 1, 0, 0, 0, 1);

	double_array_set_4(clampConst, 0, 0, 0, 1);

	double_array_set_20(casterConst, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);


	direction = new Vector3(0, 0, -1);


	boundVertexList = Vertex::createList(8);

	light = new_DirectionalLight(0);

	textureScaleU = 0;
	textureScaleV = 0;
	textureOffsetU = 0;
	textureOffsetV = 0;


	{
		QVector<double> list = {0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1};
		volumeVertexBuffer = new VertexBufferResource(list, 3);
	}

	{
		QVector<uint32_t> list = {0, 1, 3, 2, 3, 1, 7, 6, 4, 5, 4, 6, 4, 5, 0, 1, 0, 5, 3, 2, 7, 6, 7, 2, 0, 3, 4, 7, 4, 3, 5, 6, 1, 2, 1, 6};
		volumeIndexBuffer = new IndexBufferResource(list);
	}


	/*
	if (param1 > ShadowAtlas.sizeLimit)
	{
		throw (new Error("Value of mapSize too big."));
	}
	*/

	/*
	double _loc8_ = MyMath::log(mapSize) / MyMath::LN2;
	if (_loc8_ != int(_loc8_))
	{
		//throw (new Error("Value of mapSize must be power of 2."));
	}
	*/

	this->mapSize = mapSize;
	this->blur = blur;
	this->attenuation = attenuation;
	this->nearDistance = nearDistance;
	this->farDistance = farDistance;
	this->color = color;
	this->alpha = alpha;

}

ProgramResource* Shadow::getCasterProgram()
{
	ProgramResource *programResource = casterProgram;
	if (programResource == nullptr)
	{
		ShaderSampler samplers;
		QByteArray vertex = ShadowCasterVertexShaderHLSL();
		QByteArray fragment = ShadowCasterFragmentShaderHLSL();
		programResource = new ProgramResource(11, ProgramResource::TYPE::HLSL, vertex, fragment, samplers);
		casterProgram = programResource;
	}
	return programResource;
}


void Shadow::renderVolume(Camera3D *camera)
{
	Device *_loc2_ = camera->device;
	volumeTransformConst[0] = this->light->matrix.ma;
	volumeTransformConst[1] = this->light->matrix.mb;
	volumeTransformConst[2] = this->light->matrix.mc;
	volumeTransformConst[3] = this->light->matrix.md;
	volumeTransformConst[4] = this->light->matrix.me;
	volumeTransformConst[5] = this->light->matrix.mf;
	volumeTransformConst[6] = this->light->matrix.mg;
	volumeTransformConst[7] = this->light->matrix.mh;
	volumeTransformConst[8] = this->light->matrix.mi;
	volumeTransformConst[9] = this->light->matrix.mj;
	volumeTransformConst[10] = this->light->matrix.mk;
	volumeTransformConst[11] = this->light->matrix.ml;
	volumeTransformConst[12] = (this->light->bound.MaxX - this->light->bound.MinX);
	volumeTransformConst[13] = (this->light->bound.MaxY - this->light->bound.MinY);
	volumeTransformConst[14] = (this->light->bound.MaxZ - this->light->bound.MinZ);
	volumeTransformConst[15] = 1;
	volumeTransformConst[16] = this->light->bound.MinX;
	volumeTransformConst[17] = this->light->bound.MinY;
	volumeTransformConst[18] = this->light->bound.MinZ;
	volumeTransformConst[19] = 1;
	_loc2_->setProgram(getVolumeProgram());
	_loc2_->setVertexBufferAt(0, volumeVertexBuffer, 0, Context3DVertexBufferFormat::FLOAT_3);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, volumeTransformConst, 5, false);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 16, camera->projection, 1);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 17, camera->correction, 1);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 13, volumeFragmentConst, 1);
	_loc2_->drawTriangles(volumeIndexBuffer, 0, 12);
}

void Shadow::renderReceivers(Camera3D *camera)
{
	Device *_loc2_ = camera->device;
	double _loc3_ = (this->light->bound.MinZ - this->offset);
	double _loc4_ = ((this->light->bound.MaxZ - this->attenuation) - _loc3_);
	double _loc5_ = (this->light->matrix_i.ma / this->planeSize);
	double _loc6_ = (this->light->matrix_i.mb / this->planeSize);
	double _loc7_ = (this->light->matrix_i.mc / this->planeSize);
	double _loc8_ = ((this->light->matrix_i.md - this->planeX) / this->planeSize);
	double _loc9_ = (this->light->matrix_i.me / this->planeSize);
	double _loc10_ = (this->light->matrix_i.mf / this->planeSize);
	double _loc11_ = (this->light->matrix_i.mg / this->planeSize);
	double _loc12_ = ((this->light->matrix_i.mh - this->planeY) / this->planeSize);
	double _loc13_ = (this->light->matrix_i.mi / _loc4_);
	double _loc14_ = (this->light->matrix_i.mj / _loc4_);
	double _loc15_ = (this->light->matrix_i.mk / _loc4_);
	double _loc16_ = ((this->light->matrix_i.ml - _loc3_) / _loc4_);
	this->transformConst[0] = (((_loc5_ * camera->matrix_g.ma) + (_loc6_ * camera->matrix_g.me)) + (_loc7_ * camera->matrix_g.mi));
	this->transformConst[1] = (((_loc5_ * camera->matrix_g.mb) + (_loc6_ * camera->matrix_g.mf)) + (_loc7_ * camera->matrix_g.mj));
	this->transformConst[2] = (((_loc5_ * camera->matrix_g.mc) + (_loc6_ * camera->matrix_g.mg)) + (_loc7_ * camera->matrix_g.mk));
	this->transformConst[3] = ((((_loc5_ * camera->matrix_g.md) + (_loc6_ * camera->matrix_g.mh)) + (_loc7_ * camera->matrix_g.ml)) + _loc8_);
	this->transformConst[4] = (((_loc9_ * camera->matrix_g.ma) + (_loc10_ * camera->matrix_g.me)) + (_loc11_ * camera->matrix_g.mi));
	this->transformConst[5] = (((_loc9_ * camera->matrix_g.mb) + (_loc10_ * camera->matrix_g.mf)) + (_loc11_ * camera->matrix_g.mj));
	this->transformConst[6] = (((_loc9_ * camera->matrix_g.mc) + (_loc10_ * camera->matrix_g.mg)) + (_loc11_ * camera->matrix_g.mk));
	this->transformConst[7] = ((((_loc9_ * camera->matrix_g.md) + (_loc10_ * camera->matrix_g.mh)) + (_loc11_ * camera->matrix_g.ml)) + _loc12_);
	this->transformConst[8] = (((_loc13_ * camera->matrix_g.ma) + (_loc14_ * camera->matrix_g.me)) + (_loc15_ * camera->matrix_g.mi));
	this->transformConst[9] = (((_loc13_ * camera->matrix_g.mb) + (_loc14_ * camera->matrix_g.mf)) + (_loc15_ * camera->matrix_g.mj));
	this->transformConst[10] = (((_loc13_ * camera->matrix_g.mc) + (_loc14_ * camera->matrix_g.mg)) + (_loc15_ * camera->matrix_g.mk));
	this->transformConst[11] = ((((_loc13_ * camera->matrix_g.md) + (_loc14_ * camera->matrix_g.mh)) + (_loc15_ * camera->matrix_g.ml)) + _loc16_);
	this->uvConst[0] = this->textureScaleU;
	this->uvConst[1] = this->textureScaleV;
	this->uvConst[4] = this->textureOffsetU;
	this->uvConst[5] = this->textureOffsetV;
	double _loc17_ = (this->nearDistance * camera->shadowsDistanceMultiplier);
	double _loc18_ = (this->farDistance * camera->shadowsDistanceMultiplier);
	double _loc19_ = (1 - ((this->minZ - _loc17_) / (_loc18_ - _loc17_)));
	if (_loc19_ < 0)
	{
		_loc19_ = 0;
	}
	if (_loc19_ > 1)
	{
		_loc19_ = 1;
	}
	this->colorConst[0] = 0;
	this->colorConst[1] = 0x0100;
	this->colorConst[2] = 1;
	this->colorConst[3] = (this->attenuation / _loc4_);
	this->colorConst[4] = 0;
	this->colorConst[5] = (this->backFadeRange / _loc4_);
	this->colorConst[6] = (this->offset / _loc4_);
	this->colorConst[7] = 1;
	this->colorConst[8] = (((this->color >> 16) & 0xFF) / 0xFF);
	this->colorConst[9] = (((this->color >> 8) & 0xFF) / 0xFF);
	this->colorConst[10] = ((this->color & 0xFF) / 0xFF);
	this->colorConst[11] = ((this->alpha * _loc19_) * camera->shadowsStrength);
	_loc2_->setProgram(getReceiverProgram(camera->view->quality, this->cameraInside, camera->view->correction));
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 11, camera->transform, 3);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 14, camera->projection, 1);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 15, this->transformConst, 3);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 18, camera->correction, 1);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 19, this->uvConst, 2);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 13, this->colorConst, 3);
	_loc2_->setProgramConstantsFromVector(Context3DProgramType::FRAGMENT, 16, this->clampConst, 1);
	int _loc20_ = 0;
	while (_loc20_ < this->receiversCount)
	{
		int _loc21_ = this->receiversBuffers[_loc20_];
		_loc2_->setVertexBufferAt(0, camera->receiversVertexBuffers->at(_loc21_).get(), 0, Context3DVertexBufferFormat::FLOAT_3);
		_loc2_->drawTriangles(camera->receiversIndexBuffers->at(_loc21_).get(), this->receiversFirstIndexes.at(_loc20_), this->receiversNumsTriangles.at(_loc20_));
		camera->numShadows++;
		_loc20_++;
	}
	this->receiversCount = 0;
}

void Shadow::addCaster(Mesh *mesh)
{
	if (this->casters.length() < (this->castersCount + 1))
		this->casters.resize(this->castersCount+1);

	this->casters.replace(this->castersCount, mesh);
	this->castersCount++;
}

void Shadow::removeCaster(Mesh *mesh)
{
	int i = 0;
	while (i < this->castersCount)
	{
		if (this->casters[i] == mesh)
		{
			this->castersCount--;
			while (i < this->castersCount)
			{
				this->casters[i] = this->casters[i + 1];
				i++;
			}
			//this->casters.length = this->castersCount;
			break;
		}
		i++;
	}
}

void Shadow::removeAllCasters()
{
	this->castersCount = 0;
	//this->casters.length = 0;
}

bool Shadow::checkVisibility(Camera3D *param1)
{
	if (this->castersCount == 0)
	{
		return false;
	}


	Vector3 dir;

	if (this->direction != nullptr)
	{
		dir.x = this->direction->x;
		dir.y = this->direction->y;
		dir.z = this->direction->z;
		dir.normalize();
	}
	else
	{
		dir.x = 0;
		dir.y = 0;
		dir.z = -1;
	}



	this->light->rotation.x = (MyMath::atan2(dir.z,
										   MyMath::sqrt(((dir.x * dir.x) + (dir.y * dir.y)))) - (MyMath::PI / 2));

	this->light->rotation.y = 0;
	this->light->rotation.z = -(MyMath::atan2(dir.x, dir.y));
	this->light->composeMatrix();
	double _loc2_ = this->light->matrix.ma;
	double _loc3_ = this->light->matrix.mb;
	double _loc4_ = this->light->matrix.mc;
	double _loc5_ = this->light->matrix.md;
	double _loc6_ = this->light->matrix.me;
	double _loc7_ = this->light->matrix.mf;
	double _loc8_ = this->light->matrix.mg;
	double _loc9_ = this->light->matrix.mh;
	double _loc10_ = this->light->matrix.mi;
	double _loc11_ = this->light->matrix.mj;
	double _loc12_ = this->light->matrix.mk;
	double _loc13_ = this->light->matrix.ml;
	this->light->invertMatrix();
	this->light->matrix_i.ma = this->light->matrix.ma;
	this->light->matrix_i.mb = this->light->matrix.mb;
	this->light->matrix_i.mc = this->light->matrix.mc;
	this->light->matrix_i.md = this->light->matrix.md;
	this->light->matrix_i.me = this->light->matrix.me;
	this->light->matrix_i.mf = this->light->matrix.mf;
	this->light->matrix_i.mg = this->light->matrix.mg;
	this->light->matrix_i.mh = this->light->matrix.mh;
	this->light->matrix_i.mi = this->light->matrix.mi;
	this->light->matrix_i.mj = this->light->matrix.mj;
	this->light->matrix_i.mk = this->light->matrix.mk;
	this->light->matrix_i.ml = this->light->matrix.ml;
	this->light->bound.MinX = 1E22;
	this->light->bound.MinY = 1E22;
	this->light->bound.MinZ = 1E22;
	this->light->bound.MaxX = -1E22;
	this->light->bound.MaxY = -1E22;
	this->light->bound.MaxZ = -1E22;
	int _loc14_ = 0;
	while (_loc14_ < this->castersCount)
	{
		Object3D *_loc24_ = this->casters.at(_loc14_);
		_loc24_->composeMatrix();
		Object3D *_loc25_ = _loc24_->_parent;
		while (_loc25_ != nullptr)
		{
			tA.composeMatrixFromSource(_loc25_);
			_loc24_->appendMatrix(&tA);
			_loc25_ = _loc25_->_parent;
		}
		_loc24_->appendMatrix(this->light.get());
		Vertex *_loc26_ = this->boundVertexList.get();
		_loc26_->x = _loc24_->bound.MinX;
		_loc26_->y = _loc24_->bound.MinY;
		_loc26_->z = _loc24_->bound.MinZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MaxX;
		_loc26_->y = _loc24_->bound.MinY;
		_loc26_->z = _loc24_->bound.MinZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MinX;
		_loc26_->y = _loc24_->bound.MaxY;
		_loc26_->z = _loc24_->bound.MinZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MaxX;
		_loc26_->y = _loc24_->bound.MaxY;
		_loc26_->z = _loc24_->bound.MinZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MinX;
		_loc26_->y = _loc24_->bound.MinY;
		_loc26_->z = _loc24_->bound.MaxZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MaxX;
		_loc26_->y = _loc24_->bound.MinY;
		_loc26_->z = _loc24_->bound.MaxZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MinX;
		_loc26_->y = _loc24_->bound.MaxY;
		_loc26_->z = _loc24_->bound.MaxZ;
		_loc26_ = _loc26_->next.get();
		_loc26_->x = _loc24_->bound.MaxX;
		_loc26_->y = _loc24_->bound.MaxY;
		_loc26_->z = _loc24_->bound.MaxZ;
		_loc26_ = this->boundVertexList.get();
		while (_loc26_ != nullptr)
		{
			_loc26_->cameraX = ((((_loc24_->matrix.ma * _loc26_->x) + (_loc24_->matrix.mb * _loc26_->y)) + (_loc24_->matrix.mc * _loc26_->z)) + _loc24_->matrix.md);
			_loc26_->cameraY = ((((_loc24_->matrix.me * _loc26_->x) + (_loc24_->matrix.mf * _loc26_->y)) + (_loc24_->matrix.mg * _loc26_->z)) + _loc24_->matrix.mh);
			_loc26_->cameraZ = ((((_loc24_->matrix.mi * _loc26_->x) + (_loc24_->matrix.mj * _loc26_->y)) + (_loc24_->matrix.mk * _loc26_->z)) + _loc24_->matrix.ml);
			if (_loc26_->cameraX < this->light->bound.MinX)
			{
				this->light->bound.MinX = _loc26_->cameraX;
			}
			if (_loc26_->cameraX > this->light->bound.MaxX)
			{
				this->light->bound.MaxX = _loc26_->cameraX;
			}
			if (_loc26_->cameraY < this->light->bound.MinY)
			{
				this->light->bound.MinY = _loc26_->cameraY;
			}
			if (_loc26_->cameraY > this->light->bound.MaxY)
			{
				this->light->bound.MaxY = _loc26_->cameraY;
			}
			if (_loc26_->cameraZ < this->light->bound.MinZ)
			{
				this->light->bound.MinZ = _loc26_->cameraZ;
			}
			if (_loc26_->cameraZ > this->light->bound.MaxZ)
			{
				this->light->bound.MaxZ = _loc26_->cameraZ;
			}
			_loc26_ = _loc26_->next.get();
		}
		_loc14_++;
	}
	int _loc15_ = ((((this->mapSize - 1) - 1) - this->blur) - this->blur);
	double _loc16_ = (this->light->bound.MaxX - this->light->bound.MinX);
	double _loc17_ = (this->light->bound.MaxY - this->light->bound.MinY);
	double _loc18_ = ((_loc16_ > _loc17_) ? (_loc16_) : (_loc17_));
	double _loc19_ = (_loc18_ / _loc15_);
	double _loc20_ = ((1 + this->blur) * _loc19_);
	double _loc21_ = ((1 + this->blur) * _loc19_);
	if (_loc16_ > _loc17_)
	{
		_loc21_ = (_loc21_ + (((MyMath::ceil(((_loc17_ - 0.01) / (_loc19_ + _loc19_))) * (_loc19_ + _loc19_)) - _loc17_) * 0.5));
	}
	else
	{
		_loc20_ = (_loc20_ + (((MyMath::ceil(((_loc16_ - 0.01) / (_loc19_ + _loc19_))) * (_loc19_ + _loc19_)) - _loc16_) * 0.5));
	}

	this->light->bound.MinX = (this->light->bound.MinX - _loc20_);
	this->light->bound.MaxX = (this->light->bound.MaxX + _loc20_);
	this->light->bound.MinY = (this->light->bound.MinY - _loc21_);
	this->light->bound.MaxY = (this->light->bound.MaxY + _loc21_);
	this->light->bound.MinZ = (this->light->bound.MinZ + this->offset);
	this->light->bound.MaxZ = (this->light->bound.MaxZ + this->attenuation);
	this->planeSize = ((_loc18_ * this->mapSize) / _loc15_);
	if (_loc16_ > _loc17_)
	{
		this->planeX = this->light->bound.MinX;
		this->planeY = (this->light->bound.MinY - (((this->light->bound.MaxX - this->light->bound.MinX) - (this->light->bound.MaxY - this->light->bound.MinY)) * 0.5));
	}
	else
	{
		this->planeX = (this->light->bound.MinX - (((this->light->bound.MaxY - this->light->bound.MinY) - (this->light->bound.MaxX - this->light->bound.MinX)) * 0.5));
		this->planeY = this->light->bound.MinY;
	}

	double _loc22_ = param1->farClipping;
	param1->farClipping = (this->farDistance * param1->shadowsDistanceMultiplier);
	this->light->matrix.ma = _loc2_;
	this->light->matrix.mb = _loc3_;
	this->light->matrix.mc = _loc4_;
	this->light->matrix.md = _loc5_;
	this->light->matrix.me = _loc6_;
	this->light->matrix.mf = _loc7_;
	this->light->matrix.mg = _loc8_;
	this->light->matrix.mh = _loc9_;
	this->light->matrix.mi = _loc10_;
	this->light->matrix.mj = _loc11_;
	this->light->matrix.mk = _loc12_;
	this->light->matrix.ml = _loc13_;
	this->light->appendMatrix(param1);
	bool _loc23_ = this->cullingInCamera(param1);
	param1->farClipping = _loc22_;
	if (_loc23_)
	{
		/*
		if (((param1.debug) && (param1.checkInDebug(this->light) & Debug.BOUNDS)))
		{
			Debug.drawBounds(param1, this->light, this->light->bound.MinX, this->light->bound.MinY, this->light->bound.MinZ, this->light->bound.MaxX, this->light->bound.MaxY, this->light->bound.MaxZ, 0xFF00FF);
		}
		*/

		this->bound.MinX = 1E22;
		this->bound.MinY = 1E22;
		this->bound.MinZ = 1E22;
		this->bound.MaxX = -1E22;
		this->bound.MaxY = -1E22;
		this->bound.MaxZ = -1E22;
		Vertex *_loc26_ = this->boundVertexList.get();
		while (_loc26_ != nullptr)
		{
			_loc26_->cameraX = ((((_loc2_ * _loc26_->x) + (_loc3_ * _loc26_->y)) + (_loc4_ * _loc26_->z)) + _loc5_);
			_loc26_->cameraY = ((((_loc6_ * _loc26_->x) + (_loc7_ * _loc26_->y)) + (_loc8_ * _loc26_->z)) + _loc9_);
			_loc26_->cameraZ = ((((_loc10_ * _loc26_->x) + (_loc11_ * _loc26_->y)) + (_loc12_ * _loc26_->z)) + _loc13_);
			if (_loc26_->cameraX < this->bound.MinX)
			{
				this->bound.MinX = _loc26_->cameraX;
			}
			if (_loc26_->cameraX > this->bound.MaxX)
			{
				this->bound.MaxX = _loc26_->cameraX;
			}
			if (_loc26_->cameraY < this->bound.MinY)
			{
				this->bound.MinY = _loc26_->cameraY;
			}
			if (_loc26_->cameraY > this->bound.MaxY)
			{
				this->bound.MaxY = _loc26_->cameraY;
			}
			if (_loc26_->cameraZ < this->bound.MinZ)
			{
				this->bound.MinZ = _loc26_->cameraZ;
			}
			if (_loc26_->cameraZ > this->bound.MaxZ)
			{
				this->bound.MaxZ = _loc26_->cameraZ;
			}
			_loc26_ = _loc26_->next.get();
		}
		this->cameraInside = false;
		if (this->minZ <= param1->nearClipping)
		{
			double _loc27_ = ((((this->light->matrix_i.ma * param1->matrix_g.md) + (this->light->matrix_i.mb * param1->matrix_g.mh)) + (this->light->matrix_i.mc * param1->matrix_g.ml)) + this->light->matrix_i.md);
			if ((((_loc27_ - param1->nearClipping) <= this->light->bound.MaxX) && ((_loc27_ + param1->nearClipping) >= this->light->bound.MinX)))
			{
				_loc27_ = ((((this->light->matrix_i.me * param1->matrix_g.md) + (this->light->matrix_i.mf * param1->matrix_g.mh)) + (this->light->matrix_i.mg * param1->matrix_g.ml)) + this->light->matrix_i.mh);
				if ((((_loc27_ - param1->nearClipping) <= this->light->bound.MaxY) && ((_loc27_ + param1->nearClipping) >= this->light->bound.MinY)))
				{
					_loc27_ = ((((this->light->matrix_i.mi * param1->matrix_g.md) + (this->light->matrix_i.mj * param1->matrix_g.mh)) + (this->light->matrix_i.mk * param1->matrix_g.ml)) + this->light->matrix_i.ml);
					if ((((_loc27_ - param1->nearClipping) <= this->light->bound.MaxZ) && ((_loc27_ + param1->nearClipping) >= this->light->bound.MinZ)))
					{
						this->cameraInside = true;
					}
				}
			}
		}
	}
	return (_loc23_);
}

void Shadow::renderCasters(Camera3D *param1)
{
	Device *_loc2_ = param1->device;
	double _loc3_ = (2. / this->planeSize);
	double _loc4_ = (-2. / this->planeSize);
	double _loc5_ = (1. / ((this->light->bound.MaxZ - this->attenuation) - (this->light->bound.MinZ - this->offset)));
	double _loc6_ = (-(this->light->bound.MinZ - this->offset) * _loc5_);
	double _loc7_ = ((this->light->bound.MinX + this->light->bound.MaxX) * 0.5);
	double _loc8_ = ((this->light->bound.MinY + this->light->bound.MaxY) * 0.5);

	int _loc9_ = 0;
	while (_loc9_ < this->castersCount)
	{
		Mesh *_loc10_ = this->casters[_loc9_];
		_loc10_->prepareResources();
		casterConst[0] = (_loc10_->matrix.ma * _loc3_);
		casterConst[1] = (_loc10_->matrix.mb * _loc3_);
		casterConst[2] = (_loc10_->matrix.mc * _loc3_);
		casterConst[3] = ((_loc10_->matrix.md - _loc7_) * _loc3_);
		casterConst[4] = (_loc10_->matrix.me * _loc4_);
		casterConst[5] = (_loc10_->matrix.mf * _loc4_);
		casterConst[6] = (_loc10_->matrix.mg * _loc4_);
		casterConst[7] = ((_loc10_->matrix.mh - _loc8_) * _loc4_);
		casterConst[8] = (_loc10_->matrix.mi * _loc5_);
		casterConst[9] = (_loc10_->matrix.mj * _loc5_);
		casterConst[10] = (_loc10_->matrix.mk * _loc5_);
		casterConst[11] = ((_loc10_->matrix.ml * _loc5_) + _loc6_);
		casterConst[12] = this->textureScaleU;
		casterConst[13] = this->textureScaleV;
		casterConst[16] = (((2 * this->textureOffsetU) - 1) + this->textureScaleU);
		casterConst[17] = -(((2 * this->textureOffsetV) - 1) + this->textureScaleV);
		_loc2_->setVertexBufferAt(0, _loc10_->vertexBuffer.get(), 0, Context3DVertexBufferFormat::FLOAT_3);
		_loc2_->setProgramConstantsFromVector(Context3DProgramType::VERTEX, 0, casterConst, 5, false);
		_loc2_->drawTriangles(_loc10_->indexBuffer.get(), 0, _loc10_->numTriangles);
		_loc9_++;
	}
	this->clampConst[0] = this->textureOffsetU;
	this->clampConst[1] = this->textureOffsetV;
	this->clampConst[2] = (this->textureOffsetU + this->textureScaleU);
	this->clampConst[3] = (this->textureOffsetV + this->textureScaleV);
}


bool Shadow::cullingInCamera(Camera3D *param1)
{
	Vertex *_loc2_ = this->boundVertexList.get();
	_loc2_->x = this->light->bound.MinX;
	_loc2_->y = this->light->bound.MinY;
	_loc2_->z = this->light->bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MaxX;
	_loc2_->y = this->light->bound.MinY;
	_loc2_->z = this->light->bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MinX;
	_loc2_->y = this->light->bound.MaxY;
	_loc2_->z = this->light->bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MaxX;
	_loc2_->y = this->light->bound.MaxY;
	_loc2_->z = this->light->bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MinX;
	_loc2_->y = this->light->bound.MinY;
	_loc2_->z = this->light->bound.MaxZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MaxX;
	_loc2_->y = this->light->bound.MinY;
	_loc2_->z = this->light->bound.MaxZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MinX;
	_loc2_->y = this->light->bound.MaxY;
	_loc2_->z = this->light->bound.MaxZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = this->light->bound.MaxX;
	_loc2_->y = this->light->bound.MaxY;
	_loc2_->z = this->light->bound.MaxZ;
	this->minZ = 1E22;
	_loc2_ = this->boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		_loc2_->cameraX = ((((this->light->matrix.ma * _loc2_->x) + (this->light->matrix.mb * _loc2_->y)) + (this->light->matrix.mc * _loc2_->z)) + this->light->matrix.md);
		_loc2_->cameraY = ((((this->light->matrix.me * _loc2_->x) + (this->light->matrix.mf * _loc2_->y)) + (this->light->matrix.mg * _loc2_->z)) + this->light->matrix.mh);
		_loc2_->cameraZ = ((((this->light->matrix.mi * _loc2_->x) + (this->light->matrix.mj * _loc2_->y)) + (this->light->matrix.mk * _loc2_->z)) + this->light->matrix.ml);
		if (_loc2_->cameraZ < this->minZ)
		{
			this->minZ = _loc2_->cameraZ;
		}
		_loc2_ = _loc2_->next.get();
	}

	double _loc5_ = param1->nearClipping;
	_loc2_ = this->boundVertexList.get();
	bool _loc3_ = false;
	bool _loc4_ = false;
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraZ > _loc5_)
		{
			_loc3_ = true;
			if (_loc4_)
			{
				break;
			}
		}
		else
		{
			_loc4_ = true;
			if (_loc3_)
			{
				break;
			}
		}
		_loc2_ = _loc2_->next.get();
	}

	if (((_loc4_) && (!(_loc3_))))
	{
		return false;
	}

	double _loc6_ = param1->farClipping;
	_loc2_ = this->boundVertexList.get();
	_loc3_ = false;
	_loc4_ = false;
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraZ < _loc6_)
		{
			_loc3_ = true;
			if (_loc4_)
			{
				break;
			}
		}
		else
		{
			_loc4_ = true;
			if (_loc3_)
			{
				break;
			}
		}
		_loc2_ = _loc2_->next.get();
	}
	if (((_loc4_) && (!(_loc3_))))
	{
		return false;
	}
	_loc2_ = this->boundVertexList.get();
	_loc3_ = false;
	_loc4_ = false;
	while (_loc2_ != nullptr)
	{
		if (-(_loc2_->cameraX) < _loc2_->cameraZ)
		{
			_loc3_ = true;
			if (_loc4_)
			{
				break;
			}
		}
		else
		{
			_loc4_ = true;
			if (_loc3_)
			{
				break;
			}
		}
		_loc2_ = _loc2_->next.get();
	}
	if (((_loc4_) && (!(_loc3_))))
	{
		return false;
	}
	_loc2_ = this->boundVertexList.get();
	_loc3_ = false;
	_loc4_ = false;
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraX < _loc2_->cameraZ)
		{
			_loc3_ = true;
			if (_loc4_)
			{
				break;
			}
		}
		else
		{
			_loc4_ = true;
			if (_loc3_)
			{
				break;
			}
		}
		_loc2_ = _loc2_->next.get();
	}
	if (((_loc4_) && (!(_loc3_))))
	{
		return false;
	}
	_loc2_ = this->boundVertexList.get();
	_loc3_ = false;
	_loc4_ = false;
	while (_loc2_ != nullptr)
	{
		if (-(_loc2_->cameraY) < _loc2_->cameraZ)
		{
			_loc3_ = true;
			if (_loc4_)
			{
				break;
			}
		}
		else
		{
			_loc4_ = true;
			if (_loc3_)
			{
				break;
			}
		}
		_loc2_ = _loc2_->next.get();
	}
	if (((_loc4_) && (!(_loc3_))))
	{
		return false;
	}
	_loc2_ = this->boundVertexList.get();
	_loc3_ = false;
	_loc4_ = false;
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraY < _loc2_->cameraZ)
		{
			_loc3_ = true;
			if (_loc4_)
			{
				break;
			}
		}
		else
		{
			_loc4_ = true;
			if (_loc3_)
			{
				break;
			}
		}
		_loc2_ = _loc2_->next.get();
	}
	if (((_loc4_) && (!(_loc3_))))
	{
		return false;
	}

	return true;
}
