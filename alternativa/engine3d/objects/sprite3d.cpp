#include "sprite3d.h"
#include "mymath.h"
#include "alternativa/engine3d/materials/texturematerial.h"
#include "flash/display/bitmapdata.h"
#include "../core/rayintersectiondata.h"
#include "../core/camera3d.h"
#include "../core/view.h"
#include "../core/face.h"
#include "../lights/omnilight.h"
#include "../lights/tubelight.h"
#include "../lights/spotlight.h"
#include "alternativa/engine3d/core/vg.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/engine3d/core/face.h"
#include "_global.h"
#include "../core/Object3DFilter.h"
#include "../core/object3dcontainer.h"
#include "alternativa/tanks/engine3d/animatedsprite3d.h"



const Object3D::T *Sprite3D::TYPE = new Object3D::T("Sprite3D");


Sprite3D::Sprite3D(const TT &t, double width, double height, std::shared_ptr<Material> material) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	originX = 0.5;
	originY = 0.5;
	sorting = 0;
	clipping = 2;
	rotation = 0;
	autoSize = false;
	perspectiveScale = true;
	topLeftU = 0;
	topLeftV = 0;
	bottomRightU = 1;
	bottomRightV = 1;
	depthTest = true;

	lightConst[0] = 0;
	lightConst[1] = 0;
	lightConst[2] = 0;
	lightConst[3] = 1;
	lighted = false;

	this->width = width;
	this->height = height;
	this->material = material;
	this->shadowMapAlphaThreshold = 100;
	//this->faces = new Vector.<Face>();
}

bool Sprite3D::intersectRay(const Vector3 *param1, const Vector3 *param2, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result)
{
	if (camera == nullptr || (filter != nullptr && filter->check(this)))
	{
		return false;
	}

	camera->composeCameraMatrix();
	Object3D *_loc6_ = camera;
	while (_loc6_->_parent != nullptr)
	{
		_loc6_ = _loc6_->_parent;
		_loc6_->composeMatrix();
		camera->appendMatrix(_loc6_);
	}

	camera->invertMatrix();
	composeMatrix();
	_loc6_ = this;
	while (_loc6_->_parent != nullptr)
	{
		_loc6_ = _loc6_->_parent;
		_loc6_->composeMatrix();
		appendMatrix(_loc6_);
	}

	appendMatrix(camera);
	calculateInverseMatrix();
	double _loc7_ = camera->nearClipping;
	double _loc8_ = camera->farClipping;
	camera->nearClipping = -1.7976931348623157081e+308; //-Number.MAX_VALUE;
	camera->farClipping = 1.7976931348623157081e+308; //Number.MAX_VALUE;
	culling = 0;
	std::shared_ptr<Face> _loc9_ = this->calculateFace(camera);
	camera->nearClipping = _loc7_;
	camera->farClipping = _loc8_;
	Wrapper *_loc10_ = _loc9_->wrapper.get();
	while (_loc10_ != nullptr)
	{
		Vertex *_loc24_ = _loc10_->vertex.get();
		_loc24_->x = ((((this->matrix_i.ma * _loc24_->cameraX) + (this->matrix_i.mb * _loc24_->cameraY)) + (this->matrix_i.mc * _loc24_->cameraZ)) + this->matrix_i.md);
		_loc24_->y = ((((this->matrix_i.me * _loc24_->cameraX) + (this->matrix_i.mf * _loc24_->cameraY)) + (this->matrix_i.mg * _loc24_->cameraZ)) + this->matrix_i.mh);
		_loc24_->z = ((((this->matrix_i.mi * _loc24_->cameraX) + (this->matrix_i.mj * _loc24_->cameraY)) + (this->matrix_i.mk * _loc24_->cameraZ)) + this->matrix_i.ml);
		_loc10_ = _loc10_->_next;
	}
	Wrapper *_loc11_ = _loc9_->wrapper.get();
	std::shared_ptr<Vertex> _loc12_ = _loc11_->vertex;
	_loc11_ = _loc11_->_next;
	std::shared_ptr<Vertex> _loc13_ = _loc11_->vertex;
	_loc11_ = _loc11_->_next;
	std::shared_ptr<Vertex> _loc14_ = _loc11_->vertex;
	_loc11_ = _loc11_->_next;
	std::shared_ptr<Vertex> _loc15_ = _loc11_->vertex;
	_loc12_->u = this->topLeftU;
	_loc12_->v = this->topLeftV;
	_loc13_->u = this->topLeftU;
	_loc13_->v = this->bottomRightV;
	_loc14_->u = this->bottomRightU;
	_loc14_->v = this->bottomRightV;
	_loc15_->u = this->bottomRightU;
	_loc15_->v = this->topLeftV;
	double _loc16_ = (_loc13_->x - _loc12_->x);
	double _loc17_ = (_loc13_->y - _loc12_->y);
	double _loc18_ = (_loc13_->z - _loc12_->z);
	double _loc19_ = (_loc14_->x - _loc12_->x);
	double _loc20_ = (_loc14_->y - _loc12_->y);
	double _loc21_ = (_loc14_->z - _loc12_->z);
	_loc9_->normalX = ((_loc21_ * _loc17_) - (_loc20_ * _loc18_));
	_loc9_->normalY = ((_loc19_ * _loc18_) - (_loc21_ * _loc16_));
	_loc9_->normalZ = ((_loc20_ * _loc16_) - (_loc19_ * _loc17_));
	double _loc22_ = (1. / MyMath::sqrt((((_loc9_->normalX * _loc9_->normalX) + (_loc9_->normalY * _loc9_->normalY)) + (_loc9_->normalZ * _loc9_->normalZ))));
	_loc9_->normalX = (_loc9_->normalX * _loc22_);
	_loc9_->normalY = (_loc9_->normalY * _loc22_);
	_loc9_->normalZ = (_loc9_->normalZ * _loc22_);
	_loc9_->offset = (((_loc12_->x * _loc9_->normalX) + (_loc12_->y * _loc9_->normalY)) + (_loc12_->z * _loc9_->normalZ));
	double _loc23_ = (((param2->x * _loc9_->normalX) + (param2->y * _loc9_->normalY)) + (param2->z * _loc9_->normalZ));

	bool n = false;

	if (_loc23_ < 0)
	{
		double _loc25_ = ((((param1->x * _loc9_->normalX) + (param1->y * _loc9_->normalY)) + (param1->z * _loc9_->normalZ)) - _loc9_->offset);
		if (_loc25_ > 0)
		{
			double _loc26_ = (-(_loc25_) / _loc23_);
			double _loc27_ = (param1->x + (param2->x * _loc26_));
			double _loc28_ = (param1->y + (param2->y * _loc26_));
			double _loc29_ = (param1->z + (param2->z * _loc26_));
			_loc10_ = _loc9_->wrapper.get();
			while (_loc10_ != nullptr)
			{
				_loc12_ = _loc10_->vertex;
				_loc13_ = ((!(_loc10_->_next == nullptr)) ? _loc10_->_next->vertex : _loc9_->wrapper->vertex);
				_loc16_ = (_loc13_->x - _loc12_->x);
				_loc17_ = (_loc13_->y - _loc12_->y);
				_loc18_ = (_loc13_->z - _loc12_->z);
				_loc19_ = (_loc27_ - _loc12_->x);
				_loc20_ = (_loc28_ - _loc12_->y);
				_loc21_ = (_loc29_ - _loc12_->z);
				if ((((((_loc21_ * _loc17_) - (_loc20_ * _loc18_)) * _loc9_->normalX) + (((_loc19_ * _loc18_) - (_loc21_ * _loc16_)) * _loc9_->normalY)) + (((_loc20_ * _loc16_) - (_loc19_ * _loc17_)) * _loc9_->normalZ)) < 0)
				{
					break;
				}
				_loc10_ = _loc10_->_next;
			}
			if (_loc10_ == nullptr)
			{
				result->object = this;
				result->face = nullptr;
				result->point.reset(_loc27_, _loc28_, _loc29_);
				_loc9_->getUV(&result->point, &result->uv);
				result->time = _loc26_;
				n = true;
			}
		}
	}

	camera->deferredDestroy();
	return n;
}

void Sprite3D::clonePropertiesFromSprite3D(Sprite3D *src)
{
	Object3D::clonePropertiesFromObject3D(src);
	Sprite3D *_loc2_ = src;
	this->width = _loc2_->width;
	this->height = _loc2_->height;
	this->autoSize = _loc2_->autoSize;
	this->material = _loc2_->material;
	this->clipping = _loc2_->clipping;
	this->sorting = _loc2_->sorting;
	this->originX = _loc2_->originX;
	this->originY = _loc2_->originY;
	this->topLeftU = _loc2_->topLeftU;
	this->topLeftV = _loc2_->topLeftV;
	this->bottomRightU = _loc2_->bottomRightU;
	this->bottomRightV = _loc2_->bottomRightV;
	this->rotation = _loc2_->rotation;
	this->perspectiveScale = _loc2_->perspectiveScale;
}

void Sprite3D::draw(Camera3D *param1)
{
	if (this->material == nullptr)
	{
		return;
	}

	std::shared_ptr<Face> _loc3_ = this->calculateFace(param1);
	if (_loc3_ != nullptr)
	{
		this->lighted = false;
		if (((((useLight) && (!(param1->view->constrained))) && (param1->deferredLighting)) && (param1->deferredLightingStrength > 0)))
		{
			this->calculateLight(param1);
		}

		/*
		int _loc2_;

		if (((param1->debug) && ((_loc2_ = param1->checkInDebug(this)) > 0)))
		{
			if ((_loc2_ & Debug.EDGES))
			{
				Debug.drawEdges(param1, _loc3_, 0xFFFFFF);
			}
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(param1, this, bound.MinX, bound.MinY, bound.MinZ, bound.MaxX, bound.MaxY, bound.MaxZ);
			}
		}
		*/

		std::shared_ptr<Object3D> object = this->ptr();
		param1->addTransparent(_loc3_, object);
	}
}


object_ptr<VG> Sprite3D::getVG(Camera3D *param1)
{
	if (this->material == nullptr)
	{
		return nullptr;
	}

	std::shared_ptr<Face> face = this->calculateFace(param1);
	if (face != nullptr)
	{
		this->lighted = false;
		if (((useLight && !param1->view->constrained) && param1->deferredLighting) && (param1->deferredLightingStrength > 0))
		{
			this->calculateLight(param1);
		}
		face->normalX = 0;
		face->normalY = 0;
		face->normalZ = -1;
		face->offset = -matrix.ml;
		std::shared_ptr<Object3D> __this = this->ptr();
		return VG::create(__this, face, this->sorting, ((!(!(param1->debug))) ? int(param1->checkInDebug(__this)) : int(0)), true);
	}

	return nullptr;
}

void Sprite3D::calculateLight(Camera3D *param1)
{
	double _loc2_ = (param1->viewSizeX / param1->focalLength);
	double _loc3_ = (param1->viewSizeY / param1->focalLength);
	if (((!(param1->view->constrained)) && (((!(param1->directionalLight == nullptr)) && (param1->directionalLightStrength > 0)) || ((!(param1->shadowMap == nullptr)) && (param1->shadowMapStrength > 0)))))
	{
		this->lightConst[0] = 0;
		this->lightConst[1] = 0;
		this->lightConst[2] = 0;
	}
	else
	{
		this->lightConst[0] = 1;
		this->lightConst[1] = 1;
		this->lightConst[2] = 1;
	}
	double _loc13_ = (matrix.md * _loc2_);
	double _loc14_ = (matrix.mh * _loc3_);
	double _loc15_ = matrix.ml;
	double _loc16_ = MyMath::sqrt((((_loc13_ * _loc13_) + (_loc14_ * _loc14_)) + (_loc15_ * _loc15_)));
	double _loc17_ = (-(_loc13_) / _loc16_);
	double _loc18_ = (-(_loc14_) / _loc16_);
	double _loc19_ = (-(_loc15_) / _loc16_);
	int _loc4_ = 0;
	while (_loc4_ < param1->omniesCount)
	{
		OmniLight* _loc20_ = param1->omnies[_loc4_];
		double _loc5_ = (_loc20_->cmd * _loc2_);
		double _loc6_ = (_loc20_->cmh * _loc3_);
		double _loc7_ = _loc20_->cml;
		double _loc8_ = _loc20_->attenuationEnd;
		if ((((((((_loc5_ - _loc8_) < _loc13_) && ((_loc5_ + _loc8_) > _loc13_)) && ((_loc6_ - _loc8_) < _loc14_)) && ((_loc6_ + _loc8_) > _loc14_)) && ((_loc7_ - _loc8_) < _loc15_)) && ((_loc7_ + _loc8_) > _loc15_)))
		{
			_loc5_ = (_loc5_ - _loc13_);
			_loc6_ = (_loc6_ - _loc14_);
			_loc7_ = (_loc7_ - _loc15_);
			_loc16_ = MyMath::sqrt((((_loc5_ * _loc5_) + (_loc6_ * _loc6_)) + (_loc7_ * _loc7_)));
			if (((_loc16_ > 0) && (_loc16_ < _loc8_)))
			{
				_loc5_ = (_loc5_ / _loc16_);
				_loc6_ = (_loc6_ / _loc16_);
				_loc7_ = (_loc7_ / _loc16_);
				double _loc9_ = ((_loc8_ - _loc16_) / (_loc20_->attenuationEnd - _loc20_->attenuationBegin));
				if (_loc9_ > 1)
				{
					_loc9_ = 1;
				}
				if (_loc9_ < 0)
				{
					_loc9_ = 0;
				}
				_loc9_ = (_loc9_ * _loc9_);
				double _loc11_ = (((_loc5_ * _loc17_) + (_loc6_ * _loc18_)) + (_loc7_ * _loc19_));
				_loc11_ = (_loc11_ * 0.5);
				_loc11_ = (_loc11_ + 0.5);
				double _loc12_ = ((((_loc9_ * _loc11_) * _loc20_->intensity) * 2) * param1->deferredLightingStrength);
				this->lightConst[0] = (this->lightConst[0] + ((_loc12_ * ((_loc20_->color >> 16) & 0xFF)) / 0xFF));
				this->lightConst[1] = (this->lightConst[1] + ((_loc12_ * ((_loc20_->color >> 8) & 0xFF)) / 0xFF));
				this->lightConst[2] = (this->lightConst[2] + ((_loc12_ * (_loc20_->color & 0xFF)) / 0xFF));
				this->lighted = true;
			}
		}
		_loc4_++;
	}
	_loc4_ = 0;
	while (_loc4_ < param1->spotsCount)
	{
		SpotLight *_loc21_ = param1->spots[_loc4_];
		double _loc5_ = (_loc21_->cmd * _loc2_);
		double _loc6_ = (_loc21_->cmh * _loc3_);
		double _loc7_ = _loc21_->cml;
		double _loc8_ = _loc21_->attenuationEnd;
		if ((((((((_loc5_ - _loc8_) < _loc13_) && ((_loc5_ + _loc8_) > _loc13_)) && ((_loc6_ - _loc8_) < _loc14_)) && ((_loc6_ + _loc8_) > _loc14_)) && ((_loc7_ - _loc8_) < _loc15_)) && ((_loc7_ + _loc8_) > _loc15_)))
		{
			_loc5_ = (_loc5_ - _loc13_);
			_loc6_ = (_loc6_ - _loc14_);
			_loc7_ = (_loc7_ - _loc15_);
			_loc16_ = MyMath::sqrt((((_loc5_ * _loc5_) + (_loc6_ * _loc6_)) + (_loc7_ * _loc7_)));
			if (((_loc16_ > 0) && (_loc16_ < _loc8_)))
			{
				_loc5_ = (_loc5_ / _loc16_);
				_loc6_ = (_loc6_ / _loc16_);
				_loc7_ = (_loc7_ / _loc16_);
				double _loc22_ = ((((-(_loc5_) * _loc21_->cmc) * _loc2_) - ((_loc6_ * _loc21_->cmg) * _loc3_)) - (_loc7_ * _loc21_->cmk));
				double _loc23_ = MyMath::cos((_loc21_->falloff * 0.5));
				if (_loc22_ > _loc23_)
				{
					double _loc11_ = (((_loc5_ * _loc17_) + (_loc6_ * _loc18_)) + (_loc7_ * _loc19_));
					_loc11_ = (_loc11_ * 0.5);
					_loc11_ = (_loc11_ + 0.5);
					double _loc9_ = ((_loc8_ - _loc16_) / (_loc21_->attenuationEnd - _loc21_->attenuationBegin));
					if (_loc9_ > 1)
					{
						_loc9_ = 1;
					}
					if (_loc9_ < 0)
					{
						_loc9_ = 0;
					}
					_loc9_ = (_loc9_ * _loc9_);
					double _loc10_ = ((_loc22_ - _loc23_) / (MyMath::cos((_loc21_->hotspot * 0.5)) - _loc23_));
					if (_loc10_ > 1)
					{
						_loc10_ = 1;
					}
					if (_loc10_ < 0)
					{
						_loc10_ = 0;
					}
					_loc10_ = (_loc10_ * _loc10_);
					double _loc12_ = (((((_loc9_ * _loc10_) * _loc11_) * _loc21_->intensity) * 2) * param1->deferredLightingStrength);
					this->lightConst[0] = (this->lightConst[0] + ((_loc12_ * ((_loc21_->color >> 16) & 0xFF)) / 0xFF));
					this->lightConst[1] = (this->lightConst[1] + ((_loc12_ * ((_loc21_->color >> 8) & 0xFF)) / 0xFF));
					this->lightConst[2] = (this->lightConst[2] + ((_loc12_ * (_loc21_->color & 0xFF)) / 0xFF));
					this->lighted = true;
				}
			}
		}
		_loc4_++;
	}
	_loc4_ = 0;
	while (_loc4_ < param1->tubesCount)
	{
		TubeLight *_loc24_ = param1->tubes[_loc4_];
		double _loc25_ = (_loc24_->length * 0.5);
		double _loc26_ = (_loc25_ + _loc24_->falloff);
		double _loc27_ = (_loc24_->cmc * _loc2_);
		double _loc28_ = (_loc24_->cmg * _loc2_);
		double _loc29_ = _loc24_->cmk;
		double _loc5_ = ((_loc24_->cmd * _loc2_) + (_loc27_ * _loc25_));
		double _loc6_ = ((_loc24_->cmh * _loc3_) + (_loc28_ * _loc25_));
		double _loc7_ = (_loc24_->cml + (_loc29_ * _loc25_));
		double _loc30_ = (((_loc27_ * (_loc13_ - _loc5_)) + (_loc28_ * (_loc14_ - _loc6_))) + (_loc29_ * (_loc15_ - _loc7_)));
		if (((_loc30_ > -(_loc26_)) && (_loc30_ < _loc26_)))
		{
			_loc5_ = (_loc5_ + ((_loc27_ * _loc30_) - _loc13_));
			_loc6_ = (_loc6_ + ((_loc28_ * _loc30_) - _loc14_));
			_loc7_ = (_loc7_ + ((_loc29_ * _loc30_) - _loc15_));
			_loc16_ = MyMath::sqrt((((_loc5_ * _loc5_) + (_loc6_ * _loc6_)) + (_loc7_ * _loc7_)));
			if (((_loc16_ > 0) && (_loc16_ < _loc24_->attenuationEnd)))
			{
				_loc5_ = (_loc5_ / _loc16_);
				_loc6_ = (_loc6_ / _loc16_);
				_loc7_ = (_loc7_ / _loc16_);
				double _loc11_ = (((_loc5_ * _loc17_) + (_loc6_ * _loc18_)) + (_loc7_ * _loc19_));
				_loc11_ = (_loc11_ * 0.5);
				_loc11_ = (_loc11_ + 0.5);
				double _loc9_ = ((_loc24_->attenuationEnd - _loc16_) / (_loc24_->attenuationEnd - _loc24_->attenuationBegin));
				if (_loc9_ > 1)
				{
					_loc9_ = 1;
				}
				if (_loc9_ < 0)
				{
					_loc9_ = 0;
				}
				_loc9_ = (_loc9_ * _loc9_);
				if (_loc30_ < 0)
				{
					_loc30_ = -(_loc30_);
				}
				double _loc10_ = ((_loc26_ - _loc30_) / (_loc26_ - _loc25_));
				if (_loc10_ > 1)
				{
					_loc10_ = 1;
				}
				if (_loc10_ < 0)
				{
					_loc10_ = 0;
				}
				_loc10_ = (_loc10_ * _loc10_);
				double _loc12_ = (((((_loc9_ * _loc10_) * _loc11_) * _loc24_->intensity) * 2) * param1->deferredLightingStrength);
				this->lightConst[0] = (this->lightConst[0] + ((_loc12_ * ((_loc24_->color >> 16) & 0xFF)) / 0xFF));
				this->lightConst[1] = (this->lightConst[1] + ((_loc12_ * ((_loc24_->color >> 8) & 0xFF)) / 0xFF));
				this->lightConst[2] = (this->lightConst[2] + ((_loc12_ * (_loc24_->color & 0xFF)) / 0xFF));
				this->lighted = true;
			}
		}
		_loc4_++;
	}
}

std::shared_ptr<Face> Sprite3D::calculateFace(Camera3D *param1)
{
	std::shared_ptr<Vertex> _loc11_;
	std::shared_ptr<Vertex> _loc12_;


	culling = (culling & 0x3C);
	double _loc2_ = matrix.ml;
	if (((_loc2_ <= param1->nearClipping) || (_loc2_ >= param1->farClipping)))
	{
		return nullptr;
	}


	double _loc13_ = this->width;
	double _loc14_ = this->height;

#ifdef GRAPHICS
	double _loc15_ = this->bottomRightU - this->topLeftU;
	double _loc16_ = this->bottomRightV - this->topLeftV;

	Material *material = this->material.get();

	if (((this->autoSize) && (material->isType(TextureMaterial::TYPE))))
	{
		TextureMaterial *textureMaterial = (TextureMaterial*)material;

		BitmapData *bitmapData = textureMaterial->getTexturePtr();
		if (bitmapData != nullptr)
		{
			_loc13_ = (bitmapData->width() * _loc15_);
			_loc14_ = (bitmapData->height() * _loc16_);
		}
	}
#endif

	//double _loc17_ = (param1->viewSizeX / _loc2_);
	//double _loc18_ = (param1->viewSizeY / _loc2_);
	double _loc19_ = (param1->focalLength / _loc2_);
	double _loc20_ = (param1->focalLength / param1->viewSizeX);
	double _loc21_ = (param1->focalLength / param1->viewSizeY);
	double _loc3_ = (matrix.ma / _loc20_);
	double _loc4_ = (matrix.me / _loc21_);
	double _loc22_ = MyMath::sqrt((((_loc3_ * _loc3_) + (_loc4_ * _loc4_)) + (matrix.mi * matrix.mi)));
	_loc3_ = (matrix.mb / _loc20_);
	_loc4_ = (matrix.mf / _loc21_);
	_loc22_ = (_loc22_ + MyMath::sqrt((((_loc3_ * _loc3_) + (_loc4_ * _loc4_)) + (matrix.mj * matrix.mj))));
	_loc3_ = (matrix.mc / _loc20_);
	_loc4_ = (matrix.mg / _loc21_);
	_loc22_ = (_loc22_ + MyMath::sqrt((((_loc3_ * _loc3_) + (_loc4_ * _loc4_)) + (matrix.mk * matrix.mk))));
	_loc22_ = (_loc22_ / 3.);

	if ((!(this->perspectiveScale)))
	{
		_loc22_ = (_loc22_ / _loc19_);
	}

	if (this->rotation == 0)
	{
		double _loc26_ = ((_loc22_ * _loc13_) * _loc20_);
		double _loc27_ = ((_loc22_ * _loc14_) * _loc21_);
		double _loc3_ = (matrix.md - (this->originX * _loc26_));
		double _loc4_ = (matrix.mh - (this->originY * _loc27_));
		double _loc7_ = (_loc3_ + _loc26_);
		double _loc8_ = (_loc4_ + _loc27_);

		if (((culling > 0) && ((((_loc3_ > _loc2_) || (_loc4_ > _loc2_)) || (_loc7_ < -(_loc2_))) || (_loc8_ < -(_loc2_)))))
		{
			return nullptr;
		}

		_loc11_ = Vertex::createList(4);
		_loc12_ = _loc11_;
		_loc12_->cameraX = _loc3_;
		_loc12_->cameraY = _loc4_;
		_loc12_->cameraZ = _loc2_;
		_loc12_->u = this->topLeftU;
		_loc12_->v = this->topLeftV;
		_loc12_ = _loc12_->next;
		_loc12_->cameraX = _loc3_;
		_loc12_->cameraY = _loc8_;
		_loc12_->cameraZ = _loc2_;
		_loc12_->u = this->topLeftU;
		_loc12_->v = this->bottomRightV;
		_loc12_ = _loc12_->next;
		_loc12_->cameraX = _loc7_;
		_loc12_->cameraY = _loc8_;
		_loc12_->cameraZ = _loc2_;
		_loc12_->u = this->bottomRightU;
		_loc12_->v = this->bottomRightV;
		_loc12_ = _loc12_->next;
		_loc12_->cameraX = _loc7_;
		_loc12_->cameraY = _loc4_;
		_loc12_->cameraZ = _loc2_;
		_loc12_->u = this->bottomRightU;
		_loc12_->v = this->topLeftV;
	}
	else
	{
		double _loc28_ = (-(MyMath::sin(this->rotation)) * _loc22_);
		double _loc29_ = (MyMath::cos(this->rotation) * _loc22_);
		double _loc30_ = ((_loc29_ * _loc13_) * _loc20_);
		double _loc31_ = ((-(_loc28_) * _loc13_) * _loc21_);
		double _loc32_ = ((_loc28_ * _loc14_) * _loc20_);
		double _loc33_ = ((_loc29_ * _loc14_) * _loc21_);
		_loc3_ = ((matrix.md - (this->originX * _loc30_)) - (this->originY * _loc32_));
		_loc4_ = ((matrix.mh - (this->originX * _loc31_)) - (this->originY * _loc33_));
		double _loc5_ = (_loc3_ + _loc32_);
		double _loc6_ = (_loc4_ + _loc33_);
		double _loc7_ = ((_loc3_ + _loc30_) + _loc32_);
		double _loc8_ = ((_loc4_ + _loc31_) + _loc33_);
		double _loc9_ = (_loc3_ + _loc30_);
		double _loc10_ = (_loc4_ + _loc31_);
		if (culling > 0)
		{
			if (this->clipping == 1)
			{
				if ((((((culling & 0x04) && (_loc2_ <= -(_loc3_))) && (_loc2_ <= -(_loc5_))) && (_loc2_ <= -(_loc7_))) && (_loc2_ <= -(_loc9_))))
				{
					return nullptr;
				}
				if ((((((culling & 0x08) && (_loc2_ <= _loc3_)) && (_loc2_ <= _loc5_)) && (_loc2_ <= _loc7_)) && (_loc2_ <= _loc9_)))
				{
					return nullptr;
				}
				if ((((((culling & 0x10) && (_loc2_ <= -(_loc4_))) && (_loc2_ <= -(_loc6_))) && (_loc2_ <= -(_loc8_))) && (_loc2_ <= -(_loc10_))))
				{
					return nullptr;
				}
				if ((((((culling & 0x20) && (_loc2_ <= _loc4_)) && (_loc2_ <= _loc6_)) && (_loc2_ <= _loc8_)) && (_loc2_ <= _loc10_)))
				{
					return nullptr;
				}
				_loc11_ = Vertex::createList(4);
				_loc12_ = _loc11_;
				_loc12_->cameraX = _loc3_;
				_loc12_->cameraY = _loc4_;
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->topLeftU;
				_loc12_->v = this->topLeftV;
				_loc12_ = _loc12_->next;
				_loc12_->cameraX = (_loc3_ + _loc32_);
				_loc12_->cameraY = (_loc4_ + _loc33_);
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->topLeftU;
				_loc12_->v = this->bottomRightV;
				_loc12_ = _loc12_->next;
				_loc12_->cameraX = ((_loc3_ + _loc30_) + _loc32_);
				_loc12_->cameraY = ((_loc4_ + _loc31_) + _loc33_);
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->bottomRightU;
				_loc12_->v = this->bottomRightV;
				_loc12_ = _loc12_->next;
				_loc12_->cameraX = (_loc3_ + _loc30_);
				_loc12_->cameraY = (_loc4_ + _loc31_);
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->bottomRightU;
				_loc12_->v = this->topLeftV;
			}
			else
			{
				if ((culling & 0x04))
				{
					if (((((_loc2_ <= -(_loc3_)) && (_loc2_ <= -(_loc5_))) && (_loc2_ <= -(_loc7_))) && (_loc2_ <= -(_loc9_))))
					{
						return nullptr;
					}
					if (((((_loc2_ > -(_loc3_)) && (_loc2_ > -(_loc5_))) && (_loc2_ > -(_loc7_))) && (_loc2_ > -(_loc9_))))
					{
						culling = (culling & 0x3B);
					}
				}
				if ((culling & 0x08))
				{
					if (((((_loc2_ <= _loc3_) && (_loc2_ <= _loc5_)) && (_loc2_ <= _loc7_)) && (_loc2_ <= _loc9_)))
					{
						return nullptr;
					}
					if (((((_loc2_ > _loc3_) && (_loc2_ > _loc5_)) && (_loc2_ > _loc7_)) && (_loc2_ > _loc9_)))
					{
						culling = (culling & 0x37);
					}
				}
				if ((culling & 0x10))
				{
					if (((((_loc2_ <= -(_loc4_)) && (_loc2_ <= -(_loc6_))) && (_loc2_ <= -(_loc8_))) && (_loc2_ <= -(_loc10_))))
					{
						return nullptr;
					}
					if (((((_loc2_ > -(_loc4_)) && (_loc2_ > -(_loc6_))) && (_loc2_ > -(_loc8_))) && (_loc2_ > -(_loc10_))))
					{
						culling = (culling & 0x2F);
					}
				}
				if ((culling & 0x20))
				{
					if (((((_loc2_ <= _loc4_) && (_loc2_ <= _loc6_)) && (_loc2_ <= _loc8_)) && (_loc2_ <= _loc10_)))
					{
						return nullptr;
					}
					if (((((_loc2_ > _loc4_) && (_loc2_ > _loc6_)) && (_loc2_ > _loc8_)) && (_loc2_ > _loc10_)))
					{
						culling = (culling & 0x1F);
					}
				}
				_loc11_ = Vertex::createList(4);
				_loc12_ = _loc11_;
				_loc12_->cameraX = _loc3_;
				_loc12_->cameraY = _loc4_;
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->topLeftU;
				_loc12_->v = this->topLeftV;
				_loc12_ = _loc12_->next;
				_loc12_->cameraX = (_loc3_ + _loc32_);
				_loc12_->cameraY = (_loc4_ + _loc33_);
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->topLeftU;
				_loc12_->v = this->bottomRightV;
				_loc12_ = _loc12_->next;
				_loc12_->cameraX = ((_loc3_ + _loc30_) + _loc32_);
				_loc12_->cameraY = ((_loc4_ + _loc31_) + _loc33_);
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->bottomRightU;
				_loc12_->v = this->bottomRightV;
				_loc12_ = _loc12_->next;
				_loc12_->cameraX = (_loc3_ + _loc30_);
				_loc12_->cameraY = (_loc4_ + _loc31_);
				_loc12_->cameraZ = _loc2_;
				_loc12_->u = this->bottomRightU;
				_loc12_->v = this->topLeftV;
			}
		}
		else
		{
			_loc11_ = Vertex::createList(4);
			_loc12_ = _loc11_;
			_loc12_->cameraX = _loc3_;
			_loc12_->cameraY = _loc4_;
			_loc12_->cameraZ = _loc2_;
			_loc12_->u = this->topLeftU;
			_loc12_->v = this->topLeftV;
			_loc12_ = _loc12_->next;
			_loc12_->cameraX = (_loc3_ + _loc32_);
			_loc12_->cameraY = (_loc4_ + _loc33_);
			_loc12_->cameraZ = _loc2_;
			_loc12_->u = this->topLeftU;
			_loc12_->v = this->bottomRightV;
			_loc12_ = _loc12_->next;
			_loc12_->cameraX = ((_loc3_ + _loc30_) + _loc32_);
			_loc12_->cameraY = ((_loc4_ + _loc31_) + _loc33_);
			_loc12_->cameraZ = _loc2_;
			_loc12_->u = this->bottomRightU;
			_loc12_->v = this->bottomRightV;
			_loc12_ = _loc12_->next;
			_loc12_->cameraX = (_loc3_ + _loc30_);
			_loc12_->cameraY = (_loc4_ + _loc31_);
			_loc12_->cameraZ = _loc2_;
			_loc12_->u = this->bottomRightU;
			_loc12_->v = this->topLeftV;
		}
	}
	param1->lastVertex->next = _loc11_;
	param1->lastVertex = _loc12_;
	std::shared_ptr<Face> _loc23_ = Face::create();
	_loc23_->material = this->material;
	param1->lastFace->next = _loc23_;
	param1->lastFace = _loc23_;
	std::shared_ptr<Wrapper> _loc24_ = Wrapper::create(26);
	_loc23_->wrapper = _loc24_;
	_loc24_->vertex = _loc11_;
	_loc11_ = _loc11_->next;

	Wrapper *_loc24_1_ = _loc24_.get();

	while (_loc11_ != nullptr)
	{
		_loc24_1_->setNext(Wrapper::create(27));
		_loc24_1_ = _loc24_1_->_next;
		_loc24_1_->vertex = _loc11_;
		_loc11_ = _loc11_->next;
	}

	this->faces.append(_loc23_);
	return _loc23_;
}

void Sprite3D::updateBounds(Object3D *param1, Object3D *param2)
{
	double _loc12_;
	double _loc13_;
	double _loc14_;
	double _loc15_;

	double _loc3_ = this->width;
	double _loc4_ = this->height;
#ifdef GRAPHICS
	if (this->autoSize && this->material->isType(TextureMaterial::TYPE))
	{
		TextureMaterial *textureMaterial = (TextureMaterial*)this->material.get();
		BitmapData *_loc11_ = textureMaterial->getTexturePtr();

		//BitmapData *_loc11_ = (this->material as TextureMaterial).texture;
		if (_loc11_ != nullptr)
		{
			_loc3_ = (_loc11_->width() * (this->bottomRightU - this->topLeftU));
			_loc4_ = (_loc11_->height() * (this->bottomRightV - this->topLeftV));
		}
	}
#endif
	double _loc5_ = (((this->originX >= 0.5) ? this->originX : (1 - this->originX)) * _loc3_);
	double _loc6_ = (((this->originY >= 0.5) ? this->originY : (1 - this->originY)) * _loc4_);
	double _loc7_ = MyMath::sqrt(((_loc5_ * _loc5_) + (_loc6_ * _loc6_)));
	double _loc8_ = 0;
	double _loc9_ = 0;
	double _loc10_ = 0;

	if (param2 != nullptr)
	{
		_loc12_ = param2->matrix.ma;
		_loc13_ = param2->matrix.me;
		_loc14_ = param2->matrix.mi;
		_loc15_ = MyMath::sqrt((((_loc12_ * _loc12_) + (_loc13_ * _loc13_)) + (_loc14_ * _loc14_)));
		_loc12_ = param2->matrix.mb;
		_loc13_ = param2->matrix.mf;
		_loc14_ = param2->matrix.mj;
		_loc15_ = (_loc15_ + MyMath::sqrt((((_loc12_ * _loc12_) + (_loc13_ * _loc13_)) + (_loc14_ * _loc14_))));
		_loc12_ = param2->matrix.mc;
		_loc13_ = param2->matrix.mg;
		_loc14_ = param2->matrix.mk;
		_loc15_ = (_loc15_ + MyMath::sqrt((((_loc12_ * _loc12_) + (_loc13_ * _loc13_)) + (_loc14_ * _loc14_))));
		_loc7_ = (_loc7_ * (_loc15_ / 3));
		_loc8_ = param2->matrix.md;
		_loc9_ = param2->matrix.mh;
		_loc10_ = param2->matrix.ml;
	}
	if ((_loc8_ - _loc7_) < param1->bound.MinX)
	{
		param1->bound.MinX = (_loc8_ - _loc7_);
	}
	if ((_loc8_ + _loc7_) > param1->bound.MaxX)
	{
		param1->bound.MaxX = (_loc8_ + _loc7_);
	}
	if ((_loc9_ - _loc7_) < param1->bound.MinY)
	{
		param1->bound.MinY = (_loc9_ - _loc7_);
	}
	if ((_loc9_ + _loc7_) > param1->bound.MaxY)
	{
		param1->bound.MaxY = (_loc9_ + _loc7_);
	}
	if ((_loc10_ - _loc7_) < param1->bound.MinZ)
	{
		param1->bound.MinZ = (_loc10_ - _loc7_);
	}
	if ((_loc10_ + _loc7_) > param1->bound.MaxZ)
	{
		param1->bound.MaxZ = (_loc10_ + _loc7_);
	}
}


void Sprite3D::destroy()
{
	this->material = nullptr;
	this->faces.clear();

	/*
	if (this->material != nullptr)
	{
		this->material->dispose();
		this->material = nullptr;
	}

	for (std::shared_ptr<Face> &face : this->faces)
	{
		face->destroy();
	}
	*/
}

std::shared_ptr<Sprite3D> new_Sprite3D(double width, double height, std::shared_ptr<Material> material)
{
	return std::make_shared<Sprite3D>(Object3D::TT(Sprite3D::TYPE), width, height, material);
}

std::shared_ptr<Sprite3D> Sprite3D_clone(const std::shared_ptr<Sprite3D> &in)
{
	if (in->isType(Sprite3D::TYPE)) {
		Sprite3D *src = in.get();
		auto out = new_Sprite3D(src->width, src->height, src->material);
		out->clonePropertiesFromSprite3D(src);
		return out;
	}

	throw 4113244;
}
