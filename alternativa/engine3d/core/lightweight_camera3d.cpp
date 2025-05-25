#include "camera3d.h"
#include "vertex.h"
#include "face.h"
#include "wrapper.h"
#include "mymath.h"
#include "IMain.h"
#include "service.h"
#include "_global.h"
#include "alternativa/engine3d/materials/material.h"
#include "alternativa/engine3d/core/object3dcontainer.h"
#include "view.h"






const Object3D::T *Camera3D::TYPE = new Object3D::T("Camera3D");


uint32_t Camera3D::renderId = 0;

static const int constantsAttributesCount = 8;
static const int constantsOffset = 16;
static const int constantsMaxTriangles = 18;


Camera3D::Camera3D(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	_function_name("Camera3D::Camera3D");

	fov = 1.5707963267949;
	//numTriangles = 0;
	//numPolygons = 0;

	//this->_diagram = this->createDiagram();

	this->firstVertex = Vertex::create();
	this->firstFace = Face::create();
	this->firstWrapper = Wrapper::create(0);
	this->lastWrapper = this->firstWrapper;
	this->lastVertex = this->firstVertex;
	this->lastFace = this->firstFace;

	this->transparentCount = 0;
	this->deferredLighting = false;
	this->deferredLightingStrength = 1;
	this->debug = false;
	this->omniesCount = 0;
	this->spotsCount = 0;
	this->directionalLight = nullptr;
	this->shadowMapStrength = 1;
	this->directionalLightStrength = 1;
	this->shadowMap = nullptr;
	this->tubesCount = 0;
	this->farClipping = 1000000;
	this->nearClipping = 1;
	this->occludedAll = false;
	this->numOccluders = 0;
	this->numDraws = 0;
	this->numShadows = 0;
	this->numTriangles = 0;
	this->lightsLength = 0;
	this->shadowsStrength = 1;
	this->decalsCount = 0;
	this->opaqueCount = 0;
	this->transparentOpaqueCount = 0;
	this->skyCount = 0;
	this->depthCount = 0;
	this->depthBufferScale = 1;
	this->softTransparency = false;
	this->softTransparencyStrength = 1;
	this->ssao = false;
	this->ssaoStrength = 1;
	this->lightMap = nullptr;
	this->casterCount = 0;
	this->ambientColor = 0;
	this->depthMap = nullptr;
	this->ssaoAlpha = 1;
	this->ssaoColor = 0;
	this->fogNear = 0;
	this->fogFar = 1000000;
	this->fogColor = 0x7F7F7F;
	this->fogAlpha = 0;
	this->fogStrength = 1;



	this->lightTransform[0] = 0;
	this->lightTransform[1] = 0;
	this->lightTransform[2] = 0;
	this->lightTransform[3] = 1;

	//fragmentConst:Vector.<Number> = Vector.<Number>([0, 0, 0, 1, 0.5, 0.5, 0, (1 / 0x1000)]);
	this->fragmentConst[0] = 0;
	this->fragmentConst[1] = 0;
	this->fragmentConst[2] = 0;
	this->fragmentConst[3] = 1;
	this->fragmentConst[4] = 0.5;
	this->fragmentConst[5] = 0.5;
	this->fragmentConst[6] = 0;
	this->fragmentConst[7] = 1. / 0x1000;

	//depthParams:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	this->depthParams[0] = 0;
	this->depthParams[1] = 0;
	this->depthParams[2] = 0;
	this->depthParams[3] = 1;


	//lightParams:Vector.<Number> = Vector.<Number>([0, 0, 0, 1, 0, 0, 0, 1]);
	this->lightParams[0] = 0;
	this->lightParams[1] = 0;
	this->lightParams[2] = 0;
	this->lightParams[3] = 1;
	this->lightParams[4] = 0;
	this->lightParams[5] = 0;
	this->lightParams[6] = 0;
	this->lightParams[7] = 1;


	//fogFragment:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	this->fogFragment[0] = 0;
	this->fogFragment[1] = 0;
	this->fogFragment[2] = 0;
	this->fogFragment[3] = 1;

	//ssaoParams:Vector.<Number> = Vector.<Number>([0, 0, 0, 1]);
	this->ssaoParams[0] = 0;
	this->ssaoParams[1] = 0;
	this->ssaoParams[2] = 0;
	this->ssaoParams[3] = 1;

	//fogParams:Vector.<Number> = Vector.<Number>([1, 1, 0, 1]);
	this->fogParams[0] = 1;
	this->fogParams[1] = 1;
	this->fogParams[2] = 0;
	this->fogParams[3] = 1;
}

Camera3D::~Camera3D()
{
	if (this->firstVertex != nullptr)
		this->firstVertex->destroy();

	if (this->firstFace != nullptr)
		this->firstFace->destroy();

	if (this->firstWrapper != nullptr)
		Wrapper::destroy(this->firstWrapper);

	//delete this->firstVertex;
	//delete this->firstFace;
	//delete this->firstWrapper;
}

void Camera3D::setViewSize(double w, double h)
{
	_function_name("Camera3D::setViewSize");
	this->viewSizeX = w * 0.5;
	this->viewSizeY = h * 0.5;
}

void Camera3D::addTransparent(std::shared_ptr<Face> &face, std::shared_ptr<Object3D> &object3D)
{
	//this->transparentFaceLists[this->transparentCount] = param1;
	//this->transparentObjects[this->transparentCount] = param2;
	transparentFaceLists.append(face.get());
	transparentObjects.append(object3D.get());
	this->transparentCount++;
}


void Camera3D::render()
{
	this->numDraws = 0;
	this->numShadows = 0;
	this->numTriangles = 0;
	if (this->view != nullptr)
	{

		renderId++;
		this->device = this->view->device;
		this->view->configure();

		if (this->nearClipping < 1)
			this->nearClipping = 1;

		if (this->farClipping > 1000000)
			this->farClipping = 1000000;

		this->viewSizeX = this->view->_width * 0.5;
		this->viewSizeY = this->view->_height * 0.5;
		this->focalLength = MyMath::sqrt(((this->viewSizeX * this->viewSizeX) + (this->viewSizeY * this->viewSizeY))) / MyMath::tan(this->fov * 0.5);
		this->correctionX = (this->viewSizeX / this->focalLength);
		this->correctionY = (this->viewSizeY / this->focalLength);

		this->projection[0] = (1 << this->view->zBufferPrecision);
		this->projection[1] = 1;
		this->projection[2] = this->farClipping / (this->farClipping - this->nearClipping);
		this->projection[3] = (this->nearClipping * this->farClipping) / (this->nearClipping - this->farClipping);

		Object3D *_loc5_ = this;


		while (_loc5_->_parent != nullptr)
		{
			_loc5_ = _loc5_->_parent;
			_loc5_->composeMatrix();
			appendMatrix(_loc5_);
		}

		this->matrix_g.copy(&this->matrix);

		this->invertMatrix();
		this->transform[0] = this->matrix.ma;
		this->transform[1] = this->matrix.mb;
		this->transform[2] = this->matrix.mc;
		this->transform[3] = this->matrix.md;
		this->transform[4] = this->matrix.me;
		this->transform[5] = this->matrix.mf;
		this->transform[6] = this->matrix.mg;
		this->transform[7] = this->matrix.mh;
		this->transform[8] = this->matrix.mi;
		this->transform[9] = this->matrix.mj;
		this->transform[10] = this->matrix.mk;
		this->transform[11] = this->matrix.ml;
		this->numOccluders = 0;
		this->occludedAll = false;


		this->composeCameraMatrix();
		matrix.invert();

		Service::getIMain()->draw();
	}
}




void Camera3D::composeCameraMatrix()
{
	_function_name("Camera3D::composeCameraMatrix");

	double _loc1_ = this->viewSizeX / this->focalLength;
	double _loc2_ = this->viewSizeY / this->focalLength;
	double _loc3_ = MyMath::cos(rotation.x);
	double _loc4_ = MyMath::sin(rotation.x);
	double _loc5_ = MyMath::cos(rotation.y);
	double _loc6_ = MyMath::sin(rotation.y);
	double _loc7_ = MyMath::cos(rotation.z);
	double _loc8_ = MyMath::sin(rotation.z);
	double _loc9_ = _loc7_ * _loc6_;
	double _loc10_ = _loc8_ * _loc6_;
	double _loc11_ = _loc5_;
	double _loc12_ = _loc4_;
	double _loc13_ = _loc3_;
	double _loc14_ = _loc3_;
	double _loc15_ = _loc4_;
	matrix.ma = _loc7_ * _loc11_ * _loc1_;
	matrix.mb = (_loc9_ * _loc12_ - _loc8_ * _loc13_) * _loc2_;
	matrix.mc = _loc9_ * _loc14_ + _loc8_ * _loc15_;
	matrix.md = position.x;
	matrix.me = _loc8_ * _loc11_ * _loc1_;
	matrix.mf = (_loc10_ * _loc12_ + _loc7_ * _loc13_) * _loc2_;
	matrix.mg = _loc10_ * _loc14_ - _loc7_ * _loc15_;
	matrix.mh = position.y;
	matrix.mi = -_loc6_ * _loc1_;
	matrix.mj = _loc5_ * _loc12_ * _loc2_;
	matrix.mk = _loc5_ * _loc14_;
	matrix.ml = position.z;
	double _loc16_ = 0. / this->viewSizeX;
	double _loc17_ = 0. / this->viewSizeY;
	matrix.mc = matrix.mc - (matrix.ma * _loc16_ + matrix.mb * _loc17_);
	matrix.mg = matrix.mg - (matrix.me * _loc16_ + matrix.mf * _loc17_);
	matrix.mk = matrix.mk - (matrix.mi * _loc16_ + matrix.mj * _loc17_);
}



int Camera3D::checkInDebug(std::shared_ptr<Object3D> &param1)
{
	(void)param1;
	return 0;
	/*
	int _loc2_ = 0;
	int _loc3_ = 1;

	while (_loc3_ <= 0x0200)
	{
		if (this->debugSet[_loc3_])
		{
			if (((this->debugSet[_loc3_][Object3D]) || (this->debugSet[_loc3_][param1])))
			{
				_loc2_ = (_loc2_ | _loc3_);
			}
			else
			{
				Class _loc4_ = (getDefinitionByName(getQualifiedClassName(param1)) as Class);
				while (_loc4_ != Object3D)
				{
					if (this->debugSet[_loc3_][_loc4_])
					{
						(_loc2_ = (_loc2_ | _loc3_));
						break;
					}
					_loc4_ = Class(getDefinitionByName(getQualifiedSuperclassName(_loc4_)));
				}
			}
		}
		_loc3_ = _loc3_ << 1;
	}

	return _loc2_;
	*/
}


Face* Camera3D::sortByAverageZ(Face*)
{
	return nullptr;
}

Face *Camera3D::sortByDynamicBSP(Face*, double, Face*)
{
	return nullptr;
}

void Camera3D::clonePropertiesFromCamera3D(Camera3D *src)
{
	clonePropertiesFromObject3D(src);
}

void Camera3D::addDecal(Decal *decal)
{
	if (this->decals.length() < (this->decalsCount + 1))
		this->decals.resize(this->decalsCount + 1);

	this->decals[this->decalsCount] = decal;
	this->decalsCount++;
}

void Camera3D::addShadow(Shadow *shadow)
{
	(void)shadow;
	//this->shadows[shadow] = true;
}


void Camera3D::drawTransparentList(Face*, Object3D*, bool)
{

}


void Camera3D::deferredDestroy()
{
	Face *_loc1_ = this->firstFace->next.get();
	while (_loc1_ != nullptr)
	{
		Wrapper *_loc2_ = _loc1_->wrapper.get();
		if (_loc2_ != nullptr)
		{
			Wrapper *_loc3_ = nullptr;
			while (_loc2_ != nullptr)
			{
				_loc2_->vertex = nullptr;
				_loc3_ = _loc2_;
				_loc2_ = _loc2_->_next;
			}
			this->lastWrapper->setNext(_loc1_->wrapper);
			this->lastWrapper = _loc3_->ptr();
		}
		_loc1_->material = nullptr;
		_loc1_->wrapper = nullptr;
		_loc1_ = _loc1_->next.get();
	}

	if (this->firstFace != this->lastFace)
	{
		this->lastFace->next = Face::collector;
		Face::collector = this->firstFace->next;
		this->firstFace->next = nullptr;
		this->lastFace = this->firstFace;
	}

	if (this->firstWrapper != this->lastWrapper)
	{
		this->lastWrapper->setNext(Wrapper::collector);
		Wrapper::collector = this->firstWrapper->next();
		this->firstWrapper->setNext(nullptr);
		this->lastWrapper = this->firstWrapper;
	}

	if (this->firstVertex != this->lastVertex)
	{
		this->lastVertex->next = Vertex::collector;
		Vertex::collector = this->firstVertex->next;
		this->firstVertex->next = nullptr;
		this->lastVertex = this->firstVertex;
	}
}

void Camera3D::clearOccluders()
{
	for (int i = 0; i < this->numOccluders; i++)
	{
		std::shared_ptr<Vertex> _loc2_ = this->occluders[i];
		std::shared_ptr<Vertex> _loc3_ = _loc2_;
		while (_loc3_->next != nullptr)
		{
			_loc3_ = _loc3_->next;
		}
		_loc3_->next = Vertex::collector;
		Vertex::collector = _loc2_;
		this->occluders[i] = nullptr;
	}
	this->numOccluders = 0;
}




void Camera3D::addOpaque(Material *param1, VertexBufferResource *param2, IndexBufferResource *param3, int param4, int param5, Object3D *object)
{
	if (this->opaqueMaterials.length() < (this->opaqueCount+1))
		this->opaqueMaterials.resize(this->opaqueCount+1);

	if (this->opaqueVertexBuffers.length() < (this->opaqueCount+1))
		this->opaqueVertexBuffers.resize(this->opaqueCount+1);

	if (this->opaqueIndexBuffers.length() < (this->opaqueCount+1))
		this->opaqueIndexBuffers.resize(this->opaqueCount+1);

	if (this->opaqueFirstIndexes.length() < (this->opaqueCount+1))
		this->opaqueFirstIndexes.resize(this->opaqueCount+1);

	if (this->opaqueNumsTriangles.length() < (this->opaqueCount+1))
		this->opaqueNumsTriangles.resize(this->opaqueCount+1);

	if (this->opaqueNumsTriangles.length() < (this->opaqueCount+1))
		this->opaqueNumsTriangles.resize(this->opaqueCount+1);

	if (this->opaqueObjects.length() < (this->opaqueCount+1))
		this->opaqueObjects.resize(this->opaqueCount+1);

	this->opaqueMaterials[this->opaqueCount] = param1;
	this->opaqueVertexBuffers[this->opaqueCount] = param2;
	this->opaqueIndexBuffers[this->opaqueCount] = param3;
	this->opaqueFirstIndexes[this->opaqueCount] = param4;
	this->opaqueNumsTriangles[this->opaqueCount] = param5;
	this->opaqueObjects[this->opaqueCount] = object;
	this->opaqueCount++;
}


std::shared_ptr<Face> Camera3D::cull(std::shared_ptr<Face> param1, int param2)
{
	std::shared_ptr<Face> _loc3_;
	std::shared_ptr<Face> _loc4_;
	std::shared_ptr<Face> _loc5_;


	double _loc12_ = MyMath::nan(); //nan
	double _loc13_ = MyMath::nan(); //nan
	double _loc15_ = MyMath::nan(); //nan
	double _loc16_ = MyMath::nan(); //nan
	double _loc18_ = MyMath::nan(); //nan
	double _loc19_ = MyMath::nan(); //nan

	bool _loc21_ = ((param2 & 0x01) > 0);
	bool _loc22_ = ((param2 & 0x02) > 0);
	bool _loc23_ = ((param2 & 0x04) > 0);
	bool _loc24_ = ((param2 & 0x08) > 0);
	bool _loc25_ = ((param2 & 0x10) > 0);
	bool _loc26_ = ((param2 & 0x20) > 0);
	bool _loc27_ = this->nearClipping;
	bool _loc28_ = this->farClipping;
	bool _loc29_ = ((_loc23_) || (_loc24_));
	bool _loc30_ = ((_loc25_) || (_loc26_));

	std::shared_ptr<Face> _loc31_ = param1;

	for (; _loc31_ != nullptr; _loc31_ = _loc5_)
	{
		_loc5_ = _loc31_->processNext;
		Wrapper *_loc9_ = _loc31_->wrapper.get();
		Vertex *_loc6_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc7_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc8_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		if (_loc29_)
		{
			(_loc12_ = _loc6_->cameraX);
			(_loc15_ = _loc7_->cameraX);
			(_loc18_ = _loc8_->cameraX);
		}
		if (_loc30_)
		{
			(_loc13_ = _loc6_->cameraY);
			(_loc16_ = _loc7_->cameraY);
			(_loc19_ = _loc8_->cameraY);
		}
		double _loc14_ = _loc6_->cameraZ;
		double _loc17_ = _loc7_->cameraZ;
		double _loc20_ = _loc8_->cameraZ;

		if (_loc21_)
		{
			if ((((_loc14_ <= _loc27_) || (_loc17_ <= _loc27_)) || (_loc20_ <= _loc27_)))
			{
				_loc31_->processNext = nullptr;
				continue;
			}
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				if (_loc11_->vertex->cameraZ <= _loc27_)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ != nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (((((_loc22_) && (_loc14_ >= _loc28_)) && (_loc17_ >= _loc28_)) && (_loc20_ >= _loc28_)))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				if (_loc11_->vertex->cameraZ < _loc28_)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}

		if (((((_loc23_) && (_loc14_ <= -(_loc12_))) && (_loc17_ <= -(_loc15_))) && (_loc20_ <= -(_loc18_))))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (-(_loc10_->cameraX) < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}

		if (((((_loc24_) && (_loc14_ <= _loc12_)) && (_loc17_ <= _loc15_)) && (_loc20_ <= _loc18_)))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (_loc10_->cameraX < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (((((_loc25_) && (_loc14_ <= -(_loc13_))) && (_loc17_ <= -(_loc16_))) && (_loc20_ <= -(_loc19_))))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (-(_loc10_->cameraY) < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (((((_loc26_) && (_loc14_ <= _loc13_)) && (_loc17_ <= _loc16_)) && (_loc20_ <= _loc19_)))
		{
			Wrapper *_loc11_ = _loc9_;
			while (_loc11_ != nullptr)
			{
				Vertex *_loc10_ = _loc11_->vertex.get();
				if (_loc10_->cameraY < _loc10_->cameraZ)
				{
					break;
				}
				_loc11_ = _loc11_->_next;
			}
			if (_loc11_ == nullptr)
			{
				_loc31_->processNext = nullptr;
				continue;
			}
		}
		if (_loc3_ != nullptr)
		{
			_loc4_->processNext = _loc31_->ptr();
		}
		else
		{
			_loc3_ = _loc31_;
		}
		_loc4_ = _loc31_;
	}
	if (_loc4_ != nullptr)
	{
		_loc4_->processNext = nullptr;
	}
	return _loc3_;
}

std::shared_ptr<Face> Camera3D::clip(std::shared_ptr<Face> param1, int param2)
{
	double _loc16_ = MyMath::nan(); //nan
	double _loc17_ = MyMath::nan(); //nan
	double _loc19_ = MyMath::nan(); //nan
	double _loc20_ = MyMath::nan(); //nan
	double _loc22_ = MyMath::nan(); //nan
	double _loc23_ = MyMath::nan(); //nan

	std::shared_ptr<Face> _loc3_;
	std::shared_ptr<Face> _loc4_;
	std::shared_ptr<Face> _loc5_;



	std::shared_ptr<Wrapper> _loc12_;
	std::shared_ptr<Wrapper> _loc13_;


	int _loc35_ = 0;

	bool _loc25_ = ((param2 & 0x01) > 0);
	bool _loc26_ = ((param2 & 0x02) > 0);
	bool _loc27_ = ((param2 & 0x04) > 0);
	bool _loc28_ = ((param2 & 0x08) > 0);
	bool _loc29_ = ((param2 & 0x10) > 0);
	bool _loc30_ = ((param2 & 0x20) > 0);

	double _loc31_ = this->nearClipping;
	double _loc32_ = this->farClipping;
	bool _loc33_ = (_loc27_) || (_loc28_);
	bool _loc34_ = (_loc29_) || (_loc30_);
	std::shared_ptr<Face> _loc37_ = param1;

	for (; _loc37_ != nullptr; _loc37_ = _loc5_)
	{
		_loc5_ = _loc37_->processNext;
		Wrapper *_loc9_ = _loc37_->wrapper.get();
		Vertex *_loc6_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc7_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		Vertex *_loc8_ = _loc9_->vertex.get();
		_loc9_ = _loc9_->_next;
		if (_loc33_)
		{
			(_loc16_ = _loc6_->cameraX);
			(_loc19_ = _loc7_->cameraX);
			(_loc22_ = _loc8_->cameraX);
		}
		if (_loc34_)
		{
			(_loc17_ = _loc6_->cameraY);
			(_loc20_ = _loc7_->cameraY);
			(_loc23_ = _loc8_->cameraY);
		}
		double _loc18_ = _loc6_->cameraZ;
		double _loc21_ = _loc7_->cameraZ;
		double _loc24_ = _loc8_->cameraZ;
		(_loc35_ = 0);
		if (_loc25_)
		{
			if ((((_loc18_ <= _loc31_) && (_loc21_ <= _loc31_)) && (_loc24_ <= _loc31_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					if (_loc11_->vertex->cameraZ > _loc31_)
					{
						(_loc35_ = (_loc35_ | 0x01));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > _loc31_) && (_loc21_ > _loc31_)) && (_loc24_ > _loc31_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						if (_loc11_->vertex->cameraZ <= _loc31_)
						{
							_loc35_ = (_loc35_ | 0x01);
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					_loc35_ = (_loc35_ | 0x01);
				}
			}
		}
		if (_loc26_)
		{
			if ((((_loc18_ >= _loc32_) && (_loc21_ >= _loc32_)) && (_loc24_ >= _loc32_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					if (_loc11_->vertex->cameraZ < _loc32_)
					{
						(_loc35_ = (_loc35_ | 0x02));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ < _loc32_) && (_loc21_ < _loc32_)) && (_loc24_ < _loc32_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						if (_loc11_->vertex->cameraZ >= _loc32_)
						{
							(_loc35_ = (_loc35_ | 0x02));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x02));
				}
			}
		}
		if (_loc27_)
		{
			if ((((_loc18_ <= -(_loc16_)) && (_loc21_ <= -(_loc19_))) && (_loc24_ <= -(_loc22_))))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (-(_loc10_->cameraX) < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x04));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > -(_loc16_)) && (_loc21_ > -(_loc19_))) && (_loc24_ > -(_loc22_))))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (-(_loc10_->cameraX) >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x04));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x04));
				}
			}
		}
		if (_loc28_)
		{
			if ((((_loc18_ <= _loc16_) && (_loc21_ <= _loc19_)) && (_loc24_ <= _loc22_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (_loc10_->cameraX < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x08));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > _loc16_) && (_loc21_ > _loc19_)) && (_loc24_ > _loc22_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (_loc10_->cameraX >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x08));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x08));
				}
			}
		}
		if (_loc29_)
		{
			if ((((_loc18_ <= -(_loc17_)) && (_loc21_ <= -(_loc20_))) && (_loc24_ <= -(_loc23_))))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (-(_loc10_->cameraY) < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x10));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > -(_loc17_)) && (_loc21_ > -(_loc20_))) && (_loc24_ > -(_loc23_))))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (-(_loc10_->cameraY) >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x10));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x10));
				}
			}
		}
		if (_loc30_)
		{
			if ((((_loc18_ <= _loc17_) && (_loc21_ <= _loc20_)) && (_loc24_ <= _loc23_)))
			{
				Wrapper *_loc11_ = _loc9_;
				while (_loc11_ != nullptr)
				{
					Vertex *_loc10_ = _loc11_->vertex.get();
					if (_loc10_->cameraY < _loc10_->cameraZ)
					{
						(_loc35_ = (_loc35_ | 0x20));
						break;
					}
					_loc11_ = _loc11_->_next;
				}
				if (_loc11_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			else
			{
				if ((((_loc18_ > _loc17_) && (_loc21_ > _loc20_)) && (_loc24_ > _loc23_)))
				{
					Wrapper *_loc11_ = _loc9_;
					while (_loc11_ != nullptr)
					{
						Vertex *_loc10_ = _loc11_->vertex.get();
						if (_loc10_->cameraY >= _loc10_->cameraZ)
						{
							(_loc35_ = (_loc35_ | 0x20));
							break;
						}
						_loc11_ = _loc11_->_next;
					}
				}
				else
				{
					(_loc35_ = (_loc35_ | 0x20));
				}
			}
		}
		if (_loc35_ > 0)
		{
			bool _loc38_ = ((!(_loc37_->material == nullptr)) && (_loc37_->material->useVerticesNormals));
			_loc12_ = nullptr;
			_loc13_ = nullptr;
			Wrapper *_loc11_ = _loc37_->wrapper.get();
			while (_loc11_ != nullptr)
			{
				std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
				_loc15_->vertex = _loc11_->vertex;
				if (_loc12_ != nullptr)
				{
					_loc13_->setNext(_loc15_);
				}
				else
				{
					_loc12_ = _loc15_;
				}
				_loc13_ = _loc15_;
				_loc11_ = _loc11_->_next;
			}
			if ((_loc35_ & 0x01))
			{
				_loc6_ = _loc13_->vertex.get();
				_loc18_ = _loc6_->cameraZ;
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > _loc31_) && (_loc18_ <= _loc31_)) || ((_loc21_ <= _loc31_) && (_loc18_ > _loc31_))))
					{
						double _loc36_ = ((_loc31_ - _loc18_) / (_loc21_ - _loc18_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));
						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							_loc12_ = _loc15_;
						}
						_loc13_ = _loc15_;
					}
					if (_loc21_ > _loc31_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							(_loc12_ = _loc11_);
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x02))
			{
				_loc6_ = _loc13_->vertex.get();
				(_loc18_ = _loc6_->cameraZ);
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ < _loc32_) && (_loc18_ >= _loc32_)) || ((_loc21_ >= _loc32_) && (_loc18_ < _loc32_))))
					{
						double _loc36_ = ((_loc32_ - _loc18_) / (_loc21_ - _loc18_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));
						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							(_loc12_ = _loc15_);
						}
						(_loc13_ = _loc15_);
					}
					if (_loc21_ < _loc32_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x04))
			{
				_loc6_ = _loc13_->vertex.get();
				_loc16_ = _loc6_->cameraX;
				_loc18_ = _loc6_->cameraZ;
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc19_ = _loc7_->cameraX;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > -(_loc19_)) && (_loc18_ <= -(_loc16_))) || ((_loc21_ <= -(_loc19_)) && (_loc18_ > -(_loc16_)))))
					{
						double _loc36_ = ((_loc16_ + _loc18_) / (((_loc16_ + _loc18_) - _loc19_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc16_ + ((_loc19_ - _loc16_) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));
						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							(_loc12_ = _loc15_);
						}
						(_loc13_ = _loc15_);
					}
					if (_loc21_ > -(_loc19_))
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					_loc6_ = _loc7_;
					_loc16_ = _loc19_;
					_loc18_ = _loc21_;
					_loc11_ = _loc14_;
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x08))
			{
				(_loc6_ = _loc13_->vertex.get());
				(_loc16_ = _loc6_->cameraX);
				(_loc18_ = _loc6_->cameraZ);
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				(_loc12_ = nullptr);
				(_loc13_ = nullptr);
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc19_ = _loc7_->cameraX;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > _loc19_) && (_loc18_ <= _loc16_)) || ((_loc21_ <= _loc19_) && (_loc18_ > _loc16_))))
					{
						double _loc36_ = ((_loc18_ - _loc16_) / (((_loc18_ - _loc16_) + _loc19_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						this->lastVertex->next = _loc10_;
						this->lastVertex = _loc10_;
						(_loc10_->cameraX = (_loc16_ + ((_loc19_ - _loc16_) * _loc36_)));
						(_loc10_->cameraY = (_loc6_->cameraY + ((_loc7_->cameraY - _loc6_->cameraY) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));
						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
						_loc15_->vertex = _loc10_;
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							(_loc12_ = _loc15_);
						}
						(_loc13_ = _loc15_);
					}
					if (_loc21_ > _loc19_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							(_loc12_ = _loc11_);
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					(_loc6_ = _loc7_);
					(_loc16_ = _loc19_);
					(_loc18_ = _loc21_);
					(_loc11_ = _loc14_);
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			if ((_loc35_ & 0x10))
			{
				_loc6_ = _loc13_->vertex.get();
				_loc17_ = _loc6_->cameraY;
				_loc18_ = _loc6_->cameraZ;
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc20_ = _loc7_->cameraY;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > -(_loc20_)) && (_loc18_ <= -(_loc17_))) || ((_loc21_ <= -(_loc20_)) && (_loc18_ > -(_loc17_)))))
					{
						double _loc36_ = ((_loc17_ + _loc18_) / (((_loc17_ + _loc18_) - _loc20_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc17_ + ((_loc20_ - _loc17_) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));
						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
						(_loc15_->vertex = _loc10_);
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							(_loc12_ = _loc15_);
						}
						(_loc13_ = _loc15_);
					}
					if (_loc21_ > -(_loc20_))
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							_loc12_ = _loc11_;
						}
						_loc13_ = _loc11_;
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					(_loc6_ = _loc7_);
					(_loc17_ = _loc20_);
					(_loc18_ = _loc21_);
					(_loc11_ = _loc14_);
				}
				if (_loc12_ == nullptr)
				{
					(_loc37_->processNext = nullptr);
					continue;
				}
			}
			if ((_loc35_ & 0x20))
			{
				_loc6_ = _loc13_->vertex.get();
				(_loc17_ = _loc6_->cameraY);
				(_loc18_ = _loc6_->cameraZ);
				std::shared_ptr<Wrapper> _loc11_ = _loc12_;
				_loc12_ = nullptr;
				_loc13_ = nullptr;
				while (_loc11_ != nullptr)
				{
					std::shared_ptr<Wrapper> _loc14_ = _loc11_->next();
					_loc7_ = _loc11_->vertex.get();
					_loc20_ = _loc7_->cameraY;
					_loc21_ = _loc7_->cameraZ;
					if ((((_loc21_ > _loc20_) && (_loc18_ <= _loc17_)) || ((_loc21_ <= _loc20_) && (_loc18_ > _loc17_))))
					{
						double _loc36_ = ((_loc18_ - _loc17_) / (((_loc18_ - _loc17_) + _loc20_) - _loc21_));
						std::shared_ptr<Vertex> _loc10_ = Vertex::create();
						(this->lastVertex->next = _loc10_);
						(this->lastVertex = _loc10_);
						(_loc10_->cameraX = (_loc6_->cameraX + ((_loc7_->cameraX - _loc6_->cameraX) * _loc36_)));
						(_loc10_->cameraY = (_loc17_ + ((_loc20_ - _loc17_) * _loc36_)));
						(_loc10_->cameraZ = (_loc18_ + ((_loc21_ - _loc18_) * _loc36_)));
						(_loc10_->x = (_loc6_->x + ((_loc7_->x - _loc6_->x) * _loc36_)));
						(_loc10_->y = (_loc6_->y + ((_loc7_->y - _loc6_->y) * _loc36_)));
						(_loc10_->z = (_loc6_->z + ((_loc7_->z - _loc6_->z) * _loc36_)));
						(_loc10_->u = (_loc6_->u + ((_loc7_->u - _loc6_->u) * _loc36_)));
						(_loc10_->v = (_loc6_->v + ((_loc7_->v - _loc6_->v) * _loc36_)));
						if (_loc38_)
						{
							(_loc10_->normalX = (_loc6_->normalX + ((_loc7_->normalX - _loc6_->normalX) * _loc36_)));
							(_loc10_->normalY = (_loc6_->normalY + ((_loc7_->normalY - _loc6_->normalY) * _loc36_)));
							(_loc10_->normalZ = (_loc6_->normalZ + ((_loc7_->normalZ - _loc6_->normalZ) * _loc36_)));
						}
						std::shared_ptr<Wrapper> _loc15_ = Wrapper::create(0);
						(_loc15_->vertex = _loc10_);
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc15_);
						}
						else
						{
							(_loc12_ = _loc15_);
						}
						(_loc13_ = _loc15_);
					}
					if (_loc21_ > _loc20_)
					{
						if (_loc12_ != nullptr)
						{
							_loc13_->setNext(_loc11_);
						}
						else
						{
							(_loc12_ = _loc11_);
						}
						(_loc13_ = _loc11_);
						_loc11_->setNext(nullptr);
					}
					else
					{
						_loc11_->vertex = nullptr;
						_loc11_->setNext(Wrapper::collector);
						Wrapper::collector = _loc11_;
					}
					(_loc6_ = _loc7_);
					(_loc17_ = _loc20_);
					(_loc18_ = _loc21_);
					(_loc11_ = _loc14_);
				}
				if (_loc12_ == nullptr)
				{
					_loc37_->processNext = nullptr;
					continue;
				}
			}
			_loc37_->processNext = nullptr;
			std::shared_ptr<Face> _loc39_ = Face::create();
			(_loc39_->material = _loc37_->material);
			(this->lastFace->next = _loc39_);
			(this->lastFace = _loc39_);
			(_loc39_->wrapper = _loc12_);
			_loc37_ = _loc39_;
		}
		if (_loc3_ != nullptr)
		{
			(_loc4_->processNext = _loc37_);
		}
		else
		{
			_loc3_ = _loc37_;
		}
		_loc4_ = _loc37_;
	}
	if (_loc4_ != nullptr)
	{
		_loc4_->processNext = nullptr;
	}
	return _loc3_;
}

void Camera3D::addSky(Material*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*)
{

}

void Camera3D::removeShadow(Shadow*)
{

}
