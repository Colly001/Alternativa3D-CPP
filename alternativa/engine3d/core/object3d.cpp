#include "object3d.h"
#include "_global.h"
#include "mymath.h"
#include "vertex.h"
#include "flash/geom/colortransform.h"
#include "object3dcontainer.h"
#include "camera3d.h"
#include <QString>
#include <QDebug>


const Object3D::T *Object3D::TYPE = new Object3D::T("Object3D");



std::shared_ptr<Vertex> Object3D::boundVertexList = Vertex::createList(8);


Object3D::Object3D(const TT &t) :
	__type(t.type),
	__type_first(t.type),
	__type_next(t.type_next)
{
	_function_name("Object3D::Object3D");

	//concatenatedAlpha = 1;
	this->_parent = nullptr;
	this->bound.infinity();
	this->position.reset(0, 0, 0);
	this->rotation.reset(0, 0, 0);
	this->scale.reset(1, 1, 1);
	this->alpha = 1.;
	this->visible = true;
	this->_name = new QString();
	this->matrix.toIdentity();
	this->matrix_i.toIdentity();
	this->shadowMapAlphaThreshold = 0;
	this->useLight = true;
	this->transformId = 0;
	this->culling = 0;
	this->concatenatedAlpha = 1;
	this->colorTransform = nullptr;
	this->concatenatedColorTransform = nullptr;
	this->useShadowMap = false;
	this->softAttenuation = 0;
	this->useDepth = false;
	this->depthMapAlphaThreshold = 1;
	this->mouseEnabled = true;

	for (int i = 0; i < 8; i++)
		colorConst[i] = 0;

	concatenatedBlendMode = BlendMode::NORMAL;
	blendMode = BlendMode::NORMAL;
}

Object3D::~Object3D()
{
	_function_name("Object3D::~Object3D");
	delete _name;
}

void Object3D::getMatrix3D(Matrix3D *matrix3D) const
{
	(void)matrix3D;
	/*
	Object3D o;
	o.composeMatrixFromSource(this);

	matrix3D->set(o.matrix.ma, o.matrix.me, o.matrix.mi, 0, o.matrix.mb,
				  o.matrix.mf, o.matrix.mj, 0, o.matrix.mc, o.matrix.mg,
				  o.matrix.mk, 0, o.matrix.md, o.matrix.mh, o.matrix.ml, 1);
				  */
}

void Object3D::setMatrix3D(const Matrix3D *)
{
	//var _loc2_:Vector.<Vector3D> = param1->decompose();
	//var position:Vector3D = _loc2_[0];
	//var rotation:Vector3D = _loc2_[1];
	//var scale:Vector3D = _loc2_[2];
	/*
	this->position.x = position.x;
	this->position.y = position.y;
	this->position.z = position.z;
	this->rotation.x = rotation.x;
	this->rotation.y = rotation.y;
	this->rotation.z = rotation.z;
	this->scale.x = scale.x;
	this->scale.y = scale.y;
	this->scale.z = scale.z;
	*/
}


void Object3D::composeMatrixFromSource(const Object3D *param1)
{
	_function_name("Object3D::composeMatrixFromSource");

	double _loc2_ = MyMath::cos(param1->rotation.x);
	double _loc3_ = MyMath::sin(param1->rotation.x);
	double _loc4_ = MyMath::cos(param1->rotation.y);
	double _loc5_ = MyMath::sin(param1->rotation.y);
	double _loc6_ = MyMath::cos(param1->rotation.z);
	double _loc7_ = MyMath::sin(param1->rotation.z);
	double _loc8_ = (_loc6_ * _loc5_);
	double _loc9_ = (_loc7_ * _loc5_);
	double _loc10_ = (_loc4_ * param1->scale.x);
	double _loc11_ = (_loc3_ * param1->scale.y);
	double _loc12_ = (_loc2_ * param1->scale.y);
	double _loc13_ = (_loc2_ * param1->scale.z);
	double _loc14_ = (_loc3_ * param1->scale.z);
	this->matrix.ma = (_loc6_ * _loc10_);
	this->matrix.mb = ((_loc8_ * _loc11_) - (_loc7_ * _loc12_));
	this->matrix.mc = ((_loc8_ * _loc13_) + (_loc7_ * _loc14_));
	this->matrix.md = param1->position.x;
	this->matrix.me = (_loc7_ * _loc10_);
	this->matrix.mf = ((_loc9_ * _loc11_) + (_loc6_ * _loc12_));
	this->matrix.mg = ((_loc9_ * _loc13_) - (_loc6_ * _loc14_));
	this->matrix.mh = param1->position.y;
	this->matrix.mi = (-(_loc5_) * param1->scale.x);
	this->matrix.mj = (_loc4_ * _loc11_);
	this->matrix.mk = (_loc4_ * _loc13_);
	this->matrix.ml = param1->position.z;
}

void Object3D::composeMatrix()
{
	_function_name("Object3D::composeMatrix");

	double cosX = MyMath::cos(this->rotation.x);
	double sinX = MyMath::sin(this->rotation.x);
	double cosY = MyMath::cos(this->rotation.y);
	double sinY = MyMath::sin(this->rotation.y);
	double cosZ = MyMath::cos(this->rotation.z);
	double sinZ = MyMath::sin(this->rotation.z);
	double cosZsinY = cosZ * sinY;
	double sinZsinY = sinZ * sinY;
	double cosYscaleX = cosY * this->scale.x;
	double sinXscaleY = sinX * this->scale.y;
	double cosXscaleY = cosX * this->scale.y;
	double cosXscaleZ = cosX * this->scale.z;
	double sinXscaleZ = sinX * this->scale.z;
	this->matrix.ma = cosZ * cosYscaleX;
	this->matrix.mb = (cosZsinY * sinXscaleY) - (sinZ * cosXscaleY);
	this->matrix.mc = (cosZsinY * cosXscaleZ) + (sinZ * sinXscaleZ);
	this->matrix.md = this->position.x;
	this->matrix.me = sinZ * cosYscaleX;
	this->matrix.mf = (sinZsinY * sinXscaleY) + (cosZ * cosXscaleY);
	this->matrix.mg = (sinZsinY * cosXscaleZ) - (cosZ * sinXscaleZ);
	this->matrix.mh = this->position.y;
	this->matrix.mi = -sinY * this->scale.x;
	this->matrix.mj = cosY * sinXscaleY;
	this->matrix.mk = cosY * cosXscaleZ;
	this->matrix.ml = this->position.z;
}


void Object3D::appendMatrix(Object3D *transform)
{
	_function_name("Object3D::appendMatrix");

	double ta = this->matrix.ma;
	double tb = this->matrix.mb;
	double tc = this->matrix.mc;
	double td = this->matrix.md;
	double te = this->matrix.me;
	double tf = this->matrix.mf;
	double tg = this->matrix.mg;
	double th = this->matrix.mh;
	double ti = this->matrix.mi;
	double tj = this->matrix.mj;
	double tk = this->matrix.mk;
	double tl = this->matrix.ml;
	this->matrix.ma = ((transform->matrix.ma * ta) + (transform->matrix.mb * te)) + (transform->matrix.mc * ti);
	this->matrix.mb = ((transform->matrix.ma * tb) + (transform->matrix.mb * tf)) + (transform->matrix.mc * tj);
	this->matrix.mc = ((transform->matrix.ma * tc) + (transform->matrix.mb * tg)) + (transform->matrix.mc * tk);
	this->matrix.md = (((transform->matrix.ma * td) + (transform->matrix.mb * th)) + (transform->matrix.mc * tl)) + transform->matrix.md;
	this->matrix.me = ((transform->matrix.me * ta) + (transform->matrix.mf * te)) + (transform->matrix.mg * ti);
	this->matrix.mf = ((transform->matrix.me * tb) + (transform->matrix.mf * tf)) + (transform->matrix.mg * tj);
	this->matrix.mg = ((transform->matrix.me * tc) + (transform->matrix.mf * tg)) + (transform->matrix.mg * tk);
	this->matrix.mh = (((transform->matrix.me * td) + (transform->matrix.mf * th)) + (transform->matrix.mg * tl)) + transform->matrix.mh;
	this->matrix.mi = ((transform->matrix.mi * ta) + (transform->matrix.mj * te)) + (transform->matrix.mk * ti);
	this->matrix.mj = ((transform->matrix.mi * tb) + (transform->matrix.mj * tf)) + (transform->matrix.mk * tj);
	this->matrix.mk = ((transform->matrix.mi * tc) + (transform->matrix.mj * tg)) + (transform->matrix.mk * tk);
	this->matrix.ml = (((transform->matrix.mi * td) + (transform->matrix.mj * th)) + (transform->matrix.mk * tl)) + transform->matrix.ml;
}

void Object3D::draw(Camera3D*)
{

}

object_ptr<VG> Object3D::getVG(Camera3D*)
{
	return nullptr;
}

void Object3D::collectPlanes(Vector3*, Vector3*, Vector3*, Vector3*, Vector3*, void*, Dictionary*)
{

}

bool Object3D::checkIntersection(double, double, double, double, double, double, double, Dictionary *)
{
	return false;
}

bool Object3D::intersectRay(const Vector3*, const Vector3*, Object3DFilter*, Camera3D*, RayIntersectionData*)
{
	return false;
}

void Object3D::split(Vector3 *, Vector3 *, Vector3 *, double, Object3D **result_1, Object3D **result_2)
{
	*result_1 = nullptr;
	*result_2 = nullptr;
}

void Object3D::composeAndAppend(Object3D *param1)
{
	_function_name("Object3D::composeAndAppend");

	double _loc2_= MyMath::cos(this->rotation.x);
	double _loc3_= MyMath::sin(this->rotation.x);
	double _loc4_= MyMath::cos(this->rotation.y);
	double _loc5_= MyMath::sin(this->rotation.y);
	double _loc6_= MyMath::cos(this->rotation.z);
	double _loc7_= MyMath::sin(this->rotation.z);
	double _loc8_= (_loc6_ * _loc5_);
	double _loc9_= (_loc7_ * _loc5_);
	double _loc10_= (_loc4_ * this->scale.x);
	double _loc11_= (_loc3_ * this->scale.y);
	double _loc12_= (_loc2_ * this->scale.y);
	double _loc13_= (_loc2_ * this->scale.z);
	double _loc14_= (_loc3_ * this->scale.z);
	double _loc15_= (_loc6_ * _loc10_);
	double _loc16_= ((_loc8_ * _loc11_) - (_loc7_ * _loc12_));
	double _loc17_= ((_loc8_ * _loc13_) + (_loc7_ * _loc14_));
	double _loc18_= this->position.x;
	double _loc19_= (_loc7_ * _loc10_);
	double _loc20_= ((_loc9_ * _loc11_) + (_loc6_ * _loc12_));
	double _loc21_= ((_loc9_ * _loc13_) - (_loc6_ * _loc14_));
	double _loc22_= this->position.y;
	double _loc23_= (-(_loc5_) * this->scale.x);
	double _loc24_= (_loc4_ * _loc11_);
	double _loc25_= (_loc4_ * _loc13_);
	double _loc26_= this->position.z;
	this->matrix.ma = (((param1->matrix.ma * _loc15_) + (param1->matrix.mb * _loc19_)) + (param1->matrix.mc * _loc23_));
	this->matrix.mb = (((param1->matrix.ma * _loc16_) + (param1->matrix.mb * _loc20_)) + (param1->matrix.mc * _loc24_));
	this->matrix.mc = (((param1->matrix.ma * _loc17_) + (param1->matrix.mb * _loc21_)) + (param1->matrix.mc * _loc25_));
	this->matrix.md = ((((param1->matrix.ma * _loc18_) + (param1->matrix.mb * _loc22_)) + (param1->matrix.mc * _loc26_)) + param1->matrix.md);
	this->matrix.me = (((param1->matrix.me * _loc15_) + (param1->matrix.mf * _loc19_)) + (param1->matrix.mg * _loc23_));
	this->matrix.mf = (((param1->matrix.me * _loc16_) + (param1->matrix.mf * _loc20_)) + (param1->matrix.mg * _loc24_));
	this->matrix.mg = (((param1->matrix.me * _loc17_) + (param1->matrix.mf * _loc21_)) + (param1->matrix.mg * _loc25_));
	this->matrix.mh = ((((param1->matrix.me * _loc18_) + (param1->matrix.mf * _loc22_)) + (param1->matrix.mg * _loc26_)) + param1->matrix.mh);
	this->matrix.mi = (((param1->matrix.mi * _loc15_) + (param1->matrix.mj * _loc19_)) + (param1->matrix.mk * _loc23_));
	this->matrix.mj = (((param1->matrix.mi * _loc16_) + (param1->matrix.mj * _loc20_)) + (param1->matrix.mk * _loc24_));
	this->matrix.mk = (((param1->matrix.mi * _loc17_) + (param1->matrix.mj * _loc21_)) + (param1->matrix.mk * _loc25_));
	this->matrix.ml = ((((param1->matrix.mi * _loc18_) + (param1->matrix.mj * _loc22_)) + (param1->matrix.mk * _loc26_)) + param1->matrix.ml);
}


void Object3D::calculateBounds()
{
	_function_name("Object3D::calculateBounds");

	this->bound.minX = 1E22;
	this->bound.minY = 1E22;
	this->bound.minZ = 1E22;
	this->bound.maxX = -1E22;
	this->bound.maxY = -1E22;
	this->bound.maxZ = -1E22;
	this->updateBounds(this, nullptr);
	if (this->bound.minX > this->bound.maxX)
	{
		this->bound.minX = -1E22;
		this->bound.minY = -1E22;
		this->bound.minZ = -1E22;
		this->bound.maxX = 1E22;
		this->bound.maxY = 1E22;
		this->bound.maxZ = 1E22;
	}
}

void Object3D::updateBounds(Object3D*, Object3D*)
{

}

void Object3D::setName(const QString &name)
{
	_function_name("Object3D::setName");
	*_name = name;
}

QString Object3D::getName() const
{
	_function_name("Object3D::getName");
	return *_name;
}

bool Object3D::compareName(const QString &name) const
{
	return *this->_name == name;
}

void Object3D::setParent(Object3DContainer *container)
{
	_parent = container;
}

void Object3D::clonePropertiesFromObject3D(const Object3D *param1)
{
	_function_name("Object3D::clonePropertiesFrom");

	this->setName(param1->getName());
	this->visible = param1->visible;
	this->alpha = param1->alpha;
	(this->blendMode = param1->blendMode);
	this->mouseEnabled = param1->mouseEnabled;
	//(this->doubleClickEnabled = param1->doubleClickEnabled);
	//(this->useHandCursor = param1->useHandCursor);
	//(this->depthMapAlphaThreshold = param1->depthMapAlphaThreshold);
	this->shadowMapAlphaThreshold = param1->shadowMapAlphaThreshold;
	this->softAttenuation = param1->softAttenuation;
	this->useShadowMap = param1->useShadowMap;
	this->useLight = param1->useLight;
	this->transformId = param1->transformId;
	//(this->distance = param1->distance);

	if (param1->colorTransform != nullptr)
	{
		this->colorTransform = std::make_shared<ColorTransform>();
		this->colorTransform->concat(param1->colorTransform.get());
	}
	/*
	if (param1->filters != nullptr)
	{
		(this->filters = new Array().concat(param1->filters));
	}
	*/

	this->position.copy(&param1->position);
	this->rotation.copy(&param1->rotation);
	this->scale.copy(&param1->scale);
	this->bound.copyFrom(&param1->bound);
}


void Object3D::concat(Object3DContainer *param1)
{
	_function_name("Object3D::concat");

	this->concatenatedAlpha = param1->concatenatedAlpha * this->alpha;
	this->concatenatedBlendMode = (!(param1->concatenatedBlendMode == BlendMode::NORMAL)) ? param1->concatenatedBlendMode : this->blendMode;

	if (param1->concatenatedColorTransform != nullptr)
	{
		if (this->colorTransform != nullptr)
		{
			this->concatenatedColorTransform = std::make_shared<ColorTransform>();
			this->concatenatedColorTransform->redMultiplier = param1->concatenatedColorTransform->redMultiplier;
			this->concatenatedColorTransform->greenMultiplier = param1->concatenatedColorTransform->greenMultiplier;
			this->concatenatedColorTransform->blueMultiplier = param1->concatenatedColorTransform->blueMultiplier;
			this->concatenatedColorTransform->redOffset = param1->concatenatedColorTransform->redOffset;
			this->concatenatedColorTransform->greenOffset = param1->concatenatedColorTransform->greenOffset;
			this->concatenatedColorTransform->blueOffset = param1->concatenatedColorTransform->blueOffset;
			this->concatenatedColorTransform->concat(this->colorTransform.get());
		}
		else
		{
			this->concatenatedColorTransform = param1->concatenatedColorTransform;
		}
	}
	else
	{
		this->concatenatedColorTransform = this->colorTransform;
	}

	if (this->concatenatedColorTransform != nullptr)
	{
		this->colorConst[0] = this->concatenatedColorTransform->redMultiplier;
		this->colorConst[1] = this->concatenatedColorTransform->greenMultiplier;
		this->colorConst[2] = this->concatenatedColorTransform->blueMultiplier;
		this->colorConst[3] = this->concatenatedAlpha;
		this->colorConst[4] = this->concatenatedColorTransform->redOffset / 0xFF;
		this->colorConst[5] = this->concatenatedColorTransform->greenOffset / 0xFF;
		this->colorConst[6] = this->concatenatedColorTransform->blueOffset / 0xFF;
	}
	else
	{
		this->colorConst[3] = this->concatenatedAlpha;
	}
}



int Object3D::cullingInCamera(Camera3D *param1, int param2)
{
	_function_name("Object3D::cullingInCamera");

	if (param1->occludedAll)
	{
		return -1;
	}


	int _loc3_ = param1->numOccluders;
	if (param2 > 0 || _loc3_ > 0)
	{
		Vertex *_loc4_ = boundVertexList.get();
		(_loc4_->x = this->bound.MinX);
		(_loc4_->y = this->bound.MinY);
		(_loc4_->z = this->bound.MinZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MaxX);
		(_loc4_->y = this->bound.MinY);
		(_loc4_->z = this->bound.MinZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MinX);
		(_loc4_->y = this->bound.MaxY);
		(_loc4_->z = this->bound.MinZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MaxX);
		(_loc4_->y = this->bound.MaxY);
		(_loc4_->z = this->bound.MinZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MinX);
		(_loc4_->y = this->bound.MinY);
		(_loc4_->z = this->bound.MaxZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MaxX);
		(_loc4_->y = this->bound.MinY);
		(_loc4_->z = this->bound.MaxZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MinX);
		(_loc4_->y = this->bound.MaxY);
		(_loc4_->z = this->bound.MaxZ);
		(_loc4_ = _loc4_->next.get());
		(_loc4_->x = this->bound.MaxX);
		(_loc4_->y = this->bound.MaxY);
		(_loc4_->z = this->bound.MaxZ);
		(_loc4_ = boundVertexList.get());
		while (_loc4_ != nullptr)
		{
			double x = _loc4_->x;
			double y = _loc4_->y;
			double z = _loc4_->z;
			_loc4_->cameraX = ((((this->matrix.ma * x) + (this->matrix.mb * y)) + (this->matrix.mc * z)) + this->matrix.md);
			_loc4_->cameraY = ((((this->matrix.me * x) + (this->matrix.mf * y)) + (this->matrix.mg * z)) + this->matrix.mh);
			_loc4_->cameraZ = ((((this->matrix.mi * x) + (this->matrix.mj * y)) + (this->matrix.mk * z)) + this->matrix.ml);
			_loc4_ = _loc4_->next.get();
		}
	}


	if (param2 > 0)
	{
		if ((param2 & 0x01))
		{
			double _loc10_ = param1->nearClipping;
			Vertex *_loc4_ = boundVertexList.get();
			bool _loc8_ = false;
			bool _loc9_ = false;
			while (_loc4_ != nullptr)
			{
				if (_loc4_->cameraZ > _loc10_)
				{
					(_loc8_ = true);
					if (_loc9_)
					{
						break;
					}
				}
				else
				{
					(_loc9_ = true);
					if (_loc8_)
					{
						break;
					}
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc9_)
			{
				if (!_loc8_)
				{
					return -1;
				}
			}
			else
			{
				(param2 = (param2 & 0x3E));
			}
		}
		if ((param2 & 0x02))
		{
			double _loc11_ = param1->farClipping;
			Vertex *_loc4_ = boundVertexList.get();
			bool _loc8_ = false;
			bool _loc9_ = false;
			while (_loc4_ != nullptr)
			{
				if (_loc4_->cameraZ < _loc11_)
				{
					_loc8_ = true;
					if (_loc9_)
					{
						break;
					}
				}
				else
				{
					(_loc9_ = true);
					if (_loc8_)
					{
						break;
					}
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc9_)
			{
				if (!_loc8_)
				{
					return -1;
				}
			}
			else
			{
				param2 = param2 & 0x3D;
			}
		}
		if (param2 & 0x04)
		{
			Vertex *_loc4_ = boundVertexList.get();
			bool _loc8_ = false;
			bool _loc9_ = false;
			while (_loc4_ != nullptr)
			{
				if (-(_loc4_->cameraX) < _loc4_->cameraZ)
				{
					(_loc8_ = true);
					if (_loc9_)
					{
						break;
					}
				}
				else
				{
					_loc9_ = true;
					if (_loc8_)
					{
						break;
					}
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc9_)
			{
				if (!_loc8_)
				{
					return -1;
				}
			}
			else
			{
				(param2 = (param2 & 0x3B));
			}
		}

		if ((param2 & 0x08))
		{
			Vertex *_loc4_ = boundVertexList.get();
			bool _loc8_ = false;
			bool _loc9_ = false;
			while (_loc4_ != nullptr)
			{
				if (_loc4_->cameraX < _loc4_->cameraZ)
				{
					_loc8_ = true;
					if (_loc9_)
					{
						break;
					}
				}
				else
				{
					_loc9_ = true;
					if (_loc8_)
					{
						break;
					}
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc9_)
			{
				if (!_loc8_)
				{
					return -1;
				}
			}
			else
			{
				param2 = param2 & 0x37;
			}
		}

		if (param2 & 0x10)
		{
			Vertex *_loc4_ = boundVertexList.get();
			bool _loc8_ = false;
			bool _loc9_ = false;
			while (_loc4_ != nullptr)
			{
				if (-_loc4_->cameraY < _loc4_->cameraZ)
				{
					_loc8_ = true;
					if (_loc9_)
					{
						break;
					}
				}
				else
				{
					_loc9_ = true;
					if (_loc8_)
					{
						break;
					}
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc9_)
			{
				if (!_loc8_)
				{
					return -1;
				}
			}
			else
			{
				param2 = param2 & 0x2F;
			}
		}

		if (param2 & 0x20)
		{
			Vertex *_loc4_ = boundVertexList.get();
			bool _loc8_ = false;
			bool _loc9_ = false;
			while (_loc4_ != nullptr)
			{
				if (_loc4_->cameraY < _loc4_->cameraZ)
				{
					_loc8_ = true;
					if (_loc9_)
					{
						break;
					}
				}
				else
				{
					_loc9_ = true;
					if (_loc8_)
					{
						break;
					}
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc9_)
			{
				if (!_loc8_)
				{
					return (-1);
				}
			}
			else
			{
				param2 = param2 & 0x1F;
			}
		}
	}

	if (_loc3_ > 0)
	{
		int _loc12_ = 0;
		while (true)
		{
			if (_loc12_ < _loc3_)
			{
				Vertex *_loc13_ = param1->occluders.at(_loc12_).get();
				while (_loc13_ != nullptr)
				{
					Vertex *_loc4_ = boundVertexList.get();
					while (_loc4_ != nullptr)
					{
						if ((((_loc13_->cameraX * _loc4_->cameraX) + (_loc13_->cameraY * _loc4_->cameraY)) + (_loc13_->cameraZ * _loc4_->cameraZ)) >= 0)
						{
							break;
						}
						_loc4_ = _loc4_->next.get();
					}
					if (_loc4_ != nullptr)
					{
						break;
					}
					_loc13_ = _loc13_->next.get();
				}
				if (_loc13_ == nullptr)
				{
					break;
				}
				_loc12_++;
			}
		}

		return -1;
	}

	this->culling = param2;
	return param2;
}

void Object3D::invertMatrix()
{
	_function_name("Object3D::invertMatrix");

	matrix.invert();
}

void Object3D::calculateInverseMatrix()
{
	_function_name("Object3D::calculateInverseMatrix");

	double _loc1_ = (1. / (((((((-(this->matrix.mc) * this->matrix.mf) * this->matrix.mi) + ((this->matrix.mb * this->matrix.mg) * this->matrix.mi)) + ((this->matrix.mc * this->matrix.me) * this->matrix.mj)) - ((this->matrix.ma * this->matrix.mg) * this->matrix.mj)) - ((this->matrix.mb * this->matrix.me) * this->matrix.mk)) + ((this->matrix.ma * this->matrix.mf) * this->matrix.mk)));
	(this->matrix_i.ma = (((-(this->matrix.mg) * this->matrix.mj) + (this->matrix.mf * this->matrix.mk)) * _loc1_));
	(this->matrix_i.mb = (((this->matrix.mc * this->matrix.mj) - (this->matrix.mb * this->matrix.mk)) * _loc1_));
	(this->matrix_i.mc = (((-(this->matrix.mc) * this->matrix.mf) + (this->matrix.mb * this->matrix.mg)) * _loc1_));
	(this->matrix_i.md = ((((((((this->matrix.md * this->matrix.mg) * this->matrix.mj) - ((this->matrix.mc * this->matrix.mh) * this->matrix.mj)) - ((this->matrix.md * this->matrix.mf) * this->matrix.mk)) + ((this->matrix.mb * this->matrix.mh) * this->matrix.mk)) + ((this->matrix.mc * this->matrix.mf) * this->matrix.ml)) - ((this->matrix.mb * this->matrix.mg) * this->matrix.ml)) * _loc1_));
	(this->matrix_i.me = (((this->matrix.mg * this->matrix.mi) - (this->matrix.me * this->matrix.mk)) * _loc1_));
	(this->matrix_i.mf = (((-(this->matrix.mc) * this->matrix.mi) + (this->matrix.ma * this->matrix.mk)) * _loc1_));
	(this->matrix_i.mg = (((this->matrix.mc * this->matrix.me) - (this->matrix.ma * this->matrix.mg)) * _loc1_));
	(this->matrix_i.mh = ((((((((this->matrix.mc * this->matrix.mh) * this->matrix.mi) - ((this->matrix.md * this->matrix.mg) * this->matrix.mi)) + ((this->matrix.md * this->matrix.me) * this->matrix.mk)) - ((this->matrix.ma * this->matrix.mh) * this->matrix.mk)) - ((this->matrix.mc * this->matrix.me) * this->matrix.ml)) + ((this->matrix.ma * this->matrix.mg) * this->matrix.ml)) * _loc1_));
	(this->matrix_i.mi = (((-(this->matrix.mf) * this->matrix.mi) + (this->matrix.me * this->matrix.mj)) * _loc1_));
	(this->matrix_i.mj = (((this->matrix.mb * this->matrix.mi) - (this->matrix.ma * this->matrix.mj)) * _loc1_));
	(this->matrix_i.mk = (((-(this->matrix.mb) * this->matrix.me) + (this->matrix.ma * this->matrix.mf)) * _loc1_));
	(this->matrix_i.ml = ((((((((this->matrix.md * this->matrix.mf) * this->matrix.mi) - ((this->matrix.mb * this->matrix.mh) * this->matrix.mi)) - ((this->matrix.md * this->matrix.me) * this->matrix.mj)) + ((this->matrix.ma * this->matrix.mh) * this->matrix.mj)) + ((this->matrix.mb * this->matrix.me) * this->matrix.ml)) - ((this->matrix.ma * this->matrix.mf) * this->matrix.ml)) * _loc1_));
}

void Object3D::prepareResources()
{

}

double Object3D::calculateResolution(int, int, int, Matrix3D*)
{
	return 1;
}

void Object3D::copyAndAppend(Object3D *param1, Object3D *param2)
{
	_function_name("Object3D::copyAndAppend");

	this->matrix.ma = (((param2->matrix.ma * param1->matrix.ma) + (param2->matrix.mb * param1->matrix.me)) + (param2->matrix.mc * param1->matrix.mi));
	this->matrix.mb = (((param2->matrix.ma * param1->matrix.mb) + (param2->matrix.mb * param1->matrix.mf)) + (param2->matrix.mc * param1->matrix.mj));
	this->matrix.mc = (((param2->matrix.ma * param1->matrix.mc) + (param2->matrix.mb * param1->matrix.mg)) + (param2->matrix.mc * param1->matrix.mk));
	this->matrix.md = ((((param2->matrix.ma * param1->matrix.md) + (param2->matrix.mb * param1->matrix.mh)) + (param2->matrix.mc * param1->matrix.ml)) + param2->matrix.md);
	this->matrix.me = (((param2->matrix.me * param1->matrix.ma) + (param2->matrix.mf * param1->matrix.me)) + (param2->matrix.mg * param1->matrix.mi));
	this->matrix.mf = (((param2->matrix.me * param1->matrix.mb) + (param2->matrix.mf * param1->matrix.mf)) + (param2->matrix.mg * param1->matrix.mj));
	this->matrix.mg = (((param2->matrix.me * param1->matrix.mc) + (param2->matrix.mf * param1->matrix.mg)) + (param2->matrix.mg * param1->matrix.mk));
	this->matrix.mh = ((((param2->matrix.me * param1->matrix.md) + (param2->matrix.mf * param1->matrix.mh)) + (param2->matrix.mg * param1->matrix.ml)) + param2->matrix.mh);
	this->matrix.mi = (((param2->matrix.mi * param1->matrix.ma) + (param2->matrix.mj * param1->matrix.me)) + (param2->matrix.mk * param1->matrix.mi));
	this->matrix.mj = (((param2->matrix.mi * param1->matrix.mb) + (param2->matrix.mj * param1->matrix.mf)) + (param2->matrix.mk * param1->matrix.mj));
	this->matrix.mk = (((param2->matrix.mi * param1->matrix.mc) + (param2->matrix.mj * param1->matrix.mg)) + (param2->matrix.mk * param1->matrix.mk));
	this->matrix.ml = ((((param2->matrix.mi * param1->matrix.md) + (param2->matrix.mj * param1->matrix.mh)) + (param2->matrix.mk * param1->matrix.ml)) + param2->matrix.ml);
}

void Object3D::destroy()
{

}

void Object3D::removeFromParent()
{
	_function_name("Object3D::removeFromParent");

	if (this->_parent != nullptr)
	{
		this->_parent->removeChild(this);
		this->_parent = nullptr;
	}
}

bool Object3D::boundIntersectRay(const Vector3 *param1, const Vector3 *param2, double param3, double param4, double param5, double param6, double param7, double param8)
{
	_function_name("Object3D::boundIntersectRay");

	double _loc9_;
	double _loc10_;
	double _loc11_;
	double _loc12_;

	if (((((((param1->x >= param3) && (param1->x <= param6)) && (param1->y >= param4)) && (param1->y <= param7)) && (param1->z >= param5)) && (param1->z <= param8)))
	{
		return true;
	}

	if ((((((((param1->x < param3) && (param2->x <= 0)) || ((param1->x > param6) && (param2->x >= 0))) || ((param1->y < param4) && (param2->y <= 0))) || ((param1->y > param7) && (param2->y >= 0))) || ((param1->z < param5) && (param2->z <= 0))) || ((param1->z > param8) && (param2->z >= 0))))
	{
		return false;
	}

	double _loc13_ = 1E-6;
	if (param2->x > _loc13_)
	{
		(_loc9_ = ((param3 - param1->x) / param2->x));
		(_loc10_ = ((param6 - param1->x) / param2->x));
	}
	else
	{
		if (param2->x < -(_loc13_))
		{
			(_loc9_ = ((param6 - param1->x) / param2->x));
			(_loc10_ = ((param3 - param1->x) / param2->x));
		}
		else
		{
			(_loc9_ = 0);
			(_loc10_ = 1E22);
		};
	};
	if (param2->y > _loc13_)
	{
		(_loc11_ = ((param4 - param1->y) / param2->y));
		(_loc12_ = ((param7 - param1->y) / param2->y));
	}
	else
	{
		if (param2->y < -(_loc13_))
		{
			(_loc11_ = ((param7 - param1->y) / param2->y));
			(_loc12_ = ((param4 - param1->y) / param2->y));
		}
		else
		{
			(_loc11_ = 0);
			(_loc12_ = 1E22);
		}
	}
	if (((_loc11_ >= _loc10_) || (_loc12_ <= _loc9_)))
	{
		return (false);
	}
	if (_loc11_ < _loc9_)
	{
		if (_loc12_ < _loc10_)
		{
			(_loc10_ = _loc12_);
		}
	}
	else
	{
		(_loc9_ = _loc11_);
		if (_loc12_ < _loc10_)
		{
			(_loc10_ = _loc12_);
		}
	}
	if (param2->z > _loc13_)
	{
		(_loc11_ = ((param5 - param1->z) / param2->z));
		(_loc12_ = ((param8 - param1->z) / param2->z));
	}
	else
	{
		if (param2->z < -(_loc13_))
		{
			(_loc11_ = ((param8 - param1->z) / param2->z));
			(_loc12_ = ((param5 - param1->z) / param2->z));
		}
		else
		{
			(_loc11_ = 0);
			(_loc12_ = 1E22);
		}
	}
	if (((_loc11_ >= _loc10_) || (_loc12_ <= _loc9_)))
	{
		return false;
	}
	return true;
}

std::shared_ptr<Object3D> new_Object3D()
{
	_function_name("new_Object3D");

	return std::make_shared<Object3D>(Object3D::TT(Object3D::TYPE));
}


std::shared_ptr<Object3D> Object3D_clone(const std::shared_ptr<Object3D> &in)
{
	_function_name("Object3D_clone");

	if (in->isType(Object3D::TYPE)) {
		auto out = new_Object3D();
		out->clonePropertiesFromObject3D(in.get());
		return out;
	}

	throw 4113241;
}
