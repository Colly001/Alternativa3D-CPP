#include "light3d.h"
#include "object3dcontainer.h"
#include "vertex.h"
#include "camera3d.h"


const Object3D::T *Light3D::TYPE = new Object3D::T("Light3D");


Light3D::Light3D(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	intensity = 1;
	nextLight = nullptr;
}

void Light3D::clonePropertiesFromLight3D(Light3D *src)
{
	clonePropertiesFromObject3D(src);
	this->color = src->color;
	this->intensity = src->intensity;
}


void Light3D::calculateCameraMatrix(Camera3D *camera)
{
	composeMatrix();
	Object3D *o = this;
	while (o->_parent != nullptr)
	{
		o = o->_parent;
		o->composeMatrix();
		appendMatrix(o);
	}

	appendMatrix(camera);

	this->cma = this->matrix.ma;
	this->cmb = this->matrix.mb;
	this->cmc = this->matrix.mc;
	this->cmd = this->matrix.md;
	this->cme = this->matrix.me;
	this->cmf = this->matrix.mf;
	this->cmg = this->matrix.mg;
	this->cmh = this->matrix.mh;
	this->cmi = this->matrix.mi;
	this->cmj = this->matrix.mj;
	this->cmk = this->matrix.mk;
	this->cml = this->matrix.ml;
}


//not used
void Light3D::calculateObjectMatrix(Object3D *param1)
{
	(void)param1;
	qDebug("used Light3D::calculateObjectMatrix");
/*
	this->oma = (((param1->matrix_i.ma * this->cma) + (param1->matrix_i.mb * this->cme)) + (param1->matrix_i.mc * this->cmi));
	this->omb = (((param1->matrix_i.ma * this->cmb) + (param1->matrix_i.mb * this->cmf)) + (param1->matrix_i.mc * this->cmj));
	this->omc = (((param1->matrix_i.ma * this->cmc) + (param1->matrix_i.mb * this->cmg)) + (param1->matrix_i.mc * this->cmk));
	this->omd = ((((param1->matrix_i.ma * this->cmd) + (param1->matrix_i.mb * this->cmh)) + (param1->matrix_i.mc * this->cml)) + param1->matrix_i.md);
	this->ome = (((param1->matrix_i.me * this->cma) + (param1->matrix_i.mf * this->cme)) + (param1->matrix_i.mg * this->cmi));
	this->omf = (((param1->matrix_i.me * this->cmb) + (param1->matrix_i.mf * this->cmf)) + (param1->matrix_i.mg * this->cmj));
	this->omg = (((param1->matrix_i.me * this->cmc) + (param1->matrix_i.mf * this->cmg)) + (param1->matrix_i.mg * this->cmk));
	this->omh = ((((param1->matrix_i.me * this->cmd) + (param1->matrix_i.mf * this->cmh)) + (param1->matrix_i.mg * this->cml)) + param1->matrix_i.mh);
	this->omi = (((param1->matrix_i.mi * this->cma) + (param1->matrix_i.mj * this->cme)) + (param1->matrix_i.mk * this->cmi));
	this->omj = (((param1->matrix_i.mi * this->cmb) + (param1->matrix_i.mj * this->cmf)) + (param1->matrix_i.mk * this->cmj));
	this->omk = (((param1->matrix_i.mi * this->cmc) + (param1->matrix_i.mj * this->cmg)) + (param1->matrix_i.mk * this->cmk));
	this->oml = ((((param1->matrix_i.mi * this->cmd) + (param1->matrix_i.mj * this->cmh)) + (param1->matrix_i.mk * this->cml)) + param1->matrix_i.ml);
*/
}

void Light3D::setParent(Object3DContainer *container)
{
	Light3D *_loc3_ = nullptr;

	if (container == nullptr)
	{
		Object3DContainer *_loc2_ = _parent;
		while (_loc2_->_parent != nullptr)
		{
			_loc2_ = _loc2_->_parent;
		}
		Light3D *_loc4_ = _loc2_->lightList;
		while (_loc4_ != nullptr)
		{
			if (_loc4_ == this)
			{
				if (_loc3_ != nullptr)
				{
					_loc3_->nextLight = this->nextLight;
				}
				else
				{
					_loc2_->lightList = this->nextLight;
				}
				this->nextLight = nullptr;
				break;
			}
			_loc3_ = _loc4_;
			_loc4_ = _loc4_->nextLight;
		}
	}
	else
	{
		Object3DContainer *temp = container;
		while (temp->_parent != nullptr)
		{
			temp = temp->_parent;
		}
		this->nextLight = temp->lightList;
		temp->lightList = this;
	}
	_parent = container;
}

void Light3D::drawDebug(Camera3D*)
{

}


void Light3D::updateBounds(Object3D *param1, Object3D *object)
{
	(void)object;
	param1->bound.MinX = -1E22;
	param1->bound.MinY = -1E22;
	param1->bound.MinZ = -1E22;
	param1->bound.MaxX = 1E22;
	param1->bound.MaxY = 1E22;
	param1->bound.MaxZ = 1E22;
}

int Light3D::cullingInCamera(Camera3D*, int)
{
	return -1;
}


bool Light3D::checkFrustumCulling(Camera3D *param1)
{
	Vertex *_loc2_ = boundVertexList.get();
	_loc2_->x = bound.MinX;
	_loc2_->y = bound.MinY;
	_loc2_->z = bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MaxX;
	_loc2_->y = bound.MinY;
	_loc2_->z = bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MinX;
	_loc2_->y = bound.MaxY;
	_loc2_->z = bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MaxX;
	_loc2_->y = bound.MaxY;
	_loc2_->z = bound.MinZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MinX;
	_loc2_->y = bound.MinY;
	_loc2_->z = bound.MaxZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MaxX;
	_loc2_->y = bound.MinY;
	_loc2_->z = bound.MaxZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MinX;
	_loc2_->y = bound.MaxY;
	_loc2_->z = bound.MaxZ;
	_loc2_ = _loc2_->next.get();
	_loc2_->x = bound.MaxX;
	_loc2_->y = bound.MaxY;
	_loc2_->z = bound.MaxZ;
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		_loc2_->cameraX = ((((this->matrix.ma * _loc2_->x) + (this->matrix.mb * _loc2_->y)) + (this->matrix.mc * _loc2_->z)) + this->matrix.md);
		_loc2_->cameraY = ((((this->matrix.me * _loc2_->x) + (this->matrix.mf * _loc2_->y)) + (this->matrix.mg * _loc2_->z)) + this->matrix.mh);
		_loc2_->cameraZ = ((((this->matrix.mi * _loc2_->x) + (this->matrix.mj * _loc2_->y)) + (this->matrix.mk * _loc2_->z)) + this->matrix.ml);
		_loc2_ = _loc2_->next.get();
	}
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraZ > param1->nearClipping)
		{
			break;
		}
		_loc2_ = _loc2_->next.get();
	}
	if (_loc2_ == nullptr)
	{
		return false;
	}
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraZ < param1->farClipping)
		{
			break;
		}
		_loc2_ = _loc2_->next.get();
	}
	if (_loc2_ == nullptr)
	{
		return false;
	}
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		if (-(_loc2_->cameraX) < _loc2_->cameraZ)
		{
			break;
		}
		_loc2_ = _loc2_->next.get();
	}
	if (_loc2_ == nullptr)
	{
		return false;
	}
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraX < _loc2_->cameraZ)
		{
			break;
		}
		_loc2_ = _loc2_->next.get();
	}
	if (_loc2_ == nullptr)
	{
		return false;
	}
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		if (-(_loc2_->cameraY) < _loc2_->cameraZ)
		{
			break;
		}
		_loc2_ = _loc2_->next.get();
	}
	if (_loc2_ == nullptr)
	{
		return false;
	}
	_loc2_ = boundVertexList.get();
	while (_loc2_ != nullptr)
	{
		if (_loc2_->cameraY < _loc2_->cameraZ)
		{
			break;
		}
		_loc2_ = _loc2_->next.get();
	}
	if (_loc2_ == nullptr)
	{
		return false;
	}
	return true;
}

//not used
bool Light3D::checkBoundsIntersection(Object3D *param1)
{
	(void)param1;
	qDebug("used Light3D::checkBoundsIntersection");
#if 0
	var _loc2_:Number = NaN;
	var _loc3_:Number = NaN;
	var _loc4_:Number = ((bound.MaxX - bound.MinX) * 0.5);
	var _loc5_:Number = ((bound.MaxY - bound.MinY) * 0.5);
	var _loc6_:Number = ((bound.MaxZ - bound.MinZ) * 0.5);
	var _loc7_:Number = (this->oma * _loc4_);
	var _loc8_:Number = (this->ome * _loc4_);
	var _loc9_:Number = (this->omi * _loc4_);
	var _loc10_:Number = (this->omb * _loc5_);
	var _loc11_:Number = (this->omf * _loc5_);
	var _loc12_:Number = (this->omj * _loc5_);
	var _loc13_:Number = (this->omc * _loc6_);
	var _loc14_:Number = (this->omg * _loc6_);
	var _loc15_:Number = (this->omk * _loc6_);
	var _loc16_:Number = ((param1->bound.MaxX - param1->bound.MinX) * 0.5);
	var _loc17_:Number = ((param1->bound.MaxY - param1->bound.MinY) * 0.5);
	var _loc18_:Number = ((param1->bound.MaxZ - param1->bound.MinZ) * 0.5);
	var _loc19_:Number = ((((((this->oma * (bound.MinX + _loc4_)) + (this->omb * (bound.MinY + _loc5_))) + (this->omc * (bound.MinZ + _loc6_))) + this->omd) - param1->bound.MinX) - _loc16_);
	var _loc20_:Number = ((((((this->ome * (bound.MinX + _loc4_)) + (this->omf * (bound.MinY + _loc5_))) + (this->omg * (bound.MinZ + _loc6_))) + this->omh) - param1->bound.MinY) - _loc17_);
	var _loc21_:Number = ((((((this->omi * (bound.MinX + _loc4_)) + (this->omj * (bound.MinY + _loc5_))) + (this->omk * (bound.MinZ + _loc6_))) + this->oml) - param1->bound.MinZ) - _loc18_);
	_loc2_ = 0;
	_loc3_ = ((_loc7_ >= 0) ? Number(_loc7_) : Number(-(_loc7_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((_loc10_ >= 0) ? Number(_loc10_) : Number(-(_loc10_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((_loc13_ >= 0) ? Number(_loc13_) : Number(-(_loc13_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc2_ = (_loc2_ + _loc16_);
	_loc3_ = ((_loc19_ >= 0) ? Number(_loc19_) : Number(-(_loc19_)));
	_loc2_ = (_loc2_ - _loc3_);
	if (_loc2_ <= 0)
	{
		return false;
	}
	_loc2_ = 0;
	_loc3_ = ((_loc8_ >= 0) ? Number(_loc8_) : Number(-(_loc8_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((_loc11_ >= 0) ? Number(_loc11_) : Number(-(_loc11_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((_loc14_ >= 0) ? Number(_loc14_) : Number(-(_loc14_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc2_ = (_loc2_ + _loc17_);
	_loc3_ = ((_loc20_ >= 0) ? Number(_loc20_) : Number(-(_loc20_)));
	_loc2_ = (_loc2_ - _loc3_);
	if (_loc2_ <= 0)
	{
		return false;
	}
	_loc2_ = 0;
	_loc3_ = ((_loc9_ >= 0) ? Number(_loc9_) : Number(-(_loc9_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((_loc12_ >= 0) ? Number(_loc12_) : Number(-(_loc12_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((_loc15_ >= 0) ? Number(_loc15_) : Number(-(_loc15_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc2_ = (_loc2_ + _loc17_);
	_loc3_ = ((_loc21_ >= 0) ? Number(_loc21_) : Number(-(_loc21_)));
	_loc2_ = (_loc2_ - _loc3_);
	if (_loc2_ <= 0)
	{
		return false;
	}
	_loc2_ = 0;
	_loc3_ = (((this->oma * _loc7_) + (this->ome * _loc8_)) + (this->omi * _loc9_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->oma * _loc10_) + (this->ome * _loc11_)) + (this->omi * _loc12_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->oma * _loc13_) + (this->ome * _loc14_)) + (this->omi * _loc15_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->oma >= 0) ? Number((this->oma * _loc16_)) : Number((-(this->oma) * _loc16_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->ome >= 0) ? Number((this->ome * _loc17_)) : Number((-(this->ome) * _loc17_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omi >= 0) ? Number((this->omi * _loc18_)) : Number((-(this->omi) * _loc18_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->oma * _loc19_) + (this->ome * _loc20_)) + (this->omi * _loc21_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ - _loc3_);
	if (_loc2_ <= 0)
	{
		return false;
	}
	_loc2_ = 0;
	_loc3_ = (((this->omb * _loc7_) + (this->omf * _loc8_)) + (this->omj * _loc9_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->omb * _loc10_) + (this->omf * _loc11_)) + (this->omj * _loc12_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->omb * _loc13_) + (this->omf * _loc14_)) + (this->omj * _loc15_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omb >= 0) ? Number((this->omb * _loc16_)) : Number((-(this->omb) * _loc16_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omf >= 0) ? Number((this->omf * _loc17_)) : Number((-(this->omf) * _loc17_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omj >= 0) ? Number((this->omj * _loc18_)) : Number((-(this->omj) * _loc18_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->omb * _loc19_) + (this->omf * _loc20_)) + (this->omj * _loc21_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ - _loc3_);
	if (_loc2_ <= 0)
	{
		return false;
	}
	_loc2_ = 0;
	_loc3_ = (((this->omc * _loc7_) + (this->omg * _loc8_)) + (this->omk * _loc9_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->omc * _loc10_) + (this->omg * _loc11_)) + (this->omk * _loc12_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->omc * _loc13_) + (this->omg * _loc14_)) + (this->omk * _loc15_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omc >= 0) ? Number((this->omc * _loc16_)) : Number((-(this->omc) * _loc16_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omg >= 0) ? Number((this->omg * _loc17_)) : Number((-(this->omg) * _loc17_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = ((this->omk >= 0) ? Number((this->omk * _loc18_)) : Number((-(this->omk) * _loc18_)));
	_loc2_ = (_loc2_ + _loc3_);
	_loc3_ = (((this->omc * _loc19_) + (this->omg * _loc20_)) + (this->omk * _loc21_));
	_loc3_ = ((_loc3_ >= 0) ? Number(_loc3_) : Number(-(_loc3_)));
	_loc2_ = (_loc2_ - _loc3_);
	if (_loc2_ <= 0)
	{
		return false;
	}
	return true;
#endif
	return false;
}
