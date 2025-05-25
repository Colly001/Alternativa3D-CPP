#include "omnilight.h"
#include "alternativa/engine3d/core/vertex.h"

const Object3D::T *OmniLight::TYPE = new Object3D::T("OmniLight");


OmniLight::OmniLight(const TT &t, uint32_t color, double attenuationBegin, double attenuationEnd) :
	Light3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	this->color = color;
	this->attenuationBegin = attenuationBegin;
	this->attenuationEnd = attenuationEnd;
	calculateBounds();
}


void OmniLight::drawDebug(Camera3D*) //override
{
#if 0
	var _loc3_:Sprite;
	var _loc4_:Number = NaN;
	var _loc5_:Number = NaN;
	var _loc6_:Number = NaN;
	var _loc7_:int;
	var _loc8_:Number = NaN;
	var _loc9_:Number = NaN;
	var _loc10_:Number = NaN;
	var _loc11_:Number = NaN;
	var _loc12_:Number = NaN;
	var _loc13_:Number = NaN;
	var _loc2_:int = param1->checkInDebug(this);
	if (_loc2_ > 0)
	{
		_loc3_ = param1->view.canvas;
		if (((_loc2_ & Debug.LIGHTS) && (ml > param1->nearClipping)))
		{
			_loc4_ = (((color >> 16) & 0xFF) * intensity);
			_loc5_ = (((color >> 8) & 0xFF) * intensity);
			_loc6_ = ((color & 0xFF) * intensity);
			_loc7_ = (((((_loc4_ > 0xFF) ? 0xFF : _loc4_) << 16) + (((_loc5_ > 0xFF) ? 0xFF : _loc5_) << 8)) + ((_loc6_ > 0xFF) ? 0xFF : _loc6_));
			_loc8_ = ((ma * param1->viewSizeX) / param1->focalLength);
			_loc9_ = ((me * param1->viewSizeY) / param1->focalLength);
			_loc10_ = Math.sqrt((((_loc8_ * _loc8_) + (_loc9_ * _loc9_)) + (mi * mi)));
			_loc8_ = ((mb * param1->viewSizeX) / param1->focalLength);
			_loc9_ = ((mf * param1->viewSizeY) / param1->focalLength);
			_loc10_ = (_loc10_ + Math.sqrt((((_loc8_ * _loc8_) + (_loc9_ * _loc9_)) + (mj * mj))));
			_loc8_ = ((mc * param1->viewSizeX) / param1->focalLength);
			_loc9_ = ((mg * param1->viewSizeY) / param1->focalLength);
			_loc10_ = (_loc10_ + Math.sqrt((((_loc8_ * _loc8_) + (_loc9_ * _loc9_)) + (mk * mk))));
			_loc10_ = (_loc10_ / 3);
			_loc11_ = Math.round(((md * param1->viewSizeX) / ml));
			_loc12_ = Math.round(((mh * param1->viewSizeY) / ml));
			_loc13_ = 8;
			_loc3_->graphics.lineStyle(1, _loc7_);
			_loc3_->graphics.moveTo((_loc11_ - _loc13_), _loc12_);
			_loc3_->graphics.lineTo((_loc11_ + _loc13_), _loc12_);
			_loc3_->graphics.moveTo(_loc11_, (_loc12_ - _loc13_));
			_loc3_->graphics.lineTo(_loc11_, (_loc12_ + _loc13_));
			_loc3_->graphics.moveTo((_loc11_ - (_loc13_ * 0.7)), (_loc12_ - (_loc13_ * 0.7)));
			_loc3_->graphics.lineTo((_loc11_ + (_loc13_ * 0.7)), (_loc12_ + (_loc13_ * 0.7)));
			_loc3_->graphics.moveTo((_loc11_ - (_loc13_ * 0.7)), (_loc12_ + (_loc13_ * 0.7)));
			_loc3_->graphics.lineTo((_loc11_ + (_loc13_ * 0.7)), (_loc12_ - (_loc13_ * 0.7)));
			_loc3_->graphics.drawCircle(_loc11_, _loc12_, (((this->attenuationBegin * _loc10_) * param1->focalLength) / ml));
			_loc3_->graphics.lineStyle(1, _loc7_, 0.5);
			_loc3_->graphics.drawCircle(_loc11_, _loc12_, (((this->attenuationEnd * _loc10_) * param1->focalLength) / ml));
		}
		if ((_loc2_ & Debug.BOUNDS))
		{
			Debug.drawBounds(param1, this, bound.MinX, bound.MinY, bound.MinZ, bound.MaxX, bound.MaxY, bound.MaxZ, 0x99FF00);
		}
	}
#endif
}

void OmniLight::updateBounds(Object3D *param1, Object3D *param2)
{
	if (param2 != nullptr)
	{
		Vertex *_loc3_ = boundVertexList.get();
		_loc3_->x = -(this->attenuationEnd);
		_loc3_->y = -(this->attenuationEnd);
		_loc3_->z = -(this->attenuationEnd);
		_loc3_ = _loc3_->next.get();
		_loc3_->x = this->attenuationEnd;
		_loc3_->y = -(this->attenuationEnd);
		_loc3_->z = -(this->attenuationEnd);
		_loc3_ = _loc3_->next.get();
		_loc3_->x = -(this->attenuationEnd);
		_loc3_->y = this->attenuationEnd;
		_loc3_->z = -(this->attenuationEnd);
		_loc3_ = _loc3_->next.get();
		_loc3_->x = this->attenuationEnd;
		_loc3_->y = this->attenuationEnd;
		_loc3_->z = -(this->attenuationEnd);
		_loc3_ = _loc3_->next.get();
		_loc3_->x = -(this->attenuationEnd);
		_loc3_->y = -(this->attenuationEnd);
		_loc3_->z = this->attenuationEnd;
		_loc3_ = _loc3_->next.get();
		_loc3_->x = this->attenuationEnd;
		_loc3_->y = -(this->attenuationEnd);
		_loc3_->z = this->attenuationEnd;
		_loc3_ = _loc3_->next.get();
		_loc3_->x = -(this->attenuationEnd);
		_loc3_->y = this->attenuationEnd;
		_loc3_->z = this->attenuationEnd;
		_loc3_ = _loc3_->next.get();
		_loc3_->x = this->attenuationEnd;
		_loc3_->y = this->attenuationEnd;
		_loc3_->z = this->attenuationEnd;
		_loc3_ = boundVertexList.get();
		while (_loc3_ != nullptr)
		{
			_loc3_->cameraX = ((((param2->matrix.ma * _loc3_->x) + (param2->matrix.mb * _loc3_->y)) + (param2->matrix.mc * _loc3_->z)) + param2->matrix.md);
			_loc3_->cameraY = ((((param2->matrix.me * _loc3_->x) + (param2->matrix.mf * _loc3_->y)) + (param2->matrix.mg * _loc3_->z)) + param2->matrix.mh);
			_loc3_->cameraZ = ((((param2->matrix.mi * _loc3_->x) + (param2->matrix.mj * _loc3_->y)) + (param2->matrix.mk * _loc3_->z)) + param2->matrix.ml);
			if (_loc3_->cameraX < param1->bound.MinX)
			{
				param1->bound.MinX = _loc3_->cameraX;
			}
			if (_loc3_->cameraX > param1->bound.MaxX)
			{
				param1->bound.MaxX = _loc3_->cameraX;
			}
			if (_loc3_->cameraY < param1->bound.MinY)
			{
				param1->bound.MinY = _loc3_->cameraY;
			}
			if (_loc3_->cameraY > param1->bound.MaxY)
			{
				param1->bound.MaxY = _loc3_->cameraY;
			}
			if (_loc3_->cameraZ < param1->bound.MinZ)
			{
				param1->bound.MinZ = _loc3_->cameraZ;
			}
			if (_loc3_->cameraZ > param1->bound.MaxZ)
			{
				param1->bound.MaxZ = _loc3_->cameraZ;
			}
			_loc3_ = _loc3_->next.get();
		}
	}
	else
	{
		if (-(this->attenuationEnd) < param1->bound.MinX)
		{
			param1->bound.MinX = -(this->attenuationEnd);
		}
		if (this->attenuationEnd > param1->bound.MaxX)
		{
			param1->bound.MaxX = this->attenuationEnd;
		}
		if (-(this->attenuationEnd) < param1->bound.MinY)
		{
			param1->bound.MinY = -(this->attenuationEnd);
		}
		if (this->attenuationEnd > param1->bound.MaxY)
		{
			param1->bound.MaxY = this->attenuationEnd;
		}
		if (-(this->attenuationEnd) < param1->bound.MinZ)
		{
			param1->bound.MinZ = -(this->attenuationEnd);
		}
		if (this->attenuationEnd > param1->bound.MaxZ)
		{
			param1->bound.MaxZ = this->attenuationEnd;
		}
	}
}

void OmniLight::clonePropertiesFromOmniLight(OmniLight *src)
{
	clonePropertiesFromLight3D(src);
}

std::shared_ptr<OmniLight> new_OmniLight(uint32_t color, double attenuationBegin, double attenuationEnd)
{
	return std::make_shared<OmniLight>(Object3D::TT(OmniLight::TYPE), color, attenuationBegin, attenuationEnd);
}


std::shared_ptr<OmniLight> OmniLight_clone(const std::shared_ptr<OmniLight> &in)
{
	if (in->isType(OmniLight::TYPE)) {
		OmniLight *src = in.get();
		auto out = new_OmniLight(src->color, src->attenuationBegin, src->attenuationEnd);
		out->clonePropertiesFromOmniLight(src);
		return out;
	}

	throw 4113244;
}
