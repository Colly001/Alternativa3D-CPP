#include "spotlight.h"
#include "mymath.h"
#include <QDebug>

const Object3D::T *SpotLight::TYPE = new Object3D::T("SpotLight");


SpotLight::SpotLight(const TT &t, uint32_t color, double attenuationBegin, double attenuationEnd, double hotspot, double falloff) :
	Light3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	this->color = color;
	this->attenuationBegin = attenuationBegin;
	this->attenuationEnd = attenuationEnd;
	this->hotspot = hotspot;
	this->falloff = falloff;
	calculateBounds();
}

//not used
void SpotLight::lookAt(double x, double y, double z)
{
	double _loc4_ = x - this->position.x;
	double _loc5_ = y - this->position.y;
	double _loc6_ = z - this->position.z;
	rotation.x = MyMath::atan2(_loc6_, MyMath::sqrt(((_loc4_ * _loc4_) + (_loc5_ * _loc5_)))) - (MyMath::PI / 2);
	rotation.y = 0;
	rotation.z = -MyMath::atan2(_loc4_, _loc5_);
}

//not used
void SpotLight::drawDebug(Camera3D *param1) //override
{
	(void)param1;

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
	var _loc14_:Number = NaN;
	var _loc15_:Number = NaN;
	var _loc16_:Number = NaN;
	var _loc17_:Number = NaN;
	var _loc18_:Number = NaN;
	var _loc19_:Number = NaN;
	var _loc20_:Number = NaN;
	var _loc21_:Number = NaN;
	var _loc22_:Number = NaN;
	var _loc23_:Number = NaN;
	var _loc24_:Number = NaN;
	var _loc25_:Number = NaN;
	var _loc26_:Number = NaN;
	var _loc27_:Number = NaN;
	var _loc28_:Number = NaN;
	var _loc29_:Number = NaN;
	var _loc30_:Number = NaN;
	var _loc31_:Number = NaN;
	var _loc32_:Number = NaN;
	var _loc33_:Number = NaN;
	var _loc34_:Number = NaN;
	var _loc35_:Number = NaN;
	var _loc36_:Number = NaN;
	var _loc37_:Number = NaN;
	var _loc38_:Number = NaN;
	var _loc39_:Number = NaN;
	var _loc40_:Number = NaN;
	var _loc41_:Number = NaN;
	var _loc42_:Number = NaN;
	var _loc43_:Number = NaN;
	var _loc44_:Number = NaN;
	var _loc45_:Number = NaN;
	var _loc46_:Number = NaN;
	var _loc47_:Number = NaN;
	var _loc2_:int = param1.checkInDebug(this);
	if (_loc2_ > 0)
	{
		_loc3_ = param1.view.canvas;
		if (((_loc2_ & Debug.LIGHTS) && (ml > param1.nearClipping)))
		{
			_loc4_ = (((color >> 16) & 0xFF) * intensity);
			_loc5_ = (((color >> 8) & 0xFF) * intensity);
			_loc6_ = ((color & 0xFF) * intensity);
			_loc7_ = (((((_loc4_ > 0xFF) ? 0xFF : _loc4_) << 16) + (((_loc5_ > 0xFF) ? 0xFF : _loc5_) << 8)) + ((_loc6_ > 0xFF) ? 0xFF : _loc6_));
			_loc8_ = 0;
			_loc9_ = ((md * param1.viewSizeX) / param1.focalLength);
			_loc10_ = ((mh * param1.viewSizeY) / param1.focalLength);
			_loc11_ = ml;
			_loc12_ = ((mc * param1.viewSizeX) / param1.focalLength);
			_loc13_ = ((mg * param1.viewSizeY) / param1.focalLength);
			_loc14_ = mk;
			_loc15_ = Math.sqrt((((_loc12_ * _loc12_) + (_loc13_ * _loc13_)) + (_loc14_ * _loc14_)));
			_loc8_ = (_loc8_ + _loc15_);
			_loc12_ = (_loc12_ / _loc15_);
			_loc13_ = (_loc13_ / _loc15_);
			_loc14_ = (_loc14_ / _loc15_);
			_loc16_ = ((ma * param1.viewSizeX) / param1.focalLength);
			_loc17_ = ((me * param1.viewSizeY) / param1.focalLength);
			_loc18_ = mi;
			_loc8_ = (_loc8_ + Math.sqrt((((_loc16_ * _loc16_) + (_loc17_ * _loc17_)) + (_loc18_ * _loc18_))));
			_loc19_ = ((_loc18_ * _loc13_) - (_loc17_ * _loc14_));
			_loc20_ = ((_loc16_ * _loc14_) - (_loc18_ * _loc12_));
			_loc21_ = ((_loc17_ * _loc12_) - (_loc16_ * _loc13_));
			_loc15_ = Math.sqrt((((_loc19_ * _loc19_) + (_loc20_ * _loc20_)) + (_loc21_ * _loc21_)));
			_loc19_ = (_loc19_ / _loc15_);
			_loc20_ = (_loc20_ / _loc15_);
			_loc21_ = (_loc21_ / _loc15_);
			_loc16_ = ((mb * param1.viewSizeX) / param1.focalLength);
			_loc17_ = ((mf * param1.viewSizeY) / param1.focalLength);
			_loc18_ = mj;
			_loc8_ = (_loc8_ + Math.sqrt((((_loc16_ * _loc16_) + (_loc17_ * _loc17_)) + (_loc18_ * _loc18_))));
			_loc8_ = (_loc8_ / 3);
			_loc16_ = ((_loc21_ * _loc13_) - (_loc20_ * _loc14_));
			_loc17_ = ((_loc19_ * _loc14_) - (_loc21_ * _loc12_));
			_loc18_ = ((_loc20_ * _loc12_) - (_loc19_ * _loc13_));
			_loc22_ = Math.cos((this.hotspot / 2));
			_loc23_ = Math.sin((this.hotspot / 2));
			_loc24_ = (_loc9_ + ((((_loc12_ * _loc22_) + (_loc16_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc25_ = (_loc10_ + ((((_loc13_ * _loc22_) + (_loc17_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc26_ = (_loc11_ + ((((_loc14_ * _loc22_) + (_loc18_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc27_ = (_loc9_ + ((((_loc12_ * _loc22_) + (((_loc16_ + _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc28_ = (_loc10_ + ((((_loc13_ * _loc22_) + (((_loc17_ + _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc29_ = (_loc11_ + ((((_loc14_ * _loc22_) + (((_loc18_ + _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc30_ = (_loc9_ + ((((_loc12_ * _loc22_) + (_loc19_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc31_ = (_loc10_ + ((((_loc13_ * _loc22_) + (_loc20_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc32_ = (_loc11_ + ((((_loc14_ * _loc22_) + (_loc21_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc33_ = (_loc9_ + ((((_loc12_ * _loc22_) - (((_loc16_ - _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc34_ = (_loc10_ + ((((_loc13_ * _loc22_) - (((_loc17_ - _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc35_ = (_loc11_ + ((((_loc14_ * _loc22_) - (((_loc18_ - _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc36_ = (_loc9_ + ((((_loc12_ * _loc22_) - (_loc16_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc37_ = (_loc10_ + ((((_loc13_ * _loc22_) - (_loc17_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc38_ = (_loc11_ + ((((_loc14_ * _loc22_) - (_loc18_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc39_ = (_loc9_ + ((((_loc12_ * _loc22_) - (((_loc16_ + _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc40_ = (_loc10_ + ((((_loc13_ * _loc22_) - (((_loc17_ + _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc41_ = (_loc11_ + ((((_loc14_ * _loc22_) - (((_loc18_ + _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc42_ = (_loc9_ + ((((_loc12_ * _loc22_) - (_loc19_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc43_ = (_loc10_ + ((((_loc13_ * _loc22_) - (_loc20_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc44_ = (_loc11_ + ((((_loc14_ * _loc22_) - (_loc21_ * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc45_ = (_loc9_ + ((((_loc12_ * _loc22_) + (((_loc16_ - _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc46_ = (_loc10_ + ((((_loc13_ * _loc22_) + (((_loc17_ - _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			_loc47_ = (_loc11_ + ((((_loc14_ * _loc22_) + (((_loc18_ - _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationBegin));
			if (((((((((_loc26_ > param1.nearClipping) && (_loc29_ > param1.nearClipping)) && (_loc32_ > param1.nearClipping)) && (_loc35_ > param1.nearClipping)) && (_loc38_ > param1.nearClipping)) && (_loc41_ > param1.nearClipping)) && (_loc44_ > param1.nearClipping)) && (_loc47_ > param1.nearClipping)))
			{
				_loc3_.graphics.lineStyle(1, _loc7_);
				_loc3_.graphics.moveTo(((_loc24_ * param1.focalLength) / _loc26_), ((_loc25_ * param1.focalLength) / _loc26_));
				_loc3_.graphics.curveTo(((_loc27_ * param1.focalLength) / _loc29_), ((_loc28_ * param1.focalLength) / _loc29_), ((_loc30_ * param1.focalLength) / _loc32_), ((_loc31_ * param1.focalLength) / _loc32_));
				_loc3_.graphics.curveTo(((_loc33_ * param1.focalLength) / _loc35_), ((_loc34_ * param1.focalLength) / _loc35_), ((_loc36_ * param1.focalLength) / _loc38_), ((_loc37_ * param1.focalLength) / _loc38_));
				_loc3_.graphics.curveTo(((_loc39_ * param1.focalLength) / _loc41_), ((_loc40_ * param1.focalLength) / _loc41_), ((_loc42_ * param1.focalLength) / _loc44_), ((_loc43_ * param1.focalLength) / _loc44_));
				_loc3_.graphics.curveTo(((_loc45_ * param1.focalLength) / _loc47_), ((_loc46_ * param1.focalLength) / _loc47_), ((_loc24_ * param1.focalLength) / _loc26_), ((_loc25_ * param1.focalLength) / _loc26_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc24_ * param1.focalLength) / _loc26_), ((_loc25_ * param1.focalLength) / _loc26_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc30_ * param1.focalLength) / _loc32_), ((_loc31_ * param1.focalLength) / _loc32_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc36_ * param1.focalLength) / _loc38_), ((_loc37_ * param1.focalLength) / _loc38_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc42_ * param1.focalLength) / _loc44_), ((_loc43_ * param1.focalLength) / _loc44_));
			}
			_loc22_ = Math.cos((this.falloff / 2));
			_loc23_ = Math.sin((this.falloff / 2));
			_loc24_ = (_loc9_ + ((((_loc12_ * _loc22_) + (_loc16_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc25_ = (_loc10_ + ((((_loc13_ * _loc22_) + (_loc17_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc26_ = (_loc11_ + ((((_loc14_ * _loc22_) + (_loc18_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc27_ = (_loc9_ + ((((_loc12_ * _loc22_) + (((_loc16_ + _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc28_ = (_loc10_ + ((((_loc13_ * _loc22_) + (((_loc17_ + _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc29_ = (_loc11_ + ((((_loc14_ * _loc22_) + (((_loc18_ + _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc30_ = (_loc9_ + ((((_loc12_ * _loc22_) + (_loc19_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc31_ = (_loc10_ + ((((_loc13_ * _loc22_) + (_loc20_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc32_ = (_loc11_ + ((((_loc14_ * _loc22_) + (_loc21_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc33_ = (_loc9_ + ((((_loc12_ * _loc22_) - (((_loc16_ - _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc34_ = (_loc10_ + ((((_loc13_ * _loc22_) - (((_loc17_ - _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc35_ = (_loc11_ + ((((_loc14_ * _loc22_) - (((_loc18_ - _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc36_ = (_loc9_ + ((((_loc12_ * _loc22_) - (_loc16_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc37_ = (_loc10_ + ((((_loc13_ * _loc22_) - (_loc17_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc38_ = (_loc11_ + ((((_loc14_ * _loc22_) - (_loc18_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc39_ = (_loc9_ + ((((_loc12_ * _loc22_) - (((_loc16_ + _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc40_ = (_loc10_ + ((((_loc13_ * _loc22_) - (((_loc17_ + _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc41_ = (_loc11_ + ((((_loc14_ * _loc22_) - (((_loc18_ + _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc42_ = (_loc9_ + ((((_loc12_ * _loc22_) - (_loc19_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc43_ = (_loc10_ + ((((_loc13_ * _loc22_) - (_loc20_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc44_ = (_loc11_ + ((((_loc14_ * _loc22_) - (_loc21_ * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc45_ = (_loc9_ + ((((_loc12_ * _loc22_) + (((_loc16_ - _loc19_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc46_ = (_loc10_ + ((((_loc13_ * _loc22_) + (((_loc17_ - _loc20_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			_loc47_ = (_loc11_ + ((((_loc14_ * _loc22_) + (((_loc18_ - _loc21_) * 0.9) * _loc23_)) * _loc8_) * this.attenuationEnd));
			if (((((((((_loc26_ > param1.nearClipping) && (_loc29_ > param1.nearClipping)) && (_loc32_ > param1.nearClipping)) && (_loc35_ > param1.nearClipping)) && (_loc38_ > param1.nearClipping)) && (_loc41_ > param1.nearClipping)) && (_loc44_ > param1.nearClipping)) && (_loc47_ > param1.nearClipping)))
			{
				_loc3_.graphics.lineStyle(1, _loc7_, 0.5);
				_loc3_.graphics.moveTo(((_loc24_ * param1.focalLength) / _loc26_), ((_loc25_ * param1.focalLength) / _loc26_));
				_loc3_.graphics.curveTo(((_loc27_ * param1.focalLength) / _loc29_), ((_loc28_ * param1.focalLength) / _loc29_), ((_loc30_ * param1.focalLength) / _loc32_), ((_loc31_ * param1.focalLength) / _loc32_));
				_loc3_.graphics.curveTo(((_loc33_ * param1.focalLength) / _loc35_), ((_loc34_ * param1.focalLength) / _loc35_), ((_loc36_ * param1.focalLength) / _loc38_), ((_loc37_ * param1.focalLength) / _loc38_));
				_loc3_.graphics.curveTo(((_loc39_ * param1.focalLength) / _loc41_), ((_loc40_ * param1.focalLength) / _loc41_), ((_loc42_ * param1.focalLength) / _loc44_), ((_loc43_ * param1.focalLength) / _loc44_));
				_loc3_.graphics.curveTo(((_loc45_ * param1.focalLength) / _loc47_), ((_loc46_ * param1.focalLength) / _loc47_), ((_loc24_ * param1.focalLength) / _loc26_), ((_loc25_ * param1.focalLength) / _loc26_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc24_ * param1.focalLength) / _loc26_), ((_loc25_ * param1.focalLength) / _loc26_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc30_ * param1.focalLength) / _loc32_), ((_loc31_ * param1.focalLength) / _loc32_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc36_ * param1.focalLength) / _loc38_), ((_loc37_ * param1.focalLength) / _loc38_));
				_loc3_.graphics.moveTo(((_loc9_ * param1.focalLength) / _loc11_), ((_loc10_ * param1.focalLength) / _loc11_));
				_loc3_.graphics.lineTo(((_loc42_ * param1.focalLength) / _loc44_), ((_loc43_ * param1.focalLength) / _loc44_));
			}
		}
		if ((_loc2_ & Debug.BOUNDS))
		{
			Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ, 0x99FF00);
		}
	}
#endif
}

void SpotLight::updateBounds(Object3D *param1, Object3D *param2) //override
{
	(void)param1;
	(void)param2;

	qDebug("used SpotLight::updateBounds");

#if 0
	var _loc5_:Vertex;
	var _loc3_:Number = ((this.falloff < Math.PI) ? Number((Math.sin((this.falloff / 2)) * this.attenuationEnd)) : Number(this.attenuationEnd));
	var _loc4_:Number = ((this.falloff < Math.PI) ? Number(0) : Number((Math.cos((this.falloff / 2)) * this.attenuationEnd)));
	if (param2 != null)
	{
		_loc5_ = boundVertexList;
		_loc5_.x = -(_loc3_);
		_loc5_.y = -(_loc3_);
		_loc5_.z = _loc4_;
		_loc5_ = _loc5_.next;
		_loc5_.x = _loc3_;
		_loc5_.y = -(_loc3_);
		_loc5_.z = _loc4_;
		_loc5_ = _loc5_.next;
		_loc5_.x = -(_loc3_);
		_loc5_.y = _loc3_;
		_loc5_.z = _loc4_;
		_loc5_ = _loc5_.next;
		_loc5_.x = _loc3_;
		_loc5_.y = _loc3_;
		_loc5_.z = _loc4_;
		_loc5_ = _loc5_.next;
		_loc5_.x = -(_loc3_);
		_loc5_.y = -(_loc3_);
		_loc5_.z = this.attenuationEnd;
		_loc5_ = _loc5_.next;
		_loc5_.x = _loc3_;
		_loc5_.y = -(_loc3_);
		_loc5_.z = this.attenuationEnd;
		_loc5_ = _loc5_.next;
		_loc5_.x = -(_loc3_);
		_loc5_.y = _loc3_;
		_loc5_.z = this.attenuationEnd;
		_loc5_ = _loc5_.next;
		_loc5_.x = _loc3_;
		_loc5_.y = _loc3_;
		_loc5_.z = this.attenuationEnd;
		_loc5_ = boundVertexList;
		while (_loc5_ != null)
		{
			_loc5_.cameraX = ((((param2.ma * _loc5_.x) + (param2.mb * _loc5_.y)) + (param2.mc * _loc5_.z)) + param2.md);
			_loc5_.cameraY = ((((param2.me * _loc5_.x) + (param2.mf * _loc5_.y)) + (param2.mg * _loc5_.z)) + param2.mh);
			_loc5_.cameraZ = ((((param2.mi * _loc5_.x) + (param2.mj * _loc5_.y)) + (param2.mk * _loc5_.z)) + param2.ml);
			if (_loc5_.cameraX < param1.boundMinX)
			{
				param1.boundMinX = _loc5_.cameraX;
			}
			if (_loc5_.cameraX > param1.boundMaxX)
			{
				param1.boundMaxX = _loc5_.cameraX;
			}
			if (_loc5_.cameraY < param1.boundMinY)
			{
				param1.boundMinY = _loc5_.cameraY;
			}
			if (_loc5_.cameraY > param1.boundMaxY)
			{
				param1.boundMaxY = _loc5_.cameraY;
			}
			if (_loc5_.cameraZ < param1.boundMinZ)
			{
				param1.boundMinZ = _loc5_.cameraZ;
			}
			if (_loc5_.cameraZ > param1.boundMaxZ)
			{
				param1.boundMaxZ = _loc5_.cameraZ;
			}
			_loc5_ = _loc5_.next;
		}
	}
	else
	{
		if (-(_loc3_) < param1.boundMinX)
		{
			param1.boundMinX = -(_loc3_);
		}
		if (_loc3_ > param1.boundMaxX)
		{
			param1.boundMaxX = _loc3_;
		}
		if (-(_loc3_) < param1.boundMinY)
		{
			param1.boundMinY = -(_loc3_);
		}
		if (_loc3_ > param1.boundMaxY)
		{
			param1.boundMaxY = _loc3_;
		}
		if (_loc4_ < param1.boundMinZ)
		{
			param1.boundMinZ = _loc4_;
		}
		if (this.attenuationEnd > param1.boundMaxZ)
		{
			param1.boundMaxZ = this.attenuationEnd;
		}
	}
#endif
}

void SpotLight::clonePropertiesFromSpotLight(SpotLight *src)
{
	clonePropertiesFromLight3D(src);
}


std::shared_ptr<SpotLight> new_SpotLight(uint32_t color, double attenuationBegin, double attenuationEnd, double hotspot, double falloff)
{
	return std::make_shared<SpotLight>(Object3D::TT(SpotLight::TYPE), color, attenuationBegin, attenuationEnd, hotspot, falloff);
}

std::shared_ptr<SpotLight> SpotLight_clone(const std::shared_ptr<SpotLight> &in)
{
	if (in->isType(SpotLight::TYPE)) {
		SpotLight *src = in.get();
		auto out = new_SpotLight(src->color, src->attenuationBegin, src->attenuationEnd, src->hotspot, src->falloff);
		out->clonePropertiesFromSpotLight(src);
		return out;
	}

	throw 4113244;
}
