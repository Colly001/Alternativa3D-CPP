#include "directionallight.h"
#include "mymath.h"

const Object3D::T *DirectionalLight::TYPE = new Object3D::T("DirectionalLight");


DirectionalLight::DirectionalLight(const TT &t, uint32_t color) :
	Light3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	this->color = color;
	calculateBounds();
}

void DirectionalLight::lookAt(double x, double y, double z)
{
	x = x - this->position.x;
	y = y - this->position.y;
	z = z - this->position.z;
	rotation.x = MyMath::atan2(z, MyMath::sqrt(((x * x) + (y * y)))) - (MyMath::PI / 2);
	rotation.y = 0;
	rotation.z = -MyMath::atan2(x, y);
}

void DirectionalLight::drawDebug(Camera3D *param1)
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
	var _loc48_:Number = NaN;
	var _loc49_:Number = NaN;
	var _loc50_:Number = NaN;
	var _loc51_:Number = NaN;
	var _loc52_:Number = NaN;
	var _loc53_:Number = NaN;
	var _loc54_:Number = NaN;
	var _loc55_:Number = NaN;
	var _loc56_:Number = NaN;
	var _loc57_:Number = NaN;
	var _loc58_:Number = NaN;
	var _loc59_:Number = NaN;
	var _loc60_:Number = NaN;
	var _loc61_:Number = NaN;
	var _loc62_:Number = NaN;
	var _loc63_:Number = NaN;
	var _loc64_:Number = NaN;
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
			_loc8_ = ((md * param1.viewSizeX) / param1.focalLength);
			_loc9_ = ((mh * param1.viewSizeY) / param1.focalLength);
			_loc10_ = ml;
			_loc11_ = ((mc * param1.viewSizeX) / param1.focalLength);
			_loc12_ = ((mg * param1.viewSizeY) / param1.focalLength);
			_loc13_ = mk;
			_loc14_ = Math.sqrt((((_loc11_ * _loc11_) + (_loc12_ * _loc12_)) + (_loc13_ * _loc13_)));
			_loc11_ = (_loc11_ / _loc14_);
			_loc12_ = (_loc12_ / _loc14_);
			_loc13_ = (_loc13_ / _loc14_);
			_loc15_ = ((ma * param1.viewSizeX) / param1.focalLength);
			_loc16_ = ((me * param1.viewSizeY) / param1.focalLength);
			_loc17_ = mi;
			_loc18_ = ((_loc17_ * _loc12_) - (_loc16_ * _loc13_));
			_loc19_ = ((_loc15_ * _loc13_) - (_loc17_ * _loc11_));
			_loc20_ = ((_loc16_ * _loc11_) - (_loc15_ * _loc12_));
			_loc14_ = Math.sqrt((((_loc18_ * _loc18_) + (_loc19_ * _loc19_)) + (_loc20_ * _loc20_)));
			_loc18_ = (_loc18_ / _loc14_);
			_loc19_ = (_loc19_ / _loc14_);
			_loc20_ = (_loc20_ / _loc14_);
			_loc15_ = ((mb * param1.viewSizeX) / param1.focalLength);
			_loc16_ = ((mf * param1.viewSizeY) / param1.focalLength);
			_loc17_ = mj;
			_loc15_ = ((_loc20_ * _loc12_) - (_loc19_ * _loc13_));
			_loc16_ = ((_loc18_ * _loc13_) - (_loc20_ * _loc11_));
			_loc17_ = ((_loc19_ * _loc11_) - (_loc18_ * _loc12_));
			_loc21_ = (ml / param1.focalLength);
			_loc11_ = (_loc11_ * _loc21_);
			_loc12_ = (_loc12_ * _loc21_);
			_loc13_ = (_loc13_ * _loc21_);
			_loc15_ = (_loc15_ * _loc21_);
			_loc16_ = (_loc16_ * _loc21_);
			_loc17_ = (_loc17_ * _loc21_);
			_loc18_ = (_loc18_ * _loc21_);
			_loc19_ = (_loc19_ * _loc21_);
			_loc20_ = (_loc20_ * _loc21_);
			_loc22_ = 16;
			_loc23_ = 24;
			_loc24_ = 4;
			_loc25_ = 8;
			_loc26_ = (_loc8_ + (_loc11_ * _loc23_));
			_loc27_ = (_loc9_ + (_loc12_ * _loc23_));
			_loc28_ = (_loc10_ + (_loc13_ * _loc23_));
			_loc29_ = ((_loc8_ + (_loc15_ * _loc24_)) + (_loc18_ * _loc24_));
			_loc30_ = ((_loc9_ + (_loc16_ * _loc24_)) + (_loc19_ * _loc24_));
			_loc31_ = ((_loc10_ + (_loc17_ * _loc24_)) + (_loc20_ * _loc24_));
			_loc32_ = ((_loc8_ - (_loc15_ * _loc24_)) + (_loc18_ * _loc24_));
			_loc33_ = ((_loc9_ - (_loc16_ * _loc24_)) + (_loc19_ * _loc24_));
			_loc34_ = ((_loc10_ - (_loc17_ * _loc24_)) + (_loc20_ * _loc24_));
			_loc35_ = ((_loc8_ - (_loc15_ * _loc24_)) - (_loc18_ * _loc24_));
			_loc36_ = ((_loc9_ - (_loc16_ * _loc24_)) - (_loc19_ * _loc24_));
			_loc37_ = ((_loc10_ - (_loc17_ * _loc24_)) - (_loc20_ * _loc24_));
			_loc38_ = ((_loc8_ + (_loc15_ * _loc24_)) - (_loc18_ * _loc24_));
			_loc39_ = ((_loc9_ + (_loc16_ * _loc24_)) - (_loc19_ * _loc24_));
			_loc40_ = ((_loc10_ + (_loc17_ * _loc24_)) - (_loc20_ * _loc24_));
			_loc41_ = (((_loc8_ + (_loc11_ * _loc22_)) + (_loc15_ * _loc24_)) + (_loc18_ * _loc24_));
			_loc42_ = (((_loc9_ + (_loc12_ * _loc22_)) + (_loc16_ * _loc24_)) + (_loc19_ * _loc24_));
			_loc43_ = (((_loc10_ + (_loc13_ * _loc22_)) + (_loc17_ * _loc24_)) + (_loc20_ * _loc24_));
			_loc44_ = (((_loc8_ + (_loc11_ * _loc22_)) - (_loc15_ * _loc24_)) + (_loc18_ * _loc24_));
			_loc45_ = (((_loc9_ + (_loc12_ * _loc22_)) - (_loc16_ * _loc24_)) + (_loc19_ * _loc24_));
			_loc46_ = (((_loc10_ + (_loc13_ * _loc22_)) - (_loc17_ * _loc24_)) + (_loc20_ * _loc24_));
			_loc47_ = (((_loc8_ + (_loc11_ * _loc22_)) - (_loc15_ * _loc24_)) - (_loc18_ * _loc24_));
			_loc48_ = (((_loc9_ + (_loc12_ * _loc22_)) - (_loc16_ * _loc24_)) - (_loc19_ * _loc24_));
			_loc49_ = (((_loc10_ + (_loc13_ * _loc22_)) - (_loc17_ * _loc24_)) - (_loc20_ * _loc24_));
			_loc50_ = (((_loc8_ + (_loc11_ * _loc22_)) + (_loc15_ * _loc24_)) - (_loc18_ * _loc24_));
			_loc51_ = (((_loc9_ + (_loc12_ * _loc22_)) + (_loc16_ * _loc24_)) - (_loc19_ * _loc24_));
			_loc52_ = (((_loc10_ + (_loc13_ * _loc22_)) + (_loc17_ * _loc24_)) - (_loc20_ * _loc24_));
			_loc53_ = (((_loc8_ + (_loc11_ * _loc22_)) + (_loc15_ * _loc25_)) + (_loc18_ * _loc25_));
			_loc54_ = (((_loc9_ + (_loc12_ * _loc22_)) + (_loc16_ * _loc25_)) + (_loc19_ * _loc25_));
			_loc55_ = (((_loc10_ + (_loc13_ * _loc22_)) + (_loc17_ * _loc25_)) + (_loc20_ * _loc25_));
			_loc56_ = (((_loc8_ + (_loc11_ * _loc22_)) - (_loc15_ * _loc25_)) + (_loc18_ * _loc25_));
			_loc57_ = (((_loc9_ + (_loc12_ * _loc22_)) - (_loc16_ * _loc25_)) + (_loc19_ * _loc25_));
			_loc58_ = (((_loc10_ + (_loc13_ * _loc22_)) - (_loc17_ * _loc25_)) + (_loc20_ * _loc25_));
			_loc59_ = (((_loc8_ + (_loc11_ * _loc22_)) - (_loc15_ * _loc25_)) - (_loc18_ * _loc25_));
			_loc60_ = (((_loc9_ + (_loc12_ * _loc22_)) - (_loc16_ * _loc25_)) - (_loc19_ * _loc25_));
			_loc61_ = (((_loc10_ + (_loc13_ * _loc22_)) - (_loc17_ * _loc25_)) - (_loc20_ * _loc25_));
			_loc62_ = (((_loc8_ + (_loc11_ * _loc22_)) + (_loc15_ * _loc25_)) - (_loc18_ * _loc25_));
			_loc63_ = (((_loc9_ + (_loc12_ * _loc22_)) + (_loc16_ * _loc25_)) - (_loc19_ * _loc25_));
			_loc64_ = (((_loc10_ + (_loc13_ * _loc22_)) + (_loc17_ * _loc25_)) - (_loc20_ * _loc25_));
			if ((((((((((((((_loc28_ > param1.nearClipping) && (_loc31_ > param1.nearClipping)) && (_loc34_ > param1.nearClipping)) && (_loc37_ > param1.nearClipping)) && (_loc40_ > param1.nearClipping)) && (_loc43_ > param1.nearClipping)) && (_loc46_ > param1.nearClipping)) && (_loc49_ > param1.nearClipping)) && (_loc52_ > param1.nearClipping)) && (_loc55_ > param1.nearClipping)) && (_loc58_ > param1.nearClipping)) && (_loc61_ > param1.nearClipping)) && (_loc64_ > param1.nearClipping)))
			{
				_loc3_.graphics.lineStyle(1, _loc7_);
				_loc3_.graphics.moveTo(((_loc29_ * param1.focalLength) / _loc31_), ((_loc30_ * param1.focalLength) / _loc31_));
				_loc3_.graphics.lineTo(((_loc32_ * param1.focalLength) / _loc34_), ((_loc33_ * param1.focalLength) / _loc34_));
				_loc3_.graphics.lineTo(((_loc35_ * param1.focalLength) / _loc37_), ((_loc36_ * param1.focalLength) / _loc37_));
				_loc3_.graphics.lineTo(((_loc38_ * param1.focalLength) / _loc40_), ((_loc39_ * param1.focalLength) / _loc40_));
				_loc3_.graphics.lineTo(((_loc29_ * param1.focalLength) / _loc31_), ((_loc30_ * param1.focalLength) / _loc31_));
				_loc3_.graphics.moveTo(((_loc41_ * param1.focalLength) / _loc43_), ((_loc42_ * param1.focalLength) / _loc43_));
				_loc3_.graphics.lineTo(((_loc44_ * param1.focalLength) / _loc46_), ((_loc45_ * param1.focalLength) / _loc46_));
				_loc3_.graphics.lineTo(((_loc47_ * param1.focalLength) / _loc49_), ((_loc48_ * param1.focalLength) / _loc49_));
				_loc3_.graphics.lineTo(((_loc50_ * param1.focalLength) / _loc52_), ((_loc51_ * param1.focalLength) / _loc52_));
				_loc3_.graphics.lineTo(((_loc41_ * param1.focalLength) / _loc43_), ((_loc42_ * param1.focalLength) / _loc43_));
				_loc3_.graphics.moveTo(((_loc53_ * param1.focalLength) / _loc55_), ((_loc54_ * param1.focalLength) / _loc55_));
				_loc3_.graphics.lineTo(((_loc56_ * param1.focalLength) / _loc58_), ((_loc57_ * param1.focalLength) / _loc58_));
				_loc3_.graphics.lineTo(((_loc59_ * param1.focalLength) / _loc61_), ((_loc60_ * param1.focalLength) / _loc61_));
				_loc3_.graphics.lineTo(((_loc62_ * param1.focalLength) / _loc64_), ((_loc63_ * param1.focalLength) / _loc64_));
				_loc3_.graphics.lineTo(((_loc53_ * param1.focalLength) / _loc55_), ((_loc54_ * param1.focalLength) / _loc55_));
				_loc3_.graphics.moveTo(((_loc26_ * param1.focalLength) / _loc28_), ((_loc27_ * param1.focalLength) / _loc28_));
				_loc3_.graphics.lineTo(((_loc53_ * param1.focalLength) / _loc55_), ((_loc54_ * param1.focalLength) / _loc55_));
				_loc3_.graphics.moveTo(((_loc26_ * param1.focalLength) / _loc28_), ((_loc27_ * param1.focalLength) / _loc28_));
				_loc3_.graphics.lineTo(((_loc56_ * param1.focalLength) / _loc58_), ((_loc57_ * param1.focalLength) / _loc58_));
				_loc3_.graphics.moveTo(((_loc26_ * param1.focalLength) / _loc28_), ((_loc27_ * param1.focalLength) / _loc28_));
				_loc3_.graphics.lineTo(((_loc59_ * param1.focalLength) / _loc61_), ((_loc60_ * param1.focalLength) / _loc61_));
				_loc3_.graphics.moveTo(((_loc26_ * param1.focalLength) / _loc28_), ((_loc27_ * param1.focalLength) / _loc28_));
				_loc3_.graphics.lineTo(((_loc62_ * param1.focalLength) / _loc64_), ((_loc63_ * param1.focalLength) / _loc64_));
				_loc3_.graphics.moveTo(((_loc29_ * param1.focalLength) / _loc31_), ((_loc30_ * param1.focalLength) / _loc31_));
				_loc3_.graphics.lineTo(((_loc41_ * param1.focalLength) / _loc43_), ((_loc42_ * param1.focalLength) / _loc43_));
				_loc3_.graphics.moveTo(((_loc32_ * param1.focalLength) / _loc34_), ((_loc33_ * param1.focalLength) / _loc34_));
				_loc3_.graphics.lineTo(((_loc44_ * param1.focalLength) / _loc46_), ((_loc45_ * param1.focalLength) / _loc46_));
				_loc3_.graphics.moveTo(((_loc35_ * param1.focalLength) / _loc37_), ((_loc36_ * param1.focalLength) / _loc37_));
				_loc3_.graphics.lineTo(((_loc47_ * param1.focalLength) / _loc49_), ((_loc48_ * param1.focalLength) / _loc49_));
				_loc3_.graphics.moveTo(((_loc38_ * param1.focalLength) / _loc40_), ((_loc39_ * param1.focalLength) / _loc40_));
				_loc3_.graphics.lineTo(((_loc50_ * param1.focalLength) / _loc52_), ((_loc51_ * param1.focalLength) / _loc52_));
			}
		}
		if ((_loc2_ & Debug.BOUNDS))
		{
			Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ, 0x99FF00);
		}
	}
#endif
}

void DirectionalLight::clonePropertiesFromDirectionalLight(DirectionalLight *src)
{
	clonePropertiesFromLight3D(src);
}


std::shared_ptr<DirectionalLight> new_DirectionalLight(uint32_t color)
{
	return std::make_shared<DirectionalLight>(Object3D::TT(DirectionalLight::TYPE), color);
}


std::shared_ptr<DirectionalLight> DirectionalLight_clone(const std::shared_ptr<DirectionalLight> &in)
{
	if (in->isType(DirectionalLight::TYPE)) {
		DirectionalLight *src = in.get();
		auto out = new_DirectionalLight(src->color);
		out->clonePropertiesFromDirectionalLight(src);
		return out;
	}

	throw 4113244;
}

