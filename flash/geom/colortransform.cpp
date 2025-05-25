#include "colortransform.h"
#include <QString>


void ColorTransform::concat(const ColorTransform *)
{

}


uint32_t ColorTransform::get(uint32_t color) const
{
	union Color
	{
		struct
		{
			uint8_t b;
			uint8_t g;
			uint8_t r;
			uint8_t a;
		};
		uint32_t u32;
	};

	Color c;
	c.u32 = color;
	c.b = std::max(0, std::min(255, int((this->blueMultiplier * c.b) + this->blueOffset)));
	c.g = std::max(0, std::min(255, int((this->greenMultiplier * c.g) + this->greenOffset)));
	c.r = std::max(0, std::min(255, int((this->redMultiplier * c.r) + this->redOffset)));
	c.a = std::max(0, std::min(255, int((this->alphaMultiplier * c.a) + this->alphaOffset)));

	return c.u32;
}
