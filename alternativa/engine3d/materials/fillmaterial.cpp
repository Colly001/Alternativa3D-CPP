#include "fillmaterial.h"
#include "alternativa/engine3d/core/face.h"
#include "graphics.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/wrapper.h"


const Material::T *FillMaterial::TYPE = new Material::T("FillMaterial");


FillMaterial::FillMaterial(const T *type, int color, double alpha, double lineThickness, int lineColor) :
	TextureMaterial(type)
{
	TextureMaterial::init();
	this->color = color;
	this->alpha = alpha;
	this->lineThickness = lineThickness;
	this->lineColor = lineColor;
}

std::shared_ptr<FillMaterial> new_FillMaterial(int color, double lineThickness, double alpha, int lineColor)
{
	auto p = std::make_shared<FillMaterial>(FillMaterial::TYPE, color, lineThickness, alpha, lineColor);
	p->ptr = p;
	return p;
}
