#ifndef FILLMATERIAL_H
#define FILLMATERIAL_H

#include "texturematerial.h"

class Camera3D;
class Canvas;
class Face;

class FillMaterial : public TextureMaterial
{
	int color;
	int lineColor;
	double alpha;
	double lineThickness;
public:

	static const T *TYPE;

	FillMaterial(const T *type, int color = 8355711, double lineThickness = 1, double alpha = -1, int lineColor = 16777215);
};


std::shared_ptr<FillMaterial> new_FillMaterial(int color = 8355711, double lineThickness = 1, double alpha = -1, int lineColor = 16777215);

#endif // FILLMATERIAL_H
