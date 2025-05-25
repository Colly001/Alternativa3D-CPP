#ifndef TUBELIGHT_H
#define TUBELIGHT_H

#include "../core/light3d.h"

class TubeLight : public Light3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	double length;
	double attenuationBegin;
	double attenuationEnd;
	double falloff;

	TubeLight(const TT &t, uint32_t color, double length, double attenuationBegin, double attenuationEnd, double falloff);
	void lookAt(double x, double y, double z);
	void drawDebug(Camera3D*);
	void updateBounds(Object3D *param1, Object3D *param2 = nullptr) override;

	void clonePropertiesFromTubeLight(TubeLight *src);
};

std::shared_ptr<TubeLight> new_TubeLight(uint32_t color, double length, double attenuationBegin, double attenuationEnd, double falloff);
std::shared_ptr<TubeLight> TubeLight_clone(const std::shared_ptr<TubeLight> &in);

#endif // TUBELIGHT_H
