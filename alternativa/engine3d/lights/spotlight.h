#ifndef SPOTLIGHT_H
#define SPOTLIGHT_H

#include "../core/light3d.h"

class SpotLight : public Light3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	double attenuationBegin;
	double attenuationEnd;
	double hotspot;
	double falloff;


	SpotLight(const TT &t, uint32_t color, double attenuationBegin, double attenuationEnd, double hotspot, double falloff);

	void lookAt(double x, double y, double z);
	void drawDebug(Camera3D*);
	void updateBounds(Object3D*, Object3D* = nullptr) override;

	void clonePropertiesFromSpotLight(SpotLight *src);
};

std::shared_ptr<SpotLight> new_SpotLight(uint32_t color, double attenuationBegin, double attenuationEnd, double hotspot, double falloff);
std::shared_ptr<SpotLight> SpotLight_clone(const std::shared_ptr<SpotLight> &in);

#endif // SPOTLIGHT_H
