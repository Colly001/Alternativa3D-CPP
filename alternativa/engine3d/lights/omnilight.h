#ifndef OMNILIGHT_H
#define OMNILIGHT_H

#include "../core/light3d.h"

class OmniLight : public Light3D
{
public:

	static const T *TYPE;
	const T *__type_next;

	double attenuationBegin;
	double attenuationEnd;

	OmniLight(const TT &t, uint32_t color, double attenuationBegin, double attenuationEnd);

	void drawDebug(Camera3D*);
	void updateBounds(Object3D*, Object3D* = nullptr) override;

	void clonePropertiesFromOmniLight(OmniLight *src);
};

std::shared_ptr<OmniLight> new_OmniLight(uint32_t color, double attenuationBegin, double attenuationEnd);
std::shared_ptr<OmniLight> OmniLight_clone(const std::shared_ptr<OmniLight> &in);

#endif // OMNILIGHT_H
