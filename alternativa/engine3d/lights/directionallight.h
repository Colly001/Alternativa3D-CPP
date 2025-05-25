#ifndef DIRECTIONALLIGHT_H
#define DIRECTIONALLIGHT_H

#include "../core/light3d.h"

class DirectionalLight : public Light3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	DirectionalLight(const TT &t, uint32_t color);
	void lookAt(double, double, double);
	void drawDebug(Camera3D*); //override
	void clonePropertiesFromDirectionalLight(DirectionalLight *src);
};

std::shared_ptr<DirectionalLight> new_DirectionalLight(uint32_t color);
std::shared_ptr<DirectionalLight> DirectionalLight_clone(const std::shared_ptr<DirectionalLight> &in);

#endif // DIRECTIONALLIGHT_H
