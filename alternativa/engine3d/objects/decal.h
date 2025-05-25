#ifndef DECAL_H
#define DECAL_H

#include "mesh.h"

class Decal : public Mesh
{
public:

	static const T *TYPE;
	const T *__type_next;


	double attenuation;
	Decal(const TT &t);

	void clonePropertiesFromDecal(Decal *src);
	void draw(Camera3D*) override;
	object_ptr<VG> getVG(Camera3D*) override;
	void prepareResources() override;
};

std::shared_ptr<Decal> new_Decal();

#endif // DECAL_H
