#ifndef SKINNEDBODY3D_H
#define SKINNEDBODY3D_H

#include "alternativa/physics/rigid/body3d.h"
#include <memory>

class Mesh;


class SkinnedBody3D : public Body3D
{
public:
	std::shared_ptr<Mesh> skin;

	SkinnedBody3D(const T *type, double invMass, const Matrix3 *invInertia);
	void updateSkin(double t) override;
};

#endif // SKINNEDBODY3D_H
