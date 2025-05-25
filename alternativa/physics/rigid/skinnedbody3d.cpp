#include "skinnedbody3d.h"
#include "alternativa/engine3d/objects/mesh.h"


SkinnedBody3D::SkinnedBody3D(const T *type, double invMass, const Matrix3 *invInertia) :
	Body3D(type, invMass, invInertia)
{
	skin = nullptr;
}


void SkinnedBody3D::updateSkin(double t)
{
	if (this->skin == nullptr)
		return;

	Vector3 v;
	Quaternion o;
	interpolate(t, &v, &o);
	this->skin->position.copy(&v);
	o.normalize();
	o.getEulerAngles(&v);
	this->skin->rotation.copy(&v);
}
