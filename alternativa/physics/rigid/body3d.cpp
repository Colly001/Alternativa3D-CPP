#include "body3d.h"

Body3D::Body3D(const T *type, double invMass, const Matrix3 *invInertia, bool localTank) :
	Body(type, invMass, invInertia, localTank)
{

}

Body3D::~Body3D()
{

}

void Body3D::destroy()
{
	delete this;
}

void Body3D::addToContainer(Scene3DContainer*)
{

}

void Body3D::removeFromContainer()
{

}

void Body3D::updateSkin(double)
{

}
