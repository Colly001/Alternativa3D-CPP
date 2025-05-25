#ifndef BODY3D_H
#define BODY3D_H

#include "../body.h"
class Scene3DContainer;

class Body3D : public Body
{
public:
	Body3D(const T *type, double invMass, const Matrix3 *invInertia, bool localTank = false);
	virtual ~Body3D();
	virtual void destroy();
	virtual void addToContainer(Scene3DContainer *container);
	virtual void removeFromContainer();
	virtual void updateSkin(double t);
};

#endif // BODY3D_H
