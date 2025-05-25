#ifndef OBJECT3DCONTAINER_H
#define OBJECT3DCONTAINER_H

#include "object3d.h"
#include <QVector>

class Light3D;
class RayIntersectionData;
class Dictionary;
class Camera3D;
class VG;


class Object3DContainer : public Object3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	bool mouseChildren;


	std::shared_ptr<Object3D> childrenList;
	Light3D *lightList;

	QVector<std::shared_ptr<Object3D>> visibleChildren;
	int numVisibleChildren;

	Object3DContainer(const TT &t);


	Object3D* addChild(Object3D *param1);
	Object3D* removeChild(Object3D *param1);
	Object3D* addChildAt(Object3D *param1, int param2);
	Object3D* removeChildAt(int param1);
	Object3D* getChildAt(int param1);
	int getChildIndex(Object3D *param1);
	void setChildIndex(Object3D *param1, int param2);
	void swapChildren(Object3D *param1, Object3D *param2);
	void swapChildrenAt(int param1, int param2);
	Object3D* getChildByName(const QString &param1);
	bool contains(Object3D *param1);
	int numChildren();


	bool checkIntersection(double param1,
						   double param2,
						   double param3,
						   double param4,
						   double param5,
						   double param6,
						   double param7,
						   Dictionary *param8) override;

	void collectPlanes(Vector3 *param1,
					   Vector3 *param2,
					   Vector3 *param3,
					   Vector3 *param4,
					   Vector3 *param5,
					   void *param6,
					   Dictionary *param7 = nullptr) override;


	void clonePropertiesFromObject3DContainer(Object3DContainer *param1);
	void draw(Camera3D *param1) override;
	void drawVisibleChildren(Camera3D *param1);
	object_ptr<VG> getVG(Camera3D *param1) override;
	void updateBounds(Object3D *param1, Object3D *param2 = nullptr) override;
	void split(Vector3 *param1, Vector3 *param2, Vector3 *param3, double param4, Object3D ** result_1, Object3D ** result_2) override;
	void addToList(Object3D *param1, Object3D *param2 = nullptr);
	void setParent(Object3DContainer *container) override;

	bool intersectRay(const Vector3*, const Vector3*, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result) override;

private:
	void transferLights(Object3DContainer *param1, Object3DContainer *param2);
};

std::shared_ptr<Object3DContainer> new_Object3DContainer();
std::shared_ptr<Object3DContainer> Object3DContainer_clone(const std::shared_ptr<Object3DContainer> &in);

#endif // OBJECT3DCONTAINER_H
