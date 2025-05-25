#ifndef KDCONTAINER_H
#define KDCONTAINER_H

#include "conflictcontainer.h"
#include <QVector>
#include <QHash>


class KDNode;
class Occluder;
class Material;
class Decal;
class Device;
class VertexBufferResource;
class IndexBufferResource;


class KDContainer : public ConflictContainer
{
	double nearPlaneX;
	double nearPlaneY;
	double nearPlaneZ;
	double nearPlaneOffset;
	double farPlaneX;
	double farPlaneY;
	double farPlaneZ;
	double farPlaneOffset;
	double leftPlaneX;
	double leftPlaneY;
	double leftPlaneZ;
	double leftPlaneOffset;
	double rightPlaneX;
	double rightPlaneY;
	double rightPlaneZ;
	double rightPlaneOffset;
	double topPlaneX;
	double topPlaneY;
	double topPlaneZ;
	double topPlaneOffset;
	double bottomPlaneX;
	double bottomPlaneY;
	double bottomPlaneZ;
	double bottomPlaneOffset;
	int numOccluders;
	std::shared_ptr<Object3D> opaqueList;
	int transparentLength;
	bool isCreated;

	QVector<Vertex*> occluders;
	QVector<std::shared_ptr<VertexBufferResource>> receiversVertexBuffers;
	QVector<std::shared_ptr<IndexBufferResource>> receiversIndexBuffers;

	QVector<std::shared_ptr<Object3D>> transparent;
	QHash<Material*, bool> materials;

public:

	static const T *TYPE;
	const T *__type_next;

	double debugAlphaFade;
	bool ignoreChildrenInCollider;
	KDNode *root;
	bool batched;


	KDContainer(const TT &t);


	void createTree(QVector<std::shared_ptr<Object3D>> *objects, QVector<std::shared_ptr<Occluder>> *occluders = nullptr);

	void destroyTree();

	bool intersectRay(const Vector3*, const Vector3*, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result) override;

	bool checkIntersection(double param1, double param2, double param3, double param4, double param5, double param6, double param7, Dictionary *param8) override;

	void collectPlanes(Vector3 *param1,
					   Vector3 *param2,
					   Vector3 *param3,
					   Vector3 *param4,
					   Vector3 *param5,
					   void *param6,
					   Dictionary *param7 = nullptr) override;

	std::shared_ptr<Decal> createDecal(const Vector3 *param1,
									   const Vector3 *param2,
									   double param3,
									   double param4,
									   double param5,
									   double param6,
									   Material *param7);

	/*
			override public function clone():Object3D
			{
				var _loc1_:KDContainer = new KDContainer();
				_loc1_.clonePropertiesFrom(this);
				return (_loc1_);
			}
	*/

	void clonePropertiesFrom(KDContainer *src);
	void draw(Camera3D *param1) override;
	object_ptr<VG> getVG(Camera3D *param1) override;
	void uploadResources(Device *param1);
	void updateBounds(Object3D *param1, Object3D *param2 = nullptr) override;

private:
	void calculateCameraPlanes(double param1, double param2);
	void updateOccluders(Camera3D *param1);
	int cullingInContainer(int param1, double param2, double param3, double param4, double param5, double param6, double param7);
	KDNode *createNode(Object3D *param1, Object3D *param2, Object3D *param3, Object3D *param4, double param5, double param6, double param7, double param8, double param9, double param10);
	void drawNode(KDNode *param1, int param2, Camera3D *param3, VG *param4);
	bool occludeGeometry(Camera3D *param1, VG *param2);
	bool intersectRayNode(KDNode *param1, const Vector3 *param2, const Vector3 *param3, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result);
};


std::shared_ptr<KDContainer> new_KDContainer();


#endif // KDCONTAINER_H
