#ifndef BSP_H
#define BSP_H

#include "../core/object3d.h"
#include <qglobal.h>
#include <memory>

class Mesh;
class Face;
class VertexBufferResource;
class IndexBufferResource;
class Vertex;
class BSP_Node;
class Material;


class BSP : public Object3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	std::shared_ptr<VertexBufferResource> vertexBuffer;
	std::shared_ptr<IndexBufferResource> indexBuffer;
	QVector<std::shared_ptr<Face>> *faces;
	std::shared_ptr<Vertex> vertexList;
	std::shared_ptr<BSP_Node> root;
	int numTriangles;
	int clipping;
	double threshold;
	bool splitAnalysis;

	BSP(const TT &t);
	~BSP();

	void createTree(std::shared_ptr<Mesh>, bool = false);
	void destroyTree();
	void deleteResources();
	void clonePropertiesFromBSP(BSP *src);
	void setMaterialToAllFaces(Material *material);
	void prepareResources();
	void draw(Camera3D*) override;
	object_ptr<VG> getVG(Camera3D*) override;
	void destroy() override;


	void updateBounds(Object3D*, Object3D*) override;

	bool intersectRay(const Vector3 *origin, const Vector3 *direction, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result) override;

private:
	std::shared_ptr<BSP_Node> createNode(std::shared_ptr<Face>&);
	void destroyNode(std::shared_ptr<BSP_Node> &node);

	void setMaterialToNode(BSP_Node *node, Material *material);
	std::shared_ptr<Face> collectNode(BSP_Node *param1, std::shared_ptr<Face> param2 = nullptr);
	std::shared_ptr<Face> prepareNode(BSP_Node*, int, Camera3D*);
};

std::shared_ptr<BSP> new_BSP();
std::shared_ptr<BSP> BSP_clone(const std::shared_ptr<BSP> &in);

#endif // BSP_H
