#ifndef MESH_H
#define MESH_H

#include "../core/object3d.h"
#include <memory>
#include <qglobal.h>
#include <QVector>

class Face;
class Vertex;
class VertexBufferResource;
class IndexBufferResource;
class Material;


class Mesh : public Object3D
{
	std::shared_ptr<Face> transparentList;

protected:
	int opaqueLength;
	QVector<Material*> opaqueMaterials;
	QVector<int> opaqueBegins;
	QVector<int> opaqueNums;

public:

	static const T *TYPE;
	const T *__type_next;

	int sorting;
	int clipping;
	double threshold;

	std::shared_ptr<Face> _faceList;
	std::shared_ptr<Vertex> vertexList;


	std::shared_ptr<Face> faceList() { return _faceList; }
	std::shared_ptr<Face> constFaceList() const{ return _faceList; }

	void setFaceList(std::shared_ptr<Face> face, int);

	std::shared_ptr<VertexBufferResource> vertexBuffer;
	std::shared_ptr<IndexBufferResource> indexBuffer;

	//Material *material;

	int numOpaqueTriangles;
	int numTriangles;


	Mesh(const TT &t);
	virtual ~Mesh();
	virtual void destroy();

	void calculateFacesNormals(bool = true);
	void updateBounds(Object3D*, Object3D*) override;
	void optimizeForDynamicBSP(int);
	void clonePropertiesFromMesh(const Mesh*);
	void weldVertices(double = 0, double = 0);
	void weldFaces(double angleThreshold = 0, double uvThreshold = 0, double convexThreshold = 0, bool pairWeld = false);
	void calculateVerticesNormalsByAngle(double, double = 0);

	void prepareResources() override;

	void setMaterialToAllFaces(Material *material);

	virtual void addOpaque(Camera3D *camera);

	void draw(Camera3D *camera) override;

	void drawFaces(Camera3D *camera, Face *face);

	std::shared_ptr<Face> prepareFaces(Camera3D *camera, std::shared_ptr<Face> face);

	object_ptr<VG> getVG(Camera3D *param1) override;

	void addVerticesAndFaces(const double*, int, const double*, int, const int*, int, bool = false, Material * = nullptr);

	int getVertices(Vertex **out, int count);

	bool intersectRay(const Vector3 *origin, const Vector3 *direction, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result) override;

	std::shared_ptr<Vertex> addVertex(double x, double y, double z, double u = 0, double v = 0);

	std::shared_ptr<Face> addQuadFace(std::shared_ptr<Vertex>&, std::shared_ptr<Vertex>&, std::shared_ptr<Vertex>&, std::shared_ptr<Vertex>&, std::shared_ptr<Material> material = nullptr);

	void calculateVerticesNormalsBySmoothingGroups(double value = 0);

private:
	void deleteResources();
	void calculateNormalsAndRemoveDegenerateFaces();
};

std::shared_ptr<Mesh> new_Mesh();
std::shared_ptr<Mesh> Mesh_clone(const std::shared_ptr<Mesh> &in);

#endif // MESH_H
