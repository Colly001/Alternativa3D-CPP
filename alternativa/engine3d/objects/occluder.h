#ifndef OCCLUDER_H
#define OCCLUDER_H

#include "../core/object3d.h"
#include <memory>

class Face;
class Edge;
class Vertex;
class Camera3D;
class Mesh;

class Occluder : public Object3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	Occluder(const TT &t);

	std::shared_ptr<Face> faceList;
	std::shared_ptr<Edge> edgeList;
	std::shared_ptr<Vertex> vertexList;
	double minSize;

	void createForm(std::shared_ptr<Mesh>, bool = false);
	void destroyForm();

	void clonePropertiesFromOccluder(Occluder*);

private:
	int calculateEdges();
	void draw(Camera3D*) override;
	void updateBounds(Object3D*, Object3D* = nullptr) override;

public:
	void destory();
};

std::shared_ptr<Occluder> new_Occluder();
std::shared_ptr<Occluder> Occluder_clone(const std::shared_ptr<Occluder> &in);

#endif // OCCLUDER_H
