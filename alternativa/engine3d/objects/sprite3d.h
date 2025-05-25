#ifndef SPRITE3D_H
#define SPRITE3D_H

#include "../core/object3d.h"
#include <QVector>

class Material;
class RayIntersectionData;
class Dictionary;
class Vector3;
class Camera3D;
class VG;
class Face;


class Sprite3D : public Object3D
{
public:

	static const T *TYPE;
	const T *__type_next;


	QVector<std::shared_ptr<Face>> faces;
	std::shared_ptr<Material> material;

	double originX;
	double originY;
	double rotation;
	double width;
	double height;
	double topLeftU;
	double topLeftV;
	double bottomRightU;
	double bottomRightV;
	double lightConst[4];

	int sorting;
	int clipping;

	bool autoSize;
	bool depthTest;
	bool perspectiveScale;
	bool lighted;

	Sprite3D(const TT &t, double width, double height, std::shared_ptr<Material>);
	bool intersectRay(const Vector3*, const Vector3*, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result) override;
	void clonePropertiesFromSprite3D(Sprite3D*);
	void draw(Camera3D*);
	object_ptr<VG> getVG(Camera3D*);

private:
	void calculateLight(Camera3D*);
	std::shared_ptr<Face> calculateFace(Camera3D*);

public:
	void updateBounds(Object3D*, Object3D* = nullptr);

public:
	void destroy();
};

std::shared_ptr<Sprite3D> new_Sprite3D(double width, double height, std::shared_ptr<Material> material);
std::shared_ptr<Sprite3D> Sprite3D_clone(const std::shared_ptr<Sprite3D> &in);

#endif // SPRITE3D_H
