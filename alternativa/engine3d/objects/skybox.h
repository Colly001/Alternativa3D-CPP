#ifndef SKYBOX_H
#define SKYBOX_H

#include "mesh.h"

class Material;
class Matrix;


class SkyBox : public Mesh
{

	std::shared_ptr<Face> leftFace;
	std::shared_ptr<Face> rightFace;
	std::shared_ptr<Face> backFace;
	std::shared_ptr<Face> frontFace;
	std::shared_ptr<Face> bottomFace;
	std::shared_ptr<Face> topFace;

public:

	static const T *TYPE;
	const T *__type_next;


	bool autoSize;

	double reduceConst[4];

	enum
	{
		LEFT,
		RIGHT,
		BACK,
		FRONT,
		BOTTOM,
		TOP,
	};

	SkyBox(const TT &t);

	void init(double,
			  std::shared_ptr<Material>,
			  std::shared_ptr<Material>,
			  std::shared_ptr<Material>,
			  std::shared_ptr<Material>,
			  std::shared_ptr<Material>,
			  std::shared_ptr<Material>,
			  double = 0);


	Face* getSide(int param1);
	void clonePropertiesFromSkyBox(SkyBox*);
	std::shared_ptr<Vertex> createVertex(double param1, double param2, double param3, double param4, double param5);
	void draw(Camera3D *param1) override;
	void prepareResources() override;
	void addOpaque(Camera3D *param1) override;
	object_ptr<VG> getVG(Camera3D *param1) override;
	int cullingInCamera(Camera3D *param1, int param2) override;
	void transformUV(int direction, const Matrix *matrix);


private:
	void calculateTransform(Camera3D *param1);
	std::shared_ptr<Face> createQuad(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2, std::shared_ptr<Vertex> v3, std::shared_ptr<Vertex> v4, std::shared_ptr<Material> material);
};


std::shared_ptr<SkyBox> new_SkyBox();

#endif // SKYBOX_H
