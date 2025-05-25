#ifndef VG_H
#define VG_H

class Vertex;
class Face;
class Object3D;
class Camera3D;
class Matrix4;


#include "object_ptr.h"
#include <memory>


class VG : public object
{
public:

	double boundMinX;
	double boundMinY;
	double boundMinZ;
	double boundMaxX;
	double boundMaxY;
	double boundMaxZ;
	double boundMin;
	double boundMax;

	object_ptr<VG> next;
	std::shared_ptr<Face> _faceStruct;

	int faceStruct_id;

	std::shared_ptr<Face> faceStruct()
	{
		return _faceStruct;
	}

	void setFaceStruct(std::shared_ptr<Face> face, int id);



	std::shared_ptr<Object3D> object;
	int sorting;
	int debug;
	int space;
	int numOccluders;
	bool viewAligned;


	std::shared_ptr<Vertex> boundVertexList;
	std::shared_ptr<Vertex> boundPlaneList;

	VG();
	~VG();


	static object_ptr<VG> create(std::shared_ptr<Object3D> &object, std::shared_ptr<Face> &faceStruct, int sorting, int debug, bool viewAligned);

	//used
	void destroy();

	//used
	void calculateAABB(const Matrix4 *matrix);

	//used
	void calculateOOBB(Object3D*);



	//not used
	void split(Camera3D *param1,
			   double param2,
			   double param3,
			   double param4,
			   double param5,
			   double param6);

	//not used
	void crop(Camera3D *param1,
			  double param2,
			  double param3,
			  double param4,
			  double param5,
			  double param6);


	//used
	void draw(Camera3D *param1, double param2, Object3D *param3);

private:

	//not used
	void destroyFaceStruct(std::shared_ptr<Face>);

	//used
	void calculateAABBStruct(Face* param1, int param2, const Matrix4 *matrix);



	//used
	void calculateOOBBStruct(Face *param1, int param2);

	//not used
	void updateAABBStruct(Face *param1, int param2);


	//not used
	void splitFaceStruct(Camera3D *param1,
						 Face *param2,
						 Face *param3,
						 double param4,
						 double param5,
						 double param6,
						 double param7,
						 double param8,
						 double param9);


	//not used
	Face *cropFaceStruct(Camera3D *param1,
						 Face *param2,
						 double param3,
						 double param4,
						 double param5,
						 double param6,
						 double param7,
						 double param8);


public:
	//used
	void transformStruct(Face *param1, int param2, const Matrix4 *matrix);
};

#endif // VG_H
