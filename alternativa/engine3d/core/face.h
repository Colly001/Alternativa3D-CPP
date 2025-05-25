#ifndef FACE_H
#define FACE_H

#include <qglobal.h>


#include <memory>
#include "object_ptr.h"

class Wrapper;
class Material;
class VG;
class Vertex;
class Point;
class Vector3;


class Face : public std::enable_shared_from_this<Face>
{
public:
	std::shared_ptr<Face> ptr()
	{
		return shared_from_this();
	}

	static inline std::shared_ptr<Face> ptr_safe(Face *t)
	{
		return t == nullptr ? nullptr : t->shared_from_this();
	}

	static std::shared_ptr<Face> collector;

	int smoothingGroups;


	std::shared_ptr<Material> material;
	std::shared_ptr<Face> next;
	std::shared_ptr<Face> processNext;
	std::shared_ptr<Face> processNegative;
	std::shared_ptr<Face> processPositive;
	std::shared_ptr<Wrapper> wrapper;
	object_ptr<VG> geometry;

	double normalX;
	double normalY;
	double normalZ;
	double distance;
	double offset;

	Face();
	~Face();

	static std::shared_ptr<Face> create();
	QVector<std::shared_ptr<Vertex>>* getVertices();
	void calculateBestSequenceAndNormal();
	void destroy();
	void getUV(Vector3 *in, Point *result);
};

#endif // FACE_H
