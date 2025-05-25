#ifndef VERTEX_H
#define VERTEX_H

#include <memory>

class Vertex : public std::enable_shared_from_this<Vertex>
{
public:

	std::shared_ptr<Vertex> ptr()
	{
		return shared_from_this();
	}

	static inline std::shared_ptr<Vertex> ptr_safe(Vertex *t)
	{
		return t == nullptr ? nullptr : t->shared_from_this();
	}

	static std::shared_ptr<Vertex> collector;


	double x;
	double y;
	double z;
	double u;
	double v;
	double normalX;
	double normalY;
	double normalZ;
	double cameraX;
	double cameraY;
	double cameraZ;
	double offset;
	int transformId;
	int index;
	int drawId;
	std::shared_ptr<Vertex> next;
	std::shared_ptr<Vertex> value;
	Vertex();
	void destroy();

	static std::shared_ptr<Vertex> createList(int count);
	static std::shared_ptr<Vertex> create();
};

#endif // VERTEX_H
