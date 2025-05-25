#ifndef PLANE_H
#define PLANE_H

#include "alternativa/engine3d/objects/mesh.h"

class Plane : public Mesh
{

public:
	static const T *TYPE;
	const T *__type_next;


	Plane(const TT &t,
		  double param1 = 100,
		  double param2 = 100,
		  uint32_t param3 = 1,
		  uint32_t param4 = 1,
		  bool param5 = true,
		  bool param6 = false,
		  bool param7 = false,
		  std::shared_ptr<Material> param8 = nullptr,
		  std::shared_ptr<Material> param9 = nullptr);
private:

	std::shared_ptr<Vertex> createVertex(double x, double y, double z, double u, double v);

	void createFace(std::shared_ptr<Vertex> param1,
					std::shared_ptr<Vertex> param2,
					std::shared_ptr<Vertex> param3,
					std::shared_ptr<Vertex> param4,
					double param5,
					double param6,
					double param7,
					double param8,
					bool param9,
					bool param10,
					std::shared_ptr<Material> param11);
};

std::shared_ptr<Plane> new_Plane(double param1 = 100,
								 double param2 = 100,
								 uint32_t param3 = 1,
								 uint32_t param4 = 1,
								 bool param5 = true,
								 bool param6 = false,
								 bool param7 = false,
								 std::shared_ptr<Material> param8 = nullptr,
								 std::shared_ptr<Material> param9 = nullptr);

#endif // PLANE_H
