#ifndef CONFLICTCONTAINER_H
#define CONFLICTCONTAINER_H

#include "../core/object3dcontainer.h"

class Face;

class ConflictContainer : public Object3DContainer
{
public:

	static const T *TYPE;
	const T *__type_next;

	bool resolveByAABB;
	bool resolveByOOBB;
	double threshold;

	ConflictContainer(const TT &t);

	void clonePropertiesFromConflictContainer(ConflictContainer *src);
	void draw(Camera3D*) override;
	void drawAABBGeometry(Camera3D *param1, object_ptr<VG> param2, bool param3 = true, bool param4 = false, bool param5 = true, int param6 = -1);
	void drawOOBBGeometry(Camera3D *param1, object_ptr<VG> param2);
	void drawConflictGeometry(Camera3D *param1, object_ptr<VG> param2);

private:
	object_ptr<VG> sortGeometry(object_ptr<VG>, bool, bool);

	Face* collectNode(Face *param1, Face *param2, Camera3D *param3, double param4, bool param5, Face *param6 = nullptr);
};

#endif // CONFLICTCONTAINER_H
