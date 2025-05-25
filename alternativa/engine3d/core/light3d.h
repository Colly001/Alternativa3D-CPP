#ifndef LIGHT3D_H
#define LIGHT3D_H

#include "object3d.h"
#include <stdint.h>

class Camera3D;

class Light3D : public Object3D
{

public:

	static const T *TYPE;
	const T *__type_next;

	uint32_t color;
	double intensity;

	double cma;
	double cmb;
	double cmc;
	double cmd;
	double cme;
	double cmf;
	double cmg;
	double cmh;
	double cmi;
	double cmj;
	double cmk;
	double cml;

	Light3D *nextLight;

	Light3D(const TT &t);

	void clonePropertiesFromLight3D(Light3D*);

public:
	void calculateCameraMatrix(Camera3D*);
	void calculateObjectMatrix(Object3D *);
	void setParent(Object3DContainer*) override;
	void drawDebug(Camera3D*);
	void updateBounds(Object3D*, Object3D* = nullptr) override;
	int cullingInCamera(Camera3D*, int) override;
	bool checkFrustumCulling(Camera3D*);
	bool checkBoundsIntersection(Object3D*);
};

#endif // LIGHT3D_H
