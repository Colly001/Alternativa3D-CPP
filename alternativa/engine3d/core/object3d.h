#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "alternativa/math/vector3.h"
#include "alternativa/math/matrix4.h"
#include "alternativa/physics/collision/types/boundbox.h"
#include "flash/display/BlendMode.h"
#include "object_ptr.h"
#include <memory>



class Object3DContainer;
class Matrix3D;
class Vertex;
class Camera3D;
class VG;
class Object3DFilter;
class RayIntersectionData;
class ColorTransform;
class Dictionary;



class Object3D : public std::enable_shared_from_this<Object3D>
{
private:
	QString *_name;

public:

	std::shared_ptr<Object3D> ptr()
	{
		return shared_from_this();
	}

	static inline std::shared_ptr<Object3D> ptr_safe(Object3D *t)
	{
		return t == nullptr ? nullptr : t->shared_from_this();
	}


	class T
	{
		const char *_name;
	public:

		T(const char *name) : _name(name) {}
		const char* getName() const { return this->_name; }
	};

	class TT
	{
	public:
		const T *type;
		const T *type_next;

		explicit TT(const T *_type, const T *_type_next = nullptr) :
			type(_type), type_next(_type_next) { }
	};

	const T *__type;
	const T *__type_first;
	const T *__type_next;


	static const T *TYPE;

	inline bool isType(const T *type)
	{
		return this->__type == type;
	}

	static std::shared_ptr<Vertex> boundVertexList;


	Object3D(const TT &t);

	virtual ~Object3D();

	void getMatrix3D(Matrix3D *matrix3D) const;
	void setMatrix3D(const Matrix3D *matrix3D);


	Vector3 position;
	Vector3 rotation;
	Vector3 scale;
	BoundBox bound;
	Matrix4 matrix;
	Matrix4 matrix_i;

	std::shared_ptr<ColorTransform> colorTransform;
	std::shared_ptr<ColorTransform> concatenatedColorTransform;

	double alpha;
	double shadowMapAlphaThreshold;
	double concatenatedAlpha;
	double colorConst[8];
	double softAttenuation;
	double transformConst[12];
	double depthMapAlphaThreshold;


	Object3DContainer *_parent;
	std::shared_ptr<Object3D> next;

	bool visible;
	bool useLight;
	bool useShadowMap;
	bool useDepth;
	bool mouseEnabled;

	BlendMode concatenatedBlendMode;
	BlendMode blendMode;

	int transformId;
	int culling;

	void calculateBounds();
	virtual void updateBounds(Object3D *, Object3D *);
	void setName(const QString &name);
	QString getName() const;
	bool compareName(const QString &name) const;


	Object3DContainer* getParent() { return _parent; }
	virtual void setParent(Object3DContainer *container);


	void clonePropertiesFromObject3D(const Object3D*);
	void composeMatrixFromSource(const Object3D*);
	void composeMatrix();

	void appendMatrix(Object3D *transform);

	virtual void draw(Camera3D*);
	virtual object_ptr<VG> getVG(Camera3D*);

	virtual void collectPlanes(Vector3*,
							   Vector3*,
							   Vector3*,
							   Vector3*,
							   Vector3*,
							   void*, //Vector.<Face>
							   Dictionary* = nullptr);


	virtual bool checkIntersection(double, double, double, double, double, double, double, Dictionary*);


	virtual bool intersectRay(const Vector3*, const Vector3*, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result);

	virtual void split(Vector3*, Vector3*, Vector3*, double, Object3D ** result_1, Object3D ** result_2);


	void composeAndAppend(Object3D*);
	void concat(Object3DContainer*);
	virtual int cullingInCamera(Camera3D*, int);
	void invertMatrix();

	void calculateInverseMatrix();

	virtual void prepareResources();

	double calculateResolution(int, int, int = 1, Matrix3D * = nullptr);

	void copyAndAppend(Object3D*, Object3D*);

	virtual void destroy();

	void removeFromParent();


	static bool boundIntersectRay(const Vector3 *param1,
								  const Vector3 *param2,
								  double param3,
								  double param4,
								  double param5,
								  double param6,
								  double param7,
								  double param8);
};

std::shared_ptr<Object3D> new_Object3D();
std::shared_ptr<Object3D> Object3D_clone(const std::shared_ptr<Object3D> &in);

#endif // OBJECT3D_H
