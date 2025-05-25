#ifndef MATERIAL_H
#define MATERIAL_H

class Camera3D;
class Canvas;
class Face;
class QString;
class VertexBufferResource;
class IndexBufferResource;
class Object3D;

#include <stdint.h>
#include <memory>

class Material
{
	QString *_name;

	bool isTransparent;

public:

	std::weak_ptr<Material> ptr;

	class T
	{
		const char *_name;
	public:
		T(const char *name) : _name(name) {}
		const char* getName() const { return this->_name; }
	};

	const T *type;

	static const T *TYPE;

	bool isType(const T *type)
	{
		return this->type == type;
	}


	Material(const T *type);
	virtual ~Material();

	void setName(const QString &name);
	QString getName() const;

	bool zOffset;
	bool uploadEveryFrame;
	bool useVerticesNormals;

	uint32_t drawId;

	double alphaTestThreshold;

	virtual bool transparent();
	void setTransparent(bool enable);
	void clonePropertiesFromMaterial(Material*);

	virtual void drawOpaque(Camera3D*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*) = 0;
	virtual void drawTransparent(Camera3D*, VertexBufferResource*, IndexBufferResource*, int, int, Object3D*, bool=false) = 0;
	virtual void dispose() = 0;	
};

#endif // MATERIAL_H
