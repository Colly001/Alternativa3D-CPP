#ifndef RESOURCE_H
#define RESOURCE_H

class Context3D;

class Resource
{
public:
	Context3D *context;
	Resource();
	virtual void dispose();
	virtual void reset();
	virtual bool available() const;
	virtual void create(Context3D *context);
	virtual bool upload();
	bool isCreated(Context3D *context) const;
	virtual void destroy(bool);
	virtual bool isLoaded();
};

#endif // RESOURCE_H
