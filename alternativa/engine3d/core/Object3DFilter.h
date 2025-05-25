#ifndef OBJECT3DFILTER_H
#define OBJECT3DFILTER_H

class Object3D;

class Object3DFilter
{
public:
	virtual bool check(Object3D*) = 0; //true = ignore
};

#endif // OBJECT3DFILTER_H
