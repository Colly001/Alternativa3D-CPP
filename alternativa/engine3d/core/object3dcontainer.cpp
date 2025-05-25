#include "object3dcontainer.h"
#include "camera3d.h"
#include "vg.h"
#include "object3d_clone.h"
#include "light3d.h"
#include "alternativa/engine3d/core/rayintersectiondata.h"
#include "Object3DFilter.h"
#include <QDebug>


const Object3D::T *Object3DContainer::TYPE = new Object3D::T("Object3DContainer");


Object3DContainer::Object3DContainer(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	this->mouseChildren = true;
	this->numVisibleChildren = 0;
	this->lightList = nullptr;
}

Object3D* Object3DContainer::addChild(Object3D *object)
{
	if (object == nullptr)
	{
		throw 234524234; //(new TypeError("Parameter child must be non-null."));
	}

	if (object == this)
	{
		throw 425453543; //(new ArgumentError("An object cannot be added as a child of itself."));
	}

	Object3DContainer *container = _parent;
	while (container != nullptr)
	{
		if (container == object)
		{
			throw 2435242; //(new ArgumentError("An object cannot be added as a child to one of it's children (or children's children, etc.)."));
		}
		container = container->_parent;
	}

	if (object->_parent != nullptr)
	{
		object->_parent->removeChild(object);
	}

	this->addToList(object);

	return object;
}

Object3D* Object3DContainer::removeChild(Object3D *param1)
{
	if (param1 == nullptr)
	{
		throw 23452432; //(new TypeError("Parameter child must be non-null."));
	}

	if (param1->_parent != this)
	{
		//trace("Object3DContainer::removeChild The supplied Object3D must be a child of the caller.");
		return nullptr;
	}

	std::shared_ptr<Object3D> _loc2_;
	std::shared_ptr<Object3D> _loc3_ = this->childrenList;

	while (_loc3_ != nullptr)
	{
		if (_loc3_.get() == param1)
		{
			if (_loc2_ != nullptr)
			{
				_loc2_->next = _loc3_->next;
			}
			else
			{
				this->childrenList = _loc3_->next;
			}

			_loc3_->next = nullptr;
			_loc3_->setParent(nullptr);
			return param1;
		}

		_loc2_ = _loc3_;
		_loc3_ = _loc3_->next;
	}

	throw 45453453;
	//throw (new ArgumentError("Cannot remove child."));
}

Object3D *Object3DContainer::addChildAt(Object3D *param1, int param2)
{
	if (param1 == nullptr)
	{
		throw 345231;
		//throw (new TypeError("Parameter child must be non-null."));
	}

	if (param1 == this)
	{
		throw 34523;
		//throw (new ArgumentError("An object cannot be added as a child of itself."));
	}

	if (param2 < 0)
	{
		throw 3452345;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	Object3DContainer *_loc3_ = _parent;
	while (_loc3_ != nullptr)
	{
		if (_loc3_ == param1)
		{
			throw 34523132;
			//throw (new ArgumentError("An object cannot be added as a child to one of it's children (or children's children, etc.)."));
		}
		_loc3_ = _loc3_->_parent;
	}

	Object3D *_loc4_ = this->childrenList.get();

	int _loc5_ = 0;

	while (_loc5_ < param2)
	{
		if (_loc4_ == nullptr)
		{
			throw 123413432;
			//throw (new RangeError("The supplied index is out of bounds."));
		}
		_loc4_ = _loc4_->next.get();
		_loc5_++;
	}

	if (param1->_parent != nullptr)
	{
		param1->_parent->removeChild(param1);
	}
	this->addToList(param1, _loc4_);
	return param1;
}

Object3D* Object3DContainer::removeChildAt(int param1)
{
	if (param1 < 0)
	{
		throw 4141313;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	Object3D *_loc2_ = this->childrenList.get();
	int _loc3_ = 0;
	while (_loc3_ < param1)
	{
		if (_loc2_ == nullptr)
		{
			throw 41413131;
			//throw (new RangeError("The supplied index is out of bounds."));
		}
		_loc2_ = _loc2_->next.get();
		_loc3_++;
	}

	if (_loc2_ == nullptr)
	{
		throw 41413132;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	this->removeChild(_loc2_);

	return _loc2_;
}

Object3D *Object3DContainer::getChildAt(int param1)
{
	if (param1 < 0)
	{
		throw 44312324;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	Object3D *_loc2_ = this->childrenList.get();

	int _loc3_ = 0;

	while (_loc3_ < param1)
	{
		if (_loc2_ == nullptr)
		{
			throw 44312325;
			//throw (new RangeError("The supplied index is out of bounds."));
		}
		_loc2_ = _loc2_->next.get();
		_loc3_++;
	}

	if (_loc2_ == nullptr)
	{
		throw 4431234322;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	return _loc2_;
}

int Object3DContainer::getChildIndex(Object3D *param1)
{
	if (param1 == nullptr)
	{
		throw 1341322;
		//throw (new TypeError("Parameter child must be non-null."));
	}

	if (param1->_parent != this)
	{
		throw 13413221;
		//throw (new ArgumentError("The supplied Object3D must be a child of the caller."));
	}

	int _loc2_ = 0;
	Object3D *_loc3_ = this->childrenList.get();
	while (_loc3_ != nullptr)
	{
		if (_loc3_ == param1)
		{
			return (_loc2_);
		}
		_loc2_++;
		_loc3_ = _loc3_->next.get();
	}
	throw 13413224543;
	//throw (new ArgumentError("Cannot get child index."));
}

void Object3DContainer::setChildIndex(Object3D *param1, int param2)
{
	if (param1 == nullptr)
	{
		throw 134132211;
		//throw (new TypeError("Parameter child must be non-null."));
	}

	if (param1->_parent != this)
	{
		throw 134132212;
		//throw (new ArgumentError("The supplied Object3D must be a child of the caller."));
	}

	if (param2 < 0)
	{
		throw 13413223;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	Object3D *_loc3_ = this->childrenList.get();
	int _loc4_ = 0;
	while (_loc4_ < param2)
	{
		if (_loc3_ == nullptr)
		{
			throw 134132231;
			//throw (new RangeError("The supplied index is out of bounds."));
		}
		_loc3_ = _loc3_->next.get();
		_loc4_++;
	}
	this->removeChild(param1);
	this->addToList(param1, _loc3_);
}

void Object3DContainer::swapChildren(Object3D *param1, Object3D *param2)
{
	if (((param1 == nullptr) || (param2 == nullptr)))
	{
		throw 134132232;
		//throw (new TypeError("Parameter child must be non-null."));
	}

	if (((!(param1->_parent == this)) || (!(param2->_parent == this))))
	{
		throw 134132233;
		//throw (new ArgumentError("The supplied Object3D must be a child of the caller."));
	}

	if (param1 != param2)
	{
		if (param1->next.get() == param2)
		{
			this->removeChild(param2);
			this->addToList(param2, param1);
		}
		else
		{
			if (param2->next.get() == param1)
			{
				this->removeChild(param1);
				this->addToList(param1, param2);
			}
			else
			{
				Object3D *_loc3_ = param1->next.get();
				this->removeChild(param1);
				this->addToList(param1, param2);
				this->removeChild(param2);
				this->addToList(param2, _loc3_);
			}
		}
	}
}

void Object3DContainer::swapChildrenAt(int param1, int param2)
{
	if (((param1 < 0) || (param2 < 0)))
	{
		throw 13411234231;
		//throw (new RangeError("The supplied index is out of bounds."));
	}

	if (param1 != param2)
	{
		Object3D *_loc4_ = this->childrenList.get();
		int _loc3_ = 0;
		while (_loc3_ < param1)
		{
			if (_loc4_ == nullptr)
			{
				throw 13411234231;
				//throw (new RangeError("The supplied index is out of bounds."));
			}
			_loc4_ = _loc4_->next.get();
			_loc3_++;
		}
		if (_loc4_ == nullptr)
		{
			throw 13411234232;
			//throw (new RangeError("The supplied index is out of bounds."));
		}
		Object3D *_loc5_ = this->childrenList.get();
		_loc3_ = 0;
		while (_loc3_ < param2)
		{
			if (_loc5_ == nullptr)
			{
				throw 13411234233;
				//throw (new RangeError("The supplied index is out of bounds."));
			}
			_loc5_ = _loc5_->next.get();
			_loc3_++;
		}
		if (_loc5_ == nullptr)
		{
			throw 13411234234;
			//throw (new RangeError("The supplied index is out of bounds."));
		}
		if (_loc4_ != _loc5_)
		{
			if (_loc4_->next.get() == _loc5_)
			{
				this->removeChild(_loc5_);
				this->addToList(_loc5_, _loc4_);
			}
			else
			{
				if (_loc5_->next.get() == _loc4_)
				{
					this->removeChild(_loc4_);
					this->addToList(_loc4_, _loc5_);
				}
				else
				{
					Object3D *_loc6_ = _loc4_->next.get();
					this->removeChild(_loc4_);
					this->addToList(_loc4_, _loc5_);
					this->removeChild(_loc5_);
					this->addToList(_loc5_, _loc6_);
				}
			}
		}
	}
}

Object3D* Object3DContainer::getChildByName(const QString &param1)
{
	if (param1 == nullptr)
	{
		throw 45342112;
		//throw (new TypeError("Parameter name must be non-null."));
	}

	Object3D *_loc2_ = this->childrenList.get();
	while (_loc2_ != nullptr)
	{
		if (_loc2_->compareName(param1))
		{
			return _loc2_;
		}
		_loc2_ = _loc2_->next.get();
	}

	return nullptr;
}

bool Object3DContainer::contains(Object3D *param1)
{
	if (param1 == nullptr)
	{
		throw 13412453;
		//throw (new TypeError("Parameter child must be non-null."));
	}

	if (param1 == this)
	{
		return true;
	}

	Object3D *_loc2_ = this->childrenList.get();
	while (_loc2_ != nullptr)
	{

		if (_loc2_->isType(Object3DContainer::TYPE))
		{
			if (((Object3DContainer*)_loc2_)->contains(param1))
			{
				return true;
			}
		}
		else
		{
			if (_loc2_ == param1)
			{
				return true;
			}
		}
		_loc2_ = _loc2_->next.get();
	}

	return false;
}

int Object3DContainer::numChildren()
{
	int i = 0;
	Object3D *o = this->childrenList.get();
	while (o != nullptr)
	{
		i++;
		o = o->next.get();
	}
	return i;
}


//not used
bool Object3DContainer::checkIntersection(double param1, double param2, double param3, double param4, double param5, double param6, double param7, Dictionary *param8)
{
	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;
	(void)param7;
	(void)param8;

	qDebug("used Object3DContainer::checkIntersection");

#if 0
	Object3D *_loc9_ = this->childrenList.get();
	while (_loc9_ != nullptr)
	{
		if (((!(param8 == nullptr)) && (!(param8[_loc9_]))))
		{
			_loc9_.composeMatrix();
			_loc9_.invertMatrix();
			double _loc10_ = ((((_loc9_.ma * param1) + (_loc9_.mb * param2)) + (_loc9_.mc * param3)) + _loc9_.md);
			double _loc11_ = ((((_loc9_.me * param1) + (_loc9_.mf * param2)) + (_loc9_.mg * param3)) + _loc9_.mh);
			double _loc12_ = ((((_loc9_.mi * param1) + (_loc9_.mj * param2)) + (_loc9_.mk * param3)) + _loc9_.ml);
			double _loc13_ = (((_loc9_.ma * param4) + (_loc9_.mb * param5)) + (_loc9_.mc * param6));
			double _loc14_ = (((_loc9_.me * param4) + (_loc9_.mf * param5)) + (_loc9_.mg * param6));
			double _loc15_ = (((_loc9_.mi * param4) + (_loc9_.mj * param5)) + (_loc9_.mk * param6));
			if (((boundCheckIntersection(_loc10_, _loc11_, _loc12_, _loc13_, _loc14_, _loc15_, param7, _loc9_.boundMinX, _loc9_.boundMinY, _loc9_.boundMinZ, _loc9_.boundMaxX, _loc9_.boundMaxY, _loc9_.boundMaxZ)) && (_loc9_.checkIntersection(_loc10_, _loc11_, _loc12_, _loc13_, _loc14_, _loc15_, param7, param8))))
			{
				return (true);
			}
		}
		_loc9_ = _loc9_.next;
	}
#endif
	return false;
}

//not used
void Object3DContainer::collectPlanes(Vector3 *param1, Vector3 *param2, Vector3 *param3, Vector3 *param4, Vector3 *param5, void* param6, Dictionary *param7)
{
	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;
	(void)param7;

	qDebug("used Object3DContainer::collectPlanes");

	//QVector<Face*> param6
#if 0
	if (((!(param7 == null)) && (param7[this])))
	{
		return;
	}
	var _loc8_:Vector3D = calculateSphere(param1, param2, param3, param4, param5);
	if ((!(boundIntersectSphere(_loc8_, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ))))
	{
		_loc8_ = null;
		return;
	}
	_loc8_ = null;
	var _loc9_:Object3D = this->childrenList;
	while (_loc9_ != null)
	{
		_loc9_.composeAndAppend(this);
		_loc9_.collectPlanes(param1, param2, param3, param4, param5, param6, param7);
		_loc9_ = _loc9_.next;
	}
#endif
}

//not used
void Object3DContainer::clonePropertiesFromObject3DContainer(Object3DContainer *src)
{
	qDebug("used Object3DContainer::clonePropertiesFromObject3DContainer");

	std::shared_ptr<Object3D> _loc4_;
	Object3D::clonePropertiesFromObject3D(src);
	this->mouseChildren = src->mouseChildren;
	std::shared_ptr<Object3D> _loc3_ = src->childrenList;
	while (_loc3_ != nullptr)
	{
		std::shared_ptr<Object3D> _loc5_ = Object3DAll_clone(_loc3_);

		if (this->childrenList != nullptr)
		{
			_loc4_->next = _loc5_;
		}
		else
		{
			this->childrenList = _loc5_;
		}
		_loc4_ = _loc5_;
		_loc5_->setParent(this);
		_loc3_ = _loc3_->next;
	}
}

void Object3DContainer::draw(Camera3D *camera)
{
	this->numVisibleChildren = 0;
	Object3D *object3D = this->childrenList.get();
	while (object3D != nullptr)
	{
		if (object3D->visible == true)
		{	
			object3D->composeAndAppend(this);
			if (object3D->cullingInCamera(camera, culling) >= 0)
			{
				object3D->concat(this);
				if (this->visibleChildren.length() < (this->numVisibleChildren + 1))
					this->visibleChildren.resize(this->numVisibleChildren+1);

				this->visibleChildren[this->numVisibleChildren] = object3D->ptr();
				this->numVisibleChildren++;
			}
		}
		object3D = object3D->next.get();
	}

	if (this->numVisibleChildren > 0)
	{
		/*
		int _loc2_;
		if (((camera->debug) && ((_loc2_ = camera->checkInDebug(this)) > 0)))
		{
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(camera, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
			}
		}
		*/
		this->drawVisibleChildren(camera);
	}
}

void Object3DContainer::drawVisibleChildren(Camera3D *camera)
{
	int i = this->numVisibleChildren - 1;
	while (i >= 0)
	{
		this->visibleChildren[i]->draw(camera);
		this->visibleChildren[i] = nullptr;
		i--;
	}
}

object_ptr<VG> Object3DContainer::getVG(Camera3D *param1)
{
	object_ptr<VG> _loc2_;
	object_ptr<VG> _loc3_;

	Object3D *_loc4_ = this->childrenList.get();
	while (_loc4_ != nullptr)
	{
		if (_loc4_->visible)
		{
			_loc4_->composeAndAppend(this);
			if (_loc4_->cullingInCamera(param1, culling) >= 0)
			{
				_loc4_->concat(this);
				object_ptr<VG> _loc5_ = _loc4_->getVG(param1);
				if (_loc5_ != nullptr)
				{
					if (_loc2_ != nullptr)
					{
						_loc3_->next = _loc5_;
					}
					else
					{
						_loc2_ = _loc5_;
						_loc3_ = _loc5_;
					}
					while (_loc3_->next != nullptr)
					{
						_loc3_ = _loc3_->next;
					}
				}
			}
		}
		_loc4_ = _loc4_->next.get();
	}
	return _loc2_;
}

void Object3DContainer::updateBounds(Object3D *param1, Object3D *param2)
{
	Object3D *o = this->childrenList.get();
	while (o != nullptr)
	{
		if (param2 != nullptr)
		{
			o->composeAndAppend(param2);
		}
		else
		{
			o->composeMatrix();
		}
		o->updateBounds(param1, o);
		o = o->next.get();
	}
}

//not used
void Object3DContainer::split(Vector3 *param1, Vector3 *param2, Vector3 *param3, double param4, Object3D ** result_1, Object3D ** result_2)
{
	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;

	qDebug("used Object3DContainer::split");

	*result_1 = nullptr;
	*result_2 = nullptr;
#if 0
	Object3D *_loc10_ = nullptr;
	Object3D *_loc11_ = nullptr;

	//var _loc18_:Vector.<Object3D>;

	//var _loc5_:Vector.<Object3D> = new Vector.<Object3D>(2);
	//Vector3 *_loc6_ = calculatePlane(param1, param2, param3);
	Object3D *_loc7_ = this->childrenList.get();
	this->childrenList = nullptr;

	std::shared_ptr<Object3DContainer> _loc8_ = Object3DContainer_clone(this->ptr.lock());
	std::shared_ptr<Object3DContainer> _loc9_ = Object3DContainer_clone(this->ptr.lock());

	Object3D *_loc12_ = _loc7_;
	while (_loc12_ != nullptr)
	{
		Object3D *_loc13_ = _loc12_->next.get();
		_loc12_.next = null;
		_loc12_.setParent(null);
		_loc12_.composeMatrix();
		_loc12_.calculateInverseMatrix();
		Vector3 *_loc14_ = new Vector3(((((_loc12_.ima * param1->x) + (_loc12_.imb * param1->y)) + (_loc12_.imc * param1->z)) + _loc12_.imd), ((((_loc12_.ime * param1->x) + (_loc12_.imf * param1->y)) + (_loc12_.img * param1->z)) + _loc12_.imh), ((((_loc12_.imi * param1->x) + (_loc12_.imj * param1->y)) + (_loc12_.imk * param1->z)) + _loc12_.iml));
		Vector3 *_loc15_ = new Vector3(((((_loc12_.ima * param2->x) + (_loc12_.imb * param2->y)) + (_loc12_.imc * param2->z)) + _loc12_.imd), ((((_loc12_.ime * param2->x) + (_loc12_.imf * param2->y)) + (_loc12_.img * param2->z)) + _loc12_.imh), ((((_loc12_.imi * param2->x) + (_loc12_.imj * param2->y)) + (_loc12_.imk * param2->z)) + _loc12_.iml));
		Vector3 *_loc16_ = new Vector3(((((_loc12_.ima * param3->x) + (_loc12_.imb * param3->y)) + (_loc12_.imc * param3->z)) + _loc12_.imd), ((((_loc12_.ime * param3->x) + (_loc12_.imf * param3->y)) + (_loc12_.img * param3->z)) + _loc12_.imh), ((((_loc12_.imi * param3->x) + (_loc12_.imj * param3->y)) + (_loc12_.imk * param3->z)) + _loc12_.iml));
		int _loc17_ = _loc12_.testSplit(_loc14_, _loc15_, _loc16_, param4);
		if (_loc17_ < 0)
		{
			if (_loc10_ != null)
			{
				_loc10_.next = _loc12_;
			}
			else
			{
				_loc8_.childrenList = _loc12_;
			}
			_loc10_ = _loc12_;
			_loc12_.setParent(_loc8_);
		}
		else
		{
			if (_loc17_ > 0)
			{
				if (_loc11_ != null)
				{
					_loc11_.next = _loc12_;
				}
				else
				{
					_loc9_.childrenList = _loc12_;
				}
				_loc11_ = _loc12_;
				_loc12_.setParent(_loc9_);
			}
			else
			{
				_loc18_ = _loc12_.split(_loc14_, _loc15_, _loc16_, param4);
				double _loc19_ = _loc12_.distance;
				if (_loc18_[0] != null)
				{
					_loc12_ = _loc18_[0];
					if (_loc10_ != null)
					{
						_loc10_.next = _loc12_;
					}
					else
					{
						_loc8_.childrenList = _loc12_;
					}
					_loc10_ = _loc12_;
					_loc12_.setParent(_loc8_);
					_loc12_.distance = _loc19_;
				}
				if (_loc18_[1] != null)
				{
					_loc12_ = _loc18_[1];
					if (_loc11_ != null)
					{
						_loc11_.next = _loc12_;
					}
					else
					{
						_loc9_.childrenList = _loc12_;
					}
					_loc11_ = _loc12_;
					_loc12_.setParent(_loc9_);
					_loc12_.distance = _loc19_;
				}
			}
		}
		_loc12_ = _loc13_;
	}
	if (_loc10_ != null)
	{
		_loc8_.calculateBounds();
		_loc5_[0] = _loc8_;
	}
	if (_loc11_ != null)
	{
		_loc9_.calculateBounds();
		_loc5_[1] = _loc9_;
	}
	return _loc5_;
#endif

}

void Object3DContainer::addToList(Object3D *object_1, Object3D *object_2)
{	
	object_1->next = Object3D::ptr_safe(object_2);

	object_1->setParent(this);

	if (object_2 == this->childrenList.get())
	{
		this->childrenList = object_1->ptr();
	}
	else
	{
		Object3D *o = this->childrenList.get();
		while (o != nullptr)
		{
			if (o->next.get() == object_2)
			{
				o->next = Object3D::ptr_safe(object_1);
				break;
			}
			o = o->next.get();
		}
	}
}

void Object3DContainer::setParent(Object3DContainer *container)
{
	Object3DContainer *temp;

	if (container == nullptr)
	{
		temp = _parent;
		while (temp->_parent != nullptr)
		{
			temp = temp->_parent;
		}
		if (temp->lightList != nullptr)
		{
			this->transferLights(temp, this);
		}
	}
	else
	{
		if (this->lightList != nullptr)
		{
			temp = container;
			while (temp->_parent != nullptr)
			{
				temp = temp->_parent;
			}
			Light3D *_loc3_ = this->lightList;
			while (_loc3_->nextLight != nullptr)
			{
				_loc3_ = _loc3_->nextLight;
			}
			_loc3_->nextLight = temp->lightList;
			temp->lightList = this->lightList;
			this->lightList = nullptr;
		}
	}
	_parent = container;
}



bool Object3DContainer::intersectRay(const Vector3 *param1, const Vector3 *param2, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result)
{
	if (filter != nullptr && filter->check(this))
	{
		return false;
	}

	if (!this->boundIntersectRay(param1, param2, bound.MinX, bound.MinY, bound.MinZ, bound.MaxX, bound.MaxY, bound.MaxZ))
	{
		return false;
	}

	bool r = false;
	RayIntersectionData _loc10_;
	RayIntersectionData data;


	double _loc8_ = 1E22;
	Object3D *object = this->childrenList.get();
	while (object != nullptr)
	{
		Vector3 _loc5_;
		Vector3 _loc6_;
		object->composeMatrix();
		object->invertMatrix();

		_loc5_.x = ((((object->matrix.ma * param1->x) + (object->matrix.mb * param1->y)) + (object->matrix.mc * param1->z)) + object->matrix.md);
		_loc5_.y = ((((object->matrix.me * param1->x) + (object->matrix.mf * param1->y)) + (object->matrix.mg * param1->z)) + object->matrix.mh);
		_loc5_.z = ((((object->matrix.mi * param1->x) + (object->matrix.mj * param1->y)) + (object->matrix.mk * param1->z)) + object->matrix.ml);
		_loc6_.x = (((object->matrix.ma * param2->x) + (object->matrix.mb * param2->y)) + (object->matrix.mc * param2->z));
		_loc6_.y = (((object->matrix.me * param2->x) + (object->matrix.mf * param2->y)) + (object->matrix.mg * param2->z));
		_loc6_.z = (((object->matrix.mi * param2->x) + (object->matrix.mj * param2->y)) + (object->matrix.mk * param2->z));

		bool rr = object->intersectRay(&_loc5_, &_loc6_, filter, camera, &_loc10_);
		if (rr == true && _loc10_.time < _loc8_)
		{
			_loc8_ = _loc10_.time;
			data.copy(&_loc10_);
			r = true;
		}
		object = object->next.get();
	}

	result->copy(&data);

	return r;
}

//not used
void Object3DContainer::transferLights(Object3DContainer *param1, Object3DContainer *param2)
{
	(void)param1;
	(void)param2;
	qDebug("used Object3DContainer::transferLights");
#if 0
	var _loc4_:Light3D;
	var _loc5_:Light3D;
	var _loc6_:Light3D;
	var _loc3_:Object3D = this->childrenList;
	while (_loc3_ != null)
	{
		if ((_loc3_ is Light3D))
		{
			_loc4_ = (_loc3_ as Light3D);
			_loc5_ = null;
			_loc6_ = param1->lightList;
			while (_loc6_ != null)
			{
				if (_loc6_ == _loc4_)
				{
					if (_loc5_ != null)
					{
						_loc5_.nextLight = _loc6_.nextLight;
					}
					else
					{
						param1->lightList = _loc6_.nextLight;
					}
					_loc6_.nextLight = param2->lightList;
					param2->lightList = _loc6_;
					break;
				}
				_loc5_ = _loc6_;
				_loc6_ = _loc6_.nextLight;
			}
		}
		else
		{
			if ((_loc3_ is Object3DContainer))
			{
				(_loc3_ as Object3DContainer).transferLights(param1, param2);
			}
		}
		if (param1->lightList == null)
		{
			break;
		}
		_loc3_ = _loc3_.next;
	}
#endif
}


std::shared_ptr<Object3DContainer> new_Object3DContainer()
{
	return std::make_shared<Object3DContainer>(Object3D::TT(Object3DContainer::TYPE));
}

std::shared_ptr<Object3DContainer> Object3DContainer_clone(const std::shared_ptr<Object3DContainer> &in)
{
	if (in->isType(Object3DContainer::TYPE)) {
		auto out = new_Object3DContainer();
		out->clonePropertiesFromObject3DContainer(in.get());
		return out;
	}

	throw 4113245;
}
