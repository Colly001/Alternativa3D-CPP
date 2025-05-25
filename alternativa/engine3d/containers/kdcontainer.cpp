#include "kdcontainer.h"
#include "alternativa/engine3d/objects/mesh.h"
#include "alternativa/engine3d/materials/material.h"
#include "alternativa/engine3d/core/object3d_clone.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/vg.h"
#include "alternativa/engine3d/objects/bsp.h"
#include "alternativa/engine3d/objects/occluder.h"
#include "kdnode.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "alternativa/engine3d/objects/decal.h"
#include "mymath.h"
#include "flash/geom/matrix3d.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/engine3d/core/view.h"
#include "alternativa/engine3d/materials/texturematerial.h"
#include "alternativa/engine3d/objects/sprite3d.h"
#include "alternativa/gfx/core/device.h"
#include "alternativa/gfx/core/bitmaptextureresource.h"
#include "alternativa/tanks/engine3d/animatedsprite3d.h"
#include "alternativa/tanks/sfx/simpleplane.h"
#include "alternativa/math/quaternion.h"
#include "alternativa/engine3d/core/rayintersectiondata.h"
#include "../core/Object3DFilter.h"
#include "alternativa/engine3d/core/shadowatlas.h"
#include "_global.h"
#include <QDebug>


const Object3D::T *KDContainer::TYPE = new Object3D::T("KDContainer");

static void destroyNode(KDNode*);
static void updateBoundsNode(KDNode *param1, Object3D *param2, Object3D *param3);

static std::shared_ptr<Object3D> createObjectBounds(Object3D *src)
{
	_function_name("KDContainer::createObjectBounds");

	std::shared_ptr<Object3D> result = new_Object3D();
	result->bound.infinity();
	src->composeMatrix();
	src->updateBounds(result.get(), src);
	result->matrix.copy(&src->matrix);
	return result;
}



KDContainer::KDContainer(const TT &t) :
	ConflictContainer(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	_function_name("KDContainer::KDContainer");
	batched = true;
	debugAlphaFade = 0.8;
	ignoreChildrenInCollider = false;
	root = nullptr;
	isCreated = false;
	transparentLength = 0;
}

void KDContainer::createTree(QVector<std::shared_ptr<Object3D>> *objects, QVector<std::shared_ptr<Occluder>> *occluders)
{
	_function_name("KDContainer::createTree");

	std::shared_ptr<Object3D> _loc8_;
	std::shared_ptr<Object3D> _loc9_;
	std::shared_ptr<Object3D> _loc10_;
	std::shared_ptr<Object3D> _loc11_;

	Vertex *_loc28_ = nullptr;

	int _loc3_;

	this->destroyTree();
	int _loc6_ = objects->length();
	int _loc7_ = ((!(occluders == nullptr)) ? int(occluders->length()) : int(0));

	QHash<Material*, std::shared_ptr<Mesh>> _loc13_;

	_loc3_ = 0;
	while (_loc3_ < _loc6_)
	{
		//_loc4_ = param1[_loc3_];
		//_loc12_ = ((_loc4_ is Mesh) ? (_loc4_ as Mesh).faceList.material : ((_loc4_ is BSP) ? (((_loc4_ as BSP).faces.length > 0) ? (_loc4_ as BSP).faces[0].material : null) : null));
		//if (_loc12_ != null)

		std::shared_ptr<Object3D> _loc4_ = objects->at(_loc3_);

		//std::shared_ptr<Mesh> _loc4_ = std::dynamic_pointer_cast<Mesh>(objects->at(_loc3_)->ptr.lock());

		Material *_loc12_;

		if (_loc4_->isType(Mesh::TYPE))
		{
			Mesh *mesh = (Mesh*)_loc4_.get();

			_loc12_ = mesh->faceList()->material.get();
		}
		else
		{
			if (_loc4_->isType(BSP::TYPE))
			{
				BSP *bsp = (BSP*)_loc4_.get();
				if (bsp->faces->length() > 0)
				{
					_loc12_ = bsp->faces->at(0)->material.get();
				}
				else
				{
					_loc12_ = nullptr;
				}
			}
			else
			{
				_loc12_ = nullptr;
			}
		}




		if (_loc12_ != nullptr)
		{
			this->materials[_loc12_] = true;
			if (_loc12_->transparent())
			{
				if (this->transparent.length() < (this->transparentLength+1))
					this->transparent.resize(this->transparentLength+1);
				this->transparent[this->transparentLength] = _loc4_;
				this->transparentLength++;
			}
			else
			{
				std::shared_ptr<Mesh> _loc29_ = _loc13_.value(_loc12_);

				if (_loc29_ == nullptr)
				{
					_loc29_ = new_Mesh();
					_loc13_.insert(_loc12_, _loc29_);
					_loc29_->next = this->opaqueList;
					this->opaqueList = _loc29_;

					//std::shared_ptr<Object3DContainer> container = std::dynamic_pointer_cast<Object3DContainer>(this->ptr.lock());

					_loc29_->setParent(this);
				}

				_loc4_ = Object3DAll_clone(_loc4_);


				_loc4_->composeMatrix();
				if (_loc4_->isType(Mesh::TYPE))
				{
					Mesh *_loc30_ = (Mesh*)_loc4_.get();
					if (_loc30_->faceList() != nullptr)
					{
						Vertex *_loc26_ = _loc30_->vertexList.get();
						while (_loc26_ != nullptr)
						{
							double _loc20_ = _loc26_->x;
							double _loc21_ = _loc26_->y;
							double _loc22_ = _loc26_->z;
							_loc26_->x = ((((_loc4_->matrix.ma * _loc20_) + (_loc4_->matrix.mb * _loc21_)) + (_loc4_->matrix.mc * _loc22_)) + _loc4_->matrix.md);
							_loc26_->y = ((((_loc4_->matrix.me * _loc20_) + (_loc4_->matrix.mf * _loc21_)) + (_loc4_->matrix.mg * _loc22_)) + _loc4_->matrix.mh);
							_loc26_->z = ((((_loc4_->matrix.mi * _loc20_) + (_loc4_->matrix.mj * _loc21_)) + (_loc4_->matrix.mk * _loc22_)) + _loc4_->matrix.ml);
							double _loc23_ = _loc26_->normalX;
							double _loc24_ = _loc26_->normalY;
							double _loc25_ = _loc26_->normalZ;
							_loc26_->normalX = (((_loc4_->matrix.ma * _loc23_) + (_loc4_->matrix.mb * _loc24_)) + (_loc4_->matrix.mc * _loc25_));
							_loc26_->normalY = (((_loc4_->matrix.me * _loc23_) + (_loc4_->matrix.mf * _loc24_)) + (_loc4_->matrix.mg * _loc25_));
							_loc26_->normalZ = (((_loc4_->matrix.mi * _loc23_) + (_loc4_->matrix.mj * _loc24_)) + (_loc4_->matrix.mk * _loc25_));
							_loc26_->transformId = 0;
							if (_loc26_->next == nullptr)
							{
								_loc28_ = _loc26_;
							}
							_loc26_ = _loc26_->next.get();
						}
						_loc28_->next = _loc29_->vertexList;
						_loc29_->vertexList = _loc30_->vertexList;
						_loc30_->vertexList = nullptr;
						Face *_loc27_ = _loc30_->faceList().get();
						while (_loc27_->next != nullptr)
						{
							_loc27_ = _loc27_->next.get();
						}
						_loc27_->next = _loc29_->faceList();
						_loc29_->setFaceList(_loc30_->faceList(), 10);
						_loc30_->setFaceList(nullptr, 11);
					}
				}
				else
				{
					if (_loc4_->isType(BSP::TYPE))
					{
						BSP *_loc31_ = (BSP*)_loc4_.get();
						if (_loc31_->root != nullptr)
						{
							Vertex *_loc26_ = _loc31_->vertexList.get();
							while (_loc26_ != nullptr)
							{
								double _loc20_ = _loc26_->x;
								double _loc21_ = _loc26_->y;
								double _loc22_ = _loc26_->z;
								_loc26_->x = ((((_loc4_->matrix.ma * _loc20_) + (_loc4_->matrix.mb * _loc21_)) + (_loc4_->matrix.mc * _loc22_)) + _loc4_->matrix.md);
								_loc26_->y = ((((_loc4_->matrix.me * _loc20_) + (_loc4_->matrix.mf * _loc21_)) + (_loc4_->matrix.mg * _loc22_)) + _loc4_->matrix.mh);
								_loc26_->z = ((((_loc4_->matrix.mi * _loc20_) + (_loc4_->matrix.mj * _loc21_)) + (_loc4_->matrix.mk * _loc22_)) + _loc4_->matrix.ml);
								double _loc23_ = _loc26_->normalX;
								double _loc24_ = _loc26_->normalY;
								double _loc25_ = _loc26_->normalZ;
								_loc26_->normalX = (((_loc4_->matrix.ma * _loc23_) + (_loc4_->matrix.mb * _loc24_)) + (_loc4_->matrix.mc * _loc25_));
								_loc26_->normalY = (((_loc4_->matrix.me * _loc23_) + (_loc4_->matrix.mf * _loc24_)) + (_loc4_->matrix.mg * _loc25_));
								_loc26_->normalZ = (((_loc4_->matrix.mi * _loc23_) + (_loc4_->matrix.mj * _loc24_)) + (_loc4_->matrix.mk * _loc25_));
								_loc26_->transformId = 0;
								if (_loc26_->next == nullptr)
								{
									_loc28_ = _loc26_;
								}
								_loc26_ = _loc26_->next.get();
							}
							_loc28_->next = _loc29_->vertexList;
							_loc29_->vertexList = _loc31_->vertexList;
							_loc31_->vertexList = nullptr;

							for (std::shared_ptr<Face> face : *_loc31_->faces)
							{
								face->next = _loc29_->faceList();
								_loc29_->setFaceList(face, 12);
							}

							_loc31_->faces->clear();
							_loc31_->root = nullptr;
						}
					}
				}
			}
		}
		_loc3_++;
	}

	{
		QHashIterator<Material*, std::shared_ptr<Mesh>> i(_loc13_);
		while (i.hasNext()) {
			i.next();
			std::shared_ptr<Mesh> mesh = i.value();
			mesh->calculateFacesNormals(true);
			mesh->calculateBounds();
		}
	}


	double _loc14_ = 1E22;
	double _loc15_ = 1E22;
	double _loc16_ = 1E22;
	double _loc17_ = -1E22;
	double _loc18_ = -1E22;
	double _loc19_ = -1E22;
	_loc3_ = 0;

	while (_loc3_ < _loc6_)
	{
		std::shared_ptr<Object3D> _loc4_ = objects->at(_loc3_);
		std::shared_ptr<Object3D> _loc5_ = ::createObjectBounds(_loc4_.get());

		if (_loc5_->bound.MinX <= _loc5_->bound.MaxX)
		{
			if (_loc4_->_parent != nullptr)
			{
				_loc4_->_parent->removeChild(_loc4_.get());
			}

			_loc4_->setParent(this);
			_loc4_->next = _loc8_;
			_loc8_ = _loc4_;
			_loc5_->next = _loc9_;
			_loc9_ = _loc5_;
			if (_loc5_->bound.MinX < _loc14_)
			{
				_loc14_ = _loc5_->bound.MinX;
			}
			if (_loc5_->bound.MaxX > _loc17_)
			{
				_loc17_ = _loc5_->bound.MaxX;
			}
			if (_loc5_->bound.MinY < _loc15_)
			{
				_loc15_ = _loc5_->bound.MinY;
			}
			if (_loc5_->bound.MaxY > _loc18_)
			{
				_loc18_ = _loc5_->bound.MaxY;
			}
			if (_loc5_->bound.MinZ < _loc16_)
			{
				_loc16_ = _loc5_->bound.MinZ;
			}
			if (_loc5_->bound.MaxZ > _loc19_)
			{
				_loc19_ = _loc5_->bound.MaxZ;
			}
		}
		_loc3_++;
	}

	_loc3_ = 0;
	while (_loc3_ < _loc7_)
	{
		std::shared_ptr<Object3D> _loc4_ = occluders->at(_loc3_);
		std::shared_ptr<Object3D> _loc5_ = createObjectBounds(_loc4_.get());
		if (_loc5_->bound.MinX <= _loc5_->bound.MaxX)
		{
			if ((!((((((_loc5_->bound.MinX < _loc14_) || (_loc5_->bound.MaxX > _loc17_)) || (_loc5_->bound.MinY < _loc15_)) || (_loc5_->bound.MaxY > _loc18_)) || (_loc5_->bound.MinZ < _loc16_)) || (_loc5_->bound.MaxZ > _loc19_))))
			{
				if (_loc4_->_parent != nullptr)
				{
					_loc4_->_parent->removeChild(_loc4_.get());
				}

				_loc4_->setParent(this);
				_loc4_->next = _loc10_;
				_loc10_ = _loc4_;
				_loc5_->next = _loc11_;
				_loc11_ = _loc5_;
			}
		}
		_loc3_++;
	}
	if (_loc8_ != nullptr)
	{
		this->root = this->createNode(_loc8_.get(), _loc9_.get(), _loc10_.get(), _loc11_.get(), _loc14_, _loc15_, _loc16_, _loc17_, _loc18_, _loc19_);

		QVector<std::shared_ptr<QVector<double>>> v;
		QVector<std::shared_ptr<QVector<uint32_t>>> p;

		v.append(std::make_shared<QVector<double>>());
		p.append(std::make_shared<QVector<uint32_t>>());

		this->root->createReceivers(&v, &p);

		for (int i = 0; i < v.length(); i++)
		{
			this->receiversVertexBuffers.append(std::make_shared<VertexBufferResource>(*v.at(i), 3));
			this->receiversIndexBuffers.append(std::make_shared<IndexBufferResource>(*p.at(i)));
		}
	}
}

void KDContainer::destroyTree()
{
	_function_name("KDContainer::destroyTree");

	{
		QHashIterator<Material*, bool> i(this->materials);
		while (i.hasNext()) {
			i.next();

			//TextureMaterial *_loc6_ = (TextureMaterial*)i.key();
			//if (_loc6_->texture != nullptr)
			//_loc6_->textureResource->reset();

			//if (_loc6_->_textureATF != nullptr)
			//_loc6_->textureATFResource->reset();

			//if (_loc6_->_textureATFAlpha != nullptr)
			//_loc6_->textureATFAlphaResource->reset();
		}
	}

	Object3D *_loc3_ = this->opaqueList.get();
	while (_loc3_ != nullptr)
	{
		if (_loc3_->isType(Mesh::TYPE))
		{
			Mesh *_loc4_ = (Mesh*)_loc3_;
			if (_loc4_->vertexBuffer != nullptr)
				_loc4_->vertexBuffer->reset();
			if (_loc4_->indexBuffer != nullptr)
				_loc4_->indexBuffer->reset();
			_loc4_->destroy();
		}
		else
		{
			if (_loc3_->isType(BSP::TYPE))
			{
				BSP *_loc5_ = (BSP*)_loc3_;

				if (_loc5_->vertexBuffer != nullptr)
					_loc5_->vertexBuffer->reset();

				if (_loc5_->indexBuffer != nullptr)
					_loc5_->indexBuffer->reset();

				_loc5_->destroy();
			}
		}
		_loc3_ = _loc3_->next.get();
	}

	int _loc1_ = 0;
	while (_loc1_ < this->transparentLength)
	{
		_loc3_ = this->transparent[_loc1_].get();
		if (_loc3_->isType(Mesh::TYPE))
		{
			Mesh *_loc4_ = (Mesh*)_loc3_;
			if (_loc4_->vertexBuffer != nullptr)
				_loc4_->vertexBuffer->reset();

			if (_loc4_->indexBuffer != nullptr)
				_loc4_->indexBuffer->reset();

			_loc4_->destroy();
		}
		else
		{
			if (_loc3_->isType(BSP::TYPE))
			{
				BSP *_loc5_ = (BSP*)_loc3_;
				if (_loc5_->vertexBuffer != nullptr)
					_loc5_->vertexBuffer.reset();

				if (_loc5_->indexBuffer != nullptr)
					_loc5_->indexBuffer.reset();

				_loc5_->destroy();
			}
		}
		_loc1_++;
	}

	this->materials.clear();
	this->opaqueList = nullptr;
	this->transparent.clear();
	this->transparentLength = 0;
	if (this->root != nullptr)
	{
		::destroyNode(this->root);
		this->root = nullptr;
	}
	_loc1_ = 0;
	while (_loc1_ < this->receiversVertexBuffers.length())
	{
		this->receiversVertexBuffers.at(_loc1_)->dispose();
		this->receiversIndexBuffers.at(_loc1_)->dispose();
		_loc1_++;
	}
	this->receiversVertexBuffers.clear();
	this->receiversIndexBuffers.clear();
	this->isCreated = false;
}


bool KDContainer::intersectRayNode(KDNode *param1, const Vector3 *param2, const Vector3 *param3, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result)
{
	_function_name("KDContainer::intersectRayNode");

	if (param1->negative != nullptr)
	{
		bool _loc12_ = param1->axis == 0;
		bool _loc13_ = param1->axis == 1;
		double _loc14_ = (((!(!(_loc12_))) ? param2->x : ((!(!(_loc13_))) ? param2->y : param2->z)) - param1->coord);
		if (_loc14_ > 0)
		{
			if (boundIntersectRay(param2,
								  param3,
								  param1->positive->bound.MinX,
								  param1->positive->bound.MinY,
								  param1->positive->bound.MinZ,
								  param1->positive->bound.MaxX,
								  param1->positive->bound.MaxY,
								  param1->positive->bound.MaxZ))
			{
				RayIntersectionData _loc6_;
				if (this->intersectRayNode(param1->positive, param2, param3, filter, camera, &_loc6_))
				{
					result->copy(&_loc6_);
					return true;
				}
			}
			double _loc7_ = ((!(!(_loc12_))) ? param3->x : ((!(!(_loc13_))) ? param3->y : param3->z));
			if (_loc7_ < 0)
			{
				Object3D *_loc8_ = param1->objectList.get();
				Object3D *_loc9_ = param1->objectBoundList.get();
				while (_loc8_ != nullptr)
				{
					if (boundIntersectRay(param2,
										  param3,
										  _loc9_->bound.MinX,
										  _loc9_->bound.MinY,
										  _loc9_->bound.MinZ,
										  _loc9_->bound.MaxX,
										  _loc9_->bound.MaxY,
										  _loc9_->bound.MaxZ))
					{
						_loc8_->composeMatrix();
						_loc8_->invertMatrix();
						Vector3 _loc10_;
						Vector3 _loc11_;
						_loc10_.x = ((((_loc8_->matrix.ma * param2->x) + (_loc8_->matrix.mb * param2->y)) + (_loc8_->matrix.mc * param2->z)) + _loc8_->matrix.md);
						_loc10_.y = ((((_loc8_->matrix.me * param2->x) + (_loc8_->matrix.mf * param2->y)) + (_loc8_->matrix.mg * param2->z)) + _loc8_->matrix.mh);
						_loc10_.z = ((((_loc8_->matrix.mi * param2->x) + (_loc8_->matrix.mj * param2->y)) + (_loc8_->matrix.mk * param2->z)) + _loc8_->matrix.ml);
						_loc11_.x = (((_loc8_->matrix.ma * param3->x) + (_loc8_->matrix.mb * param3->y)) + (_loc8_->matrix.mc * param3->z));
						_loc11_.y = (((_loc8_->matrix.me * param3->x) + (_loc8_->matrix.mf * param3->y)) + (_loc8_->matrix.mg * param3->z));
						_loc11_.z = (((_loc8_->matrix.mi * param3->x) + (_loc8_->matrix.mj * param3->y)) + (_loc8_->matrix.mk * param3->z));
						RayIntersectionData _loc6_;
						if (_loc8_->intersectRay(&_loc10_, &_loc11_, filter, camera, &_loc6_))
						{
							result->copy(&_loc6_);
							return true;
						}
					}
					_loc8_ = _loc8_->next.get();
					_loc9_ = _loc9_->next.get();
				}
				if (boundIntersectRay(param2,
									  param3,
									  param1->negative->bound.MinX,
									  param1->negative->bound.MinY,
									  param1->negative->bound.MinZ,
									  param1->negative->bound.MaxX,
									  param1->negative->bound.MaxY,
									  param1->negative->bound.MaxZ))
				{
					return this->intersectRayNode(param1->negative, param2, param3, filter, camera, result);
				}
			}
		}
		else
		{
			if (boundIntersectRay(param2,
								  param3,
								  param1->negative->bound.MinX,
								  param1->negative->bound.MinY,
								  param1->negative->bound.MinZ,
								  param1->negative->bound.MaxX,
								  param1->negative->bound.MaxY,
								  param1->negative->bound.MaxZ))
			{
				RayIntersectionData _loc6_;
				if (this->intersectRayNode(param1->negative, param2, param3, filter, camera, &_loc6_))
				{
					result->copy(&_loc6_);
					return true;
				}
			}
			double _loc7_ = ((!(!(_loc12_))) ? param3->x : ((!(!(_loc13_))) ? param3->y : param3->z));
			if (_loc7_ > 0)
			{
				Object3D *_loc8_ = param1->objectList.get();
				Object3D *_loc9_ = param1->objectBoundList.get();
				while (_loc8_ != nullptr)
				{
					if (boundIntersectRay(param2,
										  param3,
										  _loc9_->bound.MinX,
										  _loc9_->bound.MinY,
										  _loc9_->bound.MinZ,
										  _loc9_->bound.MaxX,
										  _loc9_->bound.MaxY,
										  _loc9_->bound.MaxZ))
					{
						_loc8_->composeMatrix();
						_loc8_->invertMatrix();

						Vector3 _loc10_;
						Vector3 _loc11_;

						_loc10_.x = ((((_loc8_->matrix.ma * param2->x) + (_loc8_->matrix.mb * param2->y)) + (_loc8_->matrix.mc * param2->z)) + _loc8_->matrix.md);
						_loc10_.y = ((((_loc8_->matrix.me * param2->x) + (_loc8_->matrix.mf * param2->y)) + (_loc8_->matrix.mg * param2->z)) + _loc8_->matrix.mh);
						_loc10_.z = ((((_loc8_->matrix.mi * param2->x) + (_loc8_->matrix.mj * param2->y)) + (_loc8_->matrix.mk * param2->z)) + _loc8_->matrix.ml);
						_loc11_.x = (((_loc8_->matrix.ma * param3->x) + (_loc8_->matrix.mb * param3->y)) + (_loc8_->matrix.mc * param3->z));
						_loc11_.y = (((_loc8_->matrix.me * param3->x) + (_loc8_->matrix.mf * param3->y)) + (_loc8_->matrix.mg * param3->z));
						_loc11_.z = (((_loc8_->matrix.mi * param3->x) + (_loc8_->matrix.mj * param3->y)) + (_loc8_->matrix.mk * param3->z));

						RayIntersectionData _loc6_;
						if (_loc8_->intersectRay(&_loc10_, &_loc11_, filter, camera, &_loc6_))
						{
							result->copy(&_loc6_);
							return true;
						}
					}
					_loc8_ = _loc8_->next.get();
					_loc9_ = _loc9_->next.get();
				}
				if (boundIntersectRay(param2,
									  param3,
									  param1->positive->bound.MinX,
									  param1->positive->bound.MinY,
									  param1->positive->bound.MinZ,
									  param1->positive->bound.MaxX,
									  param1->positive->bound.MaxY,
									  param1->positive->bound.MaxZ))
				{
					return this->intersectRayNode(param1->positive, param2, param3, filter, camera, result);
				}
			}
		}
		return false;
	}

	bool n = false;


	double _loc15_ = 1E22;
	Object3D *_loc8_ = param1->objectList.get();
	while (_loc8_ != nullptr)
	{
		_loc8_->composeMatrix();
		_loc8_->invertMatrix();

		Vector3 _loc10_;
		Vector3 _loc11_;

		_loc10_.x = ((((_loc8_->matrix.ma * param2->x) + (_loc8_->matrix.mb * param2->y)) + (_loc8_->matrix.mc * param2->z)) + _loc8_->matrix.md);
		_loc10_.y = ((((_loc8_->matrix.me * param2->x) + (_loc8_->matrix.mf * param2->y)) + (_loc8_->matrix.mg * param2->z)) + _loc8_->matrix.mh);
		_loc10_.z = ((((_loc8_->matrix.mi * param2->x) + (_loc8_->matrix.mj * param2->y)) + (_loc8_->matrix.mk * param2->z)) + _loc8_->matrix.ml);
		_loc11_.x = (((_loc8_->matrix.ma * param3->x) + (_loc8_->matrix.mb * param3->y)) + (_loc8_->matrix.mc * param3->z));
		_loc11_.y = (((_loc8_->matrix.me * param3->x) + (_loc8_->matrix.mf * param3->y)) + (_loc8_->matrix.mg * param3->z));
		_loc11_.z = (((_loc8_->matrix.mi * param3->x) + (_loc8_->matrix.mj * param3->y)) + (_loc8_->matrix.mk * param3->z));
		RayIntersectionData _loc6_;
		bool r = _loc8_->intersectRay(&_loc10_, &_loc11_, filter, camera, &_loc6_);
		if (r == true && _loc6_.time < _loc15_)
		{
			_loc15_ = _loc6_.time;
			result->copy(&_loc6_);
			n = true;
		}
		_loc8_ = _loc8_->next.get();
	}

	return n;
}


bool KDContainer::intersectRay(const Vector3 *param1, const Vector3 *param2, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result)
{
	_function_name("KDContainer::intersectRay");

	if (filter != nullptr && filter->check(this))
	{
		return false;
	}

	if (!boundIntersectRay(param1, param2, bound.MinX, bound.MinY, bound.MinZ, bound.MaxX, bound.MaxY, bound.MaxZ))
	{
		return false;
	}


	bool r5 = ConflictContainer::intersectRay(param1, param2, filter, camera, result);

	if (this->root != nullptr && boundIntersectRay(param1,
												   param2,
												   this->root->bound.MinX,
												   this->root->bound.MinY,
												   this->root->bound.MinZ,
												   this->root->bound.MaxX,
												   this->root->bound.MaxY,
												   this->root->bound.MaxZ))
	{
		RayIntersectionData _loc6_;
		bool rr = this->intersectRayNode(this->root, param1, param2, filter, camera, &_loc6_);
		if (rr == true && (r5 == false || (_loc6_.time < result->time)))
		{
			result->copy(&_loc6_);
			r5 = true;
		}
	}

	return r5;
}



//not used
bool KDContainer::checkIntersection(double param1,
									double param2,
									double param3,
									double param4,
									double param5,
									double param6,
									double param7,
									Dictionary *param8)
{
	_function_name("KDContainer::checkIntersection");

	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;
	(void)param7;
	(void)param8;

	qDebug("used KDContainer::checkIntersection");

#if 0
	if (ConflictContainer::checkIntersection(param1, param2, param3, param4, param5, param6, param7, param8))
	{
		return (true);
	}
	if (((!(this->root == null)) && (boundCheckIntersection(param1, param2, param3, param4, param5, param6, param7, this->root.boundMinX, this->root.boundMinY, this->root.boundMinZ, this->root.boundMaxX, this->root.boundMaxY, this->root.boundMaxZ))))
	{
		return (this->checkIntersectionNode(this->root, param1, param2, param3, param4, param5, param6, param7, param8));
	}
#endif
	return false;
}


//not used
void KDContainer::collectPlanes(Vector3 *param1,
								Vector3 *param2,
								Vector3 *param3,
								Vector3 *param4,
								Vector3 *param5,
								void *param6,
								Dictionary *param7)
{
	_function_name("KDContainer::collectPlanes");

	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;
	(void)param7;

	qDebug("used KDContainer::collectPlanes");

#if 0
	//QVector<Face *> *param6, Dictionary *param7

	var _loc9_:Object3D;
	if (((!(param7 == null)) && (param7[this])))
	{
		return;
	}
	var _loc8_:Vector3D = calculateSphere(param1, param2, param3, param4, param5, treeSphere);
	if ((!(this->ignoreChildrenInCollider)))
	{
		if ((!(boundIntersectSphere(_loc8_, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ))))
		{
			_loc8_ = null;
			return;
		}
		_loc8_ = null;
		_loc9_ = childrenList;
		while (_loc9_ != null)
		{
			_loc9_->composeAndAppend(this);
			_loc9_->collectPlanes(param1, param2, param3, param4, param5, param6, param7);
			_loc9_ = _loc9_->next;
		}
	}
	if (((!(this->root == null)) && (boundIntersectSphere(_loc8_, this->root.boundMinX, this->root.boundMinY, this->root.boundMinZ, this->root.boundMaxX, this->root.boundMaxY, this->root.boundMaxZ))))
	{
		this->collectPlanesNode(this->root, _loc8_, param1, param2, param3, rotation, param5, param6, param7);
	}
#endif
}


std::shared_ptr<Decal> KDContainer::createDecal(const Vector3 *hitPosition,
												const Vector3 *param2,
												double radius,
												double rotation,
												double param5,
												double attenuation,
												Material *param7)
{
	_function_name("KDContainer::createDecal");

	std::shared_ptr<Decal> decal = new_Decal();
	decal->attenuation = attenuation;

	{
		Quaternion q1, q2, q3, qr;

		q1.setFromAxisAngle(&Vector3::Z_AXIS, rotation + 1);
		q2.setFromAxisAngle(&Vector3::X_AXIS, MyMath::atan2(-param2->z, MyMath::sqrt((param2->x * param2->x) + (param2->y * param2->y))) - 1.5707963267945);
		q3.setFromAxisAngle(&Vector3::Z_AXIS, -MyMath::atan2(-param2->x, -param2->y));

		qr.reset();
		qr.append(&q1);
		qr.append(&q2);
		qr.append(&q3);
		qr.normalize();

		Vector3 eulerAngles;
		qr.getEulerAngles(&eulerAngles);
		decal->rotation.copy(&eulerAngles);
	}

	decal->position.copy(hitPosition);

	decal->composeMatrix();

	decal->bound.MinX = -radius;
	decal->bound.MaxX = radius;
	decal->bound.MinY = -radius;
	decal->bound.MaxY = radius;
	decal->bound.MinZ = -attenuation;
	decal->bound.MaxZ = attenuation;
	double _loc10_ = 1E22;
	double _loc11_ = 1E22;
	double _loc12_ = 1E22;
	double _loc13_ = -1E22;
	double _loc14_ = -1E22;
	double _loc15_ = -1E22;
	Vertex *_loc16_ = boundVertexList.get();
	_loc16_->x = decal->bound.MinX;
	_loc16_->y = decal->bound.MinY;
	_loc16_->z = decal->bound.MinZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MaxX;
	_loc16_->y = decal->bound.MinY;
	_loc16_->z = decal->bound.MinZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MinX;
	_loc16_->y = decal->bound.MaxY;
	_loc16_->z = decal->bound.MinZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MaxX;
	_loc16_->y = decal->bound.MaxY;
	_loc16_->z = decal->bound.MinZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MinX;
	_loc16_->y = decal->bound.MinY;
	_loc16_->z = decal->bound.MaxZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MaxX;
	_loc16_->y = decal->bound.MinY;
	_loc16_->z = decal->bound.MaxZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MinX;
	_loc16_->y = decal->bound.MaxY;
	_loc16_->z = decal->bound.MaxZ;
	_loc16_ = _loc16_->next.get();
	_loc16_->x = decal->bound.MaxX;
	_loc16_->y = decal->bound.MaxY;
	_loc16_->z = decal->bound.MaxZ;
	_loc16_ = boundVertexList.get();
	while (_loc16_ != nullptr)
	{
		_loc16_->cameraX = ((((decal->matrix.ma * _loc16_->x) + (decal->matrix.mb * _loc16_->y)) + (decal->matrix.mc * _loc16_->z)) + decal->matrix.md);
		_loc16_->cameraY = ((((decal->matrix.me * _loc16_->x) + (decal->matrix.mf * _loc16_->y)) + (decal->matrix.mg * _loc16_->z)) + decal->matrix.mh);
		_loc16_->cameraZ = ((((decal->matrix.mi * _loc16_->x) + (decal->matrix.mj * _loc16_->y)) + (decal->matrix.mk * _loc16_->z)) + decal->matrix.ml);
		if (_loc16_->cameraX < _loc10_)
		{
			_loc10_ = _loc16_->cameraX;
		}
		if (_loc16_->cameraX > _loc13_)
		{
			_loc13_ = _loc16_->cameraX;
		}
		if (_loc16_->cameraY < _loc11_)
		{
			_loc11_ = _loc16_->cameraY;
		}
		if (_loc16_->cameraY > _loc14_)
		{
			_loc14_ = _loc16_->cameraY;
		}
		if (_loc16_->cameraZ < _loc12_)
		{
			_loc12_ = _loc16_->cameraZ;
		}
		if (_loc16_->cameraZ > _loc15_)
		{
			_loc15_ = _loc16_->cameraZ;
		}
		_loc16_ = _loc16_->next.get();
	}

	decal->invertMatrix();
	if (param5 > (MyMath::PI / 2))
	{
		param5 = (MyMath::PI / 2);
	}

	if (this->root != nullptr)
	{
		this->root->collectPolygons(decal.get(), MyMath::sqrt((((radius * radius) + (radius * radius)) + (attenuation * attenuation))), (MyMath::cos(param5) - 0.001), _loc10_, _loc13_, _loc11_, _loc14_, _loc12_, _loc15_);
	}
	if (decal->faceList() != nullptr)
	{
		decal->calculateBounds();
	}
	else
	{
		decal->bound.MinX = -1;
		decal->bound.MinY = -1;
		decal->bound.MinZ = -1;
		decal->bound.MaxX = 1;
		decal->bound.MaxY = 1;
		decal->bound.MaxZ = 1;
	}
	decal->setMaterialToAllFaces(param7);
	return decal;
}


//not used
void KDContainer::clonePropertiesFrom(KDContainer *src)
{
	_function_name("KDContainer::clonePropertiesFrom");
	qDebug("used KDContainer::clonePropertiesFrom");

	(void)src;
	/*
	super.clonePropertiesFrom(src);
	var _loc2_:KDContainer = (param1 as KDContainer);
	this->debugAlphaFade = _loc2_->debugAlphaFade;
	if (_loc2_->root != null)
	{
		this->root = _loc2_->cloneNode(_loc2_->root, this);
	}
	*/
}

void KDContainer::draw(Camera3D *param1)
{
	_function_name("KDContainer::draw");

	int _loc10_ = 0;

	this->uploadResources(param1->device);
	calculateInverseMatrix();

	/*
	int _loc6_ = ((!(!(param1->debug))) ? int(param1->checkInDebug(this)) : int(0));
	if ((_loc6_ & Debug.BOUNDS))
	{
		Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
	}
	*/

	if (this->batched)
	{
		bool _loc7_ = param1->debug;
		/*
		if (((_loc6_) && (_loc6_ & Debug.NODES)))
		{
			param1->debug = false;
		}
		*/

		Object3D *_loc3_ = this->opaqueList.get();
		while (_loc3_ != nullptr)
		{
			_loc3_->matrix.copy(&this->matrix);


			_loc3_->concat(this);

			_loc3_->draw(param1);
			if (((((!(param1->view->constrained)) && (!(param1->shadowMap == nullptr))) && (param1->shadowMapStrength > 0)) && (_loc3_->concatenatedAlpha >= _loc3_->shadowMapAlphaThreshold)))
			{
				if (param1->casterObjects.length() < (param1->casterCount+1))
					param1->casterObjects.resize(param1->casterCount+1);


				param1->casterObjects[param1->casterCount] = _loc3_;
				param1->casterCount++;
			}
			_loc3_ = _loc3_->next.get();
		}

		param1->debug = _loc7_;
		object_ptr<VG> _loc5_ = ConflictContainer::getVG(param1);
		if ((((!(param1->view->constrained)) && (!(param1->shadowMap == nullptr))) && (param1->shadowMapStrength > 0)))
		{
			Object3D *_loc3_ = childrenList.get();
			while (_loc3_ != nullptr)
			{
				if (_loc3_->visible)
				{
					if (_loc3_->isType(Mesh::TYPE) || _loc3_->isType(BSP::TYPE) || _loc3_->isType(Sprite3D::TYPE))
					{
						if (_loc3_->concatenatedAlpha >= _loc3_->shadowMapAlphaThreshold)
						{
							if (param1->casterObjects.length() < (param1->casterCount+1))
								param1->casterObjects.resize(param1->casterCount+1);

							param1->casterObjects[param1->casterCount] = _loc3_;
							param1->casterCount++;
						}
					}
					else
					{
						if (_loc3_->isType(Object3DContainer::TYPE))
						{
							std::shared_ptr<Object3D> _loc8_ = ((Object3DContainer*)_loc3_)->childrenList;
							while (_loc8_ != nullptr)
							{
								if (((_loc8_->isType(Mesh::TYPE) || _loc8_->isType(BSP::TYPE)) || _loc8_->isType(Sprite3D::TYPE)) && (_loc8_->concatenatedAlpha >= _loc8_->shadowMapAlphaThreshold))
								{
									if (param1->casterObjects.length() < (param1->casterCount+1))
										param1->casterObjects.resize(param1->casterCount+1);

									param1->casterObjects[param1->casterCount] = _loc8_.get();
									param1->casterCount++;
								}
								_loc8_ = _loc8_->next;
							}
						}
					}
				}
				_loc3_ = _loc3_->next.get();
			}
		}
		int _loc2_ = 0;
		while (_loc2_ < this->transparentLength)
		{
			Object3D *_loc3_ = this->transparent[_loc2_].get();
			_loc3_->composeAndAppend(this);
			if (_loc3_->cullingInCamera(param1, culling) >= 0)
			{
				_loc3_->concat(this);
				object_ptr<VG> _loc9_ = _loc3_->getVG(param1);
				if (_loc9_ != nullptr)
				{
					_loc9_->next = _loc5_;
					_loc5_ = _loc9_;
				}
			}
			if (((((!(param1->view->constrained)) && (!(param1->shadowMap == nullptr))) && (param1->shadowMapStrength > 0))/* && (_loc3_->concatenatedAlpha >= _loc3_->shadowMapAlphaThreshold) */))
			{
				if (param1->casterObjects.length() < (param1->casterCount+1))
					param1->casterObjects.resize(param1->casterCount+1);

				param1->casterObjects[param1->casterCount] = _loc3_;
				param1->casterCount++;
			}
			_loc2_++;
		}
		if (_loc5_ != nullptr)
		{
			if (_loc5_->next != nullptr)
			{
				if (resolveByAABB)
				{
					object_ptr<VG> _loc4_ = _loc5_;
					while (_loc4_ != nullptr)
					{
						_loc4_->calculateAABB(&matrix_i);

						_loc4_ = _loc4_->next;
					}
					drawAABBGeometry(param1, _loc5_);
				}
				else
				{
					if (resolveByOOBB)
					{
						VG *_loc4_ = _loc5_.get();
						while (_loc4_ != nullptr)
						{
							if (_loc4_->faceStruct() != nullptr)
							{
								_loc4_->calculateOOBB(this);
							}
							else
							{

							}
							_loc4_ = _loc4_->next.get();
						}
						this->drawOOBBGeometry(param1, _loc5_);
					}
					else
					{
						drawConflictGeometry(param1, _loc5_);
					}
				}
			}
			else
			{
				//2
				_loc5_->draw(param1, threshold, this);
				_loc5_->destroy();
			}
		}
	}
	else
	{
		if (this->root != nullptr)
		{
			this->calculateCameraPlanes(param1->nearClipping, param1->farClipping);
			_loc10_ = this->cullingInContainer(culling,
											   this->root->bound.MinX,
											   this->root->bound.MinY,
											   this->root->bound.MinZ,
											   this->root->bound.MaxX,
											   this->root->bound.MaxY,
											   this->root->bound.MaxZ);
			if (_loc10_ >= 0)
			{
				this->numOccluders = 0;
				if (param1->numOccluders > 0)
				{
					this->updateOccluders(param1);
				}
				object_ptr<VG> _loc5_ = ConflictContainer::getVG(param1);
				object_ptr<VG> _loc4_ = _loc5_;
				while (_loc4_ != nullptr)
				{
					_loc4_->calculateAABB(&this->matrix_i);

					_loc4_ = _loc4_->next;
				}
				this->drawNode(this->root, _loc10_, param1, _loc5_.get());
				int i = 0;

				while (i < this->numOccluders)
				{
					Vertex *_loc11_ = this->occluders[i];
					Vertex *_loc12_ = _loc11_;
					while (_loc12_->next != nullptr)
						_loc12_ = _loc12_->next.get();

					_loc12_->next = Vertex::collector;
					Vertex::collector = _loc11_->ptr();
					this->occluders[i] = nullptr;
					i++;
				}

				this->numOccluders = 0;
			}
			else
			{
				//3
				ConflictContainer::draw(param1);
			}
		}
		else
		{
			//4
			ConflictContainer::draw(param1);
		}
	}
	/*
	if (((!(this->root == nullptr)) && (_loc6_ & Debug.NODES)))
	{
		this->debugNode(this->root, _loc10_, param1, 1);
		Debug.drawBounds(param1, this, this->root.boundMinX, this->root.boundMinY, this->root.boundMinZ, this->root.boundMaxX, this->root.boundMaxY, this->root.boundMaxZ, 14496733);
	}
	*/
	if (this->root != nullptr)
	{
		param1->receiversVertexBuffers = &this->receiversVertexBuffers;
		param1->receiversIndexBuffers = &this->receiversIndexBuffers;


		for (ShadowAtlas *shadowAtlas : param1->shadowAtlases)
		{
			if (shadowAtlas != nullptr)
			{
				for (int i = 0; i < shadowAtlas->shadowsCount; i++)
					this->root->collectReceivers(shadowAtlas->shadows.at(i));
			}
		}
	}
}


//not used
object_ptr<VG> KDContainer::getVG(Camera3D *camera)
{
	(void)camera;

	qDebug() << "used KDContainer::getVG";
#if 0
	var _loc3_:int;
	var _loc2_:VG = super.getVG(param1);
	if (this->root != null)
	{
		this->numOccluders = 0;
		calculateInverseMatrix();
		this->calculateCameraPlanes(param1->nearClipping, param1->farClipping);
		_loc3_ = this->cullingInContainer(culling, this->root.boundMinX, this->root.boundMinY, this->root.boundMinZ, this->root.boundMaxX, this->root.boundMaxY, this->root.boundMaxZ);
		if (_loc3_ >= 0)
		{
			_loc2_ = this->collectVGNode(this->root, _loc3_, param1, _loc2_);
		}
	}
	return (_loc2_);
#endif
	return nullptr;
}


void KDContainer::uploadResources(Device *device)
{
	_function_name("KDContainer::uploadResources");

	if (this->isCreated)
		return;

	this->isCreated = true;

	QHashIterator<Material*, bool> i(this->materials);
	while (i.hasNext()) {
		i.next();
		TextureMaterial *material = (TextureMaterial*)i.key();

		if (material->getTexture() != nullptr)
			device->uploadResource(material->textureResource);

		//if (material->_textureATF != nullptr)
		//device->uploadResource(material->textureATFResource);

		//if (material->_textureATFAlpha != nullptr)
		//device->uploadResource(material->textureATFAlphaResource);
	}


	Object3D *object3D = this->opaqueList.get();
	while (object3D != nullptr)
	{
		if (object3D->isType(Mesh::TYPE))
		{
			Mesh *mesh = (Mesh*)object3D;
			mesh->prepareResources();
			device->uploadResource(mesh->vertexBuffer.get());
			device->uploadResource(mesh->indexBuffer.get());
		}
		else
		{
			if (object3D->isType(BSP::TYPE))
			{

				BSP *bsp = (BSP*)object3D;
				bsp->prepareResources();
				device->uploadResource(bsp->vertexBuffer.get());
				device->uploadResource(bsp->indexBuffer.get());

			}
		}
		object3D = object3D->next.get();
	}

	int _loc6_ = 0;
	while (_loc6_ < this->transparentLength)
	{
		object3D = this->transparent.at(_loc6_).get();
		if (object3D->isType(Mesh::TYPE))
		{
			Mesh *mesh = (Mesh*)object3D;
			mesh->prepareResources();
			device->uploadResource(mesh->vertexBuffer.get());
			device->uploadResource(mesh->indexBuffer.get());
		}
		else
		{
			if (object3D->isType(BSP::TYPE))
			{

				BSP *bsp = (BSP*)object3D;
				bsp->prepareResources();
				device->uploadResource(bsp->vertexBuffer.get());
				device->uploadResource(bsp->indexBuffer.get());

			}
		}
		_loc6_++;
	}
	_loc6_ = 0;
	while (_loc6_ < this->receiversVertexBuffers.length())
	{
		device->uploadResource(this->receiversVertexBuffers.at(_loc6_).get());
		device->uploadResource(this->receiversIndexBuffers.at(_loc6_).get());
		_loc6_++;
	}
}



void KDContainer::updateBounds(Object3D *param1, Object3D *param2)
{
	_function_name("KDContainer::updateBounds");

	ConflictContainer::updateBounds(param1, param2);
	if (this->root != nullptr)
	{
		if (param2 != nullptr)
		{
			updateBoundsNode(this->root, param1, param2);
		}
		else
		{
			if (this->root->bound.MinX < param1->bound.MinX)
				param1->bound.MinX = this->root->bound.MinX;

			if (this->root->bound.MaxX > param1->bound.MaxX)
				param1->bound.MaxX = this->root->bound.MaxX;

			if (this->root->bound.MinY < param1->bound.MinY)
				param1->bound.MinY = this->root->bound.MinY;

			if (this->root->bound.MaxY > param1->bound.MaxY)
				param1->bound.MaxY = this->root->bound.MaxY;

			if (this->root->bound.MinZ < param1->bound.MinZ)
				param1->bound.MinZ = this->root->bound.MinZ;

			if (this->root->bound.MaxZ > param1->bound.MaxZ)
				param1->bound.MaxZ = this->root->bound.MaxZ;
		}
	}
}

//not used
void KDContainer::calculateCameraPlanes(double param1, double param2)
{
	qDebug("used KDContainer::calculateCameraPlanes");
	(void)param1;
	(void)param2;
#if 0
	this->nearPlaneX = imc;
	this->nearPlaneY = img;
	this->nearPlaneZ = imk;
	this->nearPlaneOffset = (((((imc * param1) + imd) * this->nearPlaneX) + (((img * param1) + imh) * this->nearPlaneY)) + (((imk * param1) + iml) * this->nearPlaneZ));
	this->farPlaneX = -(imc);
	this->farPlaneY = -(img);
	this->farPlaneZ = -(imk);
	this->farPlaneOffset = (((((imc * param2) + imd) * this->farPlaneX) + (((img * param2) + imh) * this->farPlaneY)) + (((imk * param2) + iml) * this->farPlaneZ));
	var _loc3_:Number = ((-(ima) - imb) + imc);
	var _loc4_:Number = ((-(ime) - imf) + img);
	var _loc5_:Number = ((-(imi) - imj) + imk);
	var _loc6_:Number = ((ima - imb) + imc);
	var _loc7_:Number = ((ime - imf) + img);
	var _loc8_:Number = ((imi - imj) + imk);
	this->topPlaneX = ((_loc8_ * _loc4_) - (_loc7_ * _loc5_));
	this->topPlaneY = ((_loc6_ * _loc5_) - (_loc8_ * _loc3_));
	this->topPlaneZ = ((_loc7_ * _loc3_) - (_loc6_ * _loc4_));
	this->topPlaneOffset = (((imd * this->topPlaneX) + (imh * this->topPlaneY)) + (iml * this->topPlaneZ));
	_loc3_ = _loc6_;
	_loc4_ = _loc7_;
	_loc5_ = _loc8_;
	_loc6_ = ((ima + imb) + imc);
	_loc7_ = ((ime + imf) + img);
	_loc8_ = ((imi + imj) + imk);
	this->rightPlaneX = ((_loc8_ * _loc4_) - (_loc7_ * _loc5_));
	this->rightPlaneY = ((_loc6_ * _loc5_) - (_loc8_ * _loc3_));
	this->rightPlaneZ = ((_loc7_ * _loc3_) - (_loc6_ * _loc4_));
	this->rightPlaneOffset = (((imd * this->rightPlaneX) + (imh * this->rightPlaneY)) + (iml * this->rightPlaneZ));
	_loc3_ = _loc6_;
	_loc4_ = _loc7_;
	_loc5_ = _loc8_;
	_loc6_ = ((-(ima) + imb) + imc);
	_loc7_ = ((-(ime) + imf) + img);
	_loc8_ = ((-(imi) + imj) + imk);
	this->bottomPlaneX = ((_loc8_ * _loc4_) - (_loc7_ * _loc5_));
	this->bottomPlaneY = ((_loc6_ * _loc5_) - (_loc8_ * _loc3_));
	this->bottomPlaneZ = ((_loc7_ * _loc3_) - (_loc6_ * _loc4_));
	this->bottomPlaneOffset = (((imd * this->bottomPlaneX) + (imh * this->bottomPlaneY)) + (iml * this->bottomPlaneZ));
	_loc3_ = _loc6_;
	_loc4_ = _loc7_;
	_loc5_ = _loc8_;
	_loc6_ = ((-(ima) - imb) + imc);
	_loc7_ = ((-(ime) - imf) + img);
	_loc8_ = ((-(imi) - imj) + imk);
	this->leftPlaneX = ((_loc8_ * _loc4_) - (_loc7_ * _loc5_));
	this->leftPlaneY = ((_loc6_ * _loc5_) - (_loc8_ * _loc3_));
	this->leftPlaneZ = ((_loc7_ * _loc3_) - (_loc6_ * _loc4_));
	this->leftPlaneOffset = (((imd * this->leftPlaneX) + (imh * this->leftPlaneY)) + (iml * this->leftPlaneZ));
#endif
}

//not used
void KDContainer::updateOccluders(Camera3D *param1)
{
	qDebug("used KDContainer::updateOccluders");

	(void)param1;
#if 0
	var _loc3_:Vertex;
	var _loc4_:Vertex;
	var _loc5_:Vertex;
	var _loc6_:Number = NaN;
	var _loc7_:Number = NaN;
	var _loc8_:Number = NaN;
	var _loc9_:Number = NaN;
	var _loc10_:Number = NaN;
	var _loc11_:Number = NaN;
	var _loc2_:int = this->numOccluders;
	while (_loc2_ < param1->numOccluders)
	{
		_loc3_ = null;
		_loc4_ = param1->occluders[_loc2_];
		while (_loc4_ != null)
		{
			_loc5_ = _loc4_->create();
			_loc5_->next = _loc3_;
			_loc3_ = _loc5_;
			_loc6_ = (((ima * _loc4_->x) + (imb * _loc4_->y)) + (imc * _loc4_->z));
			_loc7_ = (((ime * _loc4_->x) + (imf * _loc4_->y)) + (img * _loc4_->z));
			_loc8_ = (((imi * _loc4_->x) + (imj * _loc4_->y)) + (imk * _loc4_->z));
			_loc9_ = (((ima * _loc4_->u) + (imb * _loc4_->v)) + (imc * _loc4_->offset));
			_loc10_ = (((ime * _loc4_->u) + (imf * _loc4_->v)) + (img * _loc4_->offset));
			_loc11_ = (((imi * _loc4_->u) + (imj * _loc4_->v)) + (imk * _loc4_->offset));
			_loc3_->x = ((_loc11_ * _loc7_) - (_loc10_ * _loc8_));
			_loc3_->y = ((_loc9_ * _loc8_) - (_loc11_ * _loc6_));
			_loc3_->z = ((_loc10_ * _loc6_) - (_loc9_ * _loc7_));
			_loc3_->offset = (((imd * _loc3_->x) + (imh * _loc3_->y)) + (iml * _loc3_->z));
			_loc4_ = _loc4_->next;
		}
		this->occluders[this->numOccluders] = _loc3_;
		this->numOccluders++;
		_loc2_++;
	}
#endif
}


//not used
int KDContainer::cullingInContainer(int param1, double param2, double param3, double param4, double param5, double param6, double param7)
{
	qDebug("used KDContainer::cullingInContainer");

	if (param1 > 0)
	{
		if ((param1 & 0x01))
		{
			if (this->nearPlaneX >= 0)
			{
				if (this->nearPlaneY >= 0)
				{
					if (this->nearPlaneZ >= 0)
					{
						if ((((param5 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
					else
					{
						if ((((param5 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
				}
				else
				{
					if (this->nearPlaneZ >= 0)
					{
						if ((((param5 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
					else
					{
						if ((((param5 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
				}
			}
			else
			{
				if (this->nearPlaneY >= 0)
				{
					if (this->nearPlaneZ >= 0)
					{
						if ((((param2 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
					else
					{
						if ((((param2 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
				}
				else
				{
					if (this->nearPlaneZ >= 0)
					{
						if ((((param2 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
					else
					{
						if ((((param2 * this->nearPlaneX) + (param3 * this->nearPlaneY)) + (param4 * this->nearPlaneZ)) <= this->nearPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->nearPlaneX) + (param6 * this->nearPlaneY)) + (param7 * this->nearPlaneZ)) > this->nearPlaneOffset)
						{
							param1 = (param1 & 0x3E);
						}
					}
				}
			}
		}
		if ((param1 & 0x02))
		{
			if (this->farPlaneX >= 0)
			{
				if (this->farPlaneY >= 0)
				{
					if (this->farPlaneZ >= 0)
					{
						if ((((param5 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param7 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param4 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
					else
					{
						if ((((param5 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param4 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param7 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
				}
				else
				{
					if (this->farPlaneZ >= 0)
					{
						if ((((param5 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param7 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param4 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
					else
					{
						if ((((param5 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param4 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param7 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
				}
			}
			else
			{
				if (this->farPlaneY >= 0)
				{
					if (this->farPlaneZ >= 0)
					{
						if ((((param2 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param7 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param4 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
					else
					{
						if ((((param2 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param4 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param7 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
				}
				else
				{
					if (this->farPlaneZ >= 0)
					{
						if ((((param2 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param7 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param4 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
					else
					{
						if ((((param2 * this->farPlaneX) + (param3 * this->farPlaneY)) + (param4 * this->farPlaneZ)) <= this->farPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->farPlaneX) + (param6 * this->farPlaneY)) + (param7 * this->farPlaneZ)) > this->farPlaneOffset)
						{
							param1 = (param1 & 0x3D);
						}
					}
				}
			}
		}
		if ((param1 & 0x04))
		{
			if (this->leftPlaneX >= 0)
			{
				if (this->leftPlaneY >= 0)
				{
					if (this->leftPlaneZ >= 0)
					{
						if ((((param5 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
					else
					{
						if ((((param5 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
				}
				else
				{
					if (this->leftPlaneZ >= 0)
					{
						if ((((param5 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
					else
					{
						if ((((param5 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
				}
			}
			else
			{
				if (this->leftPlaneY >= 0)
				{
					if (this->leftPlaneZ >= 0)
					{
						if ((((param2 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
					else
					{
						if ((((param2 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
				}
				else
				{
					if (this->leftPlaneZ >= 0)
					{
						if ((((param2 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
					else
					{
						if ((((param2 * this->leftPlaneX) + (param3 * this->leftPlaneY)) + (param4 * this->leftPlaneZ)) <= this->leftPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->leftPlaneX) + (param6 * this->leftPlaneY)) + (param7 * this->leftPlaneZ)) > this->leftPlaneOffset)
						{
							param1 = (param1 & 0x3B);
						}
					}
				}
			}
		}
		if ((param1 & 0x08))
		{
			if (this->rightPlaneX >= 0)
			{
				if (this->rightPlaneY >= 0)
				{
					if (this->rightPlaneZ >= 0)
					{
						if ((((param5 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
					else
					{
						if ((((param5 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
				}
				else
				{
					if (this->rightPlaneZ >= 0)
					{
						if ((((param5 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
					else
					{
						if ((((param5 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
				}
			}
			else
			{
				if (this->rightPlaneY >= 0)
				{
					if (this->rightPlaneZ >= 0)
					{
						if ((((param2 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
					else
					{
						if ((((param2 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
				}
				else
				{
					if (this->rightPlaneZ >= 0)
					{
						if ((((param2 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
					else
					{
						if ((((param2 * this->rightPlaneX) + (param3 * this->rightPlaneY)) + (param4 * this->rightPlaneZ)) <= this->rightPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->rightPlaneX) + (param6 * this->rightPlaneY)) + (param7 * this->rightPlaneZ)) > this->rightPlaneOffset)
						{
							param1 = (param1 & 0x37);
						}
					}
				}
			}
		}
		if ((param1 & 0x10))
		{
			if (this->topPlaneX >= 0)
			{
				if (this->topPlaneY >= 0)
				{
					if (this->topPlaneZ >= 0)
					{
						if ((((param5 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param7 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param4 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
					else
					{
						if ((((param5 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param4 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param7 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
				}
				else
				{
					if (this->topPlaneZ >= 0)
					{
						if ((((param5 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param7 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param4 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
					else
					{
						if ((((param5 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param4 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param7 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
				}
			}
			else
			{
				if (this->topPlaneY >= 0)
				{
					if (this->topPlaneZ >= 0)
					{
						if ((((param2 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param7 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param4 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
					else
					{
						if ((((param2 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param4 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param7 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
				}
				else
				{
					if (this->topPlaneZ >= 0)
					{
						if ((((param2 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param7 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param4 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
					else
					{
						if ((((param2 * this->topPlaneX) + (param3 * this->topPlaneY)) + (param4 * this->topPlaneZ)) <= this->topPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->topPlaneX) + (param6 * this->topPlaneY)) + (param7 * this->topPlaneZ)) > this->topPlaneOffset)
						{
							param1 = (param1 & 0x2F);
						}
					}
				}
			}
		}
		if ((param1 & 0x20))
		{
			if (this->bottomPlaneX >= 0)
			{
				if (this->bottomPlaneY >= 0)
				{
					if (this->bottomPlaneZ >= 0)
					{
						if ((((param5 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
					else
					{
						if ((((param5 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
				}
				else
				{
					if (this->bottomPlaneZ >= 0)
					{
						if ((((param5 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
					else
					{
						if ((((param5 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param2 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
				}
			}
			else
			{
				if (this->bottomPlaneY >= 0)
				{
					if (this->bottomPlaneZ >= 0)
					{
						if ((((param2 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
					else
					{
						if ((((param2 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
				}
				else
				{
					if (this->bottomPlaneZ >= 0)
					{
						if ((((param2 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
					else
					{
						if ((((param2 * this->bottomPlaneX) + (param3 * this->bottomPlaneY)) + (param4 * this->bottomPlaneZ)) <= this->bottomPlaneOffset)
						{
							return (-1);
						}
						if ((((param5 * this->bottomPlaneX) + (param6 * this->bottomPlaneY)) + (param7 * this->bottomPlaneZ)) > this->bottomPlaneOffset)
						{
							param1 = (param1 & 0x1F);
						}
					}
				}
			}
		}
	}
	int _loc8_ = 0;
	while (true)
	{
		if (_loc8_ >= this->numOccluders)
		{
			return (param1);
		}
		Vertex *_loc9_ = this->occluders.at(_loc8_);
		while (_loc9_ != nullptr)
		{
			if (_loc9_->x >= 0)
			{
				if (_loc9_->y >= 0)
				{
					if (_loc9_->z >= 0)
					{
						if ((((param5 * _loc9_->x) + (param6 * _loc9_->y)) + (param7 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param5 * _loc9_->x) + (param6 * _loc9_->y)) + (param4 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
				}
				else
				{
					if (_loc9_->z >= 0)
					{
						if ((((param5 * _loc9_->x) + (param3 * _loc9_->y)) + (param7 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param5 * _loc9_->x) + (param3 * _loc9_->y)) + (param4 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
				}
			}
			else
			{
				if (_loc9_->y >= 0)
				{
					if (_loc9_->z >= 0)
					{
						if ((((param2 * _loc9_->x) + (param6 * _loc9_->y)) + (param7 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param2 * _loc9_->x) + (param6 * _loc9_->y)) + (param4 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
				}
				else
				{
					if (_loc9_->z >= 0)
					{
						if ((((param2 * _loc9_->x) + (param3 * _loc9_->y)) + (param7 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param2 * _loc9_->x) + (param3 * _loc9_->y)) + (param4 * _loc9_->z)) > _loc9_->offset)
						{
							break;
						}
					}
				}
			}
			_loc9_ = _loc9_->next.get();
		}
		if (_loc9_ == nullptr)
		{
			break;
		}
		_loc8_++;
	}

	return -1;
}


//not used
#if 0
private function checkIntersectionNode(param1:KDNode, param2:Number, param3:Number, param4:Number, param5:Number, param6:Number, param7:Number, param8:Number, param9:Dictionary):Boolean
{
																																													  var _loc10_:Object3D;
var _loc11_:Object3D;
var _loc12_:Number = NaN;
var _loc13_:Number = NaN;
var _loc14_:Number = NaN;
var _loc15_:Number = NaN;
var _loc16_:Number = NaN;
var _loc17_:Number = NaN;
var _loc18_:Number = NaN;
var _loc19_:Boolean;
var _loc20_:Boolean;
var _loc21_:Number = NaN;
var _loc22_:Number = NaN;
if (param1->negative != null)
{
	_loc19_ = (param1->axis == 0);
	_loc20_ = (param1->axis == 1);
	_loc21_ = (((!(!(_loc19_))) ? param2 : ((!(!(_loc20_))) ? param3 : param4)) - param1->coord);
	_loc22_ = ((!(!(_loc19_))) ? Number(param5) : ((!(!(_loc20_))) ? Number(param6) : Number(param7)));
	if (_loc21_ > 0)
	{
		if (_loc22_ < 0)
		{
			_loc18_ = (-(_loc21_) / _loc22_);
			if (_loc18_ < param8)
			{
				_loc10_ = param1->objectList;
				_loc11_ = param1->objectBoundList;
				while (_loc10_ != null)
				{
					if (boundCheckIntersection(param2, param3, param4, param5, param6, param7, param8, _loc11_->boundMinX, _loc11_->boundMinY, _loc11_->boundMinZ, _loc11_->boundMaxX, _loc11_->boundMaxY, _loc11_->boundMaxZ))
					{
						_loc10_->composeMatrix();
						_loc10_->invertMatrix();
						_loc12_ = ((((_loc10_->ma * param2) + (_loc10_->mb * param3)) + (_loc10_->mc * param4)) + _loc10_->md);
						_loc13_ = ((((_loc10_->me * param2) + (_loc10_->mf * param3)) + (_loc10_->mg * param4)) + _loc10_->mh);
						_loc14_ = ((((_loc10_->mi * param2) + (_loc10_->mj * param3)) + (_loc10_->mk * param4)) + _loc10_->ml);
						_loc15_ = (((_loc10_->ma * param5) + (_loc10_->mb * param6)) + (_loc10_->mc * param7));
						_loc16_ = (((_loc10_->me * param5) + (_loc10_->mf * param6)) + (_loc10_->mg * param7));
						_loc17_ = (((_loc10_->mi * param5) + (_loc10_->mj * param6)) + (_loc10_->mk * param7));
						if (_loc10_->checkIntersection(_loc12_, _loc13_, _loc14_, _loc15_, _loc16_, _loc17_, param8, param9))
						{
							return (true);
						}
					}
					_loc10_ = _loc10_->next;
					_loc11_ = _loc11_->next;
				}
				if (((boundCheckIntersection(param2, param3, param4, param5, param6, param7, param8, param1->negative.boundMinX, param1->negative.boundMinY, param1->negative.boundMinZ, param1->negative.boundMaxX, param1->negative.boundMaxY, param1->negative.boundMaxZ)) && (this->checkIntersectionNode(param1->negative, param2, param3, param4, param5, param6, param7, param8, param9))))
				{
					return (true);
				}
			}
		}
		return ((boundCheckIntersection(param2, param3, param4, param5, param6, param7, param8, param1->positive.boundMinX, param1->positive.boundMinY, param1->positive.boundMinZ, param1->positive.boundMaxX, param1->positive.boundMaxY, param1->positive.boundMaxZ)) && (this->checkIntersectionNode(param1->positive, param2, param3, param4, param5, param6, param7, param8, param9)));
	}
	if (_loc22_ > 0)
	{
		_loc18_ = (-(_loc21_) / _loc22_);
		if (_loc18_ < param8)
		{
			_loc10_ = param1->objectList;
			_loc11_ = param1->objectBoundList;
			while (_loc10_ != null)
			{
				if (boundCheckIntersection(param2, param3, param4, param5, param6, param7, param8, _loc11_->boundMinX, _loc11_->boundMinY, _loc11_->boundMinZ, _loc11_->boundMaxX, _loc11_->boundMaxY, _loc11_->boundMaxZ))
				{
					_loc10_->composeMatrix();
					_loc10_->invertMatrix();
					_loc12_ = ((((_loc10_->ma * param2) + (_loc10_->mb * param3)) + (_loc10_->mc * param4)) + _loc10_->md);
					_loc13_ = ((((_loc10_->me * param2) + (_loc10_->mf * param3)) + (_loc10_->mg * param4)) + _loc10_->mh);
					_loc14_ = ((((_loc10_->mi * param2) + (_loc10_->mj * param3)) + (_loc10_->mk * param4)) + _loc10_->ml);
					_loc15_ = (((_loc10_->ma * param5) + (_loc10_->mb * param6)) + (_loc10_->mc * param7));
					_loc16_ = (((_loc10_->me * param5) + (_loc10_->mf * param6)) + (_loc10_->mg * param7));
					_loc17_ = (((_loc10_->mi * param5) + (_loc10_->mj * param6)) + (_loc10_->mk * param7));
					if (_loc10_->checkIntersection(_loc12_, _loc13_, _loc14_, _loc15_, _loc16_, _loc17_, param8, param9))
					{
						return (true);
					}
				}
				_loc10_ = _loc10_->next;
				_loc11_ = _loc11_->next;
			}
			if (((boundCheckIntersection(param2, param3, param4, param5, param6, param7, param8, param1->positive.boundMinX, param1->positive.boundMinY, param1->positive.boundMinZ, param1->positive.boundMaxX, param1->positive.boundMaxY, param1->positive.boundMaxZ)) && (this->checkIntersectionNode(param1->positive, param2, param3, param4, param5, param6, param7, param8, param9))))
			{
				return (true);
			}
		}
	}
	return ((boundCheckIntersection(param2, param3, param4, param5, param6, param7, param8, param1->negative.boundMinX, param1->negative.boundMinY, param1->negative.boundMinZ, param1->negative.boundMaxX, param1->negative.boundMaxY, param1->negative.boundMaxZ)) && (this->checkIntersectionNode(param1->negative, param2, param3, param4, param5, param6, param7, param8, param9)));
}
_loc10_ = param1->objectList;
while (_loc10_ != null)
{
	_loc10_->composeMatrix();
	_loc10_->invertMatrix();
	_loc12_ = ((((_loc10_->ma * param2) + (_loc10_->mb * param3)) + (_loc10_->mc * param4)) + _loc10_->md);
	_loc13_ = ((((_loc10_->me * param2) + (_loc10_->mf * param3)) + (_loc10_->mg * param4)) + _loc10_->mh);
	_loc14_ = ((((_loc10_->mi * param2) + (_loc10_->mj * param3)) + (_loc10_->mk * param4)) + _loc10_->ml);
	_loc15_ = (((_loc10_->ma * param5) + (_loc10_->mb * param6)) + (_loc10_->mc * param7));
	_loc16_ = (((_loc10_->me * param5) + (_loc10_->mf * param6)) + (_loc10_->mg * param7));
	_loc17_ = (((_loc10_->mi * param5) + (_loc10_->mj * param6)) + (_loc10_->mk * param7));
	if (_loc10_->checkIntersection(_loc12_, _loc13_, _loc14_, _loc15_, _loc16_, _loc17_, param8, param9))
	{
		return (true);
	}
	_loc10_ = _loc10_->next;
}
return (false);
}
#endif


//not used
#if 0
private function collectPlanesNode(param1:KDNode, param2:Vector3D, param3:Vector3D, param4:Vector3D, param5:Vector3D, param6:Vector3D, param7:Vector3D, param8:Vector.<Face>, param9:Dictionary=null):void
{
	var _loc10_:Object3D;
	var _loc11_:Object3D;
	var _loc12_:Boolean;
	var _loc13_:Boolean;
	var _loc14_:Number = NaN;
	if (param1->negative != null)
	{
		_loc12_ = (param1->axis == 0);
		_loc13_ = (param1->axis == 1);
		_loc14_ = (((!(!(_loc12_))) ? param2->x : ((!(!(_loc13_))) ? param2->y : param2->z)) - param1->coord);
		if (_loc14_ >= param2->w)
		{
			if (boundIntersectSphere(param2, param1->positive.boundMinX, param1->positive.boundMinY, param1->positive.boundMinZ, param1->positive.boundMaxX, param1->positive.boundMaxY, param1->positive.boundMaxZ))
			{
				this->collectPlanesNode(param1->positive, param2, param3, param4, param5, param6, param7, param8, param9);
			}
		}
		else
		{
			if (_loc14_ <= -(param2->w))
			{
				if (boundIntersectSphere(param2, param1->negative.boundMinX, param1->negative.boundMinY, param1->negative.boundMinZ, param1->negative.boundMaxX, param1->negative.boundMaxY, param1->negative.boundMaxZ))
				{
					this->collectPlanesNode(param1->negative, param2, param3, param4, param5, param6, param7, param8, param9);
				}
			}
			else
			{
				_loc10_ = param1->objectList;
				_loc11_ = param1->objectBoundList;
				while (_loc10_ != null)
				{
					if (boundIntersectSphere(param2, _loc11_->boundMinX, _loc11_->boundMinY, _loc11_->boundMinZ, _loc11_->boundMaxX, _loc11_->boundMaxY, _loc11_->boundMaxZ))
					{
						_loc10_->composeAndAppend(this);
						_loc10_->collectPlanes(param3, param4, param5, param6, param7, param8, param9);
					}
					_loc10_ = _loc10_->next;
					_loc11_ = _loc11_->next;
				}
				if (boundIntersectSphere(param2, param1->positive.boundMinX, param1->positive.boundMinY, param1->positive.boundMinZ, param1->positive.boundMaxX, param1->positive.boundMaxY, param1->positive.boundMaxZ))
				{
					this->collectPlanesNode(param1->positive, param2, param3, param4, param5, param6, param7, param8, param9);
				}
				if (boundIntersectSphere(param2, param1->negative.boundMinX, param1->negative.boundMinY, param1->negative.boundMinZ, param1->negative.boundMaxX, param1->negative.boundMaxY, param1->negative.boundMaxZ))
				{
					this->collectPlanesNode(param1->negative, param2, param3, param4, param5, param6, param7, param8, param9);
				}
			}
		}
	}
	else
	{
		_loc10_ = param1->objectList;
		while (_loc10_ != null)
		{
			_loc10_->composeAndAppend(this);
			_loc10_->collectPlanes(param3, param4, param5, param6, param7, param8, param9);
			_loc10_ = _loc10_->next;
		}
	}
}
#endif


//not used
#if 0
KDNode* cloneNode(KDNode *param1, Object3DContainer *param2)
{
	var _loc4_:Object3D;
	var _loc5_:Object3D;
	var _loc6_:Object3D;
	var _loc3_:KDNode = new KDNode();
	_loc3_->axis = param1->axis;
	_loc3_->coord = param1->coord;
	_loc3_->minCoord = param1->minCoord;
	_loc3_->maxCoord = param1->maxCoord;
	_loc3_->boundMinX = param1->boundMinX;
	_loc3_->boundMinY = param1->boundMinY;
	_loc3_->boundMinZ = param1->boundMinZ;
	_loc3_->boundMaxX = param1->boundMaxX;
	_loc3_->boundMaxY = param1->boundMaxY;
	_loc3_->boundMaxZ = param1->boundMaxZ;
	_loc4_ = param1->objectList;
	_loc5_ = null;
	while (_loc4_ != null)
	{
		_loc6_ = _loc4_->clone();
		if (_loc3_->objectList != null)
		{
			_loc5_->next = _loc6_;
		}
		else
		{
			_loc3_->objectList = _loc6_;
		}
		_loc5_ = _loc6_;
		_loc6_->setParent(param2);
		_loc4_ = _loc4_->next;
	}
	_loc4_ = param1->objectBoundList;
	_loc5_ = null;
	while (_loc4_ != null)
	{
		_loc6_ = _loc4_->clone();
		if (_loc3_->objectBoundList != null)
		{
			_loc5_->next = _loc6_;
		}
		else
		{
			_loc3_->objectBoundList = _loc6_;
		}
		_loc5_ = _loc6_;
		_loc4_ = _loc4_->next;
	}
	_loc4_ = param1->occluderList;
	_loc5_ = null;
	while (_loc4_ != null)
	{
		_loc6_ = _loc4_->clone();
		if (_loc3_->occluderList != null)
		{
			_loc5_->next = _loc6_;
		}
		else
		{
			_loc3_->occluderList = _loc6_;
		}
		_loc5_ = _loc6_;
		_loc6_->setParent(param2);
		_loc4_ = _loc4_->next;
	}
	_loc4_ = param1->occluderBoundList;
	_loc5_ = null;
	while (_loc4_ != null)
	{
		_loc6_ = _loc4_->clone();
		if (_loc3_->occluderBoundList != null)
		{
			_loc5_->next = _loc6_;
		}
		else
		{
			_loc3_->occluderBoundList = _loc6_;
		}
		_loc5_ = _loc6_;
		_loc4_ = _loc4_->next;
	}
	if (param1->negative != null)
	{
		_loc3_->negative = this->cloneNode(param1->negative, param2);
	}
	if (param1->positive != null)
	{
		_loc3_->positive = this->cloneNode(param1->positive, param2);
	}
	return (_loc3_);
}
#endif


//not used
#if 0
private function collectVGNode(param1:KDNode, param2:int, param3:Camera3D, param4:VG=null):VG
{
																							   var _loc5_:VG;
var _loc6_:VG;
var _loc9_:VG;
var _loc10_:int;
var _loc11_:int;
var _loc7_:Object3D = param1->objectList;
var _loc8_:Object3D = param1->objectBoundList;
while (_loc7_ != null)
{
	if (((_loc7_->visible) && (((_loc7_->culling = param2) == 0) || ((_loc7_->culling = this->cullingInContainer(param2, _loc8_->boundMinX, _loc8_->boundMinY, _loc8_->boundMinZ, _loc8_->boundMaxX, _loc8_->boundMaxY, _loc8_->boundMaxZ)) >= 0))))
	{
		_loc7_->composeAndAppend(this);
		_loc7_->concat(this);
		_loc9_ = _loc7_->getVG(param3);
		if (_loc9_ != null)
		{
			if (_loc5_ != null)
			{
				_loc6_->next = _loc9_;
			}
			else
			{
				_loc5_ = _loc9_;
				_loc6_ = _loc9_;
			}
			while (_loc6_->next != null)
			{
				_loc6_ = _loc6_->next;
			}
		}
	}
	_loc7_ = _loc7_->next;
	_loc8_ = _loc8_->next;
}
if (_loc5_ != null)
{
	_loc6_->next = param4;
	param4 = _loc5_;
}
if (param1->negative != null)
{
	_loc10_ = ((param2 > 0) ? int(this->cullingInContainer(param2, param1->negative.boundMinX, param1->negative.boundMinY, param1->negative.boundMinZ, param1->negative.boundMaxX, param1->negative.boundMaxY, param1->negative.boundMaxZ)) : int(0));
	_loc11_ = ((param2 > 0) ? int(this->cullingInContainer(param2, param1->positive.boundMinX, param1->positive.boundMinY, param1->positive.boundMinZ, param1->positive.boundMaxX, param1->positive.boundMaxY, param1->positive.boundMaxZ)) : int(0));
	if (_loc10_ >= 0)
	{
		param4 = this->collectVGNode(param1->negative, _loc10_, param3, param4);
	}
	if (_loc11_ >= 0)
	{
		param4 = this->collectVGNode(param1->positive, _loc11_, param3, param4);
	}
}
return (param4);
}
#endif

//not used
static void updateBoundsNode(KDNode *param1, Object3D *param2, Object3D *param3)
{
	_function_name("KDContainer::updateBoundsNode");

	qDebug("used updateBoundsNode");

	Object3D *o = param1->objectList.get();
	while (o != nullptr)
	{
		if (param3 != nullptr)
		{
			o->composeAndAppend(param3);
		}
		else
		{
			o->composeMatrix();
		}
		o->updateBounds(param2, o);
		o = o->next.get();
	}
	if (param1->negative != nullptr)
	{
		updateBoundsNode(param1->negative, param2, param3);
		updateBoundsNode(param1->positive, param2, param3);
	}
}



//not used
#if 0
void debugNode(KDNode *param1, int param2, Camera3D *param3, double param4)
{
	var _loc5_:int;
	var _loc6_:int;
	if (((!(param1 == null)) && (!(param1->negative == null))))
	{
		_loc5_ = ((param2 > 0) ? int(this->cullingInContainer(param2, param1->negative.boundMinX, param1->negative.boundMinY, param1->negative.boundMinZ, param1->negative.boundMaxX, param1->negative.boundMaxY, param1->negative.boundMaxZ)) : int(0));
		_loc6_ = ((param2 > 0) ? int(this->cullingInContainer(param2, param1->positive.boundMinX, param1->positive.boundMinY, param1->positive.boundMinZ, param1->positive.boundMaxX, param1->positive.boundMaxY, param1->positive.boundMaxZ)) : int(0));
		if (_loc5_ >= 0)
		{
			this->debugNode(param1->negative, _loc5_, param3, (param4 * this->debugAlphaFade));
		}
		Debug.drawKDNode(param3, this, param1->axis, param1->coord, param1->boundMinX, param1->boundMinY, param1->boundMinZ, param1->boundMaxX, param1->boundMaxY, param1->boundMaxZ, param4);
		if (_loc6_ >= 0)
		{
			this->debugNode(param1->positive, _loc6_, param3, (param4 * this->debugAlphaFade));
		}
	}
}
#endif


//not used
void KDContainer::drawNode(KDNode *param1, int param2, Camera3D *param3, VG *param4)
{
	VG *_loc7_ = nullptr;
	VG *_loc8_ = nullptr;
	VG *_loc9_ = nullptr;
	qDebug("used KDContainer::drawNode");


	if (param3->occludedAll)
	{
		while (param4 != nullptr)
		{
			object_ptr<VG> vg = param4->next;
			param4->destroy();
			param4 = vg.get();
		}
		return;
	}

	if (param1->negative != nullptr)
	{
		int _loc12_ = (((param2 > 0) || (this->numOccluders > 0)) ? int(this->cullingInContainer(param2, param1->negative->bound.MinX, param1->negative->bound.MinY, param1->negative->bound.MinZ, param1->negative->bound.MaxX, param1->negative->bound.MaxY, param1->negative->bound.MaxZ)) : int(0));
		int _loc13_ = (((param2 > 0) || (this->numOccluders > 0)) ? int(this->cullingInContainer(param2, param1->positive->bound.MinX, param1->positive->bound.MinY, param1->positive->bound.MinZ,  param1->positive->bound.MaxX, param1->positive->bound.MaxY, param1->positive->bound.MaxZ)) : int(0));
		bool _loc14_ = (param1->axis == 0);
		bool _loc15_ = (param1->axis == 1);
		if (((_loc12_ >= 0) && (_loc13_ >= 0)))
		{
			while (param4 != nullptr)
			{
				VG *_loc6_ = param4->next.get();
				if (((param4->numOccluders < this->numOccluders) && (this->occludeGeometry(param3, param4))))
				{
					param4->destroy();
				}
				else
				{
					double _loc16_ = ((!(!(_loc14_))) ? param4->boundMinX : ((!(!(_loc15_))) ? param4->boundMinY : param4->boundMinZ));
					double _loc17_ = ((!(!(_loc14_))) ? param4->boundMaxX : ((!(!(_loc15_))) ? param4->boundMaxY : param4->boundMaxZ));
					if (_loc17_ <= param1->maxCoord)
					{
						if (_loc16_ < param1->minCoord)
						{
							param4->next = _loc7_;
							_loc7_ = param4;
						}
						else
						{
							param4->next = _loc8_;
							_loc8_ = param4;
						}
					}
					else
					{
						if (_loc16_ >= param1->minCoord)
						{
							param4->next = _loc9_;
							_loc9_ = param4;
						}
						else
						{
							param4->split(param3, ((param1->axis == 0) ? 1 : 0), ((param1->axis == 1) ? 1 : 0), ((param1->axis == 2) ? 1 : 0), param1->coord, threshold);
							if (param4->next != nullptr)
							{
								param4->next->next = _loc7_;
								_loc7_ = param4->next.get();
							}
							if (param4->faceStruct() != nullptr)
							{
								param4->next = _loc9_;
								_loc9_ = param4;
							}
							else
							{
								param4->destroy();
							}
						}
					}
				}
				param4 = _loc6_;
			}

			if (((((_loc14_) && (this->matrix_i.md > param1->coord)) || ((_loc15_) && (this->matrix_i.mh > param1->coord))) || (((!(_loc14_)) && (!(_loc15_))) && (this->matrix_i.ml > param1->coord))))
			{
				this->drawNode(param1->positive, _loc13_, param3, _loc9_);
				while (_loc8_ != nullptr)
				{
					object_ptr<VG> _loc6_ = _loc8_->next.get();
					if (((_loc8_->numOccluders >= this->numOccluders) || (!(this->occludeGeometry(param3, _loc8_)))))
					{
						_loc8_->draw(param3, threshold, this);
					}
					_loc8_->destroy();
					_loc8_ = _loc6_.get();
				}
				Object3D *_loc10_ = param1->objectList.get();
				Object3D *_loc11_ = param1->objectBoundList.get();
				while (_loc10_ != nullptr)
				{
					if (((_loc10_->visible) && ((((_loc10_->culling = param2) == 0) && (this->numOccluders == 0)) || ((_loc10_->culling = this->cullingInContainer(param2, _loc11_->bound.MinX, _loc11_->bound.MinY, _loc11_->bound.MinZ, _loc11_->bound.MaxX, _loc11_->bound.MaxY, _loc11_->bound.MaxZ)) >= 0))))
					{
						_loc10_->copyAndAppend(_loc11_, this);
						_loc10_->concat(this);
						_loc10_->draw(param3);
					}
					_loc10_ = _loc10_->next.get();
					_loc11_ = _loc11_->next.get();
				}
				_loc10_ = param1->occluderList.get();
				_loc11_ = param1->occluderBoundList.get();
				while (_loc10_ != nullptr)
				{
					if (((_loc10_->visible) && ((((_loc10_->culling = param2) == 0) && (this->numOccluders == 0)) || ((_loc10_->culling = this->cullingInContainer(param2, _loc11_->bound.MinX, _loc11_->bound.MinY, _loc11_->bound.MinZ, _loc11_->bound.MaxX, _loc11_->bound.MaxY, _loc11_->bound.MaxZ)) >= 0))))
					{
						_loc10_->copyAndAppend(_loc11_, this);
						_loc10_->concat(this);
						_loc10_->draw(param3);
					}
					_loc10_ = _loc10_->next.get();
					_loc11_ = _loc11_->next.get();
				}
				if (param1->occluderList != nullptr)
				{
					this->updateOccluders(param3);
				}
				this->drawNode(param1->negative, _loc12_, param3, _loc7_);
			}
			else
			{
				this->drawNode(param1->negative, _loc12_, param3, _loc7_);
				while (_loc8_ != nullptr)
				{
					VG *_loc6_ = _loc8_->next.get();
					if (((_loc8_->numOccluders >= this->numOccluders) || (!(this->occludeGeometry(param3, _loc8_)))))
					{
						_loc8_->draw(param3, threshold, this);
					}
					_loc8_->destroy();
					_loc8_ = _loc6_;
				}
				Object3D *_loc10_ = param1->objectList.get();
				Object3D *_loc11_ = param1->objectBoundList.get();
				while (_loc10_ != nullptr)
				{
					if (((_loc10_->visible) && ((((_loc10_->culling = param2) == 0) && (this->numOccluders == 0)) || ((_loc10_->culling = this->cullingInContainer(param2, _loc11_->bound.MinX, _loc11_->bound.MinY, _loc11_->bound.MinZ, _loc11_->bound.MaxX, _loc11_->bound.MaxY, _loc11_->bound.MaxZ)) >= 0))))
					{
						_loc10_->copyAndAppend(_loc11_, this);
						_loc10_->concat(this);
						_loc10_->draw(param3);
					}
					_loc10_ = _loc10_->next.get();
					_loc11_ = _loc11_->next.get();
				}
				_loc10_ = param1->occluderList.get();
				_loc11_ = param1->occluderBoundList.get();
				while (_loc10_ != nullptr)
				{
					if (((_loc10_->visible) && ((((_loc10_->culling = param2) == 0) && (this->numOccluders == 0)) || ((_loc10_->culling = this->cullingInContainer(param2, _loc11_->bound.MinX, _loc11_->bound.MinY, _loc11_->bound.MinZ, _loc11_->bound.MaxX, _loc11_->bound.MaxY, _loc11_->bound.MaxZ)) >= 0))))
					{
						_loc10_->copyAndAppend(_loc11_, this);
						_loc10_->concat(this);
						_loc10_->draw(param3);
					}
					_loc10_ = _loc10_->next.get();
					_loc11_ = _loc11_->next.get();
				}
				if (param1->occluderList != nullptr)
				{
					this->updateOccluders(param3);
				}
				this->drawNode(param1->positive, _loc13_, param3, _loc9_);
			}
		}
		else
		{
			if (_loc12_ >= 0)
			{
				while (param4 != nullptr)
				{
					VG *_loc6_ = param4->next.get();
					if (((param4->numOccluders < this->numOccluders) && (this->occludeGeometry(param3, param4))))
					{
						param4->destroy();
					}
					else
					{
						double _loc16_ = ((!(!(_loc14_))) ? param4->boundMinX : ((!(!(_loc15_))) ? param4->boundMinY : param4->boundMinZ));
						double _loc17_ = ((!(!(_loc14_))) ? param4->boundMaxX : ((!(!(_loc15_))) ? param4->boundMaxY : param4->boundMaxZ));
						if (_loc17_ <= param1->maxCoord)
						{
							param4->next = _loc7_;
							_loc7_ = param4;
						}
						else
						{
							if (_loc16_ >= param1->minCoord)
							{
								param4->destroy();
							}
							else
							{
								param4->crop(param3, ((param1->axis == 0) ? -1 : 0), ((param1->axis == 1) ? -1 : 0), ((param1->axis == 2) ? -1 : 0), -(param1->coord), threshold);
								if (param4->faceStruct() != nullptr)
								{
									param4->next = _loc7_;
									_loc7_ = param4;
								}
								else
								{
									param4->destroy();
								}
							}
						}
					}
					param4 = _loc6_;
				}
				this->drawNode(param1->negative, _loc12_, param3, _loc7_);
			}
			else
			{
				if (_loc13_ >= 0)
				{
					while (param4 != nullptr)
					{
						VG *_loc6_ = param4->next.get();
						if (((param4->numOccluders < this->numOccluders) && (this->occludeGeometry(param3, param4))))
						{
							param4->destroy();
						}
						else
						{
							double _loc16_ = ((!(!(_loc14_))) ? param4->boundMinX : ((!(!(_loc15_))) ? param4->boundMinY : param4->boundMinZ));
							double _loc17_ = ((!(!(_loc14_))) ? param4->boundMaxX : ((!(!(_loc15_))) ? param4->boundMaxY : param4->boundMaxZ));

							if (_loc17_ <= param1->maxCoord)
							{
								param4->destroy();
							}
							else
							{
								if (_loc16_ >= param1->minCoord)
								{
									param4->next = _loc9_;
									_loc9_ = param4;
								}
								else
								{
									param4->crop(param3, ((param1->axis == 0) ? 1 : 0), ((param1->axis == 1) ? 1 : 0), ((param1->axis == 2) ? 1 : 0), param1->coord, threshold);
									if (param4->faceStruct() != nullptr)
									{
										param4->next = _loc9_;
										_loc9_ = param4;
									}
									else
									{
										param4->destroy();
									}
								}
							}
						}
						param4 = _loc6_;
					}
					this->drawNode(param1->positive, _loc13_, param3, _loc9_);
				}
				else
				{
					while (param4 != nullptr)
					{
						object_ptr<VG> _loc6_ = param4->next;
						param4->destroy();
						param4 = _loc6_.get();
					}
				}
			}
		}
	}
	else
	{
		if (param1->objectList != nullptr)
		{
			if (((!(param1->objectList->next == nullptr)) || (!(param4 == nullptr))))
			{
				while (param4 != nullptr)
				{
					VG *_loc6_ = param4->next.get();
					if (((param4->numOccluders < this->numOccluders) && (this->occludeGeometry(param3, param4))))
					{
						param4->destroy();
					}
					else
					{
						param4->next = _loc8_;
						_loc8_ = param4;
					}
					param4 = _loc6_;
				}
				Object3D *_loc10_ = param1->objectList.get();
				Object3D *_loc11_ = param1->objectBoundList.get();
				while (_loc10_ != nullptr)
				{
					if (((_loc10_->visible) && ((((_loc10_->culling = param2) == 0) && (this->numOccluders == 0)) || ((_loc10_->culling = this->cullingInContainer(param2, _loc11_->bound.MinX, _loc11_->bound.MinY, _loc11_->bound.MinZ, _loc11_->bound.MaxX, _loc11_->bound.MaxY, _loc11_->bound.MaxZ)) >= 0))))
					{
						_loc10_->copyAndAppend(_loc11_, this);
						_loc10_->concat(this);
						param4 = _loc10_->getVG(param3).get();
						while (param4 != nullptr)
						{
							VG *_loc6_ = param4->next.get();
							param4->next = _loc8_;
							_loc8_ = param4;
							param4 = _loc6_;
						}
					}
					_loc10_ = _loc10_->next.get();
					_loc11_ = _loc11_->next.get();
				}
				if (_loc8_ != nullptr)
				{
					if (_loc8_->next != nullptr)
					{
						drawConflictGeometry(param3, _loc8_);
					}
					else
					{
						_loc8_->draw(param3, threshold, this);
						_loc8_->destroy();
					}
				}
			}
			else
			{
				Object3D *_loc10_ = param1->objectList.get();
				if (_loc10_->visible)
				{
					_loc10_->copyAndAppend(param1->objectBoundList.get(), this);
					_loc10_->culling = param2;
					_loc10_->concat(this);
					_loc10_->draw(param3);
				}
			}
		}
		else
		{
			if (param4 != nullptr)
			{
				if (param4->next != nullptr)
				{
					if (this->numOccluders > 0)
					{
						while (param4 != nullptr)
						{
							VG *_loc6_ = param4->next.get();
							if (((param4->numOccluders < this->numOccluders) && (this->occludeGeometry(param3, param4))))
							{
								param4->destroy();
							}
							else
							{
								param4->next = _loc8_;
								_loc8_ = param4;
							}
							param4 = _loc6_;
						}
						if (_loc8_ != nullptr)
						{
							if (_loc8_->next != nullptr)
							{
								if (resolveByAABB)
								{
									drawAABBGeometry(param3, _loc8_);
								}
								else
								{
									if (resolveByOOBB)
									{
										param4 = _loc8_;
										while (param4 != nullptr)
										{
											param4->calculateOOBB(this);
											param4 = param4->next.get();
										}
										drawOOBBGeometry(param3, _loc8_);
									}
									else
									{
										drawConflictGeometry(param3, _loc8_);
									}
								}
							}
							else
							{
								_loc8_->draw(param3, threshold, this);
								_loc8_->destroy();
							}
						}
					}
					else
					{
						_loc8_ = param4;
						if (resolveByAABB)
						{
							drawAABBGeometry(param3, _loc8_);
						}
						else
						{
							if (resolveByOOBB)
							{
								param4 = _loc8_;
								while (param4 != nullptr)
								{
									param4->calculateOOBB(this);
									param4 = param4->next.get();
								}
								drawOOBBGeometry(param3, _loc8_);
							}
							else
							{
								drawConflictGeometry(param3, _loc8_);
							}
						}
					}
				}
				else
				{
					if (((param4->numOccluders >= this->numOccluders) || (!(this->occludeGeometry(param3, param4)))))
					{
						param4->draw(param3, threshold, this);
					}
					param4->destroy();
				}
			}
		}
		Object3D *_loc10_ = param1->occluderList.get();
		Object3D *_loc11_ = param1->occluderBoundList.get();
		while (_loc10_ != nullptr)
		{
			if (((_loc10_->visible) && ((((_loc10_->culling = param2) == 0) && (this->numOccluders == 0)) || ((_loc10_->culling = this->cullingInContainer(param2, _loc11_->bound.MinX, _loc11_->bound.MinY, _loc11_->bound.MinZ, _loc11_->bound.MaxX, _loc11_->bound.MaxY, _loc11_->bound.MaxZ)) >= 0))))
			{
				_loc10_->copyAndAppend(_loc11_, this);
				_loc10_->concat(this);
				_loc10_->draw(param3);
			}
			_loc10_ = _loc10_->next.get();
			_loc11_ = _loc11_->next.get();
		}
		if (param1->occluderList != nullptr)
		{
			this->updateOccluders(param3);
		}
	}
}





//used
KDNode* KDContainer::createNode(Object3D *param1, Object3D *param2, Object3D *param3, Object3D *param4, double param5, double param6, double param7, double param8, double param9, double param10)
{
	_function_name("KDContainer::createNode");

	double _loc21_ = MyMath::nan(); //nan
	double _loc25_;

	std::shared_ptr<Object3D> _loc29_;
	std::shared_ptr<Object3D> _loc30_;
	std::shared_ptr<Object3D> _loc31_;
	std::shared_ptr<Object3D> _loc32_;
	std::shared_ptr<Object3D> _loc33_;
	std::shared_ptr<Object3D> _loc34_;
	std::shared_ptr<Object3D> _loc35_;
	std::shared_ptr<Object3D> _loc36_;



	KDNode *_loc11_ = new KDNode();
	_loc11_->bound.MinX = param5;
	_loc11_->bound.MinY = param6;
	_loc11_->bound.MinZ = param7;
	_loc11_->bound.MaxX = param8;
	_loc11_->bound.MaxY = param9;
	_loc11_->bound.MaxZ = param10;

	if (param1 == nullptr)
	{
		if (param3 != nullptr)
		{
		}
		return _loc11_;
	}

	int _loc17_ = 0;
	int _loc18_ = 0;
	int _loc19_ = 0;


	QHash<int, double> splitCoordsX;
	QHash<int, double> splitCoordsY;
	QHash<int, double> splitCoordsZ;


	std::shared_ptr<Object3D> _loc15_ = param2->ptr();
	while (_loc15_ != nullptr)
	{
		if (_loc15_->bound.MinX > (param5 + this->threshold))
		{
			int _loc13_ = 0;
			while (_loc13_ < _loc17_)
			{
				if (((_loc15_->bound.MinX >= (splitCoordsX[_loc13_] - threshold)) && (_loc15_->bound.MinX <= (splitCoordsX[_loc13_] + threshold))))
				{
					break;
				}
				_loc13_++;
			}
			if (_loc13_ == _loc17_)
			{
				int _local_53 = _loc17_++;
				splitCoordsX[_local_53] = _loc15_->bound.MinX;
			}
		}
		if (_loc15_->bound.MaxX < (param8 - threshold))
		{
			int _loc13_ = 0;
			while (_loc13_ < _loc17_)
			{
				if (((_loc15_->bound.MaxX >= (splitCoordsX[_loc13_] - threshold)) && (_loc15_->bound.MaxX <= (splitCoordsX[_loc13_] + threshold))))
				{
					break;
				}
				_loc13_++;
			}
			if (_loc13_ == _loc17_)
			{
				int _local_53 = _loc17_++;
				splitCoordsX[_local_53] = _loc15_->bound.MaxX;
			}
		}
		if (_loc15_->bound.MinY > (param6 + threshold))
		{
			int _loc13_ = 0;
			while (_loc13_ < _loc18_)
			{
				if (((_loc15_->bound.MinY >= (splitCoordsY[_loc13_] - threshold)) && (_loc15_->bound.MinY <= (splitCoordsY[_loc13_] + threshold))))
				{
					break;
				}
				_loc13_++;
			}
			if (_loc13_ == _loc18_)
			{
				int _local_53 = _loc18_++;
				splitCoordsY[_local_53] = _loc15_->bound.MinY;
			}
		}
		if (_loc15_->bound.MaxY < (param9 - threshold))
		{
			int _loc13_ = 0;
			while (_loc13_ < _loc18_)
			{
				if (((_loc15_->bound.MaxY >= (splitCoordsY[_loc13_] - threshold)) && (_loc15_->bound.MaxY <= (splitCoordsY[_loc13_] + threshold))))
				{
					break;
				}
				_loc13_++;
			}
			if (_loc13_ == _loc18_)
			{
				int _local_53 = _loc18_++;
				splitCoordsY[_local_53] = _loc15_->bound.MaxY;
			}
		}
		if (_loc15_->bound.MinZ > (param7 + threshold))
		{
			int _loc13_ = 0;
			while (_loc13_ < _loc19_)
			{
				if (((_loc15_->bound.MinZ >= (splitCoordsZ[_loc13_] - threshold)) && (_loc15_->bound.MinZ <= (splitCoordsZ[_loc13_] + threshold))))
				{
					break;
				}
				_loc13_++;
			}
			if (_loc13_ == _loc19_)
			{
				int _local_53 = _loc19_++;
				splitCoordsZ[_local_53] = _loc15_->bound.MinZ;
			}
		}
		if (_loc15_->bound.MaxZ < (param10 - threshold))
		{
			int _loc13_ = 0;
			while (_loc13_ < _loc19_)
			{
				if (((_loc15_->bound.MaxZ >= (splitCoordsZ[_loc13_] - threshold)) && (_loc15_->bound.MaxZ <= (splitCoordsZ[_loc13_] + threshold))))
				{
					break;
				}
				_loc13_++;
			}
			if (_loc13_ == _loc19_)
			{
				int _local_53 = _loc19_++;
				splitCoordsZ[_local_53] = _loc15_->bound.MaxZ;
			}
		}
		_loc15_ = _loc15_->next;
	}
	int _loc20_ = -1;
	double _loc22_ = 1E22;
	_loc25_ = ((param9 - param6) * (param10 - param7));
	int _loc12_ = 0;
	while (_loc12_ < _loc17_)
	{
		double _loc16_ = splitCoordsX[_loc12_];
		double _loc26_ = (_loc25_ * (_loc16_ - param5));
		double _loc27_ = (_loc25_ * (param8 - _loc16_));
		int _loc23_ = 0;
		int _loc24_ = 0;
		_loc15_ = param2->ptr();
		while (_loc15_ != nullptr)
		{
			if (_loc15_->bound.MaxX <= (_loc16_ + threshold))
			{
				if (_loc15_->bound.MinX < (_loc16_ - threshold))
				{
					_loc23_++;
				}
			}
			else
			{
				if (_loc15_->bound.MinX >= (_loc16_ - threshold))
				{
					_loc24_++;
				}
				else
				{
					break;
				}
			}
			_loc15_ = _loc15_->next;
		}
		if (_loc15_ == nullptr)
		{
			double _loc28_ = ((_loc26_ * _loc23_) + (_loc27_ * _loc24_));
			if (_loc28_ < _loc22_)
			{
				_loc22_ = _loc28_;
				_loc20_ = 0;
				_loc21_ = _loc16_;
			}
		}
		_loc12_++;
	}
	_loc25_ = ((param8 - param5) * (param10 - param7));
	_loc12_ = 0;
	while (_loc12_ < _loc18_)
	{
		double _loc16_ = splitCoordsY[_loc12_];
		double _loc26_ = (_loc25_ * (_loc16_ - param6));
		double _loc27_ = (_loc25_ * (param9 - _loc16_));
		int _loc23_ = 0;
		int _loc24_ = 0;
		_loc15_ = param2->ptr();
		while (_loc15_ != nullptr)
		{
			if (_loc15_->bound.MaxY <= (_loc16_ + threshold))
			{
				if (_loc15_->bound.MinY < (_loc16_ - threshold))
				{
					_loc23_++;
				}
			}
			else
			{
				if (_loc15_->bound.MinY >= (_loc16_ - threshold))
				{
					_loc24_++;
				}
				else
				{
					break;
				}
			}
			_loc15_ = _loc15_->next;
		}
		if (_loc15_ == nullptr)
		{
			double _loc28_ = ((_loc26_ * _loc23_) + (_loc27_ * _loc24_));
			if (_loc28_ < _loc22_)
			{
				_loc22_ = _loc28_;
				_loc20_ = 1;
				_loc21_ = _loc16_;
			}
		}
		_loc12_++;
	}
	_loc25_ = (param8 - param5) * (param9 - param6);
	_loc12_ = 0;
	while (_loc12_ < _loc19_)
	{
		double _loc16_ = splitCoordsZ[_loc12_];
		double _loc26_ = (_loc25_ * (_loc16_ - param7));
		double _loc27_ = (_loc25_ * (param10 - _loc16_));
		int _loc23_ = 0;
		int _loc24_ = 0;
		_loc15_ = param2->ptr();
		while (_loc15_ != nullptr)
		{
			if (_loc15_->bound.MaxZ <= (_loc16_ + threshold))
			{
				if (_loc15_->bound.MinZ < (_loc16_ - threshold))
				{
					_loc23_++;
				}
			}
			else
			{
				if (_loc15_->bound.MinZ >= (_loc16_ - threshold))
				{
					_loc24_++;
				}
				else
				{
					break;
				}
			}
			_loc15_ = _loc15_->next;
		}
		if (_loc15_ == nullptr)
		{
			double _loc28_ = ((_loc26_ * _loc23_) + (_loc27_ * _loc24_));
			if (_loc28_ < _loc22_)
			{
				_loc22_ = _loc28_;
				_loc20_ = 2;
				_loc21_ = _loc16_;
			}
		}
		_loc12_++;
	}
	if (_loc20_ < 0)
	{
		_loc11_->objectList = Object3D::ptr_safe(param1);
		_loc11_->objectBoundList = Object3D::ptr_safe(param2);
		_loc11_->occluderList = Object3D::ptr_safe(param3);
		_loc11_->occluderBoundList = Object3D::ptr_safe(param4);
	}
	else
	{
		_loc11_->axis = _loc20_;
		_loc11_->coord = _loc21_;
		_loc11_->minCoord = (_loc21_ - threshold);
		_loc11_->maxCoord = (_loc21_ + threshold);
		std::shared_ptr<Object3D> _loc14_ = Object3D::ptr_safe(param1);
		_loc15_ = Object3D::ptr_safe(param2);
		while (_loc14_ != nullptr)
		{
			std::shared_ptr<Object3D> _loc39_ = _loc14_->next;
			std::shared_ptr<Object3D> _loc40_ = _loc15_->next;
			_loc14_->next = nullptr;
			_loc15_->next = nullptr;
			double _loc37_ = (_loc20_ == 0) ? _loc15_->bound.MinX : ((_loc20_ == 1) ? _loc15_->bound.MinY : _loc15_->bound.MinZ);
			double _loc38_ = (_loc20_ == 0) ? _loc15_->bound.MaxX : ((_loc20_ == 1) ? _loc15_->bound.MaxY : _loc15_->bound.MaxZ);
			if (_loc38_ <= (_loc21_ + threshold))
			{
				if (_loc37_ < (_loc21_ - threshold))
				{
					_loc14_->next = _loc29_;
					_loc29_ = _loc14_;
					_loc15_->next = _loc30_;
					_loc30_ = _loc15_;
				}
				else
				{
					_loc14_->next = _loc11_->objectList;
					_loc11_->objectList = _loc14_;
					_loc15_->next = _loc11_->objectBoundList;
					_loc11_->objectBoundList = _loc15_;
				}
			}
			else
			{
				if (_loc37_ >= (_loc21_ - threshold))
				{
					_loc14_->next = _loc33_;
					_loc33_ = _loc14_;
					_loc15_->next = _loc34_;
					_loc34_ = _loc15_;
				}
			}
			_loc14_ = _loc39_;
			_loc15_ = _loc40_;
		}
		_loc14_ = Object3D::ptr_safe(param3);
		_loc15_ = Object3D::ptr_safe(param4);
		while (_loc14_ != nullptr)
		{
			std::shared_ptr<Object3D> _loc39_ = _loc14_->next;
			std::shared_ptr<Object3D> _loc40_ = _loc15_->next;
			_loc14_->next = nullptr;
			_loc15_->next = nullptr;
			double _loc37_ = ((_loc20_ == 0) ? _loc15_->bound.MinX : ((_loc20_ == 1) ? _loc15_->bound.MinY : _loc15_->bound.MinZ));
			double _loc38_ = ((_loc20_ == 0) ? _loc15_->bound.MaxX : ((_loc20_ == 1) ? _loc15_->bound.MaxY : _loc15_->bound.MaxZ));
			if (_loc38_ <= (_loc21_ + threshold))
			{
				if (_loc37_ < (_loc21_ - threshold))
				{
					_loc14_->next = _loc31_;
					_loc31_ = _loc14_;
					_loc15_->next = _loc32_;
					_loc32_ = _loc15_;
				}
				else
				{
					_loc14_->next = _loc11_->occluderList;
					_loc11_->occluderList = _loc14_;
					_loc15_->next = _loc11_->occluderBoundList;
					_loc11_->occluderBoundList = _loc15_;
				}
			}
			else
			{
				if (_loc37_ >= (_loc21_ - threshold))
				{
					_loc14_->next = _loc35_;
					_loc35_ = _loc14_;
					_loc15_->next = _loc36_;
					_loc36_ = _loc15_;
				}
			}
			_loc14_ = _loc39_;
			_loc15_ = _loc40_;
		}
		double _loc41_ = _loc11_->bound.MinX;
		double _loc42_ = _loc11_->bound.MinY;
		double _loc43_ = _loc11_->bound.MinZ;
		double _loc44_ = _loc11_->bound.MaxX;
		double _loc45_ = _loc11_->bound.MaxY;
		double _loc46_ = _loc11_->bound.MaxZ;
		double _loc47_ = _loc11_->bound.MinX;
		double _loc48_ = _loc11_->bound.MinY;
		double _loc49_ = _loc11_->bound.MinZ;
		double _loc50_ = _loc11_->bound.MaxX;
		double _loc51_ = _loc11_->bound.MaxY;
		double _loc52_ = _loc11_->bound.MaxZ;
		if (_loc20_ == 0)
		{
			_loc44_ = _loc21_;
			_loc47_ = _loc21_;
		}
		else
		{
			if (_loc20_ == 1)
			{
				_loc45_ = _loc21_;
				_loc48_ = _loc21_;
			}
			else
			{
				_loc46_ = _loc21_;
				_loc49_ = _loc21_;
			}
		}
		_loc11_->negative = this->createNode(_loc29_.get(), _loc30_.get(), _loc31_.get(), _loc32_.get(), _loc41_, _loc42_, _loc43_, _loc44_, _loc45_, _loc46_);
		_loc11_->positive = this->createNode(_loc33_.get(), _loc34_.get(), _loc35_.get(), _loc36_.get(), _loc47_, _loc48_, _loc49_, _loc50_, _loc51_, _loc52_);
	}

	return _loc11_;
}


//used
static void destroyNode(KDNode *param1)
{	
	_function_name("KDContainer::destroyNode");

	if (param1->negative != nullptr)
	{
		destroyNode(param1->negative);
		param1->negative = nullptr;
	}

	if (param1->positive != nullptr)
	{
		destroyNode(param1->positive);
		param1->positive = nullptr;
	}

	std::shared_ptr<Object3D> o = param1->objectList;
	while (o != nullptr)
	{
		std::shared_ptr<Object3D> next = o->next;
		o->setParent(nullptr);
		o->next = nullptr;
		o = next;
	}

	o = param1->objectBoundList;
	while (o != nullptr)
	{
		std::shared_ptr<Object3D> next = o->next;
		o->next = nullptr;
		o = next;
	}

	o = param1->occluderList;
	while (o != nullptr)
	{
		std::shared_ptr<Object3D> next = o->next;
		o->setParent(nullptr);
		o->next = nullptr;
		o = next;
	}

	o = param1->occluderBoundList;
	while (o != nullptr)
	{
		std::shared_ptr<Object3D> next = o->next;
		o->next = nullptr;
		o = next;
	}

	std::shared_ptr<Receiver> r = param1->receiverList;
	while (r != nullptr)
	{
		std::shared_ptr<Receiver> next = r->next;
		r->next = nullptr;
		r = next;
	}

	param1->objectList = nullptr;
	param1->objectBoundList = nullptr;
	param1->occluderList = nullptr;
	param1->occluderBoundList = nullptr;
	param1->receiverList = nullptr;
}



//not used
bool KDContainer::occludeGeometry(Camera3D *param1, VG *param2)
{
	(void)param1;
	(void)param2;

	qDebug("used KDContainer::occludeGeometry");


	int _loc3_ = param2->numOccluders;
	while (true)
	{
		if (_loc3_ >= this->numOccluders)
		{
			param2->numOccluders = this->numOccluders;
			return false;
		}

		Vertex *_loc4_ = this->occluders[_loc3_];
		while (_loc4_ != nullptr)
		{
			if (_loc4_->x >= 0)
			{
				if (_loc4_->y >= 0)
				{
					if (_loc4_->z >= 0)
					{
						if ((((param2->boundMaxX * _loc4_->x) + (param2->boundMaxY * _loc4_->y)) + (param2->boundMaxZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param2->boundMaxX * _loc4_->x) + (param2->boundMaxY * _loc4_->y)) + (param2->boundMinZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
				}
				else
				{
					if (_loc4_->z >= 0)
					{
						if ((((param2->boundMaxX * _loc4_->x) + (param2->boundMinY * _loc4_->y)) + (param2->boundMaxZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param2->boundMaxX * _loc4_->x) + (param2->boundMinY * _loc4_->y)) + (param2->boundMinZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
				}
			}
			else
			{
				if (_loc4_->y >= 0)
				{
					if (_loc4_->z >= 0)
					{
						if ((((param2->boundMinX * _loc4_->x) + (param2->boundMaxY * _loc4_->y)) + (param2->boundMaxZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param2->boundMinX * _loc4_->x) + (param2->boundMaxY * _loc4_->y)) + (param2->boundMinZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
				}
				else
				{
					if (_loc4_->z >= 0)
					{
						if ((((param2->boundMinX * _loc4_->x) + (param2->boundMinY * _loc4_->y)) + (param2->boundMaxZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
					else
					{
						if ((((param2->boundMinX * _loc4_->x) + (param2->boundMinY * _loc4_->y)) + (param2->boundMinZ * _loc4_->z)) > _loc4_->offset)
						{
							break;
						}
					}
				}
			}
			_loc4_ = _loc4_->next.get();
		}
		if (_loc4_ == nullptr)
		{
			break;
		}
		_loc3_++;
	}

	return true;
}


std::shared_ptr<KDContainer> new_KDContainer()
{
	_function_name("new_KDContainer");

	return std::make_shared<KDContainer>(Object3D::TT(KDContainer::TYPE));
}
