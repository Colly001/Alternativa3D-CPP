#include "conflictcontainer.h"
#include "alternativa/engine3d/core/vg.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/engine3d/materials/material.h"
#include "mymath.h"


const Object3D::T *ConflictContainer::TYPE = new Object3D::T("ConflictContainer");



ConflictContainer::ConflictContainer(const TT &t) :
	Object3DContainer(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	resolveByAABB = true;
	resolveByOOBB = true;
	threshold = 0.01;
}

void ConflictContainer::clonePropertiesFromConflictContainer(ConflictContainer *src)
{
	this->clonePropertiesFromObject3DContainer(src);
	this->resolveByAABB = src->resolveByAABB;
	this->resolveByOOBB = src->resolveByOOBB;
	this->threshold = src->threshold;
}

void ConflictContainer::draw(Camera3D *param1)
{
	object_ptr<VG> _loc3_ = getVG(param1);
	if (_loc3_ != nullptr)
	{
		/*
		if (((param1->debug) && ((_loc2_ = param1->checkInDebug(this)) > 0)))
		{
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
			}
		}
		*/

		if (_loc3_->next != nullptr)
		{
			calculateInverseMatrix();
			if (this->resolveByAABB)
			{
				VG* _loc4_ = _loc3_.get();
				while (_loc4_ != nullptr)
				{
					_loc4_->calculateAABB(&matrix_i);

					_loc4_ = _loc4_->next.get();
				}
				this->drawAABBGeometry(param1, _loc3_.get());
			}
			else
			{
				if (this->resolveByOOBB)
				{
					VG *_loc4_ = _loc3_.get();
					while (_loc4_ != nullptr)
					{
						_loc4_->calculateOOBB(this);
						_loc4_ = _loc4_->next.get();
					}
					this->drawOOBBGeometry(param1, _loc3_.get());
				}
				else
				{
					this->drawConflictGeometry(param1, _loc3_.get());
				}
			}
		}
		else
		{
			_loc3_->draw(param1, this->threshold, this);
			_loc3_->destroy();
		}
	}

}

void ConflictContainer::drawAABBGeometry(Camera3D *param1, object_ptr<VG> param2, bool param3, bool param4, bool param5, int param6)
{
	bool _loc7_ = false;
	bool _loc8_ = false;
	bool _loc9_ = false;


	object_ptr<VG> _loc10_ = ((!(!(param5))) ? this->sortGeometry(param2, param3, param4) : param2);
	object_ptr<VG> _loc11_ = _loc10_;
	object_ptr<VG> _loc12_ = _loc10_->next;
	double _loc13_ = _loc10_->boundMax;

	while (_loc12_ != nullptr)
	{
		bool _loc14_ = (_loc12_->boundMin >= (_loc13_ - this->threshold));
		if (((_loc14_) || (_loc12_->next == nullptr)))
		{
			if (_loc14_)
			{
				_loc11_->next = nullptr;
				param6 = 0;
			}
			else
			{
				_loc12_ = nullptr;
				param6++;
			}
			if (param3)
			{
				_loc7_ = (matrix_i.md < _loc13_);
				_loc8_ = false;
				_loc9_ = true;
			}
			else
			{
				if (param4)
				{
					_loc7_ = (matrix_i.mh < _loc13_);
					_loc8_ = false;
					_loc9_ = false;
				}
				else
				{
					_loc7_ = (matrix_i.ml < _loc13_);
					_loc8_ = true;
					_loc9_ = false;
				}
			}
			if (_loc7_)
			{
				if (_loc10_->next != nullptr)
				{
					if (param6 < 2)
					{
						this->drawAABBGeometry(param1, _loc10_, _loc8_, _loc9_, true, param6);
					}
					else
					{
						if (this->resolveByOOBB)
						{
							_loc11_ = _loc10_;
							while (_loc11_ != nullptr)
							{
								_loc11_->calculateOOBB(this);
								_loc11_ = _loc11_->next;
							}
							this->drawOOBBGeometry(param1, _loc10_);
						}
						else
						{
							this->drawConflictGeometry(param1, _loc10_);
						}
					}
				}
				else
				{
					_loc10_->draw(param1, this->threshold, this);
					_loc10_->destroy();
				}
				if (_loc12_ != nullptr)
				{
					if (_loc12_->next != nullptr)
					{
						this->drawAABBGeometry(param1, _loc12_, param3, param4, false, -1);
					}
					else
					{
						_loc12_->draw(param1, this->threshold, this);
						_loc12_->destroy();
					}
				}
			}
			else
			{
				if (_loc12_ != nullptr)
				{
					if (_loc12_->next != nullptr)
					{
						this->drawAABBGeometry(param1, _loc12_, param3, param4, false, -1);
					}
					else
					{
						_loc12_->draw(param1, this->threshold, this);
						_loc12_->destroy();
					}
				}
				if (_loc10_->next != nullptr)
				{
					if (param6 < 2)
					{
						this->drawAABBGeometry(param1, _loc10_, _loc8_, _loc9_, true, param6);
					}
					else
					{
						if (this->resolveByOOBB)
						{
							_loc11_ = _loc10_;
							while (_loc11_ != nullptr)
							{
								_loc11_->calculateOOBB(this);
								_loc11_ = _loc11_->next;
							}
							this->drawOOBBGeometry(param1, _loc10_);
						}
						else
						{
							this->drawConflictGeometry(param1, _loc10_);
						}
					}
				}
				else
				{
					_loc10_->draw(param1, this->threshold, this);
					_loc10_->destroy();
				}
			}
			break;
		}

		if (_loc12_->boundMax > _loc13_)
		{
			_loc13_ = _loc12_->boundMax;
		}
		_loc11_ = _loc12_;
		_loc12_ = _loc12_->next.get();
	}
}

object_ptr<VG> ConflictContainer::sortGeometry(object_ptr<VG> param1, bool param2, bool param3)
{
	object_ptr<VG> _loc4_ = param1;
	object_ptr<VG> _loc5_ = param1->next;


	while (((!(_loc5_ == nullptr)) && (!(_loc5_->next == nullptr))))
	{
		param1 = param1->next;
		_loc5_ = _loc5_->next->next;
	}

	_loc5_ = param1->next;
	param1->next = nullptr;

	if (_loc4_->next != nullptr)
	{
		_loc4_ = this->sortGeometry(_loc4_, param2, param3);
	}
	else
	{
		if (param2)
		{
			_loc4_->boundMin = _loc4_->boundMinX;
			_loc4_->boundMax = _loc4_->boundMaxX;
		}
		else
		{
			if (param3)
			{
				_loc4_->boundMin = _loc4_->boundMinY;
				_loc4_->boundMax = _loc4_->boundMaxY;
			}
			else
			{
				_loc4_->boundMin = _loc4_->boundMinZ;
				_loc4_->boundMax = _loc4_->boundMaxZ;
			}
		}
	}
	if (_loc5_->next != nullptr)
	{
		_loc5_ = this->sortGeometry(_loc5_, param2, param3);
	}
	else
	{
		if (param2)
		{
			_loc5_->boundMin = _loc5_->boundMinX;
			_loc5_->boundMax = _loc5_->boundMaxX;
		}
		else
		{
			if (param3)
			{
				_loc5_->boundMin = _loc5_->boundMinY;
				_loc5_->boundMax = _loc5_->boundMaxY;
			}
			else
			{
				_loc5_->boundMin = _loc5_->boundMinZ;
				_loc5_->boundMax = _loc5_->boundMaxZ;
			}
		}
	}

	bool _loc6_ = (_loc4_->boundMin < _loc5_->boundMin);
	if (_loc6_)
	{
		param1 = _loc4_;
		_loc4_ = _loc4_->next;
	}
	else
	{
		param1 = _loc5_;
		_loc5_ = _loc5_->next;
	}

	object_ptr<VG> _loc7_ = param1;
	for (;;)
	{
		if (_loc4_ == nullptr)
		{
			_loc7_->next = _loc5_;
			return param1;
		}

		if (_loc5_ == nullptr)
		{
			_loc7_->next = _loc4_;
			return param1;
		}

		if (_loc6_)
		{
			if (_loc4_->boundMin < _loc5_->boundMin)
			{
				_loc7_ = _loc4_;
				_loc4_ = _loc4_->next;
			}
			else
			{
				_loc7_->next = _loc5_;
				_loc7_ = _loc5_;
				_loc5_ = _loc5_->next;
				_loc6_ = false;
			}
		}
		else
		{
			if (_loc5_->boundMin < _loc4_->boundMin)
			{
				_loc7_ = _loc5_;
				_loc5_ = _loc5_->next;
			}
			else
			{
				_loc7_->next = _loc4_;
				_loc7_ = _loc4_;
				_loc4_ = _loc4_->next;
				_loc6_ = true;
			}
		}
	}
	return nullptr;
}

void ConflictContainer::drawOOBBGeometry(Camera3D *param1, object_ptr<VG> param2)
{
	object_ptr<VG> _loc17_ = nullptr;
	object_ptr<VG> _loc18_ = nullptr;
	object_ptr<VG> _loc19_ = nullptr;

	object_ptr<VG> _loc13_ = param2;
	double _loc10_;
	double _loc7_ = 0;
	double _loc8_ = 0;
	double _loc9_ = 0;

	while (_loc13_ != nullptr)
	{
		if (_loc13_->viewAligned)
		{
			_loc10_ = _loc13_->object->matrix.ml;
			object_ptr<VG> _loc14_ = param2;
			while (_loc14_ != nullptr)
			{
				if (!_loc14_->viewAligned)
				{
					bool _loc11_ = false;
					bool _loc12_ = false;
					Vertex *_loc3_ = _loc14_->boundVertexList.get();
					while (_loc3_ != nullptr)
					{
						if (_loc3_->cameraZ > _loc10_)
						{
							if (_loc11_)
							{
								break;
							}
							_loc12_ = true;
						}
						else
						{
							if (_loc12_)
							{
								break;
							}
							_loc11_ = true;
						}
						_loc3_ = _loc3_->next.get();
					}
					if (_loc3_ != nullptr)
					{
						break;
					}
				}
				_loc14_ = _loc14_->next;
			}
			if (_loc14_ == nullptr)
			{
				break;
			}
		}
		else
		{
			Vertex *_loc4_ = _loc13_->boundPlaneList.get();
			while (_loc4_ != nullptr)
			{
				_loc7_ = _loc4_->cameraX;
				_loc8_ = _loc4_->cameraY;
				_loc9_ = _loc4_->cameraZ;
				_loc10_ = _loc4_->offset;
				bool _loc15_ = false;
				object_ptr<VG> _loc14_ = param2;
				while (_loc14_ != nullptr)
				{
					if (_loc13_ != _loc14_)
					{
						bool _loc11_ = false;
						bool _loc12_ = false;
						if (_loc14_->viewAligned)
						{
							Wrapper *_loc5_ = _loc14_->faceStruct()->wrapper.get();
							while (_loc5_ != nullptr)
							{
								Vertex *_loc3_ = _loc5_->vertex.get();
								if ((((_loc3_->cameraX * _loc7_) + (_loc3_->cameraY * _loc8_)) + (_loc3_->cameraZ * _loc9_)) >= (_loc10_ - this->threshold))
								{
									if (_loc11_)
									{
										break;
									}
									_loc15_ = true;
									_loc12_ = true;
								}
								else
								{
									if (_loc12_)
									{
										break;
									}
									_loc11_ = true;
								}
								_loc5_ = _loc5_->_next;
							}
							if (_loc5_ != nullptr)
							{
								break;
							}
						}
						else
						{
							Vertex *_loc3_ = _loc14_->boundVertexList.get();
							while (_loc3_ != nullptr)
							{
								if ((((_loc3_->cameraX * _loc7_) + (_loc3_->cameraY * _loc8_)) + (_loc3_->cameraZ * _loc9_)) >= (_loc10_ - this->threshold))
								{
									if (_loc11_)
									{
										break;
									}
									_loc15_ = true;
									_loc12_ = true;
								}
								else
								{
									if (_loc12_)
									{
										break;
									}
									_loc11_ = true;
								}
								_loc3_ = _loc3_->next.get();
							}
							if (_loc3_ != nullptr)
							{
								break;
							}
						}
					}
					_loc14_ = _loc14_->next;
				}
				if (((_loc14_ == nullptr) && (_loc15_)))
				{
					break;
				}
				_loc4_ = _loc4_->next.get();
			}
			if (_loc4_ != nullptr)
			{
				break;
			}
		}
		_loc13_ = _loc13_->next;
	}

	if (_loc13_ != nullptr)
	{
		if (_loc13_->viewAligned)
		{
			while (param2 != nullptr)
			{
				object_ptr<VG> _loc16_ = param2->next;
				if (param2->viewAligned)
				{
					double _loc6_ = param2->object->matrix.ml - _loc10_;
					if (_loc6_ < -(this->threshold))
					{
						param2->next = _loc19_;
						_loc19_ = param2;
					}
					else
					{
						if (_loc6_ > this->threshold)
						{
							param2->next = _loc17_;
							_loc17_ = param2;
						}
						else
						{
							param2->next = _loc18_;
							_loc18_ = param2;
						}
					}
				}
				else
				{
					Vertex *_loc3_ = param2->boundVertexList.get();
					while (_loc3_ != nullptr)
					{
						double _loc6_ = _loc3_->cameraZ - _loc10_;
						if (_loc6_ < -(this->threshold))
						{
							param2->next = _loc19_;
							_loc19_ = param2;
							break;
						}
						if (_loc6_ > this->threshold)
						{
							param2->next = _loc17_;
							_loc17_ = param2;
							break;
						}
						_loc3_ = _loc3_->next.get();
					}

					if (_loc3_ == nullptr)
					{
						param2->next = _loc18_;
						_loc18_ = param2;
					}
				}

				param2 = _loc16_;
			}
		}
		else
		{
			while (param2 != nullptr)
			{
				object_ptr<VG> _loc16_ = param2->next;
				if (param2->viewAligned)
				{
					Wrapper *_loc5_ = param2->faceStruct()->wrapper.get();
					while (_loc5_ != nullptr)
					{
						Vertex *_loc3_ = _loc5_->vertex.get();
						double _loc6_ = ((((_loc3_->cameraX * _loc7_) + (_loc3_->cameraY * _loc8_)) + (_loc3_->cameraZ * _loc9_)) - _loc10_);
						if (_loc6_ < -(this->threshold))
						{
							param2->next = _loc17_;
							_loc17_ = param2;
							break;
						}
						if (_loc6_ > this->threshold)
						{
							param2->next = _loc19_;
							_loc19_ = param2;
							break;
						}
						_loc5_ = _loc5_->_next;
					}
					if (_loc5_ == nullptr)
					{
						param2->next = _loc18_;
						_loc18_ = param2;
					}
				}
				else
				{
					Vertex *_loc3_ = param2->boundVertexList.get();
					while (_loc3_ != nullptr)
					{
						double _loc6_ = ((((_loc3_->cameraX * _loc7_) + (_loc3_->cameraY * _loc8_)) + (_loc3_->cameraZ * _loc9_)) - _loc10_);
						if (_loc6_ < -(this->threshold))
						{
							param2->next = _loc17_;
							_loc17_ = param2;
							break;
						}
						if (_loc6_ > this->threshold)
						{
							param2->next = _loc19_;
							_loc19_ = param2;
							break;
						}
						_loc3_ = _loc3_->next.get();
					}
					if (_loc3_ == nullptr)
					{
						param2->next = _loc18_;
						_loc18_ = param2;
					}
				}
				param2 = _loc16_;
			}
		}

		if (((_loc13_->viewAligned) || (_loc10_ < 0)))
		{
			if (_loc19_ != nullptr)
			{
				if (_loc19_->next != nullptr)
				{
					this->drawOOBBGeometry(param1, _loc19_);
				}
				else
				{
					_loc19_->draw(param1, this->threshold, this);
					_loc19_->destroy();
				}
			}

			while (_loc18_ != nullptr)
			{
				object_ptr<VG> next = _loc18_->next;
				_loc18_->draw(param1, this->threshold, this);
				_loc18_->destroy();
				_loc18_ = next;
			}

			if (_loc17_ != nullptr)
			{
				if (_loc17_->next != nullptr)
				{
					this->drawOOBBGeometry(param1, _loc17_);
				}
				else
				{
					_loc17_->draw(param1, this->threshold, this);
					_loc17_->destroy();
				}
			}
		}
		else
		{
			if (_loc17_ != nullptr)
			{
				if (_loc17_->next != nullptr)
				{

					//try
					//{
						this->drawOOBBGeometry(param1, _loc17_);
					//}
					//catch(e:Error)
					//{
						//_loc17_->draw(param1, this->threshold, this);
						//_loc17_->destroy();
					//}

				}
				else
				{
					_loc17_->draw(param1, this->threshold, this);
					_loc17_->destroy();
				}
			}

			while (_loc18_ != nullptr)
			{
				object_ptr<VG> next = _loc18_->next;
				_loc18_->draw(param1, this->threshold, this);
				_loc18_->destroy();
				_loc18_ = next;
			}

			if (_loc19_ != nullptr)
			{
				if (_loc19_->next != nullptr)
				{
					this->drawOOBBGeometry(param1, _loc19_);
				}
				else
				{
					_loc19_->draw(param1, this->threshold, this);
					_loc19_->destroy();
				}
			}
		}
	}
	else
	{
		this->drawConflictGeometry(param1, param2);
	}
}

void ConflictContainer::drawConflictGeometry(Camera3D *param1, object_ptr<VG> param2)
{
	object_ptr<VG> _loc6_;
	object_ptr<VG> _loc7_;

	std::shared_ptr<Face> _loc8_;
	std::shared_ptr<Face> _loc9_;
	std::shared_ptr<Face> _loc10_;

	Face *_loc11_ = nullptr;

	Face *_loc13_ = nullptr;
	Face *_loc14_ = nullptr;
	Face *_loc15_ = nullptr;

	while (param2 != nullptr)
	{
		object_ptr<VG> _loc5_ = param2->next;
		if (param2->space == 1)
		{
			param2->transformStruct(param2->faceStruct().get(), ++param2->object->transformId, &this->matrix);
		}
		if (param2->sorting == 3)
		{
			param2->next = _loc6_;
			_loc6_ = param2;
		}
		else
		{
			if (param2->sorting == 2)
			{
				if (_loc8_ != nullptr)
				{
					_loc9_->processNext = param2->faceStruct();
				}
				else
				{
					_loc8_ = param2->faceStruct();
				}
				_loc9_ = param2->faceStruct();
				//_loc9_ null
				_loc9_->geometry = param2;
				while (_loc9_->processNext != nullptr)
				{
					_loc9_ = _loc9_->processNext;
					_loc9_->geometry = param2;
				}
			}
			else
			{
				if (_loc10_ != nullptr)
				{
					_loc11_->processNext = param2->faceStruct();
				}
				else
				{
					_loc10_ = param2->faceStruct();
				}

				_loc11_ = param2->faceStruct().get();
				_loc11_->geometry = param2;
				while (_loc11_->processNext != nullptr)
				{
					_loc11_ = _loc11_->processNext.get();
					_loc11_->geometry = param2;
				}
			}
			param2->setFaceStruct(nullptr, 3);
			param2->next = _loc7_;
			_loc7_ = param2;
		}
		param2 = _loc5_;
	}

	if (_loc7_ != nullptr)
	{
		param2 = _loc7_;
		while (param2->next != nullptr)
		{
			param2 = param2->next;
		}
		param2->next = _loc6_;
	}
	else
	{
		_loc7_ = _loc6_;
	}

	Face *_loc12_;

	if (_loc8_ != nullptr)
	{
		_loc12_ = _loc8_.get();
		_loc9_->processNext = _loc10_;
	}
	else
	{
		_loc12_ = _loc10_.get();
	}

	if (_loc6_ != nullptr)
	{
		_loc6_->faceStruct()->geometry = _loc6_;
		_loc12_ = this->collectNode(_loc6_->faceStruct().get(), _loc12_, param1, this->threshold, true);
		_loc6_->setFaceStruct(nullptr, 1);
		_loc6_ = _loc6_->next;
		while (_loc6_ != nullptr)
		{
			_loc6_->faceStruct()->geometry = _loc6_;
			_loc12_ = this->collectNode(_loc6_->faceStruct().get(), _loc12_, param1, this->threshold, false);
			_loc6_->setFaceStruct(nullptr, 2);
			_loc6_ = _loc6_->next;
		}
	}
	else
	{
		if (_loc8_ != nullptr)
		{
			_loc12_ = this->collectNode(nullptr, _loc12_, param1, this->threshold, true);
		}
		else
		{
			if (_loc10_ != nullptr)
			{
				_loc12_ = param1->sortByAverageZ(_loc12_);
			}
		}
	}

	Face *_loc3_ = _loc12_;
	while (_loc3_ != nullptr)
	{
		Face *_loc4_ = _loc3_->processNext.get();
		param2 = _loc3_->geometry;
		_loc3_->geometry = nullptr;
		bool _loc17_ = ((_loc4_ == nullptr) || (!(param2 == _loc4_->geometry)));
		if (((_loc17_) || (!(_loc3_->material == _loc4_->material))))
		{
			_loc3_->processNext = nullptr;
			if (_loc17_)
			{
				if (_loc13_ != nullptr)
				{
					_loc14_->processNegative = Face::ptr_safe(_loc12_);
					_loc13_ = nullptr;
					_loc14_ = nullptr;
				}
				else
				{
					_loc12_->processPositive = Face::ptr_safe(_loc15_);
					_loc15_ = _loc12_;
					_loc15_->geometry = param2;
				}
			}
			else
			{
				if (_loc13_ != nullptr)
				{
					_loc14_->processNegative = Face::ptr_safe(_loc12_);
				}
				else
				{
					_loc12_->processPositive = Face::ptr_safe(_loc15_);
					_loc15_ = _loc12_;
					_loc15_->geometry = param2;
					_loc13_ = _loc12_;
				}
				_loc14_ = _loc12_;
			}
			_loc12_ = _loc4_;
		}
		_loc3_ = _loc4_;
	}

	/*
	if (param1->debug)
	{
		_loc12_ = _loc15_;
		while (_loc12_ != null)
		{
			if ((_loc12_->geometry.debug & Debug.EDGES))
			{
				_loc3_ = _loc12_;
				while (_loc3_ != null)
				{
					Debug.drawEdges(param1, _loc3_, 0xFF0000);
					_loc3_ = _loc3_->processNegative;
				}
			}
			_loc12_ = _loc12_->processPositive;
		}
	}
	*/

	while (_loc15_ != nullptr)
	{
		_loc12_ = _loc15_;
		_loc15_ = _loc12_->processPositive.get();
		_loc12_->processPositive = nullptr;
		param2 = _loc12_->geometry;
		_loc12_->geometry = nullptr;
		Face *_loc16_ = nullptr;
		while (_loc12_ != nullptr)
		{
			Face *_loc4_ = _loc12_->processNegative.get();
			if (_loc12_->material != nullptr)
			{
				_loc12_->processNegative = Face::ptr_safe(_loc16_);
				_loc16_ = _loc12_;
			}
			else
			{
				_loc12_->processNegative = nullptr;
				while (_loc12_ != nullptr)
				{
					_loc3_ = _loc12_->processNext.get();
					_loc12_->processNext = nullptr;
					_loc12_ = _loc3_;
				}
			}
			_loc12_ = _loc4_;
		}

		_loc12_ = _loc16_;
		while (_loc12_ != nullptr)
		{
			Face *_loc4_ = _loc12_->processNegative.get();
			_loc12_->processNegative = nullptr;
			std::shared_ptr<Face> f = Face::ptr_safe(_loc12_);
			param1->addTransparent(f, param2->object);
			_loc12_ = _loc4_;
		}
	}

	param2 = _loc7_;
	while (param2 != nullptr)
	{
		object_ptr<VG> next = param2->next;
		param2->destroy();
		param2 = next;
	}
}

Face* ConflictContainer::collectNode(Face *param1, Face *param2, Camera3D *param3, double param4, bool param5, Face *param6)
{
	double _loc12_;
	double _loc13_;
	double _loc14_;
	double _loc15_;

	Face *_loc16_;
	std::shared_ptr<Face> _loc17_;
	std::shared_ptr<Face> _loc18_;
	object_ptr<VG> _loc19_;

	Face *_loc22_ = nullptr;
	Face *_loc23_ = nullptr;
	Face *_loc24_ = nullptr;
	std::shared_ptr<Face> _loc25_;
	Face *_loc26_;
	std::shared_ptr<Wrapper> _loc52_;


	if (param1 != nullptr)
	{
		_loc19_ = param1->geometry;
		if (param1->offset < 0)
		{
			_loc17_ = param1->processNegative;
			_loc18_ = param1->processPositive;
			_loc12_ = param1->normalX;
			_loc13_ = param1->normalY;
			_loc14_ = param1->normalZ;
			_loc15_ = param1->offset;
		}
		else
		{
			_loc17_ = param1->processPositive;
			_loc18_ = param1->processNegative;
			_loc12_ = -(param1->normalX);
			_loc13_ = -(param1->normalY);
			_loc14_ = -(param1->normalZ);
			_loc15_ = -(param1->offset);
		}
		param1->processNegative = nullptr;
		param1->processPositive = nullptr;
		if (param1->wrapper != nullptr)
		{
			_loc16_ = param1;
			while (_loc16_->processNext != nullptr)
			{
				_loc16_ = _loc16_->processNext.get();
				_loc16_->geometry = _loc19_;
			}
		}
		else
		{
			param1->geometry = nullptr;
			param1 = nullptr;
		}
	}
	else
	{
		param1 = param2;
		param2 = param1->processNext.get();
		_loc16_ = param1;
		Wrapper *_loc7_ = param1->wrapper.get();
		Vertex *_loc8_ = _loc7_->vertex.get();
		_loc7_ = _loc7_->_next;
		Vertex *_loc9_ = _loc7_->vertex.get();
		double _loc28_ = _loc8_->cameraX;
		double _loc29_ = _loc8_->cameraY;
		double _loc30_ = _loc8_->cameraZ;
		double _loc31_ = (_loc9_->cameraX - _loc28_);
		double _loc32_ = (_loc9_->cameraY - _loc29_);
		double _loc33_ = (_loc9_->cameraZ - _loc30_);
		_loc12_ = 0;
		_loc13_ = 0;
		_loc14_ = 1;
		_loc15_ = _loc30_;
		double _loc34_ = 0;
		_loc7_ = _loc7_->_next;
		while (_loc7_ != nullptr)
		{
			Vertex *_loc11_ = _loc7_->vertex.get();
			double _loc35_ = (_loc11_->cameraX - _loc28_);
			double _loc36_ = (_loc11_->cameraY - _loc29_);
			double _loc37_ = (_loc11_->cameraZ - _loc30_);
			double _loc38_ = ((_loc37_ * _loc32_) - (_loc36_ * _loc33_));
			double _loc39_ = ((_loc35_ * _loc33_) - (_loc37_ * _loc31_));
			double _loc40_ = ((_loc36_ * _loc31_) - (_loc35_ * _loc32_));
			double _loc41_ = (((_loc38_ * _loc38_) + (_loc39_ * _loc39_)) + (_loc40_ * _loc40_));
			if (_loc41_ > param4)
			{
				_loc41_ = (1. / MyMath::sqrt(_loc41_));
				_loc12_ = (_loc38_ * _loc41_);
				_loc13_ = (_loc39_ * _loc41_);
				_loc14_ = (_loc40_ * _loc41_);
				_loc15_ = (((_loc28_ * _loc12_) + (_loc29_ * _loc13_)) + (_loc30_ * _loc14_));
				break;
			}
			if (_loc41_ > _loc34_)
			{
				_loc41_ = (1. / MyMath::sqrt(_loc41_));
				_loc12_ = (_loc38_ * _loc41_);
				_loc13_ = (_loc39_ * _loc41_);
				_loc14_ = (_loc40_ * _loc41_);
				_loc15_ = (((_loc28_ * _loc12_) + (_loc29_ * _loc13_)) + (_loc30_ * _loc14_));
				_loc34_ = _loc41_;
			}
			_loc7_ = _loc7_->_next;
		}
	}
	double _loc20_ = (_loc15_ - param4);
	double _loc21_ = (_loc15_ + param4);
	Face *_loc27_ = param2;
	while (_loc27_ != nullptr)
	{
		_loc26_ = _loc27_->processNext.get();
		Wrapper *_loc7_ = _loc27_->wrapper.get();
		Vertex *_loc8_ = _loc7_->vertex.get();
		_loc7_ = _loc7_->_next;
		Vertex *_loc9_ = _loc7_->vertex.get();
		_loc7_ = _loc7_->_next;
		Vertex *_loc10_ = _loc7_->vertex.get();
		_loc7_ = _loc7_->_next;
		double _loc42_ = (((_loc8_->cameraX * _loc12_) + (_loc8_->cameraY * _loc13_)) + (_loc8_->cameraZ * _loc14_));
		double _loc43_ = (((_loc9_->cameraX * _loc12_) + (_loc9_->cameraY * _loc13_)) + (_loc9_->cameraZ * _loc14_));
		double _loc44_ = (((_loc10_->cameraX * _loc12_) + (_loc10_->cameraY * _loc13_)) + (_loc10_->cameraZ * _loc14_));
		bool _loc45_ = (((_loc42_ < _loc20_) || (_loc43_ < _loc20_)) || (_loc44_ < _loc20_));
		bool _loc46_ = (((_loc42_ > _loc21_) || (_loc43_ > _loc21_)) || (_loc44_ > _loc21_));
		while (_loc7_ != nullptr)
		{
			Vertex *_loc11_ = _loc7_->vertex.get();
			double _loc47_ = (((_loc11_->cameraX * _loc12_) + (_loc11_->cameraY * _loc13_)) + (_loc11_->cameraZ * _loc14_));
			if (_loc47_ < _loc20_)
			{
				_loc45_ = true;
			}
			else
			{
				if (_loc47_ > _loc21_)
				{
					_loc46_ = true;
				}
			}
			_loc11_->offset = _loc47_;
			_loc7_ = _loc7_->_next;
		}
		if (!_loc45_)
		{
			if (!_loc46_)
			{
				if (param1 != nullptr)
				{
					_loc16_->processNext = _loc27_->ptr();
				}
				else
				{
					param1 = _loc27_;
				}
				_loc16_ = _loc27_;
			}
			else
			{
				if (_loc24_ != nullptr)
				{
					_loc25_->processNext = _loc27_->ptr();
				}
				else
				{
					_loc24_ = _loc27_;
				}
				_loc25_ = _loc27_->ptr();
			}
		}
		else
		{
			if (!_loc46_)
			{
				if (_loc22_ != nullptr)
				{
					_loc23_->processNext = _loc27_->ptr();
				}
				else
				{
					_loc22_ = _loc27_;
				}
				_loc23_ = _loc27_;
			}
			else
			{
				_loc8_->offset = _loc42_;
				_loc9_->offset = _loc43_;
				_loc10_->offset = _loc44_;
				std::shared_ptr<Face> _loc48_ = _loc27_->create();
				_loc48_->material = _loc27_->material;
				_loc48_->geometry = _loc27_->geometry;
				param3->lastFace->next = _loc48_;
				param3->lastFace = _loc48_;
				std::shared_ptr<Face> _loc49_ = _loc27_->create();
				_loc49_->material = _loc27_->material;
				_loc49_->geometry = _loc27_->geometry;
				param3->lastFace->next = _loc49_;
				param3->lastFace = _loc49_;
				std::shared_ptr<Wrapper> _loc50_ = nullptr;
				std::shared_ptr<Wrapper> _loc51_ = nullptr;
				_loc7_ = _loc27_->wrapper->_next->_next;
				while (_loc7_->_next != nullptr)
				{
					_loc7_ = _loc7_->_next;
				}
				_loc8_ = _loc7_->vertex.get();
				_loc42_ = _loc8_->offset;
				bool _loc53_ = ((!(_loc27_->material == nullptr)) && (_loc27_->material->useVerticesNormals));
				_loc7_ = _loc27_->wrapper.get();
				while (_loc7_ != nullptr)
				{
					_loc9_ = _loc7_->vertex.get();
					_loc43_ = _loc9_->offset;
					if ((((_loc42_ < _loc20_) && (_loc43_ > _loc21_)) || ((_loc42_ > _loc21_) && (_loc43_ < _loc20_))))
					{
						double _loc54_ = ((_loc15_ - _loc42_) / (_loc43_ - _loc42_));
						std::shared_ptr<Vertex> _loc11_ = Vertex::create();
						param3->lastVertex->next = _loc11_;
						param3->lastVertex = _loc11_;
						_loc11_->cameraX = (_loc8_->cameraX + ((_loc9_->cameraX - _loc8_->cameraX) * _loc54_));
						_loc11_->cameraY = (_loc8_->cameraY + ((_loc9_->cameraY - _loc8_->cameraY) * _loc54_));
						_loc11_->cameraZ = (_loc8_->cameraZ + ((_loc9_->cameraZ - _loc8_->cameraZ) * _loc54_));
						_loc11_->u = (_loc8_->u + ((_loc9_->u - _loc8_->u) * _loc54_));
						_loc11_->v = (_loc8_->v + ((_loc9_->v - _loc8_->v) * _loc54_));
						if (_loc53_)
						{
							_loc11_->x = (_loc8_->x + ((_loc9_->x - _loc8_->x) * _loc54_));
							_loc11_->y = (_loc8_->y + ((_loc9_->y - _loc8_->y) * _loc54_));
							_loc11_->z = (_loc8_->z + ((_loc9_->z - _loc8_->z) * _loc54_));
							_loc11_->normalX = (_loc8_->normalX + ((_loc9_->normalX - _loc8_->normalX) * _loc54_));
							_loc11_->normalY = (_loc8_->normalY + ((_loc9_->normalY - _loc8_->normalY) * _loc54_));
							_loc11_->normalZ = (_loc8_->normalZ + ((_loc9_->normalZ - _loc8_->normalZ) * _loc54_));
						}
						_loc52_ = Wrapper::create(22);
						_loc52_->vertex = _loc11_;
						if (_loc50_ != nullptr)
						{
							_loc50_->setNext(_loc52_);
						}
						else
						{
							_loc48_->wrapper = _loc52_;
						}
						_loc50_ = _loc52_;
						_loc52_ = Wrapper::create(23);
						_loc52_->vertex = _loc11_;
						if (_loc51_ != nullptr)
						{
							_loc51_->setNext(_loc52_);
						}
						else
						{
							_loc49_->wrapper = _loc52_;
						}
						_loc51_ = _loc52_;
					}
					if (_loc43_ <= _loc21_)
					{
						_loc52_ = Wrapper::create(24);
						_loc52_->vertex = _loc9_->ptr();
						if (_loc50_ != nullptr)
						{
							_loc50_->setNext(_loc52_);
						}
						else
						{
							_loc48_->wrapper = _loc52_;
						}
						_loc50_ = _loc52_;
					}
					if (_loc43_ >= _loc20_)
					{
						_loc52_ = Wrapper::create(25);
						_loc52_->vertex = _loc9_->ptr();
						if (_loc51_ != nullptr)
						{
							_loc51_->setNext(_loc52_);
						}
						else
						{
							_loc49_->wrapper = _loc52_;
						}
						_loc51_ = _loc52_;
					}
					_loc8_ = _loc9_;
					_loc42_ = _loc43_;
					_loc7_ = _loc7_->_next;
				}
				if (_loc22_ != nullptr)
				{
					_loc23_->processNext = _loc48_;
				}
				else
				{
					_loc22_ = _loc48_.get();
				}
				_loc23_ = _loc48_.get();
				if (_loc24_ != nullptr)
				{
					_loc25_->processNext = _loc49_;
				}
				else
				{
					_loc24_ = _loc49_.get();
				}
				_loc25_ = _loc49_;
				_loc27_->processNext = nullptr;
				_loc27_->geometry = nullptr;
			}
		}
		_loc27_ = _loc26_;
	}
	if (_loc18_ != nullptr)
	{
		_loc18_->geometry = _loc19_;
		if (_loc25_ != nullptr)
		{
			_loc25_->processNext = nullptr;
		}
		param6 = this->collectNode(_loc18_.get(), _loc24_, param3, param4, param5, param6);
	}
	else
	{
		if (_loc24_ != nullptr)
		{
			if (((param5) && (!(_loc24_ == _loc25_.get()))))
			{
				if (_loc25_ != nullptr)
				{
					_loc25_->processNext = nullptr;
				}
				if (_loc24_->geometry->sorting == 2)
				{
					param6 = this->collectNode(nullptr, _loc24_, param3, param4, param5, param6);
				}
				else
				{
					_loc24_ = param3->sortByAverageZ(_loc24_);
					_loc25_ = _loc24_->processNext;
					while (_loc25_->processNext != nullptr)
					{
						_loc25_ = _loc25_->processNext;
					}
					_loc25_->processNext = Face::ptr_safe(param6);
					param6 = _loc24_;
				}
			}
			else
			{
				_loc25_->processNext = Face::ptr_safe(param6);
				param6 = _loc24_;
			}
		}
	}
	if (param1 != nullptr)
	{
		_loc16_->processNext = Face::ptr_safe(param6);
		param6 = param1;
	}
	if (_loc17_ != nullptr)
	{
		_loc17_->geometry = _loc19_;
		if (_loc23_ != nullptr)
		{
			_loc23_->processNext = nullptr;
		}
		param6 = this->collectNode(_loc17_.get(), _loc22_, param3, param4, param5, param6);
	}
	else
	{
		if (_loc22_ != nullptr)
		{
			if (param5 && _loc22_ != _loc23_)
			{
				if (_loc23_ != nullptr)
				{
					_loc23_->processNext = nullptr;
				}
				if (_loc22_->geometry->sorting == 2)
				{
					param6 = this->collectNode(nullptr, _loc22_, param3, param4, param5, param6);
				}
				else
				{
					_loc22_ = param3->sortByAverageZ(_loc22_);
					_loc23_ = _loc22_->processNext.get();
					while (_loc23_->processNext != nullptr)
					{
						_loc23_ = _loc23_->processNext.get();
					}
					_loc23_->processNext = Face::ptr_safe(param6);
					param6 = _loc22_;
				}
			}
			else
			{
				_loc23_->processNext = Face::ptr_safe(param6);
				param6 = _loc22_;
			}
		}
	}
	return param6;
}
