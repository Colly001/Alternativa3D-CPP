#include "occluder.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/vertex.h"
#include "../core/object3d_clone.h"
#include "mesh.h"


const Object3D::T *Occluder::TYPE = new Object3D::T("Occluder");


class Edge
{
public:
	std::shared_ptr<Edge> next;
	std::shared_ptr<Vertex> a;
	std::shared_ptr<Vertex> b;
	std::shared_ptr<Face> left;
	std::shared_ptr<Face> right;

	Edge()
	{

	}

	/*
	public function destroy():*
	{
		if (this.left != null)
		{
			this.left.destroy();
		}
		this.left = null;
		if (this.right != null)
		{
			this.right.destroy();
		}
		this.right = null;
		this.a = null;
		this.b = null;
	}
	*/
};



Occluder::Occluder(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	minSize = 0.;
}

void Occluder::createForm(std::shared_ptr<Mesh> param1, bool param2)
{
	this->destroyForm();

	if (param2 == false)
	{
		param1 = Mesh_clone(param1);
		//param1 = (param1->clone() as Mesh);
	}

	this->faceList = param1->faceList();
	this->vertexList = param1->vertexList;
	param1->setFaceList(nullptr, 8);
	param1->vertexList = nullptr;

	std::shared_ptr<Vertex> _loc3_ = this->vertexList;

	while (_loc3_ != nullptr)
	{
		_loc3_->transformId = 0;
		//_loc3_->id = nullptr;
		_loc3_ = _loc3_->next;
	}

	std::shared_ptr<Face> _loc4_ = this->faceList;
	while (_loc4_ != nullptr)
	{
		//_loc4_->id = nullptr;
		_loc4_ = _loc4_->next;
	}

	int status = this->calculateEdges();
	if (status != 0)
	{
		this->destroyForm();
		//throw (new ArgumentError(_loc5_));
	}

	calculateBounds();
}

void Occluder::destroyForm()
{
	this->faceList = nullptr;
	this->edgeList = nullptr;
	this->vertexList = nullptr;
}

void Occluder::clonePropertiesFromOccluder(Occluder *param1)
{
	std::shared_ptr<Vertex> vertex_temp;
	std::shared_ptr<Face> _loc6_;
	std::shared_ptr<Edge> _loc7_;


	Object3D::clonePropertiesFromObject3D(param1);
	//super.clonePropertiesFrom(param1);

	Occluder *_loc2_;

	if (param1->isType(Occluder::TYPE))
	{
		_loc2_ = (Occluder*)param1;
	}
	else
	{
		throw 34123;
	}

	this->minSize = _loc2_->minSize;
	std::shared_ptr<Vertex> _loc3_ = _loc2_->vertexList;
	while (_loc3_ != nullptr)
	{
		std::shared_ptr<Vertex> vertex = Vertex::create();
		vertex->x = _loc3_->x;
		vertex->y = _loc3_->y;
		vertex->z = _loc3_->z;
		vertex->u = _loc3_->u;
		vertex->v = _loc3_->v;
		vertex->normalX = _loc3_->normalX;
		vertex->normalY = _loc3_->normalY;
		vertex->normalZ = _loc3_->normalZ;
		_loc3_->value = vertex;
		if (vertex_temp != nullptr)
		{
			vertex_temp->next = vertex;
		}
		else
		{
			this->vertexList = vertex;
		}
		vertex_temp = vertex;
		_loc3_ = _loc3_->next;
	}

	std::shared_ptr<Face> _loc4_ = _loc2_->faceList;
	while (_loc4_ != nullptr)
	{
		std::shared_ptr<Face> _loc10_ = Face::create();
		_loc10_->material = _loc4_->material;
		_loc10_->normalX = _loc4_->normalX;
		_loc10_->normalY = _loc4_->normalY;
		_loc10_->normalZ = _loc4_->normalZ;
		_loc10_->offset = _loc4_->offset;
		_loc4_->processNext = _loc10_;
		std::shared_ptr<Wrapper> wrapper_temp = nullptr;
		Wrapper *wrapper_src = _loc4_->wrapper.get();
		while (wrapper_src != nullptr)
		{
			std::shared_ptr<Wrapper> wrapper = Wrapper::create(42);
			wrapper->vertex = wrapper_src->vertex->value;
			if (wrapper_temp != nullptr)
			{
				wrapper_temp->setNext(wrapper);
			}
			else
			{
				_loc10_->wrapper = wrapper;
			}
			wrapper_temp = wrapper;
			wrapper_src = wrapper_src->_next;
		}

		if (_loc6_ != nullptr)
		{
			_loc6_->next = _loc10_;
		}
		else
		{
			this->faceList = _loc10_;
		}

		_loc6_ = _loc10_;
		_loc4_ = _loc4_->next;
	}

	std::shared_ptr<Edge> _loc8_ = _loc2_->edgeList;
	while (_loc8_ != nullptr)
	{
		std::shared_ptr<Edge> _loc14_(new Edge());
		_loc14_->a = _loc8_->a->value;
		_loc14_->b = _loc8_->b->value;
		_loc14_->left = _loc8_->left->processNext;
		_loc14_->right = _loc8_->right->processNext;
		if (_loc7_ != nullptr)
		{
			_loc7_->next = _loc14_;
		}
		else
		{
			this->edgeList = _loc14_;
		}
		_loc7_ = _loc14_;
		_loc8_ = _loc8_->next;
	}
	_loc3_ = _loc2_->vertexList;
	while (_loc3_ != nullptr)
	{
		_loc3_->value = nullptr;
		_loc3_ = _loc3_->next;
	}
	_loc4_ = _loc2_->faceList;
	while (_loc4_ != nullptr)
	{
		_loc4_->processNext = nullptr;
		_loc4_ = _loc4_->next;
	}
}

int Occluder::calculateEdges()
{
#if 1

	std::shared_ptr<Face> _loc1_ = this->faceList;

	while (_loc1_ != nullptr)
	{
		_loc1_->calculateBestSequenceAndNormal();
		Wrapper *_loc2_ = _loc1_->wrapper.get();
_loop_1:
		for (;_loc2_ != nullptr;)
		{
			std::shared_ptr<Vertex> _loc4_ = _loc2_->vertex;
			std::shared_ptr<Vertex> _loc5_ = _loc2_->_next != nullptr ? _loc2_->_next->vertex : _loc1_->wrapper->vertex;
			std::shared_ptr<Edge> _loc3_ = this->edgeList;

			for (;true;)
			{
				if (_loc3_ != nullptr)
				{
					if (((_loc3_->a == _loc4_) && (_loc3_->b == _loc5_)))
					{
						break;
					}
					if ((!((_loc3_->a == _loc5_) && (_loc3_->b == _loc4_))))
					{
						_loc3_ = _loc3_->next;
						continue;
					}
				}

				if (_loc3_ != nullptr)
				{
					_loc3_->right = _loc1_;
				}
				else
				{
					_loc3_ = std::make_shared<Edge>();
					_loc3_->a = _loc4_;
					_loc3_->b = _loc5_;
					_loc3_->left = _loc1_;
					_loc3_->next = this->edgeList;
					this->edgeList = _loc3_;
				}

				_loc2_ = _loc2_->_next;
				_loc4_ = _loc5_;
				goto _loop_1;
			}
			//return ("The supplied geometry is not valid.");
			return 2;
		}
		_loc1_ = _loc1_->next;

	}

	std::shared_ptr<Edge> _loc3_ = this->edgeList;

	while (_loc3_ != nullptr)
	{
		if (((_loc3_->left == nullptr) || (_loc3_->right == nullptr)))
		{
			//return ("The supplied geometry is non whole.");
			return 1;
		}
		double _loc6_ = (_loc3_->b->x - _loc3_->a->x);
		double _loc7_ = (_loc3_->b->y - _loc3_->a->y);
		double _loc8_ = (_loc3_->b->z - _loc3_->a->z);
		double _loc9_ = ((_loc3_->right->normalZ * _loc3_->left->normalY) - (_loc3_->right->normalY * _loc3_->left->normalZ));
		double _loc10_ = ((_loc3_->right->normalX * _loc3_->left->normalZ) - (_loc3_->right->normalZ * _loc3_->left->normalX));
		double _loc11_ = ((_loc3_->right->normalY * _loc3_->left->normalX) - (_loc3_->right->normalX * _loc3_->left->normalY));
		if ((((_loc6_ * _loc9_) + (_loc7_ * _loc10_)) + (_loc8_ * _loc11_)) < 0)
		{
		}
		_loc3_ = _loc3_->next;
	}

#else

	std::shared_ptr<Face> _loc1_ = this->faceList;
	while(_loc1_ != nullptr)
	{
		_loc1_->calculateBestSequenceAndNormal();
		std::shared_ptr<Wrapper> _loc2_ = _loc1_->wrapper;
		while(_loc2_ != nullptr)
		{
			std::shared_ptr<Vertex> _loc4_ = _loc2_->vertex;
			std::shared_ptr<Vertex> _loc5_ = _loc2_->next != nullptr ? _loc2_->next->vertex : _loc1_->wrapper->vertex;
			std::shared_ptr<Edge> _loc3_ = this->edgeList;
			for (;;)
			{
				if (true)
				{
					if(_loc3_ == nullptr)
					{
						break;
					}
					if(!(_loc3_->a == _loc4_ && _loc3_->b == _loc5_))
					{
						continue;
					}
				}

				//return "The supplied geometry is not valid.";
				return 2;
			}

			if (_loc3_ != nullptr)
			{
				_loc3_->right = _loc1_;
			}
			else
			{
				_loc3_ = std::make_shared<Edge>();
				_loc3_->a = _loc4_;
				_loc3_->b = _loc5_;
				_loc3_->left = _loc1_;
				_loc3_->next = this->edgeList;
				this->edgeList = _loc3_;
			}

			_loc2_ = _loc2_->next;
			_loc4_ = _loc5_;

		}
		_loc1_ = _loc1_->next;
	}



	std::shared_ptr<Edge> _loc3_ = this->edgeList;
	while(_loc3_ != nullptr)
	{
		if(_loc3_->left == nullptr || _loc3_->right == nullptr)
		{
			//return "The supplied geometry is non whole.";
			return 1;
		}
		double _loc6_ = _loc3_->b->x - _loc3_->a->x;
		double _loc7_ = _loc3_->b->y - _loc3_->a->y;
		double _loc8_ = _loc3_->b->z - _loc3_->a->z;
		double _loc9_ = _loc3_->right->normalZ * _loc3_->left->normalY - _loc3_->right->normalY * _loc3_->left->normalZ;
		double _loc10_ = _loc3_->right->normalX * _loc3_->left->normalZ - _loc3_->right->normalZ * _loc3_->left->normalX;
		double _loc11_ = _loc3_->right->normalY * _loc3_->left->normalX - _loc3_->right->normalX * _loc3_->left->normalY;
		if(_loc6_ * _loc9_ + _loc7_ * _loc10_ + _loc8_ * _loc11_ < 0)
		{
		}
		_loc3_ = _loc3_->next;
	}

	#endif

	return 0;
}

//not used
void Occluder::draw(Camera3D *param1)
{
	qDebug("used Occluder::draw");
	(void)param1;
#if 0
	var _loc2_:int;
	var _loc3_:Sprite;
	var _loc6_:Vertex;
	var _loc11_:Vertex;
	var _loc12_:Vertex;
	var _loc13_:Number = NaN;
	var _loc14_:Number = NaN;
	var _loc15_:Number = NaN;
	var _loc16_:Number = NaN;
	var _loc17_:Number = NaN;
	var _loc18_:Number = NaN;
	var _loc19_:Number = NaN;
	var _loc21_:Vertex;
	var _loc22_:Vertex;
	var _loc23_:Number = NaN;
	if (((this->faceList == nullptr) || (this->edgeList == nullptr)))
	{
		return;
	}
	calculateInverseMatrix();
	var _loc4_:Boolean = true;
	var _loc5_:Face = this->faceList;
	while (_loc5_ != nullptr)
	{
		if ((((_loc5_->normalX * imd) + (_loc5_->normalY * imh)) + (_loc5_->normalZ * iml)) > _loc5_->offset)
		{
			_loc5_->distance = 1;
			_loc4_ = false;
		}
		else
		{
			_loc5_->distance = 0;
		}
		_loc5_ = _loc5_->next;
	}
	if (_loc4_)
	{
		return;
	}
	var _loc7_:int;
	var _loc8_:Boolean = true;
	var _loc9_:Number = param1->viewSizeX;
	var _loc10_:Number = param1->viewSizeY;
	var _loc20_:Edge = this->edgeList;
	for (;_loc20_ != null;(_loc20_ = _loc20_.next))
	{
		if (_loc20_.left.distance != _loc20_.right.distance)
		{
			if (_loc20_.left.distance > 0)
			{
				_loc11_ = _loc20_.a;
				_loc12_ = _loc20_.b;
			}
			else
			{
				_loc11_ = _loc20_.b;
				_loc12_ = _loc20_.a;
			}
			double _loc13_ = ((((ma * _loc11_->x) + (mb * _loc11_->y)) + (mc * _loc11_->z)) + md);
			_loc14_ = ((((me * _loc11_->x) + (mf * _loc11_->y)) + (mg * _loc11_->z)) + mh);
			_loc15_ = ((((mi * _loc11_->x) + (mj * _loc11_->y)) + (mk * _loc11_->z)) + ml);
			_loc16_ = ((((ma * _loc12_->x) + (mb * _loc12_->y)) + (mc * _loc12_->z)) + md);
			_loc17_ = ((((me * _loc12_->x) + (mf * _loc12_->y)) + (mg * _loc12_->z)) + mh);
			_loc18_ = ((((mi * _loc12_->x) + (mj * _loc12_->y)) + (mk * _loc12_->z)) + ml);
			if (culling > 0)
			{
				if (((_loc15_ <= -(_loc13_)) && (_loc18_ <= -(_loc16_))))
				{
					if (((_loc8_) && (((_loc17_ * _loc13_) - (_loc16_ * _loc14_)) > 0)))
					{
						_loc8_ = false;
					}
					continue;
				}
				if (((_loc18_ > -(_loc16_)) && (_loc15_ <= -(_loc13_))))
				{
					_loc19_ = ((_loc13_ + _loc15_) / (((_loc13_ + _loc15_) - _loc16_) - _loc18_));
					_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
					_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
					_loc15_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
				}
				else
				{
					if (((_loc18_ <= -(_loc16_)) && (_loc15_ > -(_loc13_))))
					{
						_loc19_ = ((_loc13_ + _loc15_) / (((_loc13_ + _loc15_) - _loc16_) - _loc18_));
						_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
						_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
						_loc18_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
					}
				}
				if (((_loc15_ <= _loc13_) && (_loc18_ <= _loc16_)))
				{
					if (((_loc8_) && (((_loc17_ * _loc13_) - (_loc16_ * _loc14_)) > 0)))
					{
						_loc8_ = false;
					}
					continue;
				}
				if (((_loc18_ > _loc16_) && (_loc15_ <= _loc13_)))
				{
					_loc19_ = ((_loc15_ - _loc13_) / (((_loc15_ - _loc13_) + _loc16_) - _loc18_));
					_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
					_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
					_loc15_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
				}
				else
				{
					if (((_loc18_ <= _loc16_) && (_loc15_ > _loc13_)))
					{
						_loc19_ = ((_loc15_ - _loc13_) / (((_loc15_ - _loc13_) + _loc16_) - _loc18_));
						_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
						_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
						_loc18_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
					}
				}
				if (((_loc15_ <= -(_loc14_)) && (_loc18_ <= -(_loc17_))))
				{
					if (((_loc8_) && (((_loc17_ * _loc13_) - (_loc16_ * _loc14_)) > 0)))
					{
						_loc8_ = false;
					}
					continue;
				}
				if (((_loc18_ > -(_loc17_)) && (_loc15_ <= -(_loc14_))))
				{
					_loc19_ = ((_loc14_ + _loc15_) / (((_loc14_ + _loc15_) - _loc17_) - _loc18_));
					_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
					_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
					_loc15_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
				}
				else
				{
					if (((_loc18_ <= -(_loc17_)) && (_loc15_ > -(_loc14_))))
					{
						_loc19_ = ((_loc14_ + _loc15_) / (((_loc14_ + _loc15_) - _loc17_) - _loc18_));
						_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
						_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
						_loc18_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
					}
				}
				if (((_loc15_ <= _loc14_) && (_loc18_ <= _loc17_)))
				{
					if (((_loc8_) && (((_loc17_ * _loc13_) - (_loc16_ * _loc14_)) > 0)))
					{
						_loc8_ = false;
					}
					continue;
				}
				if (((_loc18_ > _loc17_) && (_loc15_ <= _loc14_)))
				{
					_loc19_ = ((_loc15_ - _loc14_) / (((_loc15_ - _loc14_) + _loc17_) - _loc18_));
					_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
					_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
					_loc15_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
				}
				else
				{
					if (((_loc18_ <= _loc17_) && (_loc15_ > _loc14_)))
					{
						_loc19_ = ((_loc15_ - _loc14_) / (((_loc15_ - _loc14_) + _loc17_) - _loc18_));
						_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
						_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
						_loc18_ = (_loc15_ + ((_loc18_ - _loc15_) * _loc19_));
					}
				}
				_loc8_ = false;
			}
			_loc11_ = _loc11_->create();
			_loc11_->next = _loc6_;
			_loc7_++;
			_loc6_ = _loc11_;
			_loc6_->cameraX = ((_loc18_ * _loc14_) - (_loc17_ * _loc15_));
			_loc6_->cameraY = ((_loc16_ * _loc15_) - (_loc18_ * _loc13_));
			_loc6_->cameraZ = ((_loc17_ * _loc13_) - (_loc16_ * _loc14_));
			_loc6_->x = _loc13_;
			_loc6_->y = _loc14_;
			_loc6_->z = _loc15_;
			_loc6_->u = _loc16_;
			_loc6_->v = _loc17_;
			_loc6_->offset = _loc18_;
		}
	}
	if (_loc6_ != nullptr)
	{
		if (this->minSize > 0)
		{
			_loc21_ = Vertex.createList(_loc7_);
			_loc11_ = _loc6_;
			_loc12_ = _loc21_;
			while (_loc11_ != nullptr)
			{
				_loc12_->x = ((_loc11_->x * _loc9_) / _loc11_->z);
				_loc12_->y = ((_loc11_->y * _loc10_) / _loc11_->z);
				_loc12_->u = ((_loc11_->u * _loc9_) / _loc11_->offset);
				_loc12_->v = ((_loc11_->v * _loc10_) / _loc11_->offset);
				_loc12_->cameraX = (_loc12_->y - _loc12_->v);
				_loc12_->cameraY = (_loc12_->u - _loc12_->x);
				_loc12_->offset = ((_loc12_->cameraX * _loc12_->x) + (_loc12_->cameraY * _loc12_->y));
				_loc11_ = _loc11_->next;
				_loc12_ = _loc12_->next;
			}
			if (culling > 0)
			{
				if ((culling & 0x04))
				{
					_loc13_ = -(param1->viewSizeX);
					_loc14_ = -(param1->viewSizeY);
					_loc16_ = -(param1->viewSizeX);
					_loc17_ = param1->viewSizeY;
					_loc11_ = _loc21_;
					while (_loc11_ != nullptr)
					{
						_loc15_ = (((_loc13_ * _loc11_->cameraX) + (_loc14_ * _loc11_->cameraY)) - _loc11_->offset);
						_loc18_ = (((_loc16_ * _loc11_->cameraX) + (_loc17_ * _loc11_->cameraY)) - _loc11_->offset);
						if (((_loc15_ < 0) || (_loc18_ < 0)))
						{
							if (((_loc15_ >= 0) && (_loc18_ < 0)))
							{
								_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
								_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
								_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
							}
							else
							{
								if (((_loc15_ < 0) && (_loc18_ >= 0)))
								{
									_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
									_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
									_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
								}
							}
							_loc11_ = _loc11_->next;
						}
						else
						{
							break;
						}
					}
					if (_loc11_ == nullptr)
					{
						_loc12_ = _loc6_->create();
						_loc12_->next = _loc22_;
						_loc22_ = _loc12_;
						_loc22_.x = _loc13_;
						_loc22_.y = _loc14_;
						_loc22_.u = _loc16_;
						_loc22_.v = _loc17_;
					}
				}
				if ((culling & 0x08))
				{
					_loc13_ = param1->viewSizeX;
					_loc14_ = param1->viewSizeY;
					_loc16_ = param1->viewSizeX;
					_loc17_ = -(param1->viewSizeY);
					_loc11_ = _loc21_;
					while (_loc11_ != nullptr)
					{
						_loc15_ = (((_loc13_ * _loc11_->cameraX) + (_loc14_ * _loc11_->cameraY)) - _loc11_->offset);
						_loc18_ = (((_loc16_ * _loc11_->cameraX) + (_loc17_ * _loc11_->cameraY)) - _loc11_->offset);
						if (((_loc15_ < 0) || (_loc18_ < 0)))
						{
							if (((_loc15_ >= 0) && (_loc18_ < 0)))
							{
								_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
								_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
								_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
							}
							else
							{
								if (((_loc15_ < 0) && (_loc18_ >= 0)))
								{
									_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
									_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
									_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
								}
							}
							_loc11_ = _loc11_->next;
						}
						else
						{
							break;
						}
					}
					if (_loc11_ == nullptr)
					{
						_loc12_ = _loc6_->create();
						_loc12_->next = _loc22_;
						_loc22_ = _loc12_;
						_loc22_.x = _loc13_;
						_loc22_.y = _loc14_;
						_loc22_.u = _loc16_;
						_loc22_.v = _loc17_;
					}
				}
				if ((culling & 0x10))
				{
					_loc13_ = param1->viewSizeX;
					_loc14_ = -(param1->viewSizeY);
					_loc16_ = -(param1->viewSizeX);
					_loc17_ = -(param1->viewSizeY);
					_loc11_ = _loc21_;
					while (_loc11_ != nullptr)
					{
						_loc15_ = (((_loc13_ * _loc11_->cameraX) + (_loc14_ * _loc11_->cameraY)) - _loc11_->offset);
						_loc18_ = (((_loc16_ * _loc11_->cameraX) + (_loc17_ * _loc11_->cameraY)) - _loc11_->offset);
						if (((_loc15_ < 0) || (_loc18_ < 0)))
						{
							if (((_loc15_ >= 0) && (_loc18_ < 0)))
							{
								_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
								_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
								_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
							}
							else
							{
								if (((_loc15_ < 0) && (_loc18_ >= 0)))
								{
									_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
									_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
									_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
								}
							}
							_loc11_ = _loc11_->next;
						}
						else
						{
							break;
						}
					}
					if (_loc11_ == nullptr)
					{
						_loc12_ = _loc6_->create();
						_loc12_->next = _loc22_;
						_loc22_ = _loc12_;
						_loc22_.x = _loc13_;
						_loc22_.y = _loc14_;
						_loc22_.u = _loc16_;
						_loc22_.v = _loc17_;
					}
				}
				if ((culling & 0x20))
				{
					_loc13_ = -(param1->viewSizeX);
					_loc14_ = param1->viewSizeY;
					_loc16_ = param1->viewSizeX;
					_loc17_ = param1->viewSizeY;
					_loc11_ = _loc21_;
					while (_loc11_ != nullptr)
					{
						_loc15_ = (((_loc13_ * _loc11_->cameraX) + (_loc14_ * _loc11_->cameraY)) - _loc11_->offset);
						_loc18_ = (((_loc16_ * _loc11_->cameraX) + (_loc17_ * _loc11_->cameraY)) - _loc11_->offset);
						if (((_loc15_ < 0) || (_loc18_ < 0)))
						{
							if (((_loc15_ >= 0) && (_loc18_ < 0)))
							{
								_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
								_loc13_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
								_loc14_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
							}
							else
							{
								if (((_loc15_ < 0) && (_loc18_ >= 0)))
								{
									_loc19_ = (_loc15_ / (_loc15_ - _loc18_));
									_loc16_ = (_loc13_ + ((_loc16_ - _loc13_) * _loc19_));
									_loc17_ = (_loc14_ + ((_loc17_ - _loc14_) * _loc19_));
								}
							}
							_loc11_ = _loc11_->next;
						}
						else
						{
							break;
						}
					}
					if (_loc11_ == nullptr)
					{
						_loc12_ = _loc6_->create();
						_loc12_->next = _loc22_;
						_loc22_ = _loc12_;
						_loc22_.x = _loc13_;
						_loc22_.y = _loc14_;
						_loc22_.u = _loc16_;
						_loc22_.v = _loc17_;
					}
				}
			}
			_loc23_ = 0;
			_loc15_ = _loc21_.x;
			_loc18_ = _loc21_.y;
			_loc11_ = _loc21_;
			while (_loc11_->next != nullptr)
			{
				_loc11_ = _loc11_->next;
			}
			_loc11_->next = _loc22_;
			_loc11_ = _loc21_;
			while (_loc11_ != nullptr)
			{
				_loc23_ = (_loc23_ + (((_loc11_->u - _loc15_) * (_loc11_->y - _loc18_)) - ((_loc11_->v - _loc18_) * (_loc11_->x - _loc15_))));
				if (_loc11_->next == nullptr)
				{
					break;
				}
				_loc11_ = _loc11_->next;
			}
			_loc11_->next = Vertex.collector;
			Vertex.collector = _loc21_;
			if ((_loc23_ / ((param1->viewSizeX * param1->viewSizeY) * 8)) < this->minSize)
			{
				_loc11_ = _loc6_;
				while (_loc11_->next != nullptr)
				{
					_loc11_ = _loc11_->next;
				}
				_loc11_->next = Vertex.collector;
				Vertex.collector = _loc6_;
				return;
			}
		}
		if (((param1->debug) && ((_loc2_ = param1->checkInDebug(this)) > 0)))
		{
			if ((_loc2_ & Debug.EDGES))
			{
				_loc11_ = _loc6_;
				while (_loc11_ != nullptr)
				{
					_loc13_ = ((_loc11_->x * _loc9_) / _loc11_->z);
					_loc14_ = ((_loc11_->y * _loc10_) / _loc11_->z);
					_loc16_ = ((_loc11_->u * _loc9_) / _loc11_->offset);
					_loc17_ = ((_loc11_->v * _loc10_) / _loc11_->offset);
					_loc3_ = param1->view.canvas;
					_loc3_->graphics.moveTo(_loc13_, _loc14_);
					_loc3_->graphics.lineStyle(3, 0xFF);
					_loc3_->graphics.lineTo((_loc13_ + ((_loc16_ - _loc13_) * 0.8)), (_loc14_ + ((_loc17_ - _loc14_) * 0.8)));
					_loc3_->graphics.lineStyle(3, 0xFF0000);
					_loc3_->graphics.lineTo(_loc16_, _loc17_);
					_loc11_ = _loc11_->next;
				}
			}
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(param1, this, boundMin.x, boundMin.y, boundMin.z, boundMaxX, boundMaxY, boundMaxZ);
			}
		}
		param1->occluders[param1->numOccluders] = _loc6_;
		param1->numOccluders++;
	}
	else
	{
		if (_loc8_)
		{
			if (((param1->debug) && ((_loc2_ = param1->checkInDebug(this)) > 0)))
			{
				if ((_loc2_ & Debug.EDGES))
				{
					_loc19_ = 1.5;
					_loc3_ = param1->view.canvas;
					_loc3_->graphics.moveTo((-(_loc9_) + _loc19_), (-(_loc10_) + _loc19_));
					_loc3_->graphics.lineStyle(3, 0xFF);
					_loc3_->graphics.lineTo((-(_loc9_) + _loc19_), (_loc10_ * 0.6));
					_loc3_->graphics.lineStyle(3, 0xFF0000);
					_loc3_->graphics.lineTo((-(_loc9_) + _loc19_), (_loc10_ - _loc19_));
					_loc3_->graphics.lineStyle(3, 0xFF);
					_loc3_->graphics.lineTo((_loc9_ * 0.6), (_loc10_ - _loc19_));
					_loc3_->graphics.lineStyle(3, 0xFF0000);
					_loc3_->graphics.lineTo((_loc9_ - _loc19_), (_loc10_ - _loc19_));
					_loc3_->graphics.lineStyle(3, 0xFF);
					_loc3_->graphics.lineTo((_loc9_ - _loc19_), (-(_loc10_) * 0.6));
					_loc3_->graphics.lineStyle(3, 0xFF0000);
					_loc3_->graphics.lineTo((_loc9_ - _loc19_), (-(_loc10_) + _loc19_));
					_loc3_->graphics.lineStyle(3, 0xFF);
					_loc3_->graphics.lineTo((-(_loc9_) * 0.6), (-(_loc10_) + _loc19_));
					_loc3_->graphics.lineStyle(3, 0xFF0000);
					_loc3_->graphics.lineTo((-(_loc9_) + _loc19_), (-(_loc10_) + _loc19_));
				}
				if ((_loc2_ & Debug.BOUNDS))
				{
					Debug.drawBounds(param1, this, boundMin.x, boundMin.y, boundMin.z, boundMaxX, boundMaxY, boundMaxZ);
				}
			}
			param1->clearOccluders();
			param1->occludedAll = true;
		}
	}
#endif
}

void Occluder::updateBounds(Object3D *param1, Object3D *param2)
{
	Vertex *_loc3_ = this->vertexList.get();
	while (_loc3_ != nullptr)
	{
		if (param2 != nullptr)
		{
			_loc3_->cameraX = ((((param2->matrix.ma * _loc3_->x) + (param2->matrix.mb * _loc3_->y)) + (param2->matrix.mc * _loc3_->z)) + param2->matrix.md);
			_loc3_->cameraY = ((((param2->matrix.me * _loc3_->x) + (param2->matrix.mf * _loc3_->y)) + (param2->matrix.mg * _loc3_->z)) + param2->matrix.mh);
			_loc3_->cameraZ = ((((param2->matrix.mi * _loc3_->x) + (param2->matrix.mj * _loc3_->y)) + (param2->matrix.mk * _loc3_->z)) + param2->matrix.ml);
		}
		else
		{
			_loc3_->cameraX = _loc3_->x;
			_loc3_->cameraY = _loc3_->y;
			_loc3_->cameraZ = _loc3_->z;
		}
		if (_loc3_->cameraX < param1->bound.MinX)
		{
			param1->bound.MinX = _loc3_->cameraX;
		}
		if (_loc3_->cameraX > param1->bound.MaxX)
		{
			param1->bound.MaxX = _loc3_->cameraX;
		}
		if (_loc3_->cameraY < param1->bound.MinY)
		{
			param1->bound.MinY = _loc3_->cameraY;
		}
		if (_loc3_->cameraY > param1->bound.MaxY)
		{
			param1->bound.MaxY = _loc3_->cameraY;
		}
		if (_loc3_->cameraZ < param1->bound.MinZ)
		{
			param1->bound.MinZ = _loc3_->cameraZ;
		}
		if (_loc3_->cameraZ > param1->bound.MaxZ)
		{
			param1->bound.MaxZ = _loc3_->cameraZ;
		}
		_loc3_ = _loc3_->next.get();
	}
}

void Occluder::destory()
{
#if 0
	var buf:Vertex;
	var buf1:Face;
	var buf2:Edge;
	if (this->vertexList != nullptr)
	{
		while (this->vertexList.next != nullptr)
		{
			buf = this->vertexList;
			this->vertexList = null;
			this->vertexList = buf.next;
		}
		this->vertexList = nullptr;
	}

	if (this->faceList != nullptr)
	{
		while (this->faceList.next != nullptr)
		{
			buf1 = this->faceList;
			this->faceList.destroy();
			this->faceList = null;
			this->faceList = buf1.next;
		}
		this->faceList = nullptr;
	}

	if (this->edgeList != nullptr)
	{
		while (this->edgeList.next != nullptr)
		{
			buf2 = this->edgeList;
			this->edgeList.destroy();
			this->edgeList = nullptr;
			this->edgeList = buf2.next;
		}
		this->edgeList = null;
	}

	captureListeners = nullptr;
	bubbleListeners = nullptr;
#endif
}

std::shared_ptr<Occluder> new_Occluder()
{
	return std::make_shared<Occluder>(Object3D::TT(Occluder::TYPE));
}

std::shared_ptr<Occluder> Occluder_clone(const std::shared_ptr<Occluder> &in)
{
	if (in->isType(Occluder::TYPE)) {
		auto out = new_Occluder();
		out->clonePropertiesFromOccluder(in.get());
		return out;
	}

	throw 4113244;
}

