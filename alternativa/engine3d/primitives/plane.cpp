#include "plane.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/wrapper.h"
#include <QHash>


const Object3D::T *Plane::TYPE = new Object3D::T("Plane");


std::shared_ptr<Vertex> Plane::createVertex(double x, double y, double z, double u, double v)
{
	std::shared_ptr<Vertex> vertex = Vertex::create();
	vertex->x = x;
	vertex->y = y;
	vertex->z = z;
	vertex->u = u;
	vertex->v = v;
	vertex->next = vertexList;
	vertexList = vertex;
	return vertex;
}


void Plane::createFace(std::shared_ptr<Vertex> param1,
					   std::shared_ptr<Vertex> param2,
					   std::shared_ptr<Vertex> param3,
					   std::shared_ptr<Vertex> param4,
					   double param5,
					   double param6,
					   double param7,
					   double param8,
					   bool param9,
					   bool param10,
					   std::shared_ptr<Material> param11)
{

	if (param9)
	{
		param5 = -param5;
		param6 = -param6;
		param7 = -param7;
		param8 = -param8;
		std::shared_ptr<Vertex> _loc12_ = param1;
		param1 = param4;
		param4 = _loc12_;
		_loc12_ = param2;
		param2 = param3;
		param3 = _loc12_;
	};

	if (param10)
	{
		std::shared_ptr<Face> _loc13_ = Face::create();
		_loc13_->material = param11;
		_loc13_->wrapper = Wrapper::create(47);
		_loc13_->wrapper->vertex = param1;
		_loc13_->wrapper->setNext(Wrapper::create(48));
		_loc13_->wrapper->_next->vertex = param2;
		_loc13_->wrapper->_next->setNext(Wrapper::create(49));
		_loc13_->wrapper->_next->_next->vertex = param3;
		_loc13_->normalX = param5;
		_loc13_->normalY = param6;
		_loc13_->normalZ = param7;
		_loc13_->offset = param8;
		_loc13_->next = faceList();
		setFaceList(_loc13_, 30);
		_loc13_ = Face::create();
		_loc13_->material = param11;
		_loc13_->wrapper = Wrapper::create(50);
		_loc13_->wrapper->vertex = param1;
		_loc13_->wrapper->setNext(Wrapper::create(51));
		_loc13_->wrapper->_next->vertex = param3;
		_loc13_->wrapper->_next->setNext(Wrapper::create(52));
		_loc13_->wrapper->_next->_next->vertex = param4;
		_loc13_->normalX = param5;
		_loc13_->normalY = param6;
		_loc13_->normalZ = param7;
		_loc13_->offset = param8;
		_loc13_->next = faceList();
		setFaceList(_loc13_, 31);
	}
	else
	{
		std::shared_ptr<Face> _loc13_ = Face::create();
		_loc13_->material = param11;
		_loc13_->wrapper = Wrapper::create(53);
		_loc13_->wrapper->vertex = param1;
		_loc13_->wrapper->setNext(Wrapper::create(54));
		_loc13_->wrapper->_next->vertex = param2;
		_loc13_->wrapper->_next->setNext(Wrapper::create(55));
		_loc13_->wrapper->_next->_next->vertex = param3;
		_loc13_->wrapper->_next->_next->setNext(Wrapper::create(56));
		_loc13_->wrapper->_next->_next->_next->vertex = param4;
		_loc13_->normalX = param5;
		_loc13_->normalY = param6;
		_loc13_->normalZ = param7;
		_loc13_->offset = param8;
		_loc13_->next = faceList();
		setFaceList(_loc13_, 32);
	}
}


Plane::Plane(const TT &t,
			 double param1,
			 double param2,
			 uint32_t param3,
			 uint32_t param4,
			 bool param5,
			 bool param6,
			 bool param7,
			 std::shared_ptr<Material> param8,
			 std::shared_ptr<Material> param9) :
	Mesh(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	/*
	///super();
	if (param3 < 1)
	{
		throw (new ArgumentError((param3 + " width segments not enough.")));
	}
	if (param4 < 1)
	{
		throw (new ArgumentError((param4 + " length segments not enough.")));
	}
	*/

	uint32_t _loc13_ = (param3 + 1);
	uint32_t _loc14_ = (param4 + 1);
	double _loc15_ = (param1 * 0.5);
	double _loc16_ = (param2 * 0.5);
	double _loc17_ = (1. / param3);
	double _loc18_ = (1. / param4);
	double _loc19_ = (param1 / param3);
	double _loc20_ = (param2 / param4);
	//QVector<std::shared_ptr<Vertex>> _loc21_ = new QVector<std::shared_ptr<Vertex>>();
	QHash<int, std::shared_ptr<Vertex>> _loc21_;

	int _loc22_ = 0;
	uint32_t _loc10_ = 0;
	while (_loc10_ < _loc13_)
	{
		uint32_t _loc11_ = 0;
		while (_loc11_ < _loc14_)
		{
			int index = _loc22_++;
			_loc21_[index] = this->createVertex(((_loc10_ * _loc19_) - _loc15_), ((_loc11_ * _loc20_) - _loc16_), 0, (_loc10_ * _loc17_), ((param4 - _loc11_) * _loc18_));
			_loc11_++;
		}
		_loc10_++;
	}

	_loc10_ = 0;
	while (_loc10_ < _loc13_)
	{
		uint32_t _loc11_ = 0;
		while (_loc11_ < _loc14_)
		{
			if (((_loc10_ < param3) && (_loc11_ < param4)))
			{
				this->createFace(_loc21_[((_loc10_ * _loc14_) + _loc11_)], _loc21_[(((_loc10_ + 1) * _loc14_) + _loc11_)], _loc21_[((((_loc10_ + 1) * _loc14_) + _loc11_) + 1)], _loc21_[(((_loc10_ * _loc14_) + _loc11_) + 1)], 0, 0, 1, 0, param6, param7, param9);
			}
			_loc11_++;
		}
		_loc10_++;
	}

	if (param5)
	{
		_loc22_ = 0;
		_loc10_ = 0;
		while (_loc10_ < _loc13_)
		{
			uint32_t _loc11_ = 0;
			while (_loc11_ < _loc14_)
			{
				int index = _loc22_++;
				_loc21_[index] = this->createVertex(((_loc10_ * _loc19_) - _loc15_), ((_loc11_ * _loc20_) - _loc16_), 0, ((param3 - _loc10_) * _loc17_), ((param4 - _loc11_) * _loc18_));
				_loc11_++;
			}
			_loc10_++;
		}
		_loc10_ = 0;
		while (_loc10_ < _loc13_)
		{
			uint32_t _loc11_ = 0;
			while (_loc11_ < _loc14_)
			{
				if (((_loc10_ < param3) && (_loc11_ < param4)))
				{
					this->createFace(_loc21_[((((_loc10_ + 1) * _loc14_) + _loc11_) + 1)], _loc21_[(((_loc10_ + 1) * _loc14_) + _loc11_)], _loc21_[((_loc10_ * _loc14_) + _loc11_)], _loc21_[(((_loc10_ * _loc14_) + _loc11_) + 1)], 0, 0, -1, 0, param6, param7, param8);
				}
				_loc11_++;
			}
			_loc10_++;
		}
	}

	bound.MinX = -_loc15_;
	bound.MinY = -_loc16_;
	bound.MinZ = 0;
	bound.MaxX = _loc15_;
	bound.MaxY = _loc16_;
	bound.MaxZ = 0;
}


std::shared_ptr<Plane> new_Plane(double param1,
								 double param2,
								 uint32_t param3,
								 uint32_t param4,
								 bool param5,
								 bool param6,
								 bool param7,
								 std::shared_ptr<Material> param8,
								 std::shared_ptr<Material> param9)
{
	return std::make_shared<Plane>(Object3D::TT(Plane::TYPE), param1, param2, param3, param4, param5, param6, param7, param8, param9);
}

