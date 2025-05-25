#include "face.h"
#include "wrapper.h"
#include "_global.h"
#include "vertex.h"
#include "mymath.h"
#include "../materials/material.h"
#include "flash/geom/point.h"
#include "alternativa/math/vector3.h"
#include <QVector>


std::shared_ptr<Face> Face::collector;


Face::Face()
{
	_function_name("Face::Face");

	smoothingGroups = 0;
	wrapper = nullptr;
	next = nullptr;
	processNext = nullptr;
	processNegative = nullptr;
	processPositive = nullptr;
	material = nullptr;
}

Face::~Face()
{

}

void Face::destroy()
{
	_function_name("Face::destroy");

	while (this->next != nullptr)
	{
		std::shared_ptr<Face> temp = this->next;
		this->next = nullptr;
		this->next = temp->next;
	}

	while (this->wrapper != nullptr)
	{
		std::shared_ptr<Wrapper> temp = this->wrapper;
		this->wrapper = nullptr;
		this->wrapper = temp->next();
	}

	while (material != nullptr)
	{
		material = nullptr;
	}

	while (this->processNext != nullptr)
	{
		std::shared_ptr<Face> temp = this->processNext;
		this->processNext = nullptr;
		this->processNext = temp->next;
	}

	while (this->processNegative != nullptr)
	{
		std::shared_ptr<Face> temp = this->processNegative;
		this->processNegative = nullptr;
		this->processNegative = temp->next;
	}

	while (this->processPositive != nullptr)
	{
		std::shared_ptr<Face> temp = this->processPositive;
		this->processPositive = nullptr;
		this->processPositive = temp->next;
	}
}

void Face::getUV(Vector3 *in, Point *result)
{
	Vertex *_loc2_ = this->wrapper->vertex.get();
	Vertex *_loc3_ = this->wrapper->_next->vertex.get();
	Vertex *_loc4_ = this->wrapper->_next->_next->vertex.get();
	double _loc5_ = (_loc3_->x - _loc2_->x);
	double _loc6_ = (_loc3_->y - _loc2_->y);
	double _loc7_ = (_loc3_->z - _loc2_->z);
	double _loc8_ = (_loc3_->u - _loc2_->u);
	double _loc9_ = (_loc3_->v - _loc2_->v);
	double _loc10_ = (_loc4_->x - _loc2_->x);
	double _loc11_ = (_loc4_->y - _loc2_->y);
	double _loc12_ = (_loc4_->z - _loc2_->z);
	double _loc13_ = (_loc4_->u - _loc2_->u);
	double _loc14_ = (_loc4_->v - _loc2_->v);
	double _loc15_ = (((((((-(this->normalX) * _loc11_) * _loc7_) + ((_loc10_ * this->normalY) * _loc7_)) + ((this->normalX * _loc6_) * _loc12_)) - ((_loc5_ * this->normalY) * _loc12_)) - ((_loc10_ * _loc6_) * this->normalZ)) + ((_loc5_ * _loc11_) * this->normalZ));
	double _loc16_ = (((-(this->normalY) * _loc12_) + (_loc11_ * this->normalZ)) / _loc15_);
	double _loc17_ = (((this->normalX * _loc12_) - (_loc10_ * this->normalZ)) / _loc15_);
	double _loc18_ = (((-(this->normalX) * _loc11_) + (_loc10_ * this->normalY)) / _loc15_);
	double _loc19_ = ((((((((_loc2_->x * this->normalY) * _loc12_) - ((this->normalX * _loc2_->y) * _loc12_)) - ((_loc2_->x * _loc11_) * this->normalZ)) + ((_loc10_ * _loc2_->y) * this->normalZ)) + ((this->normalX * _loc11_) * _loc2_->z)) - ((_loc10_ * this->normalY) * _loc2_->z)) / _loc15_);
	double _loc20_ = (((this->normalY * _loc7_) - (_loc6_ * this->normalZ)) / _loc15_);
	double _loc21_ = (((-(this->normalX) * _loc7_) + (_loc5_ * this->normalZ)) / _loc15_);
	double _loc22_ = (((this->normalX * _loc6_) - (_loc5_ * this->normalY)) / _loc15_);
	double _loc23_ = ((((((((this->normalX * _loc2_->y) * _loc7_) - ((_loc2_->x * this->normalY) * _loc7_)) + ((_loc2_->x * _loc6_) * this->normalZ)) - ((_loc5_ * _loc2_->y) * this->normalZ)) - ((this->normalX * _loc6_) * _loc2_->z)) + ((_loc5_ * this->normalY) * _loc2_->z)) / _loc15_);
	double _loc24_ = ((_loc8_ * _loc16_) + (_loc13_ * _loc20_));
	double _loc25_ = ((_loc8_ * _loc17_) + (_loc13_ * _loc21_));
	double _loc26_ = ((_loc8_ * _loc18_) + (_loc13_ * _loc22_));
	double _loc27_ = (((_loc8_ * _loc19_) + (_loc13_ * _loc23_)) + _loc2_->u);
	double _loc28_ = ((_loc9_ * _loc16_) + (_loc14_ * _loc20_));
	double _loc29_ = ((_loc9_ * _loc17_) + (_loc14_ * _loc21_));
	double _loc30_ = ((_loc9_ * _loc18_) + (_loc14_ * _loc22_));
	double _loc31_ = (((_loc9_ * _loc19_) + (_loc14_ * _loc23_)) + _loc2_->v);

	result->x = (_loc24_ * in->x) + (_loc25_ * in->y) + (_loc26_ * in->z) + _loc27_;
	result->y = (_loc28_ * in->x) + (_loc29_ * in->y) + (_loc30_ * in->z) + _loc31_;
}


std::shared_ptr<Face> Face::create()
{
	_function_name("Face::create");

	if (collector != nullptr)
	{
		std::shared_ptr<Face> face = collector;
		collector = face->next;
		face->next = nullptr;
		return face;
	}

	auto p = std::make_shared<Face>();
	return p;
}

#if 0


Vector3D *Face::get_normal()
{
	function_name("Face::get_normal");
	Wrapper *_loc1_ = this->wrapper;
	Vertex *_loc2_ = _loc1_->vertex;
	_loc1_ = _loc1_->next;
	Vertex *_loc3_ = _loc1_->vertex;
	_loc1_ = _loc1_->next;
	Vertex *_loc4_ = _loc1_->vertex;
	double _loc5_ = (_loc3_->x - _loc2_->x);
	double _loc6_ = (_loc3_->y - _loc2_->y);
	double _loc7_ = (_loc3_->z - _loc2_->z);
	double _loc8_ = (_loc4_->x - _loc2_->x);
	double _loc9_ = (_loc4_->y - _loc2_->y);
	double _loc10_ = (_loc4_->z - _loc2_->z);
	double _loc11_ = ((_loc10_ * _loc6_) - (_loc9_ * _loc7_));
	double _loc12_ = ((_loc8_ * _loc7_) - (_loc10_ * _loc5_));
	double _loc13_ = ((_loc9_ * _loc5_) - (_loc8_ * _loc6_));
	double _loc14_ = (((_loc11_ * _loc11_) + (_loc12_ * _loc12_)) + (_loc13_ * _loc13_));
	if (_loc14_ > 0.001)
	{
		_loc14_ = (1. / MyMath::sqrt(_loc14_));
		_loc11_ = (_loc11_ * _loc14_);
		_loc12_ = (_loc12_ * _loc14_);
		_loc13_ = (_loc13_ * _loc14_);
	}
	return new Vector3(_loc11_, _loc12_, _loc13_, (((_loc2_->x * _loc11_) + (_loc2_->y * _loc12_)) + (_loc2_->z * _loc13_)));
}
#endif

QVector<std::shared_ptr<Vertex>>* Face::getVertices()
{
	_function_name("Face::get_vertices");
	auto list = new QVector<std::shared_ptr<Vertex>>;
	Wrapper *wrapper = this->wrapper.get();
	while (wrapper != nullptr)
	{
		list->append(wrapper->vertex);
		wrapper = wrapper->_next;
	}
	return list;
}

#if 0
Point *Face::getUV(Vector3 *param1)
{
	function_name("Face::Face");
	Vertex* _loc2_ = this->wrapper->vertex;
	Vertex* _loc3_ = this->wrapper->next->vertex;
	Vertex* _loc4_ = this->wrapper->next->next->vertex;
	double _loc5_ = (_loc3_->x - _loc2_->x);
	double _loc6_ = (_loc3_->y - _loc2_->y);
	double _loc7_ = (_loc3_->z - _loc2_->z);
	double _loc8_ = (_loc3_->u - _loc2_->u);
	double _loc9_ = (_loc3_->v - _loc2_->v);
	double _loc10_ = (_loc4_->x - _loc2_->x);
	double _loc11_ = (_loc4_->y - _loc2_->y);
	double _loc12_ = (_loc4_->z - _loc2_->z);
	double _loc13_ = (_loc4_->u - _loc2_->u);
	double _loc14_ = (_loc4_->v - _loc2_->v);
	double _loc15_ = (((((((-(this->normalX) * _loc11_) * _loc7_) + ((_loc10_ * this->normalY) * _loc7_)) + ((this->normalX * _loc6_) * _loc12_)) - ((_loc5_ * this->normalY) * _loc12_)) - ((_loc10_ * _loc6_) * this->normalZ)) + ((_loc5_ * _loc11_) * this->normalZ));
	double _loc16_ = (((-(this->normalY) * _loc12_) + (_loc11_ * this->normalZ)) / _loc15_);
	double _loc17_ = (((this->normalX * _loc12_) - (_loc10_ * this->normalZ)) / _loc15_);
	double _loc18_ = (((-(this->normalX) * _loc11_) + (_loc10_ * this->normalY)) / _loc15_);
	double _loc19_ = ((((((((_loc2_->x * this->normalY) * _loc12_) - ((this->normalX * _loc2_->y) * _loc12_)) - ((_loc2_->x * _loc11_) * this->normalZ)) + ((_loc10_ * _loc2_->y) * this->normalZ)) + ((this->normalX * _loc11_) * _loc2_->z)) - ((_loc10_ * this->normalY) * _loc2_->z)) / _loc15_);
	double _loc20_ = (((this->normalY * _loc7_) - (_loc6_ * this->normalZ)) / _loc15_);
	double _loc21_ = (((-(this->normalX) * _loc7_) + (_loc5_ * this->normalZ)) / _loc15_);
	double _loc22_ = (((this->normalX * _loc6_) - (_loc5_ * this->normalY)) / _loc15_);
	double _loc23_ = ((((((((this->normalX * _loc2_->y) * _loc7_) - ((_loc2_->x * this->normalY) * _loc7_)) + ((_loc2_->x * _loc6_) * this->normalZ)) - ((_loc5_ * _loc2_->y) * this->normalZ)) - ((this->normalX * _loc6_) * _loc2_->z)) + ((_loc5_ * this->normalY) * _loc2_->z)) / _loc15_);
	double _loc24_ = ((_loc8_ * _loc16_) + (_loc13_ * _loc20_));
	double _loc25_ = ((_loc8_ * _loc17_) + (_loc13_ * _loc21_));
	double _loc26_ = ((_loc8_ * _loc18_) + (_loc13_ * _loc22_));
	double _loc27_ = (((_loc8_ * _loc19_) + (_loc13_ * _loc23_)) + _loc2_->u);
	double _loc28_ = ((_loc9_ * _loc16_) + (_loc14_ * _loc20_));
	double _loc29_ = ((_loc9_ * _loc17_) + (_loc14_ * _loc21_));
	double _loc30_ = ((_loc9_ * _loc18_) + (_loc14_ * _loc22_));
	double _loc31_ = (((_loc9_ * _loc19_) + (_loc14_ * _loc23_)) + _loc2_->v);
	return (new Point(((((_loc24_ * param1->x) + (_loc25_ * param1->y)) + (_loc26_ * param1->z)) + _loc27_), ((((_loc28_ * param1->x) + (_loc29_ * param1->y)) + (_loc30_ * param1->z)) + _loc31_)));
}
#endif


void Face::calculateBestSequenceAndNormal()
{
	_function_name("Face::calculateBestSequenceAndNormal");

	std::shared_ptr<Wrapper> _loc16_;
	//Wrapper* _loc16_ = nullptr;
	if (this->wrapper->_next->_next->_next != nullptr)
	{
		double _loc15_ = -1E22;
		std::shared_ptr<Wrapper> _loc1_ = this->wrapper;
		while (_loc1_ != nullptr)
		{
			std::shared_ptr<Wrapper> _loc19_((!(_loc1_->_next == nullptr)) ? _loc1_->next() : this->wrapper);
			std::shared_ptr<Wrapper> _loc20_((!(_loc19_->_next == nullptr)) ? _loc19_->next() : this->wrapper);
			Vertex *_loc2_ = _loc1_->vertex.get();
			Vertex *_loc3_ = _loc19_->vertex.get();
			Vertex *_loc4_ = _loc20_->vertex.get();
			double _loc5_ = (_loc3_->x - _loc2_->x);
			double _loc6_ = (_loc3_->y - _loc2_->y);
			double _loc7_ = (_loc3_->z - _loc2_->z);
			double _loc8_ = (_loc4_->x - _loc2_->x);
			double _loc9_ = (_loc4_->y - _loc2_->y);
			double _loc10_ = (_loc4_->z - _loc2_->z);
			double _loc11_ = ((_loc10_ * _loc6_) - (_loc9_ * _loc7_));
			double _loc12_ = ((_loc8_ * _loc7_) - (_loc10_ * _loc5_));
			double _loc13_ = ((_loc9_ * _loc5_) - (_loc8_ * _loc6_));
			double _loc14_ = (((_loc11_ * _loc11_) + (_loc12_ * _loc12_)) + (_loc13_ * _loc13_));
			if (_loc14_ > _loc15_)
			{
				_loc15_ = _loc14_;
				_loc16_ = _loc1_;
			}
			_loc1_ = _loc1_->next();
		}
		if (_loc16_ != this->wrapper)
		{
			std::shared_ptr<Wrapper> _loc17_ = this->wrapper->_next->_next->next();
			while (_loc17_->_next != nullptr)
			{
				_loc17_ = _loc17_->next();
			}
			std::shared_ptr<Wrapper> _loc18_ = this->wrapper;
			while (((!(_loc18_->next() == _loc16_)) && (!(_loc18_->_next == nullptr))))
			{
				_loc18_ = _loc18_->next();
			}
			_loc17_->setNext(this->wrapper);
			_loc18_->setNext(nullptr);
			this->wrapper = _loc16_;
		}
	}
	std::shared_ptr<Wrapper> _loc1_ = this->wrapper;
	std::shared_ptr<Vertex> _loc2_ = _loc1_->vertex;
	_loc1_ = _loc1_->next();
	std::shared_ptr<Vertex> _loc3_ = _loc1_->vertex;
	_loc1_ = _loc1_->next();
	std::shared_ptr<Vertex> _loc4_ = _loc1_->vertex;
	double _loc5_ = (_loc3_->x - _loc2_->x);
	double _loc6_ = (_loc3_->y - _loc2_->y);
	double _loc7_ = (_loc3_->z - _loc2_->z);
	double _loc8_ = (_loc4_->x - _loc2_->x);
	double _loc9_ = (_loc4_->y - _loc2_->y);
	double _loc10_ = (_loc4_->z - _loc2_->z);
	double _loc11_ = ((_loc10_ * _loc6_) - (_loc9_ * _loc7_));
	double _loc12_ = ((_loc8_ * _loc7_) - (_loc10_ * _loc5_));
	double _loc13_ = ((_loc9_ * _loc5_) - (_loc8_ * _loc6_));
	double _loc14_ = (((_loc11_ * _loc11_) + (_loc12_ * _loc12_)) + (_loc13_ * _loc13_));
	if (_loc14_ > 0)
	{
		_loc14_ = 1. / MyMath::sqrt(_loc14_);
		_loc11_ = _loc11_ * _loc14_;
		_loc12_ = _loc12_ * _loc14_;
		_loc13_ = _loc13_ * _loc14_;
		this->normalX = _loc11_;
		this->normalY = _loc12_;
		this->normalZ = _loc13_;
	}
	this->offset = (((_loc2_->x * _loc11_) + (_loc2_->y * _loc12_)) + (_loc2_->z * _loc13_));
}
