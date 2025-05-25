#include "mesh.h"
#include "../core/wrapper.h"
#include "../core/face.h"
#include "../core/vertex.h"
#include "mymath.h"
#include "_global.h"
#include "../materials/material.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "../core/camera3d.h"
#include "../core/view.h"
#include "../core/vg.h"
#include <QVector>
#include <QHash>
//#include <QDebug>
#include "qhash_ptr.h"
#include "../core/Object3DFilter.h"
#include "../core/rayintersectiondata.h"



const Object3D::T *Mesh::TYPE = new Object3D::T("Mesh");



void Mesh::setFaceList(std::shared_ptr<Face> face, int d)
{
	_function_name("Mesh::Mesh");

	(void)d;

	//qDebug() << "Mesh::setFaceList" << this << d << face.get();

	_faceList = face;
}

Mesh::Mesh(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	_function_name("Mesh::Mesh");
	this->sorting = 1;
	this->_faceList = nullptr;
	this->vertexList = nullptr;
	this->threshold = 0.01;
	this->vertexBuffer = nullptr;
	this->indexBuffer = nullptr;
	this->numOpaqueTriangles = 0;
	this->numTriangles = 0;
	this->clipping = 2;
	//this->material = nullptr;
	this->opaqueLength = 0;
	this->transparentList = nullptr;
}

Mesh::~Mesh()
{
	_function_name("Mesh::~Mesh");
	vertexList = nullptr;
	_faceList = nullptr;
}

void Mesh::destroy()
{
	_function_name("Mesh::destroy");

	if (faceList() != nullptr)
	{
		std::shared_ptr<Face> temp = faceList();
		setFaceList(nullptr, 3);
		temp->destroy();
	}

	if (vertexList != nullptr)
	{
		std::shared_ptr<Vertex> temp = vertexList;
		vertexList = nullptr;
		temp->destroy();
	}
}

void Mesh::calculateFacesNormals(bool param1)
{
	_function_name("Mesh::calculateFacesNormals");

	this->deleteResources();
	Face *_loc2_ = this->faceList().get();
	while (_loc2_ != nullptr)
	{
		Wrapper *_loc3_ = _loc2_->wrapper.get();
		Vertex *_loc4_ = _loc3_->vertex.get();
		_loc3_ = _loc3_->_next;
		Vertex* _loc5_ = _loc3_->vertex.get();
		_loc3_ = _loc3_->_next;
		Vertex *_loc6_ = _loc3_->vertex.get();
		double _loc7_ = (_loc5_->x - _loc4_->x);
		double _loc8_ = (_loc5_->y - _loc4_->y);
		double _loc9_ = (_loc5_->z - _loc4_->z);
		double _loc10_ = (_loc6_->x - _loc4_->x);
		double _loc11_ = (_loc6_->y - _loc4_->y);
		double _loc12_ = (_loc6_->z - _loc4_->z);
		double _loc13_ = ((_loc12_ * _loc8_) - (_loc11_ * _loc9_));
		double _loc14_ = ((_loc10_ * _loc9_) - (_loc12_ * _loc7_));
		double _loc15_ = ((_loc11_ * _loc7_) - (_loc10_ * _loc8_));
		if (param1)
		{
			double _loc16_ = (((_loc13_ * _loc13_) + (_loc14_ * _loc14_)) + (_loc15_ * _loc15_));
			if (_loc16_ > 0.001)
			{
				_loc16_ = (1. / MyMath::sqrt(_loc16_));
				_loc13_ = (_loc13_ * _loc16_);
				_loc14_ = (_loc14_ * _loc16_);
				_loc15_ = (_loc15_ * _loc16_);
			}
		}
		_loc2_->normalX = _loc13_;
		_loc2_->normalY = _loc14_;
		_loc2_->normalZ = _loc15_;
		_loc2_->offset = (((_loc4_->x * _loc13_) + (_loc4_->y * _loc14_)) + (_loc4_->z * _loc15_));
		_loc2_ = _loc2_->next.get();
	}
}

void Mesh::updateBounds(Object3D *param1, Object3D *param2)
{
	_function_name("Mesh::updateBounds");

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

void Mesh::clonePropertiesFromMesh(const Mesh *param1)
{
	_function_name("Mesh::clonePropertiesFrom");

	std::shared_ptr<Vertex> _loc4_ = nullptr;
	std::shared_ptr<Face> _loc5_ = nullptr;
	this->clonePropertiesFromObject3D(param1);
	const Mesh *_loc2_ = (const Mesh*)param1;
	this->clipping = _loc2_->clipping;
	this->sorting = _loc2_->sorting;
	this->threshold = _loc2_->threshold;
	std::shared_ptr<Vertex> _loc3_ = _loc2_->vertexList;
	while (_loc3_ != nullptr)
	{
		std::shared_ptr<Vertex> _loc7_ = Vertex::create();
		_loc7_->x = _loc3_->x;
		_loc7_->y = _loc3_->y;
		_loc7_->z = _loc3_->z;
		_loc7_->u = _loc3_->u;
		_loc7_->v = _loc3_->v;
		_loc7_->normalX = _loc3_->normalX;
		_loc7_->normalY = _loc3_->normalY;
		_loc7_->normalZ = _loc3_->normalZ;
		_loc7_->offset = _loc3_->offset;
		//_loc7_->id = _loc3_->id;
		_loc3_->value = _loc7_;
		if (_loc4_ != nullptr)
		{
			_loc4_->next = _loc7_;
		}
		else
		{
			this->vertexList = _loc7_;
		}
		_loc4_ = _loc7_;
		_loc3_ = _loc3_->next;
	}
	std::shared_ptr<Face> _loc6_ =  std::static_pointer_cast<Face>(_loc2_->constFaceList());

	while (_loc6_ != nullptr)
	{
		std::shared_ptr<Face> _loc8_ = Face::create();
		_loc8_->material = _loc6_->material;
		_loc8_->smoothingGroups = _loc6_->smoothingGroups;
		//_loc8_->id = _loc6_->id;
		_loc8_->normalX = _loc6_->normalX;
		_loc8_->normalY = _loc6_->normalY;
		_loc8_->normalZ = _loc6_->normalZ;
		_loc8_->offset = _loc6_->offset;
		std::shared_ptr<Wrapper> wrapper_tmp = nullptr;
		std::shared_ptr<Wrapper> wrapper_src = _loc6_->wrapper;
		while (wrapper_src != nullptr)
		{
			std::shared_ptr<Wrapper> wrapper = Wrapper::create(4);
			wrapper->vertex = wrapper_src->vertex->value;

			if (wrapper_tmp != nullptr)
			{
				wrapper_tmp->setNext(wrapper);
			}
			else
			{
				_loc8_->wrapper = wrapper;
			}
			wrapper_tmp = wrapper;
			wrapper_src = wrapper_src->next();
		}
		if (_loc5_ != nullptr)
		{
			_loc5_->next = _loc8_;
		}
		else
		{
			this->setFaceList(_loc8_, 4);
		}
		_loc5_ = _loc8_;
		_loc6_ = _loc6_->next;
	}
	_loc3_ = _loc2_->vertexList;
	while (_loc3_ != nullptr)
	{
		_loc3_->value = nullptr;
		_loc3_ = _loc3_->next;
	}
}


void Mesh::deleteResources()
{
	_function_name("Mesh::deleteResources");

	if (this->vertexBuffer != nullptr)
	{
		this->vertexBuffer->dispose();
		this->vertexBuffer = nullptr;
		this->indexBuffer->dispose();
		this->indexBuffer = nullptr;
		this->numTriangles = 0;
		this->numOpaqueTriangles = 0;
		this->opaqueMaterials.clear();
		this->opaqueBegins.clear();
		this->opaqueNums.clear();
		this->opaqueLength = 0;
		this->transparentList = nullptr;
	}

}



void Mesh::optimizeForDynamicBSP(int param1)
{
	_function_name("Mesh::optimizeForDynamicBSP");

	Face *_loc3_ = nullptr;

	this->deleteResources();
	std::shared_ptr<Face> _loc2_ = this->faceList();

	int _loc4_ = 0;

	while (_loc4_ < param1)
	{
		Face *_loc5_ = nullptr;
		std::shared_ptr<Face> _loc6_ = _loc2_;
		while (_loc6_ != nullptr)
		{
			double _loc7_ = _loc6_->normalX;
			double _loc8_ = _loc6_->normalY;
			double _loc9_ = _loc6_->normalZ;
			double _loc10_ = _loc6_->offset;
			double _loc11_ = (_loc10_ - this->threshold);
			double _loc12_ = (_loc10_ + this->threshold);
			int _loc13_ = 0;
			Face *_loc14_ = _loc2_.get();
			while (_loc14_ != nullptr)
			{
				if (_loc14_ != _loc6_.get())
				{
					Wrapper *_loc15_ = _loc14_->wrapper.get();
					std::shared_ptr<Vertex> _loc16_ = _loc15_->vertex;
					_loc15_ = _loc15_->_next;
					Vertex *_loc17_ = _loc15_->vertex.get();
					_loc15_ = _loc15_->_next;
					Vertex *_loc18_ = _loc15_->vertex.get();
					_loc15_ = _loc15_->_next;
					double _loc19_ = (((_loc16_->x * _loc7_) + (_loc16_->y * _loc8_)) + (_loc16_->z * _loc9_));
					double _loc20_ = (((_loc17_->x * _loc7_) + (_loc17_->y * _loc8_)) + (_loc17_->z * _loc9_));
					double _loc21_ = (((_loc18_->x * _loc7_) + (_loc18_->y * _loc8_)) + (_loc18_->z * _loc9_));
					bool _loc22_ = (((_loc19_ < _loc11_) || (_loc20_ < _loc11_)) || (_loc21_ < _loc11_));
					bool _loc23_ = (((_loc19_ > _loc12_) || (_loc20_ > _loc12_)) || (_loc21_ > _loc12_));
					while (_loc15_ != nullptr)
					{
						Vertex *_loc24_ = _loc15_->vertex.get();
						double _loc25_ = (((_loc24_->x * _loc7_) + (_loc24_->y * _loc8_)) + (_loc24_->z * _loc9_));
						if (_loc25_ < _loc11_)
						{
							_loc22_ = true;
							if (_loc23_)
							{
								break;
							}
						}
						else
						{
							if (_loc25_ > _loc12_)
							{
								_loc23_ = true;
								if (_loc22_)
								{
									break;
								}
							}
						}
						_loc15_ = _loc15_->_next;
					}
					if (((_loc23_) && (_loc22_)))
					{
						_loc13_++;
						if (_loc13_ > _loc4_)
						{
							break;
						}
					}
				}
				_loc14_ = _loc14_->next.get();
			}
			if (_loc14_ == nullptr)
			{
				if (_loc5_ != nullptr)
				{
					_loc5_->next = _loc6_->next;
				}
				else
				{
					_loc2_ = _loc6_->next;
				}
				if (_loc3_ != nullptr)
				{
					_loc3_->next = _loc6_;
				}
				else
				{
					this->setFaceList(_loc6_, 5);
				}
				_loc3_ = _loc6_.get();
			}
			else
			{
				_loc5_ = _loc6_.get();
			}
			_loc6_ = _loc6_->next;
		}
		if (_loc2_ == nullptr)
		{
			break;
		}
		_loc4_++;
	}

	if (_loc3_ != nullptr)
	{
		_loc3_->next = _loc2_;
	}
}

static QVector<std::shared_ptr<Face>> getFaces(Mesh *mesh)
{
	_function_name("Mesh::getFaces");

	QVector<std::shared_ptr<Face>> faces;

	std::shared_ptr<Face> face = mesh->faceList();

	while (face != nullptr)
	{
		faces.append(face);
		face = face->next;
	}

	return faces;
}



static void group(QVector<std::shared_ptr<Vertex>> &param1, int param2, int param3, int param4, double param5, double param6, QHash<int, int> &param7)
{
	_function_name("Mesh::group");

	int _loc8_;
	Vertex *_loc13_;
	double _loc11_;

	switch (param4)
	{
		case 0:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				Vertex *_loc10_ = param1[_loc8_].get();
				_loc10_->offset = _loc10_->x;
				_loc8_++;
			}
			_loc11_ = param5;
			break;
		case 1:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				Vertex *_loc10_ = param1[_loc8_].get();
				_loc10_->offset = _loc10_->y;
				_loc8_++;
			}
			_loc11_ = param5;
			break;
		case 2:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				Vertex *_loc10_ = param1[_loc8_].get();
				_loc10_->offset = _loc10_->z;
				_loc8_++;
			}
			_loc11_ = param5;
			break;
		case 3:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				Vertex *_loc10_ = param1[_loc8_].get();
				_loc10_->offset = _loc10_->u;
				_loc8_++;
			}
			_loc11_ = param6;
			break;
		case 4:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				Vertex *_loc10_ = param1[_loc8_].get();
				_loc10_->offset = _loc10_->v;
				_loc8_++;
			}
			_loc11_ = param6;
	}
	param7[0] = param2;
	param7[1] = (param3 - 1);
	int _loc12_ = 2;
	while (_loc12_ > 0)
	{
		_loc12_--;
		int _loc14_ = param7[_loc12_];
		int _loc9_ = _loc14_;
		_loc12_--;
		int _loc15_ = param7[_loc12_];
		_loc8_ = _loc15_;
		Vertex *_loc10_ = param1[((_loc14_ + _loc15_) >> 1)].get();
		double _loc16_ = _loc10_->offset;
		while (_loc8_ <= _loc9_)
		{
			std::shared_ptr<Vertex> _loc17_ = param1[_loc8_];
			while (_loc17_->offset > _loc16_)
			{
				_loc8_++;
				_loc17_ = param1[_loc8_];
			}
			std::shared_ptr<Vertex> _loc18_ = param1[_loc9_];
			while (_loc18_->offset < _loc16_)
			{
				_loc9_--;
				_loc18_ = param1[_loc9_];
			}
			if (_loc8_ <= _loc9_)
			{
				param1[_loc8_] = _loc18_;
				param1[_loc9_] = _loc17_;
				_loc8_++;
				_loc9_--;
			}
		}
		if (_loc15_ < _loc9_)
		{
			param7[_loc12_] = _loc15_;
			_loc12_++;
			param7[_loc12_] = _loc9_;
			_loc12_++;
		}
		if (_loc8_ < _loc14_)
		{
			param7[_loc12_] = _loc8_;
			_loc12_++;
			param7[_loc12_] = _loc14_;
			_loc12_++;
		}
	}
	_loc8_ = param2;
	std::shared_ptr<Vertex> _loc10_ = param1[_loc8_];

	int _loc9_ = _loc8_ + 1;
	while (_loc9_ <= param3)
	{
		if (_loc9_ < param3)
		{
			_loc13_ = param1[_loc9_].get();
		}
		if (((_loc9_ == param3) || ((_loc10_->offset - _loc13_->offset) > _loc11_)))
		{
			if (((param4 < 4) && ((_loc9_ - _loc8_) > 1)))
			{
				::group(param1, _loc8_, _loc9_, (param4 + 1), param5, param6, param7);
			}
			if (_loc9_ < param3)
			{
				_loc8_ = _loc9_;
				_loc10_ = param1[_loc8_];
			}
		}
		else
		{
			if (param4 == 4)
			{
				_loc13_->value = _loc10_;
			}
		}
		_loc9_++;
	}
}


void Mesh::weldVertices(double param1, double param2)
{
	_function_name("Mesh::weldVertices");

	this->deleteResources();
	QVector<std::shared_ptr<Vertex>> _loc5_;

	int _loc6_ = 0;
	std::shared_ptr<Vertex> _loc3_ = this->vertexList;
	while (_loc3_ != nullptr)
	{
		std::shared_ptr<Vertex> _loc4_ = _loc3_->next;
		_loc3_->next = nullptr;
		_loc5_.append(_loc3_);
		_loc6_++;
		_loc3_ = _loc4_;
	}

	this->vertexList = nullptr;
	QHash<int, int> v;

	::group(_loc5_, 0, _loc6_, 0, param1, param2, v);

	Face *_loc7_ = this->faceList().get();
	while (_loc7_ != nullptr)
	{
		Wrapper *_loc9_ = _loc7_->wrapper.get();
		while (_loc9_ != nullptr)
		{
			if (_loc9_->vertex->value != nullptr)
			{
				_loc9_->vertex = _loc9_->vertex->value;
			}
			_loc9_ = _loc9_->_next;
		}
		_loc7_ = _loc7_->next.get();
	}

	for (int i = 0; i < _loc6_; i++)
	{
		std::shared_ptr<Vertex> vertex = _loc5_.at(i);
		if (vertex->value == nullptr)
		{
			vertex->next = this->vertexList;
			this->vertexList = vertex;
		}
	}
}

void Mesh::weldFaces(double angleThreshold, double uvThreshold, double convexThreshold, bool pairWeld)
{
	_function_name("Mesh::weldFaces");

	bool weld = false;

	QVector<std::shared_ptr<Face>> island;

	std::shared_ptr<Face> face = nullptr;

	int num;
	std::shared_ptr<Face> sibling;

	double digitThreshold = 0.001;
	angleThreshold = (MyMath::cos(angleThreshold) - digitThreshold);
	uvThreshold = (uvThreshold + digitThreshold);
	convexThreshold = MyMath::cos((MyMath::PI - convexThreshold)) - digitThreshold;

	QVector<std::shared_ptr<Face>> faces = getFaces(this);

	QSet<std::shared_ptr<Face>> faceSet;
	QHash<double, std::shared_ptr<Face>> faceSet_d;

	//var faceSet:Dictionary = new Dictionary();
	//var map:Dictionary = new Dictionary();
	QHash<Vertex*, QSet<std::shared_ptr<Face>>*> map;

	for (std::shared_ptr<Face> &face : faces)
	{
		Vertex *a = face->wrapper->vertex.get();
		Vertex *b = face->wrapper->_next->vertex.get();
		Vertex *c = face->wrapper->_next->_next->vertex.get();

		double abx = (b->x - a->x);
		double aby = (b->y - a->y);
		double abz = (b->z - a->z);
		double acx = (c->x - a->x);
		double acy = (c->y - a->y);
		double acz = (c->z - a->z);
		double nx = ((acz * aby) - (acy * abz));
		double ny = ((acx * abz) - (acz * abx));
		double nz = ((acy * abx) - (acx * aby));
		double nl = (((nx * nx) + (ny * ny)) + (nz * nz));

		if (nl > digitThreshold)
		{
			nl = (1. / MyMath::sqrt(nl));
			nx = (nx * nl);
			ny = (ny * nl);
			nz = (nz * nl);
			face->normalX = nx;
			face->normalY = ny;
			face->normalZ = nz;
			face->offset = (((a->x * nx) + (a->y * ny)) + (a->z * nz));
			faceSet.insert(face);
			Wrapper *wn = face->wrapper.get();
			while (wn != nullptr)
			{
				Vertex *vertex = wn->vertex.get();
				QSet<std::shared_ptr<Face>> *dictionary = map.value(vertex);
				if (dictionary == nullptr)
				{
					dictionary = new QSet<std::shared_ptr<Face>>;
					map.insert(vertex, dictionary);
				}
				dictionary->insert(face);
				wn = wn->_next;
			}
		}
	}

	faceSet.clear();

	double faceIdCounter = 0;

	QSet<std::shared_ptr<Face>> unfit;
	QSet<std::shared_ptr<Face>> siblings;


	for (;;)
	{
		face = nullptr;

		{
			QSetIterator<std::shared_ptr<Face>> i(faceSet);
			while (i.hasNext())
			{
				face = i.next();
			}
		}

		faceSet.clear();
		faceSet_d.clear();

		/*
		for (key in faceSet)
		{
			face = key;
			delete faceSet[key];
			break;
		}
		*/


		if (face == nullptr)
			break;

		num = 0;
		//island[num] = face;
		island.append(face);
		num++;

		Vertex *a = face->wrapper->vertex.get();
		Vertex *b = face->wrapper->_next->vertex.get();
		Vertex *c = face->wrapper->_next->_next->vertex.get();
		double abx = (b->x - a->x);
		double aby = (b->y - a->y);
		double abz = (b->z - a->z);
		double abu = (b->u - a->u);
		double abv = (b->v - a->v);
		double acx = (c->x - a->x);
		double acy = (c->y - a->y);
		double acz = (c->z - a->z);
		double acu = (c->u - a->u);
		double acv = (c->v - a->v);
		double nx = face->normalX;
		double ny = face->normalY;
		double nz = face->normalZ;
		double det = (((((((-(nx) * acy) * abz) + ((acx * ny) * abz)) + ((nx * aby) * acz)) - ((abx * ny) * acz)) - ((acx * aby) * nz)) + ((abx * acy) * nz));
		double ima = (((-(ny) * acz) + (acy * nz)) / det);
		double imb = (((nx * acz) - (acx * nz)) / det);
		double imc = (((-(nx) * acy) + (acx * ny)) / det);
		double imd = ((((((((a->x * ny) * acz) - ((nx * a->y) * acz)) - ((a->x * acy) * nz)) + ((acx * a->y) * nz)) + ((nx * acy) * a->z)) - ((acx * ny) * a->z)) / det);
		double ime = (((ny * abz) - (aby * nz)) / det);
		double imf = (((-(nx) * abz) + (abx * nz)) / det);
		double img = (((nx * aby) - (abx * ny)) / det);
		double imh = ((((((((nx * a->y) * abz) - ((a->x * ny) * abz)) + ((a->x * aby) * nz)) - ((abx * a->y) * nz)) - ((nx * aby) * a->z)) + ((abx * ny) * a->z)) / det);
		double ma = ((abu * ima) + (acu * ime));
		double mb = ((abu * imb) + (acu * imf));
		double mc = ((abu * imc) + (acu * img));
		double md = (((abu * imd) + (acu * imh)) + a->u);
		double me = ((abv * ima) + (acv * ime));
		double mf = ((abv * imb) + (acv * imf));
		double mg = ((abv * imc) + (acv * img));
		double mh = (((abv * imd) + (acv * imh)) + a->v);

		unfit.clear();
		//for (key in unfit)
		//{
		//delete unfit[key];
		//}

		int i = 0;
		while (i < num)
		{
			face = island[i];

			siblings.clear();

			//for (key in siblings)
			//{
			//delete siblings[key];
			//}

			Wrapper *w = face->wrapper.get();
			while (w != nullptr)
			{
				//QHash<Vertex*, QHash<Face*, bool>*> map;

				//for (key in map[w.vertex])
				QSetIterator<std::shared_ptr<Face>> i(*map[w->vertex.get()]);
				while (i.hasNext())
				{
					std::shared_ptr<Face> key = i.next();

					if (faceSet.contains(key) && !unfit.contains(key))
					{
						siblings.insert(key);
					}
				}
				w = w->_next;
			}

			QSetIterator<std::shared_ptr<Face>> iterator(siblings);
			while (iterator.hasNext())
			{
				std::shared_ptr<Face> key = iterator.next();

				sibling = key;
				if ((((nx * sibling->normalX) + (ny * sibling->normalY)) + (nz * sibling->normalZ)) >= angleThreshold)
				{
					Wrapper *s = sibling->wrapper.get();
					while (s != nullptr)
					{
						Vertex *vertex = s->vertex.get();
						double du = (((((ma * vertex->x) + (mb * vertex->y)) + (mc * vertex->z)) + md) - vertex->u);
						double dv = (((((me * vertex->x) + (mf * vertex->y)) + (mg * vertex->z)) + mh) - vertex->v);
						if (((((du > uvThreshold) || (du < -(uvThreshold))) || (dv > uvThreshold)) || (dv < -(uvThreshold)))) break;
						s = s->_next;
					}

					if (s == nullptr)
					{
						Wrapper *w = face->wrapper.get();
						while (w != nullptr)
						{
							Wrapper *wn = ((w->_next != nullptr) ? w->_next : face->wrapper.get());
							Wrapper *s = sibling->wrapper.get();
							while (s != nullptr)
							{
								Wrapper *sn = (s->_next != nullptr) ? s->_next : sibling->wrapper.get();
								if (((w->vertex == sn->vertex) && (wn->vertex == s->vertex)))
									break;
								s = s->_next;
							}

							if (s != nullptr)
								break;

							w = w->_next;
						}

						if (w != nullptr)
						{
							//island[num] = sibling;
							island.append(face);
							num++;
							//delete faceSet[sibling];
							faceSet.remove(sibling);
						}
					}
					else
					{
						unfit.insert(sibling);
					}
				}
				else
				{
					unfit.insert(sibling);
				}
			}
			i++;
		}
		if (num == 1)
		{
			faceSet_d[faceIdCounter] = island[0];
			faceIdCounter++;
		}
		else
		{
			while (true)
			{
				weld = false;
				i = 0;
				while (i < (num - 1))
				{
					face = island[i];
					if (face != nullptr)
					{
						int j = 1;
						for (; j < num; j++)
						{
							sibling = island[j];
							if (sibling != nullptr)
							{
								Wrapper *w = face->wrapper.get();
								Wrapper *wn;
								Wrapper *s;
								Wrapper *sn;

								while (w != nullptr)
								{
									wn = ((w->_next != nullptr) ? w->_next : face->wrapper.get());
									s = sibling->wrapper.get();
									while (s != nullptr)
									{
										sn = (s->_next != nullptr) ? s->_next : sibling->wrapper.get();
										if (((w->vertex == sn->vertex) && (wn->vertex == s->vertex)))
											break;
										s = s->_next;
									}
									if (s != nullptr)
										break;
									w = w->_next;
								}

								Wrapper *wm;
								Wrapper *sp;

								if (w != nullptr)
								{
									while (true)
									{
										wm = ((wn->_next != nullptr) ? wn->_next : face->wrapper.get());
										sp = sibling->wrapper.get();
										while (((!(sp->_next == s)) && (!(sp->_next == nullptr))))
										{
											sp = sp->_next;
										}
										if (wm->vertex == sp->vertex)
										{
											wn = wm;
											s = sp;
										}
										else
										{
											break;
										}
									}

									Wrapper *sm;
									Wrapper *wp;
									Wrapper *w = nullptr;

									while (true)
									{
										wp = face->wrapper.get();
										while (((!(wp->_next == w)) && (!(wp->_next == nullptr))))
										{
											wp = wp->_next;
										}
										sm = ((sn->_next != nullptr) ? sn->_next : sibling->wrapper.get());
										if (wp->vertex == sm->vertex)
										{
											w = wp;
											sn = sm;
										}
										else
										{
											break;
										}
									}

									a = w->vertex.get();
									b = sm->vertex.get();
									c = wp->vertex.get();
									abx = (b->x - a->x);
									aby = (b->y - a->y);
									abz = (b->z - a->z);
									acx = (c->x - a->x);
									acy = (c->y - a->y);
									acz = (c->z - a->z);
									nx = ((acz * aby) - (acy * abz));
									ny = ((acx * abz) - (acz * abx));
									nz = ((acy * abx) - (acx * aby));

									if (((((((nx < digitThreshold) && (nx > -(digitThreshold))) && (ny < digitThreshold)) && (ny > -(digitThreshold))) && (nz < digitThreshold)) && (nz > -(digitThreshold))))
									{
										if ((((abx * acx) + (aby * acy)) + (abz * acz)) > 0.)
											continue;
									}
									else
									{
										if ((((face->normalX * nx) + (face->normalY * ny)) + (face->normalZ * nz)) < 0)
											continue;
									}

									double nl = (1. / MyMath::sqrt((((abx * abx) + (aby * aby)) + (abz * abz))));
									abx = (abx * nl);
									aby = (aby * nl);
									abz = (abz * nl);
									nl = (1. / MyMath::sqrt((((acx * acx) + (acy * acy)) + (acz * acz))));
									acx = (acx * nl);
									acy = (acy * nl);
									acz = (acz * nl);
									if ((((abx * acx) + (aby * acy)) + (abz * acz)) >= convexThreshold)
									{
										a = s->vertex.get();
										b = wm->vertex.get();
										c = sp->vertex.get();
										abx = (b->x - a->x);
										aby = (b->y - a->y);
										abz = (b->z - a->z);
										acx = (c->x - a->x);
										acy = (c->y - a->y);
										acz = (c->z - a->z);
										nx = ((acz * aby) - (acy * abz));
										ny = ((acx * abz) - (acz * abx));
										nz = ((acy * abx) - (acx * aby));
										if (((((((nx < digitThreshold) && (nx > -(digitThreshold))) && (ny < digitThreshold)) && (ny > -(digitThreshold))) && (nz < digitThreshold)) && (nz > -(digitThreshold))))
										{
											if ((((abx * acx) + (aby * acy)) + (abz * acz)) > 0)
												continue;
										}
										else
										{
											if ((((face->normalX * nx) + (face->normalY * ny)) + (face->normalZ * nz)) < 0)
												continue;
										}

										nl = (1. / MyMath::sqrt((((abx * abx) + (aby * aby)) + (abz * abz))));
										abx = (abx * nl);
										aby = (aby * nl);
										abz = (abz * nl);
										nl = (1. / MyMath::sqrt((((acx * acx) + (acy * acy)) + (acz * acz))));
										acx = (acx * nl);
										acy = (acy * nl);
										acz = (acz * nl);
										if ((((abx * acx) + (aby * acy)) + (abz * acz)) >= convexThreshold)
										{
											weld = true;
											std::shared_ptr<Face> newFace = Face::create();
											newFace->material = face->material;
											newFace->normalX = face->normalX;
											newFace->normalY = face->normalY;
											newFace->normalZ = face->normalZ;
											newFace->offset = face->offset;
											wm = nullptr;

											while (wn != w)
											{
												std::shared_ptr<Wrapper> sm = Wrapper::create(5);
												sm->vertex = wn->vertex;
												if (wm != nullptr)
												{
													wm->setNext(sm);
												}
												else
												{
													newFace->wrapper = sm;
												}
												wm = sm.get();
												wn = ((wn->_next != nullptr) ? wn->_next : face->wrapper.get());
											}

											while (sn != s)
											{
												std::shared_ptr<Wrapper> sm = Wrapper::create(6);
												sm->vertex = sn->vertex;
												if (wm != nullptr)
												{
													wm->setNext(sm);
												}
												else
												{
													newFace->wrapper = sm;
												}
												wm = sm.get();
												sn = (sn->_next != nullptr) ? sn->_next : sibling->wrapper.get();
											}

											island[i] = newFace;
											island[j] = nullptr;
											face = newFace;
											if (pairWeld)
												break;
										}
									}
								}
							}
						}
					}
					i++;
				}
				if (!weld)
					break;
			}
			i = 0;
			while (i < num)
			{
				face = island[i];
				if (face != nullptr)
				{
					face->calculateBestSequenceAndNormal();
					faceSet_d[faceIdCounter] = face;
					faceIdCounter++;
				}
				i++;
			}
		}
	}

	{
		QHashIterator<Vertex*, QSet<std::shared_ptr<Face>>*> i(map);
		while (i.hasNext())
		{
			i.next();
			delete i.value();
		}
	}
}

static QVector<Vertex*> getVertices(Mesh *mesh)
{
	_function_name("Mesh::getVertices");

	QVector<Vertex*> list;
	Vertex *vertex = mesh->vertexList.get();
	while (vertex != nullptr)
	{
		list.append(vertex);
		vertex = vertex->next.get();
	}
	return list;
}

int Mesh::getVertices(Vertex** out, int count)
{
	_function_name("Mesh::getVertices");

	int i = 0;
	Vertex *vertex = this->vertexList.get();
	while (vertex != nullptr && i < count)
	{
		out[i++] = vertex;
		vertex = vertex->next.get();
	}
	return i;
}


bool Mesh::intersectRay(const Vector3 *origin, const Vector3 *direction, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result)
{
	(void)camera;

	Vector3 _loc11_;
	Face *face = nullptr;


	if (filter != nullptr && filter->check(this))
	{
		return false;
	}

	if (!boundIntersectRay(origin, direction, bound.MinX, bound.MinY, bound.MinZ, bound.MaxX, bound.MaxY, bound.MaxZ))
	{
		return false;
	}

	double _loc5_ = origin->x;
	double _loc6_ = origin->y;
	double _loc7_ = origin->z;
	double _loc8_ = direction->x;
	double _loc9_ = direction->y;
	double _loc10_ = direction->z;
	double _loc13_ = 1E22;
	Face *_loc14_ = this->faceList().get();

	while (_loc14_ != nullptr)
	{
		double _loc15_ = _loc14_->normalX;
		double _loc16_ = _loc14_->normalY;
		double _loc17_ = _loc14_->normalZ;
		double _loc18_ = (((_loc8_ * _loc15_) + (_loc9_ * _loc16_)) + (_loc10_ * _loc17_));
		if (_loc18_ < 0)
		{
			double _loc19_ = ((((_loc5_ * _loc15_) + (_loc6_ * _loc16_)) + (_loc7_ * _loc17_)) - _loc14_->offset);
			if (_loc19_ > 0)
			{
				double _loc20_ = (-(_loc19_) / _loc18_);
				if (((face == nullptr) || (_loc20_ < _loc13_)))
				{
					double _loc21_ = (_loc5_ + (_loc8_ * _loc20_));
					double _loc22_ = (_loc6_ + (_loc9_ * _loc20_));
					double _loc23_ = (_loc7_ + (_loc10_ * _loc20_));
					Wrapper *_loc24_ = _loc14_->wrapper.get();
					while (_loc24_ != nullptr)
					{
						Vertex *_loc25_ = _loc24_->vertex.get();
						Vertex *_loc26_ = ((!(_loc24_->_next == nullptr)) ? _loc24_->_next->vertex.get() : _loc14_->wrapper->vertex.get());
						double _loc27_ = (_loc26_->x - _loc25_->x);
						double _loc28_ = (_loc26_->y - _loc25_->y);
						double _loc29_ = (_loc26_->z - _loc25_->z);
						double _loc30_ = (_loc21_ - _loc25_->x);
						double _loc31_ = (_loc22_ - _loc25_->y);
						double _loc32_ = (_loc23_ - _loc25_->z);
						if ((((((_loc32_ * _loc28_) - (_loc31_ * _loc29_)) * _loc15_) + (((_loc30_ * _loc29_) - (_loc32_ * _loc27_)) * _loc16_)) + (((_loc31_ * _loc27_) - (_loc30_ * _loc28_)) * _loc17_)) < 0)
						{
							break;
						}
						_loc24_ = _loc24_->_next;
					}
					if (_loc24_ == nullptr)
					{
						if (_loc20_ < _loc13_)
						{
							_loc13_ = _loc20_;
							_loc11_.x = _loc21_;
							_loc11_.y = _loc22_;
							_loc11_.z = _loc23_;
							face = _loc14_;
						}
					}
				}
			}
		}
		_loc14_ = _loc14_->next.get();
	}

	if (face != nullptr)
	{
		result->object = this;
		result->face = face;
		result->point.copy(&_loc11_);
		face->getUV(&_loc11_, &result->uv);
		result->time = _loc13_;
		return true;
	}

	return false;
}


static void shareFaces(QVector<Vertex*> &param1, int param2, int param3, int param4, double param5, QHash<int, int> &param6, QHash<Vertex*, QSet<Face*>*> &param7)
{
	_function_name("Mesh::shareFaces");

	int _loc8_;
	Vertex *_loc11_;
	Vertex *_loc13_ = nullptr;

	switch (param4)
	{
		case 0:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				_loc11_ = param1[_loc8_];
				_loc11_->offset = _loc11_->x;
				_loc8_++;
			}
			break;
		case 1:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				_loc11_ = param1[_loc8_];
				_loc11_->offset = _loc11_->y;
				_loc8_++;
			}
			break;
		case 2:
			_loc8_ = param2;
			while (_loc8_ < param3)
			{
				_loc11_ = param1[_loc8_];
				_loc11_->offset = _loc11_->z;
				_loc8_++;
			}
			break;
	}
	param6[0] = param2;
	param6[1] = (param3 - 1);
	int _loc12_ = 2;
	while (_loc12_ > 0)
	{
		_loc12_--;
		int _loc14_ = param6[_loc12_];
		int _loc9_ = _loc14_;
		_loc12_--;
		int _loc15_ = param6[_loc12_];
		_loc8_ = _loc15_;
		_loc11_ = param1[((_loc14_ + _loc15_) >> 1)];
		double _loc16_ = _loc11_->offset;
		while (_loc8_ <= _loc9_)
		{
			Vertex *_loc17_ = param1[_loc8_];
			while (_loc17_->offset > _loc16_)
			{
				_loc8_++;
				_loc17_ = param1[_loc8_];
			}
			Vertex *_loc18_ = param1[_loc9_];
			while (_loc18_->offset < _loc16_)
			{
				_loc9_--;
				_loc18_ = param1[_loc9_];
			}
			if (_loc8_ <= _loc9_)
			{
				param1[_loc8_] = _loc18_;
				param1[_loc9_] = _loc17_;
				_loc8_++;
				_loc9_--;
			}
		}
		if (_loc15_ < _loc9_)
		{
			param6[_loc12_] = _loc15_;
			_loc12_++;
			param6[_loc12_] = _loc9_;
			_loc12_++;
		}
		if (_loc8_ < _loc14_)
		{
			param6[_loc12_] = _loc8_;
			_loc12_++;
			param6[_loc12_] = _loc14_;
			_loc12_++;
		}
	}
	_loc8_ = param2;
	_loc11_ = param1[_loc8_];
	int _loc9_ = (_loc8_ + 1);
	while (_loc9_ <= param3)
	{
		if (_loc9_ < param3)
		{
			_loc13_ = param1[_loc9_];
		}
		if (((_loc9_ == param3) || ((_loc11_->offset - _loc13_->offset) > param5)))
		{
			if ((_loc9_ - _loc8_) > 1)
			{
				if (param4 < 2)
				{
					shareFaces(param1, _loc8_, _loc9_, (param4 + 1), param5, param6, param7);
				}
				else
				{
					int _loc10_ = (_loc8_ + 1);
					while (_loc10_ < _loc9_)
					{

						_loc13_ = param1.at(_loc10_);

						QSetIterator<Face*> i(*param7.value(_loc13_));

						while (i.hasNext())
						{
							Face *face = i.next();
							param7.value(_loc11_)->insert(face);
						}

						_loc10_++;
					}
					_loc10_ = (_loc8_ + 1);
					while (_loc10_ < _loc9_)
					{
						_loc13_ = param1.at(_loc10_);

						QSetIterator<Face*> i(*param7.value(_loc11_));

						while (i.hasNext())
						{
							Face *face = i.next();
							param7.value(_loc13_)->insert(face);
						}

						_loc10_++;
					}
				}
			}
			if (_loc9_ < param3)
			{
				_loc8_ = _loc9_;
				_loc11_ = param1[_loc8_];
			}
		}
		_loc9_++;
	}
}



void Mesh::calculateVerticesNormalsByAngle(double param1, double param2)
{
	_function_name("Mesh::calculateVerticesNormalsByAngle");

	this->deleteResources();
	this->calculateNormalsAndRemoveDegenerateFaces();

	QHash<Vertex*, QSet<Face*>*> _loc6_;

	Vertex *_loc5_ = this->vertexList.get();
	while (_loc5_ != nullptr)
	{
		_loc6_.insert(_loc5_, new QSet<Face*>);
		_loc5_ = _loc5_->next.get();
	}

	Face *_loc3_ = this->faceList().get();
	while (_loc3_ != nullptr)
	{
		Wrapper *_loc4_ = _loc3_->wrapper.get();
		while (_loc4_ != nullptr)
		{
			_loc5_ = _loc4_->vertex.get();
			QSet<Face*> *set = _loc6_.value(_loc5_);
			set->insert(_loc3_);
			_loc4_ = _loc4_->_next;
		}
		_loc3_ = _loc3_->next.get();
	}

	QVector<Vertex*> _loc7_ = ::getVertices(this);

	QHash<int, int> v;

	shareFaces(_loc7_, 0, _loc7_.length(), 0, param2, v, _loc6_);

	this->vertexList = nullptr;
	param1 = MyMath::cos(param1);

	_loc3_ = this->faceList().get();
	while (_loc3_ != nullptr)
	{
		Wrapper *_loc4_ = _loc3_->wrapper.get();

		while (_loc4_ != nullptr)
		{
			Vertex *_loc5_ = _loc4_->vertex.get();

			std::shared_ptr<Vertex> vertex =  Vertex::create();

			vertex->x = _loc5_->x;
			vertex->y = _loc5_->y;
			vertex->z = _loc5_->z;
			vertex->u = _loc5_->u;
			vertex->v = _loc5_->v;
			//vertex->id = _loc5_->id;
			vertex->normalX = _loc3_->normalX;
			vertex->normalY = _loc3_->normalY;
			vertex->normalZ = _loc3_->normalZ;

			QSetIterator<Face*> i(*_loc6_.value(_loc5_));

			while (i.hasNext())
			{
				Face *_loc11_ = i.next();
				if (_loc11_ == nullptr)
					continue;

				if (((!(_loc3_ == _loc11_)) && ((((_loc3_->normalX * _loc11_->normalX) + (_loc3_->normalY * _loc11_->normalY)) + (_loc3_->normalZ * _loc11_->normalZ)) >= param1)))
				{
					vertex->normalX = (vertex->normalX + _loc11_->normalX);
					vertex->normalY = (vertex->normalY + _loc11_->normalY);
					vertex->normalZ = (vertex->normalZ + _loc11_->normalZ);
				}
			}

			double _loc10_ = ((vertex->normalX * vertex->normalX) + (vertex->normalY * vertex->normalY)) + (vertex->normalZ * vertex->normalZ);
			if (_loc10_ > 0.001)
			{
				_loc10_ = 1. / MyMath::sqrt(_loc10_);
				vertex->normalX = (vertex->normalX * _loc10_);
				vertex->normalY = (vertex->normalY * _loc10_);
				vertex->normalZ = (vertex->normalZ * _loc10_);
			}

			_loc4_->vertex = vertex;
			vertex->next = this->vertexList;
			this->vertexList = vertex;
			_loc4_ = _loc4_->_next;
		}
		_loc3_ = _loc3_->next.get();
	}

	QHashIterator<Vertex*, QSet<Face*>*> i(_loc6_);
	while (i.hasNext())
	{
		i.next();
		delete i.value();
	}
}

void Mesh::prepareResources()
{
	_function_name("Mesh::prepareResources");

	if (this->vertexBuffer != nullptr)
	{
		return;
	}

	QVector<double> list_vertex;

	int count_vertex = 0;
	Vertex *vertex = this->vertexList.get();
	while (vertex != nullptr)
	{
		list_vertex.append(vertex->x);
		list_vertex.append(vertex->y);
		list_vertex.append(vertex->z);
		list_vertex.append(vertex->u);
		list_vertex.append(vertex->v);
		list_vertex.append(vertex->normalX);
		list_vertex.append(vertex->normalY);
		list_vertex.append(vertex->normalZ);

		vertex->index = count_vertex;
		count_vertex++;
		vertex = vertex->next.get();
	}

	if (count_vertex > 0)
	{
		this->vertexBuffer = std::make_shared<VertexBufferResource>(list_vertex, 8);
	}

	std::shared_ptr<Face> _loc10_ = nullptr;

	QHash<Material*, std::shared_ptr<Face>> _loc12_;

	std::shared_ptr<Face> _loc8_face = this->faceList();
	while (_loc8_face != nullptr)
	{
		std::shared_ptr<Face> next = _loc8_face->next;

		_loc8_face->next = nullptr;
		if (((!(_loc8_face->material == nullptr)) &&
			 ((!(_loc8_face->material->transparent())) ||
			  (_loc8_face->material->alphaTestThreshold > 0))))
		{
			std::shared_ptr<Face> face = _loc12_.value(_loc8_face->material.get());
			_loc8_face->next = face;

			_loc12_.insert(_loc8_face->material.get(), _loc8_face);
		}
		else
		{
			if (_loc10_ != nullptr)
			{
				_loc10_->next = _loc8_face;
			}
			else
			{
				this->transparentList = _loc8_face;
			}
			_loc10_ = _loc8_face;
		}
		_loc8_face = next;
	}

	this->setFaceList(this->transparentList, 6);

	QVector<uint32_t> list_index;



	QHashIterator<Material*, std::shared_ptr<Face>> i(_loc12_);
	while (i.hasNext())
	{
		i.next();

		std::shared_ptr<Face> _loc16_ = i.value();


		if (this->opaqueMaterials.length() < (this->opaqueLength+1))
			this->opaqueMaterials.resize(this->opaqueLength+1);

		if (this->opaqueBegins.length() < (this->opaqueLength+1))
			this->opaqueBegins.resize(this->opaqueLength+1);


		this->opaqueMaterials[this->opaqueLength] = _loc16_->material.get();
		this->opaqueBegins[this->opaqueLength] = this->numTriangles * 3;

		std::shared_ptr<Face> face = _loc16_;
		while (face != nullptr)
		{
			Wrapper *wrapper = face->wrapper.get();
			int index_1 = wrapper->vertex->index;
			wrapper = wrapper->_next;
			int index_2 = wrapper->vertex->index;
			wrapper = wrapper->_next;
			while (wrapper != nullptr)
			{
				int index_3 = wrapper->vertex->index;
				list_index.append(index_1);
				list_index.append(index_2);
				list_index.append(index_3);
				index_2 = index_3;
				this->numTriangles++;
				wrapper = wrapper->_next;
			}
			if (face->next == nullptr)
			{
				_loc10_ = face;
			}
			face = face->next;
		}

		if (this->opaqueNums.length() < (this->opaqueLength + 1))
			this->opaqueNums.resize(this->opaqueLength + 1);


		this->opaqueNums[this->opaqueLength] = (this->numTriangles - (this->opaqueBegins[this->opaqueLength] / 3));
		this->opaqueLength++;
		_loc10_->next = this->faceList();
		this->setFaceList(_loc16_, 7);
	}


	this->numOpaqueTriangles = this->numTriangles;
	_loc8_face = this->transparentList;

	while (_loc8_face != nullptr)
	{
		Wrapper *wrapper = _loc8_face->wrapper.get();
		int index_1 = wrapper->vertex->index;
		wrapper = wrapper->_next;
		int index_2 = wrapper->vertex->index;
		wrapper = wrapper->_next;
		while (wrapper != nullptr)
		{
			int index_3 = wrapper->vertex->index;
			list_index.append(index_1);
			list_index.append(index_2);
			list_index.append(index_3);

			index_2 = index_3;
			this->numTriangles++;
			wrapper = wrapper->_next;
		}
		_loc8_face = _loc8_face->next;
	}

	if (!list_index.isEmpty())
	{
		this->indexBuffer = std::make_shared<IndexBufferResource>(list_index);
	}
}


void Mesh::setMaterialToAllFaces(Material *material)
{
	_function_name("Mesh::setMaterialToAllFaces");

	//var tracksMat:Material;
	this->deleteResources();
	//this->material = material;

	std::shared_ptr<Material> m = material ? material->ptr.lock() : nullptr;

	Face *face = this->faceList().get();
	while (face != nullptr)
	{
		if (!(((!(material == nullptr)) && (!(face->material == nullptr))) && (face->material->getName() == "track")))
			//if (!(material != nullptr && face->material != nullptr && face->material->getName() == "track"))
		{
			face->material = m;
		}
		face = face->next.get();
	}
}



void Mesh::addOpaque(Camera3D *camera)
{
	_function_name("Mesh::addOpaque");

	for (int i = 0; i < this->opaqueLength; i++)
	{
		camera->addOpaque(this->opaqueMaterials[i],
						  this->vertexBuffer.get(),
						  this->indexBuffer.get(),
						  this->opaqueBegins[i],
						  this->opaqueNums[i],
						  this);
	}
}



void Mesh::draw(Camera3D *param1)
{
	_function_name("Mesh::draw");

	if (this->faceList() == nullptr)
		return;

	if (this->clipping == 0)
	{
		if (culling & 0x01)
		{
			return;
		}
		culling = 0;
	}

	this->prepareResources();

	if ((useDepth = (((!(param1->view->constrained)) &&
					  ((((param1->softTransparency) &&
						 (param1->softTransparencyStrength > 0)) ||
						((param1->ssao) && (param1->ssaoStrength > 0))) ||
					   ((param1->deferredLighting) && (param1->deferredLightingStrength > 0))))
					 && (concatenatedAlpha >= this->depthMapAlphaThreshold))))
	{
		if (param1->depthObjects.length() < (param1->depthCount+1))
			param1->depthObjects.resize(param1->depthCount+1);

		param1->depthObjects[param1->depthCount] = this;
		param1->depthCount++;
	}

	std::shared_ptr<Face> face;


	if (((concatenatedAlpha >= 1) && (concatenatedBlendMode == BlendMode::NORMAL)))
	{


		this->addOpaque(param1);
		face = this->transparentList;
	}
	else
	{
		face = this->faceList();
	}

	transformConst[0] = this->matrix.ma;
	transformConst[1] = this->matrix.mb;
	transformConst[2] = this->matrix.mc;
	transformConst[3] = this->matrix.md;
	transformConst[4] = this->matrix.me;
	transformConst[5] = this->matrix.mf;
	transformConst[6] = this->matrix.mg;
	transformConst[7] = this->matrix.mh;
	transformConst[8] = this->matrix.mi;
	transformConst[9] = this->matrix.mj;
	transformConst[10] = this->matrix.mk;
	transformConst[11] = this->matrix.ml;

	/*
	int _loc3_ = ((!(!(param1->debug))) ? int(param1->checkInDebug(this)) : int(0));
	if ((_loc3_ & Debug.BOUNDS))
	{
		Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
	}
	*/

	if (face == nullptr)
	{
		return;
	}


	if (transformId > 500000000)
	{
		transformId = 0;
		Vertex *vertex = this->vertexList.get();
		while (vertex != nullptr)
		{
			vertex->transformId = 0;
			vertex = vertex->next.get();
		}
	}

	transformId++;
	calculateInverseMatrix();
	face = this->prepareFaces(param1, face);

	if (face == nullptr)
	{
		return;
	}


	if (culling > 0)
	{
		if (this->clipping == 1)
		{
			face = param1->cull(face, culling);
		}
		else
		{
			face = param1->clip(face, culling);
		}

		if (face == nullptr)
			return;
	}

	Face *face2 = face.get();

	if (face2->processNext != nullptr)
	{
		if (this->sorting == 1)
		{
			face2 = param1->sortByAverageZ(face2);
		}
		else
		{
			if (this->sorting == 2)
			{
				face2 = param1->sortByDynamicBSP(face2, this->threshold);
			}
		}
	}


	/*
	if ((_loc3_ & Debug.EDGES))
	{
		Debug.drawEdges(param1, _loc2_, 0xFFFFFF);
	}
	*/

	this->drawFaces(param1, face2);

}


void Mesh::drawFaces(Camera3D *param1, Face *param2)
{
	_function_name("Mesh::drawFaces");

	Face *_loc4_ = nullptr;
	Face *_loc5_ = param2;
	while (_loc5_ != nullptr)
	{
		Face *_loc3_ = _loc5_->processNext.get();
		if (((_loc3_ == nullptr) || (!(_loc3_->material == param2->material))))
		{
			_loc5_->processNext = nullptr;
			if (param2->material != nullptr)
			{
				param2->processNegative = _loc4_->ptr();
				_loc4_ = param2;
			}
			else
			{
				while (param2 != nullptr)
				{
					_loc5_ = param2->processNext.get();
					param2->processNext = nullptr;
					param2 = _loc5_;
				}
			}
			param2 = _loc3_;
		}
		_loc5_ = _loc3_;
	}
	param2 = _loc4_;
	while (param2 != nullptr)
	{
		Face *_loc3_ = param2->processNegative.get();
		param2->processNegative = nullptr;

		std::shared_ptr<Face> face = param2->ptr();
		std::shared_ptr<Object3D> object3D = this->ptr();

		param1->addTransparent(face, object3D);
		param2 = _loc3_;
	}
}


std::shared_ptr<Face> Mesh::prepareFaces(Camera3D *param1, std::shared_ptr<Face> param2)
{
	_function_name("Mesh::prepareFaces");

	(void)param1;

	std::shared_ptr<Face> _loc3_;
	std::shared_ptr<Face> _loc4_;
	std::shared_ptr<Face> _loc5_ = param2;

	while (_loc5_ != nullptr)
	{
		if ((((_loc5_->normalX * matrix_i.md) + (_loc5_->normalY * matrix_i.mh)) + (_loc5_->normalZ * matrix_i.ml)) > _loc5_->offset)
		{
			Wrapper *_loc6_ = _loc5_->wrapper.get();
			while (_loc6_ != nullptr)
			{
				Vertex *_loc7_ = _loc6_->vertex.get();
				if (_loc7_->transformId != transformId)
				{
					double _loc8_ = _loc7_->x;
					double _loc9_ = _loc7_->y;
					double _loc10_ = _loc7_->z;
					_loc7_->cameraX = ((((matrix.ma * _loc8_) + (matrix.mb * _loc9_)) + (matrix.mc * _loc10_)) + matrix.md);
					_loc7_->cameraY = ((((matrix.me * _loc8_) + (matrix.mf * _loc9_)) + (matrix.mg * _loc10_)) + matrix.mh);
					_loc7_->cameraZ = ((((matrix.mi * _loc8_) + (matrix.mj * _loc9_)) + (matrix.mk * _loc10_)) + matrix.ml);
					_loc7_->transformId = transformId;
					_loc7_->drawId = 0;
				}
				_loc6_ = _loc6_->_next;
			}
			if (_loc3_ != nullptr)
			{
				_loc4_->processNext = _loc5_;
			}
			else
			{
				_loc3_ = _loc5_;
			}
			_loc4_ = _loc5_;
		}
		_loc5_ = _loc5_->next;
	}

	if (_loc4_ != nullptr)
	{
		_loc4_->processNext = nullptr;
	}

	return _loc3_;
}

object_ptr<VG> Mesh::getVG(Camera3D *param1)
{
	_function_name("Mesh::getVG");

	if (this->faceList() == nullptr)
	{
		return nullptr;
	}


	if (this->clipping == 0)
	{
		if ((culling & 0x01))
		{
			return nullptr;
		}
		culling = 0;
	}

	this->prepareResources();

	if ((useDepth = (((!(param1->view->constrained)) &&
					  ((((param1->softTransparency) && (param1->softTransparencyStrength > 0)) ||
						((param1->ssao) && (param1->ssaoStrength > 0))) ||
					   ((param1->deferredLighting) && (param1->deferredLightingStrength > 0)))) &&
					 (concatenatedAlpha >= depthMapAlphaThreshold))))
	{

		if (param1->depthObjects.length() < (param1->depthCount+1))
			param1->depthObjects.resize(param1->depthCount+1);

		param1->depthObjects[param1->depthCount] = this;
		param1->depthCount++;
	}


	std::shared_ptr<Face> _loc2_;

	if (concatenatedAlpha >= 1 && concatenatedBlendMode == BlendMode::NORMAL)
	{
		this->addOpaque(param1);
		_loc2_ = this->transparentList;
	}
	else
	{
		_loc2_ = this->faceList();
	}


	transformConst[0] = this->matrix.ma;
	transformConst[1] = this->matrix.mb;
	transformConst[2] = this->matrix.mc;
	transformConst[3] = this->matrix.md;
	transformConst[4] = this->matrix.me;
	transformConst[5] = this->matrix.mf;
	transformConst[6] = this->matrix.mg;
	transformConst[7] = this->matrix.mh;
	transformConst[8] = this->matrix.mi;
	transformConst[9] = this->matrix.mj;
	transformConst[10] = this->matrix.mk;
	transformConst[11] = this->matrix.ml;
	int _loc3_ = 0; //((!(!(param1->debug))) ? int(param1.checkInDebug(this)) : int(0));
	/*
	if ((_loc3_ & Debug.BOUNDS))
	{
		Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
	}
	*/

	if (_loc2_ == nullptr)
	{
		return nullptr;
	}

	if (transformId > 500000000)
	{
		transformId = 0;
		Vertex *vertex = this->vertexList.get();
		while (vertex != nullptr)
		{
			vertex->transformId = 0;
			vertex = vertex->next.get();
		}
	}


	transformId++;
	calculateInverseMatrix();
	_loc2_ = this->prepareFaces(param1, _loc2_);
	if (_loc2_ == nullptr)
	{
		return nullptr;
	}


	if (culling > 0)
	{
		if (this->clipping == 1)
		{
			_loc2_ = param1->cull(_loc2_, culling);
		}
		else
		{
			_loc2_ = param1->clip(_loc2_, culling);
		}
		if (_loc2_ == nullptr)
		{
			return nullptr;
		}
	}


	std::shared_ptr<Object3D> object = this->ptr();
	std::shared_ptr<Face> faceStruct = _loc2_->ptr();
	return VG::create(object, faceStruct, this->sorting, _loc3_, false);
}


void Mesh::calculateNormalsAndRemoveDegenerateFaces()
{
	_function_name("Mesh::calculateNormalsAndRemoveDegenerateFaces");

	std::shared_ptr<Face> _loc1_ = this->faceList();
	this->setFaceList(nullptr, 21);
	while (_loc1_ != nullptr)
	{
		std::shared_ptr<Face> _loc2_ = _loc1_->next;
		Wrapper *_loc3_ = _loc1_->wrapper.get();
		Vertex *_loc4_ = _loc3_->vertex.get();
		_loc3_ = _loc3_->_next;
		Vertex *_loc5_ = _loc3_->vertex.get();
		_loc3_ = _loc3_->_next;
		Vertex *_loc6_ = _loc3_->vertex.get();
		double _loc7_ = (_loc5_->x - _loc4_->x);
		double _loc8_ = (_loc5_->y - _loc4_->y);
		double _loc9_ = (_loc5_->z - _loc4_->z);
		double _loc10_ = (_loc6_->x - _loc4_->x);
		double _loc11_ = (_loc6_->y - _loc4_->y);
		double _loc12_ = (_loc6_->z - _loc4_->z);
		_loc1_->normalX = ((_loc12_ * _loc8_) - (_loc11_ * _loc9_));
		_loc1_->normalY = ((_loc10_ * _loc9_) - (_loc12_ * _loc7_));
		_loc1_->normalZ = ((_loc11_ * _loc7_) - (_loc10_ * _loc8_));
		double _loc13_ = (((_loc1_->normalX * _loc1_->normalX) + (_loc1_->normalY * _loc1_->normalY)) + (_loc1_->normalZ * _loc1_->normalZ));
		if (_loc13_ > 0.001)
		{
			_loc13_ = 1. / MyMath::sqrt(_loc13_);
			_loc1_->normalX = (_loc1_->normalX * _loc13_);
			_loc1_->normalY = (_loc1_->normalY * _loc13_);
			_loc1_->normalZ = (_loc1_->normalZ * _loc13_);
			_loc1_->offset = (((_loc4_->x * _loc1_->normalX) + (_loc4_->y * _loc1_->normalY)) + (_loc4_->z * _loc1_->normalZ));
			_loc1_->next = this->faceList();
			this->setFaceList(_loc1_, 22);
		}
		else
		{
			_loc1_->next = nullptr;
		}
		_loc1_ = _loc2_;
	}
}


std::shared_ptr<Mesh> new_Mesh()
{
	_function_name("new_Mesh");

	return std::make_shared<Mesh>(Object3D::TT(Mesh::TYPE));
}


std::shared_ptr<Mesh> Mesh_clone(const std::shared_ptr<Mesh> &in)
{
	_function_name("Mesh_clone");

	if (in->isType(Mesh::TYPE)) {
		auto out = new_Mesh();
		out->clonePropertiesFromMesh(in.get());
		return out;
	}

	throw 41132422;
}


void Mesh::addVerticesAndFaces(const double *param1, int param1_length,
							   const double *param2, int param2_length,
							   const int *param3, int param3_length,
							   bool param4,
							   Material *param5)
{
	std::shared_ptr<Vertex> _loc11_;
	std::shared_ptr<Face> _loc13_;
	std::shared_ptr<Face> _loc14_;
	std::shared_ptr<Wrapper> _loc15_;

	this->deleteResources();

	if (param1 == nullptr)
	{
		throw 3434123413;
		//throw (new TypeError("Parameter vertices must be non-null."));
	}

	if (param2 == nullptr)
	{
		throw 3434123414;
		//throw (new TypeError("Parameter uvs must be non-null."));
	}

	if (param3 == nullptr)
	{
		throw 3434123415;
		//throw (new TypeError("Parameter indices must be non-null."));
	}

	int _loc9_ = int((param1_length / 3));

	if (_loc9_ != (param2_length / 2))
	{
		throw 3434123416;
		//throw (new ArgumentError("Vertices count and uvs count doesn't match."));
	}

	int _loc10_ = param3_length;

	if (((!(param4)) && (_loc10_ % 3)))
	{
		throw 3434123417;
		//throw (new ArgumentError("Incorrect indices."));
	}

	int _loc6_ = 0;
	int _loc8_ = 0;

	while (_loc6_ < _loc10_)
	{
		if (_loc6_ == _loc8_)
		{
			int _loc17_ = ((!(!(param4))) ? int(param3[_loc6_]) : int(3));
			if (_loc17_ < 3)
			{
				throw 3434123418;
				//throw (new ArgumentError((_loc17_ + " vertices not enough.")));
			}
			_loc8_ = ((!(!(param4))) ? int((_loc17_ + ++_loc6_)) : int((_loc6_ + _loc17_)));
			if (_loc8_ > _loc10_)
			{
				throw 3434123419;
				//throw (new ArgumentError("Incorrect indices."));
			}
		}

		int _loc16_ = param3[_loc6_];
		if (((_loc16_ < 0) || (_loc16_ >= _loc9_)))
		{
			throw 3434123420;
			//throw (new RangeError("Index is out of bounds."));
		}
		_loc6_++;
	}

	if (this->vertexList != nullptr)
	{
		_loc11_ = this->vertexList;
		while (_loc11_->next != nullptr)
		{
			_loc11_ = _loc11_->next;
		}
	}

	//var _loc12_:Vector.<Vertex> = new Vector.<Vertex>(_loc9_);
	QVector<std::shared_ptr<Vertex>> _loc12_;
	_loc12_.reserve(_loc9_);


	_loc6_ = 0;
	int _loc7_ = 0;
	_loc8_ = 0;
	while (_loc6_ < _loc9_)
	{
		std::shared_ptr<Vertex> vertex = Vertex::create();
		vertex->x = param1[_loc7_];
		_loc7_++;
		vertex->y = param1[_loc7_];
		_loc7_++;
		vertex->z = param1[_loc7_];
		_loc7_++;
		vertex->u = param2[_loc8_];
		_loc8_++;
		vertex->v = param2[_loc8_];
		_loc8_++;
		//_loc12_[_loc6_] = vertex;
		_loc12_.append(vertex);

		if (_loc11_ != nullptr)
		{
			_loc11_->next = vertex;
		}
		else
		{
			this->vertexList = vertex;
		}
		_loc11_ = vertex;
		_loc6_++;
	}

	if (this->faceList() != nullptr)
	{
		_loc13_ = this->faceList();
		while (_loc13_->next != nullptr)
		{
			_loc13_ = _loc13_->next;
		}
	}

	_loc6_ = 0;
	_loc8_ = 0;
	while (_loc6_ < _loc10_)
	{
		if (_loc6_ == _loc8_)
		{
			if (param4 == true)
			{
				int v = _loc6_;
				_loc8_ = param3[v] + ++_loc6_;
			}
			else
			{
				_loc8_ = _loc6_ + 3;
			}

			//_loc8_ = ((!(!(param4))) ? int((param3[v] + ++_loc6_)) : int((_loc6_ + 3)));

			_loc15_ = nullptr;
			_loc14_ = Face::create();
			_loc14_->material = param5 ? param5->ptr.lock() : nullptr;
			if (_loc13_ != nullptr)
			{
				_loc13_->next = _loc14_;
			}
			else
			{
				this->setFaceList(_loc14_, 30);
			}
			_loc13_ = _loc14_;
		}

		std::shared_ptr<Wrapper> _loc19_ = Wrapper::create(7);
		_loc19_->vertex = _loc12_[param3[_loc6_]];
		if (_loc15_ != nullptr)
		{
			_loc15_->setNext(_loc19_);
		}
		else
		{
			_loc14_->wrapper = _loc19_;
		}

		_loc15_ = _loc19_;
		_loc6_++;
	}

}


std::shared_ptr<Vertex> Mesh::addVertex(double x, double y, double z, double u, double v)
{
	this->deleteResources();
	std::shared_ptr<Vertex> result = Vertex::create();
	result->x = x;
	result->y = y;
	result->z = z;
	result->u = u;
	result->v = v;
	//result->id = param6;
	if (this->vertexList != nullptr)
	{
		Vertex *temp = this->vertexList.get();
		while (temp->next != nullptr)
		{
			temp = temp->next.get();
		}
		temp->next = result;
	}
	else
	{
		this->vertexList = result;
	}
	return result;
}


std::shared_ptr<Face> Mesh::addQuadFace(std::shared_ptr<Vertex> &param1,
										std::shared_ptr<Vertex> &param2,
										std::shared_ptr<Vertex> &param3,
										std::shared_ptr<Vertex> &param4,
										std::shared_ptr<Material> material)
{
	this->deleteResources();

	/*
	if (param1 == null)
		throw (new TypeError("Parameter v1 must be non-null."));
	if (param2 == null)
		throw (new TypeError("Parameter v2 must be non-null."));
	if (param3 == null)
		throw (new TypeError("Parameter v3 must be non-null."));
	if (param4 == null)
		throw (new TypeError("Parameter v4 must be non-null."));
	if ((!(this.containsVertex(param1))))
		throw (new ArgumentError("Vertex not found."));
	if ((!(this.containsVertex(param2))))
		throw (new ArgumentError("Vertex not found."));
	if ((!(this.containsVertex(param3))))
		throw (new ArgumentError("Vertex not found."));
	if ((!(this.containsVertex(param4))))
		throw (new ArgumentError("Vertex not found."));
	*/

	std::shared_ptr<Face> result = Face::create();
	result->material = material;
	//result->id = param6;
	result->wrapper = Wrapper::create(8);
	result->wrapper->vertex = param1;
	result->wrapper->setNext(Wrapper::create(9));
	result->wrapper->_next->vertex = param2;
	result->wrapper->_next->setNext(Wrapper::create(10));
	result->wrapper->_next->_next->vertex = param3;
	result->wrapper->_next->_next->setNext(Wrapper::create(11));
	result->wrapper->_next->_next->_next->vertex = param4;
	if (this->_faceList != nullptr)
	{
		Face *temp = this->_faceList.get();
		while (temp->next != nullptr)
		{
			temp = temp->next.get();
		}
		temp->next = result;
	}
	else
	{
		this->setFaceList(result, 0);
	}
	return result;
}



void Mesh::calculateVerticesNormalsBySmoothingGroups(double value)
{
	(void)value;
	this->deleteResources();
	this->calculateNormalsAndRemoveDegenerateFaces();
	QHash<Vertex*, QSet<Face*>*> _loc6_;
	Face *_loc3_ = this->faceList().get();
	while (_loc3_ != nullptr)
	{
		if (_loc3_->smoothingGroups > 0)
		{
			Wrapper *_loc5_ = _loc3_->wrapper.get();
			while (_loc5_ != nullptr)
			{
				Vertex *_loc4_ = _loc5_->vertex.get();
				if (_loc6_[_loc4_] == nullptr)
				{
					_loc6_[_loc4_] = new QSet<Face*>();
				}
				_loc6_.value(_loc4_)->insert(_loc3_);
				_loc5_ = _loc5_->_next;
			}
		}
		_loc3_ = _loc3_->next.get();
	}

	QVector<Vertex*> _loc7_;
	int _loc8_ = 0;

	{
		QHashIterator<Vertex*, QSet<Face*>*> i(_loc6_);
		while (i.hasNext())
		{
			i.next();
			_loc7_.append(i.key());
			_loc8_++;
		}
	}


	if (_loc8_ > 0)
	{
		//QVector<Vertex*> &param1, int param2, int param3, int param4, double param5, QHash<int, int> &param6, QHash<Vertex*, QSet<Face*>*> &param7
		QHash<int, int> param6;

		shareFaces(_loc7_, 0, _loc8_, 0, value, param6, _loc6_);
	}
	this->vertexList = nullptr;
	_loc3_ = this->faceList().get();
	while (_loc3_ != nullptr)
	{
		Wrapper *_loc5_ = _loc3_->wrapper.get();
		while (_loc5_ != nullptr)
		{
			Vertex *_loc4_ = _loc5_->vertex.get();
			std::shared_ptr<Vertex> _loc9_ = Vertex::create();
			_loc9_->x = _loc4_->x;
			_loc9_->y = _loc4_->y;
			_loc9_->z = _loc4_->z;
			_loc9_->u = _loc4_->u;
			_loc9_->v = _loc4_->v;
			//_loc9_->id = _loc4_->id;
			_loc9_->normalX = _loc3_->normalX;
			_loc9_->normalY = _loc3_->normalY;
			_loc9_->normalZ = _loc3_->normalZ;
			if (_loc3_->smoothingGroups > 0)
			{
				QSetIterator<Face*> i(*_loc6_[_loc4_]);
				while (i.hasNext())
				{
					Face *_loc11_ = i.next();
					if (((!(_loc3_ == _loc11_)) && ((_loc3_->smoothingGroups & _loc11_->smoothingGroups) > 0)))
					{
						_loc9_->normalX = (_loc9_->normalX + _loc11_->normalX);
						_loc9_->normalY = (_loc9_->normalY + _loc11_->normalY);
						_loc9_->normalZ = (_loc9_->normalZ + _loc11_->normalZ);
					}
				}
				double _loc10_ = (((_loc9_->normalX * _loc9_->normalX) + (_loc9_->normalY * _loc9_->normalY)) + (_loc9_->normalZ * _loc9_->normalZ));
				if (_loc10_ > 0.001)
				{
					_loc10_ = 1. / MyMath::sqrt(_loc10_);
					_loc9_->normalX = (_loc9_->normalX * _loc10_);
					_loc9_->normalY = (_loc9_->normalY * _loc10_);
					_loc9_->normalZ = (_loc9_->normalZ * _loc10_);
				}
			}
			_loc5_->vertex = _loc9_;
			_loc9_->next = this->vertexList;
			this->vertexList = _loc9_;
			_loc5_ = _loc5_->_next;
		}
		_loc3_ = _loc3_->next.get();
	}

	{
		QHashIterator<Vertex*, QSet<Face*>*> i(_loc6_);
		while (i.hasNext())
		{
			i.next();
			delete i.value();
		}
	}
}
