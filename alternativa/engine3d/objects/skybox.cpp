#include "skybox.h"
#include "skybox.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "flash/geom/point.h"
#include "flash/geom/matrix.h"
#include "alternativa/engine3d/core/Sorting.h"
#include "alternativa/engine3d/core/Clipping.h"
#include "alternativa/engine3d/materials/material.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "_global.h"


const Object3D::T *SkyBox::TYPE = new Object3D::T("SkyBox");



//Тестура за картой

SkyBox::SkyBox(const TT &t) :
	Mesh(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{

}

void SkyBox::init(double param1,
				  std::shared_ptr<Material> param2,
				  std::shared_ptr<Material> param3,
				  std::shared_ptr<Material> param4,
				  std::shared_ptr<Material> param5,
				  std::shared_ptr<Material> param6,
				  std::shared_ptr<Material> param7,
				  double param8)
{
	_function_name("SkyBox::init()");

	autoSize = true;

	reduceConst[0] = 0;
	reduceConst[1] = 0;
	reduceConst[2] = 0;
	reduceConst[3] = 1;


	param1 = (param1 * 0.5);
	std::shared_ptr<Vertex> _loc9_ = this->createVertex(-(param1), -(param1), param1, param8, param8);
	std::shared_ptr<Vertex> _loc10_ = this->createVertex(-(param1), -(param1), -(param1), param8, (1 - param8));
	std::shared_ptr<Vertex> _loc11_ = this->createVertex(-(param1), param1, -(param1), (1 - param8), (1 - param8));
	std::shared_ptr<Vertex> _loc12_ = this->createVertex(-(param1), param1, param1, (1 - param8), param8);
	this->leftFace = this->createQuad(_loc9_, _loc10_, _loc11_, _loc12_, param2);
	_loc9_ = this->createVertex(param1, param1, param1, param8, param8);
	_loc10_ = this->createVertex(param1, param1, -(param1), param8, (1 - param8));
	_loc11_ = this->createVertex(param1, -(param1), -(param1), (1 - param8), (1 - param8));
	_loc12_ = this->createVertex(param1, -(param1), param1, (1 - param8), param8);
	this->rightFace = this->createQuad(_loc9_, _loc10_, _loc11_, _loc12_, param3);
	_loc9_ = this->createVertex(param1, -(param1), param1, param8, param8);
	_loc10_ = this->createVertex(param1, -(param1), -(param1), param8, (1 - param8));
	_loc11_ = this->createVertex(-(param1), -(param1), -(param1), (1 - param8), (1 - param8));
	_loc12_ = this->createVertex(-(param1), -(param1), param1, (1 - param8), param8);
	this->backFace = this->createQuad(_loc9_, _loc10_, _loc11_, _loc12_, param4);
	_loc9_ = this->createVertex(-(param1), param1, param1, param8, param8);
	_loc10_ = this->createVertex(-(param1), param1, -(param1), param8, (1 - param8));
	_loc11_ = this->createVertex(param1, param1, -(param1), (1 - param8), (1 - param8));
	_loc12_ = this->createVertex(param1, param1, param1, (1 - param8), param8);
	this->frontFace = this->createQuad(_loc9_, _loc10_, _loc11_, _loc12_, param5);
	_loc9_ = this->createVertex(-(param1), param1, -(param1), param8, param8);
	_loc10_ = this->createVertex(-(param1), -(param1), -(param1), param8, (1 - param8));
	_loc11_ = this->createVertex(param1, -(param1), -(param1), (1 - param8), (1 - param8));
	_loc12_ = this->createVertex(param1, param1, -(param1), (1 - param8), param8);
	this->bottomFace = this->createQuad(_loc9_, _loc10_, _loc11_, _loc12_, param6);
	_loc9_ = this->createVertex(-(param1), -(param1), param1, param8, param8);
	_loc10_ = this->createVertex(-(param1), param1, param1, param8, (1 - param8));
	_loc11_ = this->createVertex(param1, param1, param1, (1 - param8), (1 - param8));
	_loc12_ = this->createVertex(param1, -(param1), param1, (1 - param8), param8);
	this->topFace = this->createQuad(_loc9_, _loc10_, _loc11_, _loc12_, param7);
	calculateBounds();
	calculateFacesNormals(true);
	clipping = Clipping::FACE_CLIPPING;
	sorting = Sorting::NONE;
	shadowMapAlphaThreshold = 0.1;
	useLight = true;

}

Face* SkyBox::getSide(int param1)
{
	switch (param1)
	{
		case LEFT:
			return this->leftFace.get();
		case RIGHT:
			return this->rightFace.get();
		case BACK:
			return this->backFace.get();
		case FRONT:
			return this->frontFace.get();
		case BOTTOM:
			return this->bottomFace.get();
		case TOP:
			return this->topFace.get();
		default:
			return nullptr;
	}
}


void SkyBox::clonePropertiesFromSkyBox(SkyBox *src)
{
	_function_name("SkyBox::prepareResources()");

	Mesh::clonePropertiesFromMesh(src);

	SkyBox *_loc2_ = src;

	this->autoSize = _loc2_->autoSize;
	std::shared_ptr<Face> _loc3_ = _loc2_->faceList();
	std::shared_ptr<Face> _loc4_ = faceList();

	while (_loc3_ != nullptr)
	{
		if (_loc3_ == _loc2_->leftFace)
		{
			this->leftFace = _loc4_;
		}
		else
		{
			if (_loc3_ == _loc2_->rightFace)
			{
				this->rightFace = _loc4_;
			}
			else
			{
				if (_loc3_ == _loc2_->backFace)
				{
					this->backFace = _loc4_;
				}
				else
				{
					if (_loc3_ == _loc2_->frontFace)
					{
						this->frontFace = _loc4_;
					}
					else
					{
						if (_loc3_ == _loc2_->bottomFace)
						{
							this->bottomFace = _loc4_;
						}
						else
						{
							if (_loc3_ == _loc2_->topFace)
							{
								this->topFace = _loc4_;
							}
						}
					}
				}
			}
		}
		_loc3_ = _loc3_->next;
		_loc4_ = _loc4_->next;
	}
}

std::shared_ptr<Vertex> SkyBox::createVertex(double param1, double param2, double param3, double param4, double param5)
{
	_function_name("SkyBox::createVertex()");

	std::shared_ptr<Vertex> _loc6_ = Vertex::create();
	_loc6_->next = vertexList;
	vertexList = _loc6_;
	_loc6_->x = param1;
	_loc6_->y = param2;
	_loc6_->z = param3;
	_loc6_->u = param4;
	_loc6_->v = param5;
	return _loc6_;
}

void SkyBox::draw(Camera3D *param1)
{
	_function_name("SkyBox::draw()");

	if (faceList() == nullptr)
	{
		return;
	}

	if (this->autoSize)
	{
		this->calculateTransform(param1);
	}

	if (clipping == 0)
	{
		if ((culling & 0x01))
		{
			return;
		}
		culling = 0;
	}

	this->prepareResources();
	this->addOpaque(param1);
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
	//var _loc2_:int = ((!(!(param1.debug))) ? int(param1.checkInDebug(this)) : int(0));
	//if ((_loc2_ & Debug.BOUNDS))
	//Debug.drawBounds(param1, this, this->bound.MinX, this->bound.MinY, this->bound.MinZ, this->bound.MaxX, this->bound.MaxY, this->bound.MaxZ);
}


void SkyBox::prepareResources()
{
	_function_name("SkyBox::prepareResources()");

	if (vertexBuffer != nullptr)
		return;

	QVector<double> vertex;

	int _loc3_ = 0;
	Vertex *_loc4_ = vertexList.get();
	while (_loc4_ != nullptr)
	{
		vertex.append(_loc4_->x);
		vertex.append(_loc4_->y);
		vertex.append(_loc4_->z);
		vertex.append(_loc4_->u);
		vertex.append(_loc4_->v);
		vertex.append(_loc4_->normalX);
		vertex.append(_loc4_->normalY);
		vertex.append(_loc4_->normalZ);
		_loc4_->index = _loc3_;
		_loc3_++;
		_loc4_ = _loc4_->next.get();
	}

	if (_loc3_ > 0)
	{
		vertexBuffer = std::make_shared<VertexBufferResource>(vertex, 8);
	}



	QHash<Material*, QVector<Face*>*> hash;

	Face *face = this->faceList().get();
	while (face != nullptr)
	{
		if (face->material != nullptr)
		{
			QVector<Face*> *list = hash[face->material.get()];
			if (list == nullptr)
			{
				list = new QVector<Face*>();
				hash.insert(face->material.get(), list);
			}
			list->append(face);
		}
		face = face->next.get();
	}

	QVector<uint32_t> index;
	int count_index = 0;

	QHashIterator<Material*, QVector<Face*>*> i(hash);
	while (i.hasNext())
	{
		i.next();

		auto key = i.key();
		auto _loc9_ = i.value();

		if (opaqueMaterials.length() < (opaqueLength+1))
			opaqueMaterials.resize(opaqueLength+1);

		if (opaqueBegins.length() < (opaqueLength+1))
			opaqueBegins.resize(opaqueLength+1);

		opaqueMaterials[opaqueLength] = key;
		opaqueBegins[opaqueLength] = numTriangles * 3;

		for (Face *_loc8_ : *_loc9_)
		{
			Wrapper *_loc10_ = _loc8_->wrapper.get();
			int index_1 = _loc10_->vertex->index;
			_loc10_ = _loc10_->_next;
			int index_2 = _loc10_->vertex->index;
			_loc10_ = _loc10_->_next;
			while (_loc10_ != nullptr)
			{
				int index_3 = _loc10_->vertex->index;
				index.append(index_1);
				index.append(index_2);
				index.append(index_3);
				count_index++;
				index_2 = index_3;
				numTriangles++;
				_loc10_ = _loc10_->_next;
			}
		}
		if (opaqueNums.length() < (opaqueLength+1))
			opaqueNums.resize(opaqueLength+1);

		opaqueNums[opaqueLength] = (numTriangles - (opaqueBegins[opaqueLength] / 3));
		opaqueLength++;
	}
	if (count_index > 0)
	{
		indexBuffer = std::make_shared<IndexBufferResource>(index);
	}
}

void SkyBox::addOpaque(Camera3D *param1)
{
	_function_name("SkyBox::addOpaque()");

	int i = 0;
	while (i < opaqueLength)
	{
		param1->addSky(opaqueMaterials[i], vertexBuffer.get(), indexBuffer.get(), opaqueBegins[i], opaqueNums[i], this);
		i++;
	}
}

object_ptr<VG> SkyBox::getVG(Camera3D *param1)
{
	_function_name("SkyBox::getVG()");

	this->draw(param1);
	return nullptr;
}

int SkyBox::cullingInCamera(Camera3D *param1, int param2)
{
	_function_name("SkyBox::cullingInCamera()");

	return Mesh::cullingInCamera(param1, (param2 = int((param2 & (~(0x03))))));
}

void SkyBox::transformUV(int direction, const Matrix *matrix)
{
	_function_name("SkyBox::transformUV()");

	Face *face = this->getSide(direction);
	if (face != nullptr)
	{
		Wrapper *wrapper = face->wrapper.get();
		while (wrapper != nullptr)
		{
			Vertex *vertex = wrapper->vertex.get();
			Point in(vertex->u, vertex->v);
			Point point;
			matrix->transformPoint(&in, &point);
			vertex->u = point.x;
			vertex->v = point.y;
			wrapper = wrapper->_next;
		}
	}
}

void SkyBox::calculateTransform(Camera3D *param1)
{
	_function_name("SkyBox::calculateTransform()");

	double _loc2_ = ((((this->matrix.mi * this->bound.MinX) + (this->matrix.mj * this->bound.MinY)) + (this->matrix.mk * this->bound.MinZ)) + this->matrix.ml);
	double _loc3_ = _loc2_;
	_loc2_ = ((((this->matrix.mi * this->bound.MaxX) + (this->matrix.mj * this->bound.MinY)) + (this->matrix.mk * this->bound.MinZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	_loc2_ = ((((this->matrix.mi * this->bound.MaxX) + (this->matrix.mj * this->bound.MaxY)) + (this->matrix.mk * this->bound.MinZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	_loc2_ = ((((this->matrix.mi * this->bound.MinX) + (this->matrix.mj * this->bound.MaxY)) + (this->matrix.mk * this->bound.MinZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	_loc2_ = ((((this->matrix.mi * this->bound.MinX) + (this->matrix.mj * this->bound.MinY)) + (this->matrix.mk * this->bound.MaxZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	_loc2_ = ((((this->matrix.mi * this->bound.MaxX) + (this->matrix.mj * this->bound.MinY)) + (this->matrix.mk * this->bound.MaxZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	_loc2_ = ((((this->matrix.mi * this->bound.MaxX) + (this->matrix.mj * this->bound.MaxY)) + (this->matrix.mk * this->bound.MaxZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	_loc2_ = ((((this->matrix.mi * this->bound.MinX) + (this->matrix.mj * this->bound.MaxY)) + (this->matrix.mk * this->bound.MaxZ)) + this->matrix.ml);
	if (_loc2_ > _loc3_)
	{
		_loc3_ = _loc2_;
	}
	double _loc4_ = 1;
	if (_loc3_ > param1->farClipping)
	{
		_loc4_ = param1->farClipping / _loc3_;
	}
	this->reduceConst[0] = _loc4_;
	this->reduceConst[1] = _loc4_;
	this->reduceConst[2] = _loc4_;
}
/*
 *
 * override public function clone():Object3D
			{
				var _loc1_:SkyBox = new SkyBox(0);
				_loc1_->clonePropertiesFrom(this);
				return (_loc1_);
			}
			*/

std::shared_ptr<Face> SkyBox::createQuad(std::shared_ptr<Vertex> v1,
										 std::shared_ptr<Vertex> v2,
										 std::shared_ptr<Vertex> v3,
										 std::shared_ptr<Vertex> v4,
										 std::shared_ptr<Material> material)
{
	_function_name("SkyBox::createQuad()");

	std::shared_ptr<Face> _loc6_ = Face::create();
	_loc6_->material = material;
	_loc6_->next = faceList();
	this->setFaceList(_loc6_, 22);
	_loc6_->wrapper = Wrapper::create(43);
	_loc6_->wrapper->vertex = v1;
	_loc6_->wrapper->setNext(Wrapper::create(44));
	_loc6_->wrapper->_next->vertex = v2;
	_loc6_->wrapper->_next->setNext(Wrapper::create(45));
	_loc6_->wrapper->_next->_next->vertex = v3;
	_loc6_->wrapper->_next->_next->setNext(Wrapper::create(46));
	_loc6_->wrapper->_next->_next->_next->vertex = v4;
	return _loc6_;
}

std::shared_ptr<SkyBox> new_SkyBox()
{
	return std::make_shared<SkyBox>(Object3D::TT(SkyBox::TYPE));
}
