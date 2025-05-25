#include "bsp.h"
#include "mesh.h"
#include "../core/face.h"
#include "../core/vertex.h"
#include "../core/wrapper.h"
#include <QVector>
#include "_global.h"
#include "../core/object3d_clone.h"
#include <QHash>
#include "alternativa/engine3d/materials/material.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "alternativa/engine3d/core/camera3d.h"
#include "alternativa/engine3d/core/view.h"
#include "alternativa/engine3d/core/vg.h"
#include "mymath.h"
#include "../core/Object3DFilter.h"
#include "../core/rayintersectiondata.h"



const Object3D::T *BSP::TYPE = new Object3D::T("BSP");

class BSP_Node
{
public:
	BSP_Node()
	{
		negative = nullptr;
		positive = nullptr;
		faceList = nullptr;
		normalX = 0.;
		normalY = 0.;
		normalZ = 0.;
		offset = 0.;
	}

	std::shared_ptr<BSP_Node> negative;
	std::shared_ptr<BSP_Node> positive;
	std::shared_ptr<Face> faceList;
	double normalX;
	double normalY;
	double normalZ;
	double offset;
};


static std::shared_ptr<BSP_Node> cloneNode(BSP_Node*, QHash<Face*, Face*>&);


BSP::BSP(const TT &t) :
	Object3D(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	_function_name("BSP::BSP");

	faces = new QVector<std::shared_ptr<Face>>;
	root = nullptr;
	vertexBuffer = nullptr;
	numTriangles = 0;
	splitAnalysis = true;
	threshold = 0.01;
	clipping = 2;
}

BSP::~BSP()
{
	this->destroy();
	delete faces;
}

void BSP::createTree(std::shared_ptr<Mesh> param1, bool param2)
{
	_function_name("BSP::createTree");

	this->destroyTree();

	std::shared_ptr<Mesh> m;

	if (!param2) {
		m = Mesh_clone(param1);
		param1 = m;
	}

	param1->optimizeForDynamicBSP(2);
	std::shared_ptr<Face> f = param1->faceList();
	this->vertexList = param1->vertexList;
	param1->setFaceList(nullptr, 2);
	param1->vertexList = nullptr;
	std::shared_ptr<Vertex> _loc4_ = this->vertexList;
	while (_loc4_ != nullptr)
	{
		_loc4_->transformId = 0;
		//_loc4_->id = nullptr;
		_loc4_ = _loc4_->next;
	}

	//int _loc5_ = 0;
	std::shared_ptr<Face> face = f;
	while (face != nullptr)
	{
		face->calculateBestSequenceAndNormal();
		//face->id = nullptr;
		this->faces->append(face);
		//_loc5_++;
		face = face->next;
	}

	if (f != nullptr)
		this->root = this->createNode(f);
	calculateBounds();
}

void BSP::destroyTree()
{
	_function_name("BSP::destroyTree");
	this->deleteResources();
	this->vertexList = nullptr;
	if (this->root != nullptr)
	{
		this->destroyNode(this->root);
		this->root = nullptr;
	}
	this->faces->clear();
}


void BSP::deleteResources()
{
	_function_name("BSP::deleteResources");
	if (this->vertexBuffer != nullptr)
	{
		//this->vertexBuffer.dispose();
		//this->vertexBuffer = null;
		//this->indexBuffer.dispose();
		//this->indexBuffer = null;
		this->numTriangles = 0;
	}
}





std::shared_ptr<BSP_Node> BSP::createNode(std::shared_ptr<Face> &param1)
{
	//_function_name("BSP::createNode");

	std::shared_ptr<Face> _loc21_ = nullptr;
	std::shared_ptr<Face> _loc22_ = nullptr;
	std::shared_ptr<Face> _loc25_ = nullptr;
	std::shared_ptr<Face> _loc26_ = nullptr;


	std::shared_ptr<BSP_Node> _loc2_ = std::make_shared<BSP_Node>();
	std::shared_ptr<Face> _loc20_ = param1;

	if (((this->splitAnalysis) && (!(param1->next == nullptr))))
	{
		int _loc27_ = 2147483647;
		std::shared_ptr<Face> _loc28_ = param1;
		while (_loc28_ != nullptr)
		{
			double _loc14_ = _loc28_->normalX;
			double _loc15_ = _loc28_->normalY;
			double _loc16_ = _loc28_->normalZ;
			double _loc17_ = _loc28_->offset;
			double _loc18_ = (_loc17_ - this->threshold);
			double _loc19_ = (_loc17_ + this->threshold);
			int _loc29_ = 0;
			std::shared_ptr<Face> _loc30_ = param1;
			while (_loc30_ != nullptr)
			{
				if (_loc30_ != _loc28_)
				{
					Wrapper *_loc3_ = _loc30_->wrapper.get();
					std::shared_ptr<Vertex> _loc4_ = _loc3_->vertex;
					_loc3_ = _loc3_->_next;
					std::shared_ptr<Vertex> _loc5_ = _loc3_->vertex;
					_loc3_ = _loc3_->_next;
					std::shared_ptr<Vertex> _loc6_ = _loc3_->vertex;
					_loc3_ = _loc3_->_next;
					double _loc10_ = (((_loc4_->x * _loc14_) + (_loc4_->y * _loc15_)) + (_loc4_->z * _loc16_));
					double _loc11_ = (((_loc5_->x * _loc14_) + (_loc5_->y * _loc15_)) + (_loc5_->z * _loc16_));
					double _loc12_ = (((_loc6_->x * _loc14_) + (_loc6_->y * _loc15_)) + (_loc6_->z * _loc16_));
					bool _loc8_ = (((_loc10_ < _loc18_) || (_loc11_ < _loc18_)) || (_loc12_ < _loc18_));
					bool _loc9_ = (((_loc10_ > _loc19_) || (_loc11_ > _loc19_)) || (_loc12_ > _loc19_));
					while (_loc3_ != nullptr)
					{
						std::shared_ptr<Vertex> _loc7_ = _loc3_->vertex;
						double _loc13_ = (((_loc7_->x * _loc14_) + (_loc7_->y * _loc15_)) + (_loc7_->z * _loc16_));
						if (_loc13_ < _loc18_)
						{
							_loc8_ = true;
							if (_loc9_)
							{
								break;
							}
						}
						else
						{
							if (_loc13_ > _loc19_)
							{
								_loc9_ = true;
								if (_loc8_)
								{
									break;
								}
							}
						}
						_loc3_ = _loc3_->_next;
					}
					if (((_loc9_) && (_loc8_)))
					{
						_loc29_++;
						if (_loc29_ >= _loc27_)
						{
							break;
						}
					}
				}
				_loc30_ = _loc30_->next;
			}
			if (_loc29_ < _loc27_)
			{
				_loc20_ = _loc28_;
				_loc27_ = _loc29_;
				if (_loc27_ == 0)
				{
					break;
				}
			}
			_loc28_ = _loc28_->next;
		}
	}
	std::shared_ptr<Face> _loc23_ = _loc20_;
	std::shared_ptr<Face> _loc24_ = _loc20_->next;
	double _loc14_ = _loc20_->normalX;
	double _loc15_ = _loc20_->normalY;
	double _loc16_ = _loc20_->normalZ;
	double _loc17_ = _loc20_->offset;
	double _loc18_ = (_loc17_ - this->threshold);
	double _loc19_ = (_loc17_ + this->threshold);
	while (param1 != nullptr)
	{
		if (param1 != _loc20_)
		{
			std::shared_ptr<Face> _loc31_ = param1->next;
			Wrapper *_loc3_ = param1->wrapper.get();
			std::shared_ptr<Vertex> _loc4_ = _loc3_->vertex;
			_loc3_ = _loc3_->_next;
			std::shared_ptr<Vertex> _loc5_ = _loc3_->vertex;
			_loc3_ = _loc3_->_next;
			std::shared_ptr<Vertex> _loc6_ = _loc3_->vertex;
			_loc3_ = _loc3_->_next;
			double _loc10_ = (((_loc4_->x * _loc14_) + (_loc4_->y * _loc15_)) + (_loc4_->z * _loc16_));
			double _loc11_ = (((_loc5_->x * _loc14_) + (_loc5_->y * _loc15_)) + (_loc5_->z * _loc16_));
			double _loc12_ = (((_loc6_->x * _loc14_) + (_loc6_->y * _loc15_)) + (_loc6_->z * _loc16_));
			bool _loc8_ = (((_loc10_ < _loc18_) || (_loc11_ < _loc18_)) || (_loc12_ < _loc18_));
			bool _loc9_ = (((_loc10_ > _loc19_) || (_loc11_ > _loc19_)) || (_loc12_ > _loc19_));
			while (_loc3_ != nullptr)
			{
				std::shared_ptr<Vertex> _loc7_ = _loc3_->vertex;
				double _loc13_ = (((_loc7_->x * _loc14_) + (_loc7_->y * _loc15_)) + (_loc7_->z * _loc16_));
				if (_loc13_ < _loc18_)
				{
					_loc8_ = true;
				}
				else
				{
					if (_loc13_ > _loc19_)
					{
						_loc9_ = true;
					}
				}
				_loc7_->offset = _loc13_;
				_loc3_ = _loc3_->_next;
			}
			if ((!(_loc8_)))
			{
				if ((!(_loc9_)))
				{
					if ((((param1->normalX * _loc14_) + (param1->normalY * _loc15_)) + (param1->normalZ * _loc16_)) > 0)
					{
						_loc23_->next = param1;
						_loc23_ = param1;
					}
					else
					{
						if (_loc21_ != nullptr)
						{
							_loc22_->next = param1;
						}
						else
						{
							_loc21_ = param1;
						}
						_loc22_ = param1;
					}
				}
				else
				{
					if (_loc25_ != nullptr)
					{
						_loc26_->next = param1;
					}
					else
					{
						_loc25_ = param1;
					}
					_loc26_ = param1;
				}
			}
			else
			{
				if ((!(_loc9_)))
				{
					if (_loc21_ != nullptr)
					{
						_loc22_->next = param1;
					}
					else
					{
						_loc21_ = param1;
					}
					_loc22_ = param1;
				}
				else
				{
					_loc4_->offset = _loc10_;
					_loc5_->offset = _loc11_;
					_loc6_->offset = _loc12_;
					std::shared_ptr<Face> _loc32_ = Face::create();
					std::shared_ptr<Face> _loc33_ = Face::create();
					std::shared_ptr<Wrapper> _loc34_ = nullptr;
					std::shared_ptr<Wrapper> _loc35_ = nullptr;
					Wrapper *_loc3_ = param1->wrapper->_next->_next;
					while (_loc3_->_next != nullptr)
					{
						_loc3_ = _loc3_->_next;
					}
					std::shared_ptr<Vertex> _loc4_ = _loc3_->vertex;
					double _loc10_ = _loc4_->offset;
					_loc3_ = param1->wrapper.get();
					while (_loc3_ != nullptr)
					{
						_loc5_ = _loc3_->vertex;
						double _loc11_ = _loc5_->offset;
						if ((((_loc10_ < _loc18_) && (_loc11_ > _loc19_)) || ((_loc10_ > _loc19_) && (_loc11_ < _loc18_))))
						{
							double _loc37_ = ((_loc17_ - _loc10_) / (_loc11_ - _loc10_));
							std::shared_ptr<Vertex> _loc7_ = Vertex::create();
							_loc7_->next = this->vertexList;
							this->vertexList = _loc7_;
							_loc7_->x = (_loc4_->x + ((_loc5_->x - _loc4_->x) * _loc37_));
							_loc7_->y = (_loc4_->y + ((_loc5_->y - _loc4_->y) * _loc37_));
							_loc7_->z = (_loc4_->z + ((_loc5_->z - _loc4_->z) * _loc37_));
							_loc7_->u = (_loc4_->u + ((_loc5_->u - _loc4_->u) * _loc37_));
							_loc7_->v = (_loc4_->v + ((_loc5_->v - _loc4_->v) * _loc37_));
							_loc7_->normalX = (_loc4_->normalX + ((_loc5_->normalX - _loc4_->normalX) * _loc37_));
							_loc7_->normalY = (_loc4_->normalY + ((_loc5_->normalY - _loc4_->normalY) * _loc37_));
							_loc7_->normalZ = (_loc4_->normalZ + ((_loc5_->normalZ - _loc4_->normalZ) * _loc37_));
							std::shared_ptr<Wrapper> _loc36_ = Wrapper::create(16);
							_loc36_->vertex = _loc7_;
							if (_loc34_ != nullptr)
							{
								_loc34_->setNext(_loc36_);
							}
							else
							{
								_loc32_->wrapper = _loc36_;
							}
							_loc34_ = _loc36_;
							_loc36_ = Wrapper::create(17);
							_loc36_->vertex = _loc7_;
							if (_loc35_ != nullptr)
							{
								_loc35_->setNext(_loc36_);
							}
							else
							{
								_loc33_->wrapper = _loc36_;
							}
							_loc35_ = _loc36_;
						}
						if (_loc11_ <= _loc19_)
						{
							std::shared_ptr<Wrapper> _loc36_ = Wrapper::create(18);
							_loc36_->vertex = _loc5_;
							if (_loc34_ != nullptr)
							{
								_loc34_->setNext(_loc36_);
							}
							else
							{
								_loc32_->wrapper = _loc36_;
							}
							_loc34_ = _loc36_;
						}
						if (_loc11_ >= _loc18_)
						{
							std::shared_ptr<Wrapper> _loc36_ = Wrapper::create(19);
							_loc36_->vertex = _loc5_;
							if (_loc35_ != nullptr)
							{
								_loc35_->setNext(_loc36_);
							}
							else
							{
								_loc33_->wrapper = _loc36_;
							}
							_loc35_ = _loc36_;
						}
						_loc4_ = _loc5_;
						_loc10_ = _loc11_;
						_loc3_ = _loc3_->_next;
					}
					_loc32_->material = param1->material;
					_loc32_->smoothingGroups = param1->smoothingGroups;
					_loc32_->calculateBestSequenceAndNormal();
					if (_loc21_ != nullptr)
					{
						_loc22_->next = _loc32_;
					}
					else
					{
						_loc21_ = _loc32_;
					}
					_loc22_ = _loc32_;
					_loc33_->material = param1->material;
					_loc33_->smoothingGroups = param1->smoothingGroups;
					_loc33_->calculateBestSequenceAndNormal();
					if (_loc25_ != nullptr)
					{
						_loc26_->next = _loc33_;
					}
					else
					{
						_loc25_ = _loc33_;
					}
					_loc26_ = _loc33_;
				}
			}
			param1 = _loc31_;
		}
		else
		{
			param1 = _loc24_;
		}
	}
	if (_loc21_ != nullptr)
	{
		_loc22_->next = nullptr;
		_loc2_->negative = this->createNode(_loc21_);
	}
	_loc23_->next = nullptr;
	_loc2_->faceList = _loc20_;
	_loc2_->normalX = _loc14_;
	_loc2_->normalY = _loc15_;
	_loc2_->normalZ = _loc16_;
	_loc2_->offset = _loc17_;
	if (_loc25_ != nullptr)
	{
		_loc26_->next = nullptr;
		_loc2_->positive = this->createNode(_loc25_);
	}
	return _loc2_;
}




void BSP::destroyNode(std::shared_ptr<BSP_Node> &node)
{
	if (node->negative != nullptr)
	{
		this->destroyNode(node->negative);
		node->negative = nullptr;
	}
	if (node->positive != nullptr)
	{
		this->destroyNode(node->positive);
		node->positive = nullptr;
	}
	std::shared_ptr<Face> _loc2_ = node->faceList;
	while (_loc2_ != nullptr)
	{
		std::shared_ptr<Face> _loc3_ = _loc2_->next;
		//_loc2_->next->destroy();
		_loc2_->next = nullptr;
		_loc2_ = _loc3_;
	}
}

void BSP::setMaterialToNode(BSP_Node *node, Material *material)
{
	Face *face = node->faceList.get();
	while (face != nullptr)
	{
		face->material = material->ptr.lock();
		face = face->next.get();
	}
	if (node->negative != nullptr)
	{
		this->setMaterialToNode(node->negative.get(), material);
	}
	if (node->positive != nullptr)
	{
		this->setMaterialToNode(node->positive.get(), material);
	}
}


void BSP::clonePropertiesFromBSP(BSP *src)
{

	clonePropertiesFromObject3D(src);

	this->clipping = src->clipping;
	this->threshold = src->threshold;
	this->splitAnalysis = src->splitAnalysis;

	Vertex *vertex_temp = nullptr;

	Vertex *vertex_src = src->vertexList.get();
	while (vertex_src != nullptr)
	{
		std::shared_ptr<Vertex> vertex = Vertex::create();
		vertex->x = vertex_src->x;
		vertex->y = vertex_src->y;
		vertex->z = vertex_src->z;
		vertex->u = vertex_src->u;
		vertex->v = vertex_src->v;
		vertex->normalX = vertex_src->normalX;
		vertex->normalY = vertex_src->normalY;
		vertex->normalZ = vertex_src->normalZ;

		//vertex->value = vertex_src->value;

		vertex_src->value = vertex; //error?

		if (vertex_temp != nullptr)
		{
			vertex_temp->next = vertex;
		}
		else
		{
			this->vertexList = vertex;
		}
		vertex_temp = vertex.get();
		vertex_src = vertex_src->next.get();
	}


	//Dictionary *_loc5_ = new Dictionary();
	QHash<Face*, Face*> _loc5_;
	int _loc6_ = src->faces->length();

	this->faces->reserve(_loc6_);

	int _loc7_ = 0;
	while (_loc7_ < _loc6_)
	{
		Face *_loc9_ = src->faces->at(_loc7_).get();
		std::shared_ptr<Face> _loc10_ = Face::create();
		_loc10_->material = _loc9_->material;
		_loc10_->smoothingGroups = _loc9_->smoothingGroups;
		_loc10_->normalX = _loc9_->normalX;
		_loc10_->normalY = _loc9_->normalY;
		_loc10_->normalZ = _loc9_->normalZ;
		_loc10_->offset = _loc9_->offset;

		Wrapper *wrapper_temp = nullptr;
		Wrapper *wrapper_src = _loc9_->wrapper.get();

		while (wrapper_src != nullptr)
		{
			std::shared_ptr<Wrapper> wrapper = Wrapper::create(20);
			wrapper->vertex = wrapper_src->vertex->value;
			if (wrapper_temp != nullptr)
			{
				wrapper_temp->setNext(wrapper);
			}
			else
			{
				_loc10_->wrapper = wrapper;
			}
			wrapper_temp = wrapper.get();
			wrapper_src = wrapper_src->_next;
		}

		this->faces->append(_loc10_);
		//this->faces[_loc7_] = _loc10_;

		_loc5_.insert(_loc9_, _loc10_.get());

		_loc7_++;
	}

	if (src->root != nullptr)
	{
		this->root = cloneNode(src->root.get(), _loc5_);
	}

	vertex_temp = src->vertexList.get();

	while (vertex_temp != nullptr)
	{
		vertex_temp->value = nullptr;
		vertex_temp = vertex_temp->next.get();
	}

}

void BSP::setMaterialToAllFaces(Material *material)
{
	this->deleteResources();
	int _loc2_ = this->faces->length();
	int i = 0;
	while (i < _loc2_)
	{
		Face *face = this->faces->at(i).get();
		face->material = material->ptr.lock();
		i++;
	}
	if (this->root != nullptr)
	{
		this->setMaterialToNode(this->root.get(), material);
	}
}

void BSP::prepareResources()
{
	if (this->vertexBuffer == nullptr)
	{
		QVector<double> vector_vertex;

		int index = 0;
		Vertex *temp = this->vertexList.get();
		while (temp != nullptr)
		{
			vector_vertex.append(temp->x);
			vector_vertex.append(temp->y);
			vector_vertex.append(temp->z);
			vector_vertex.append(temp->u);
			vector_vertex.append(temp->v);
			vector_vertex.append(temp->normalX);
			vector_vertex.append(temp->normalY);
			vector_vertex.append(temp->normalZ);
			temp->index = index;
			index++;
			temp = temp->next.get();
		}

		this->vertexBuffer = std::make_shared<VertexBufferResource>(vector_vertex, 8);
		QVector<uint> vector_index;
		this->numTriangles = 0;

		for (std::shared_ptr<Face> &face : *this->faces)
		{
			Wrapper *temp = face->wrapper.get();
			uint index_1 = temp->vertex->index;
			temp = temp->_next;
			uint index_2 = temp->vertex->index;
			temp = temp->_next;
			while (temp != nullptr)
			{
				uint index_3 = temp->vertex->index;
				vector_index.append(index_1);
				vector_index.append(index_2);
				vector_index.append(index_3);
				index_2 = index_3;
				this->numTriangles++;
				temp = temp->_next;
			}
		}

		this->indexBuffer = std::make_shared<IndexBufferResource>(vector_index);
	}
}



std::shared_ptr<Face> BSP::collectNode(BSP_Node *param1, std::shared_ptr<Face> param2)
{
	if ((((param1->normalX * this->matrix_i.md) + (param1->normalY * this->matrix_i.mh)) + (param1->normalZ * this->matrix_i.ml)) > param1->offset)
	{
		if (param1->positive != nullptr)
		{
			param2 = this->collectNode(param1->positive.get(), param2);
		}

		std::shared_ptr<Face> _loc3_ = param1->faceList;
		while (_loc3_ != nullptr)
		{
			Wrapper *_loc4_ = _loc3_->wrapper.get();
			while (_loc4_ != nullptr)
			{
				Vertex *_loc5_ = _loc4_->vertex.get();
				if (_loc5_->transformId != transformId)
				{
					double _loc6_ = _loc5_->x;
					double _loc7_ = _loc5_->y;
					double _loc8_ = _loc5_->z;
					_loc5_->cameraX = ((((this->matrix.ma * _loc6_) + (this->matrix.mb * _loc7_)) + (this->matrix.mc * _loc8_)) + this->matrix.md);
					_loc5_->cameraY = ((((this->matrix.me * _loc6_) + (this->matrix.mf * _loc7_)) + (this->matrix.mg * _loc8_)) + this->matrix.mh);
					_loc5_->cameraZ = ((((this->matrix.mi * _loc6_) + (this->matrix.mj * _loc7_)) + (this->matrix.mk * _loc8_)) + this->matrix.ml);
					_loc5_->transformId = transformId;
					_loc5_->drawId = 0;
				}
				_loc4_ = _loc4_->_next;
			}
			_loc3_->processNext = param2;
			param2 = _loc3_;
			_loc3_ = _loc3_->next;
		}
		if (param1->negative != nullptr)
		{
			param2 = this->collectNode(param1->negative.get(), param2);
		}
	}
	else
	{
		if (param1->negative != nullptr)
		{
			param2 = this->collectNode(param1->negative.get(), param2);
		}
		if (param1->positive != nullptr)
		{
			param2 = this->collectNode(param1->positive.get(), param2);
		}
	}
	return param2;
}


void BSP::draw(Camera3D *param1)
{
	std::shared_ptr<Face> _loc6_;

	if (this->root == nullptr)
	{
		return;
	}

	if (this->clipping == 0)
	{
		if ((culling & 0x01))
		{
			return;
		}
		culling = 0;
	}

	this->prepareResources();
	if ((useDepth = (((!(param1->view->constrained)) && ((((param1->softTransparency) && (param1->softTransparencyStrength > 0)) || ((param1->ssao) && (param1->ssaoStrength > 0))) || ((param1->deferredLighting) && (param1->deferredLightingStrength > 0)))) && (concatenatedAlpha >= depthMapAlphaThreshold))))
	{
		param1->depthObjects[param1->depthCount] = this;
		param1->depthCount++;
	}

	//int _loc2_ = 0; //((!(!(param1->debug))) ? int(param1->checkInDebug(this)) : int(0));

	std::shared_ptr<Face> _loc3_ = this->faces->at(0);

	if (((((concatenatedAlpha >= 1) && (concatenatedBlendMode == BlendMode::NORMAL)) && (!(_loc3_->material == nullptr))) && ((!(_loc3_->material->transparent())) || (_loc3_->material->alphaTestThreshold > 0))))
	{
		param1->addOpaque(_loc3_->material.get(), this->vertexBuffer.get(), this->indexBuffer.get(), 0, this->numTriangles, this);
		/*
		if (_loc2_ > 0)
		{
			if ((_loc2_ & Debug.EDGES))
			{
				Debug.drawEdges(param1, null, 0xFFFFFF);
			}
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
			}
		}
		*/
	}
	else
	{
		if (transformId > 0x1DCD6500)
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
		std::shared_ptr<Face> _loc4_ = this->collectNode(this->root.get());

		if (_loc4_ == nullptr)
		{
			return;
		}

		if (culling > 0)
		{
			if (this->clipping == 1)
			{
				_loc4_ = param1->cull(_loc4_, culling);
			}
			else
			{
				_loc4_ = param1->clip(_loc4_, culling);
			}
			if (_loc4_ == nullptr)
			{
				return;
			}
		}

		/*
		if (_loc2_ > 0)
		{
			if ((_loc2_ & Debug.EDGES))
			{
				Debug.drawEdges(param1, _loc4_, 0xFFFFFF);
			}
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
			}
		}
		*/

		std::shared_ptr<Face> _loc7_ = _loc4_;
		while (_loc7_ != nullptr)
		{
			std::shared_ptr<Face> _loc5_ = _loc7_->processNext;
			if (((_loc5_ == nullptr) || (!(_loc5_->material == _loc4_->material))))
			{
				_loc7_->processNext = nullptr;
				if (_loc4_->material != nullptr)
				{
					_loc4_->processNegative = _loc6_;
					_loc6_ = _loc4_;
				}
				else
				{
					while (_loc4_ != nullptr)
					{
						_loc7_ = _loc4_->processNext;
						_loc4_->processNext = nullptr;
						_loc4_ = _loc7_;
					}
				}
				_loc4_ = _loc5_;
			}
			_loc7_ = _loc5_;
		}
		_loc4_ = _loc6_;
		while (_loc4_ != nullptr)
		{
			std::shared_ptr<Face> next = _loc4_->processNegative;
			_loc4_->processNegative = nullptr;

			std::shared_ptr<Object3D> object = this->ptr();
			param1->addTransparent(_loc4_, object);
			_loc4_ = next;
		}
	}

	transformConst[0] = matrix.ma;
	transformConst[1] = matrix.mb;
	transformConst[2] = matrix.mc;
	transformConst[3] = matrix.md;
	transformConst[4] = matrix.me;
	transformConst[5] = matrix.mf;
	transformConst[6] = matrix.mg;
	transformConst[7] = matrix.mh;
	transformConst[8] = matrix.mi;
	transformConst[9] = matrix.mj;
	transformConst[10] = matrix.mk;
	transformConst[11] = matrix.ml;
}

object_ptr<VG> BSP::getVG(Camera3D *param1)
{
	if (this->root == nullptr)
	{
		return nullptr;
	}

	if (this->clipping == 0)
	{
		if (culling & 0x01)
		{
			return nullptr;
		}
		culling = 0;
	}

	this->prepareResources();
	if ((useDepth = (((!(param1->view->constrained)) && ((((param1->softTransparency) && (param1->softTransparencyStrength > 0)) || ((param1->ssao) && (param1->ssaoStrength > 0))) || ((param1->deferredLighting) && (param1->deferredLightingStrength > 0)))) && (concatenatedAlpha >= depthMapAlphaThreshold))))
	{
		if (param1->depthObjects.length() < (param1->depthCount+1))
			param1->depthObjects.resize(param1->depthCount+1);

		param1->depthObjects[param1->depthCount] = this;
		param1->depthCount++;
	}

	//int _loc2_ = ((!(!(param1->debug))) ? int(param1->checkInDebug(this)) : int(0));
	std::shared_ptr<Face> _loc3_ = this->faces->at(0);

	if (((((concatenatedAlpha >= 1) && (concatenatedBlendMode == BlendMode::NORMAL)) && (!(_loc3_->material == nullptr))) && ((!(_loc3_->material->transparent())) || (_loc3_->material->alphaTestThreshold > 0))))
	{
		param1->addOpaque(_loc3_->material.get(), this->vertexBuffer.get(), this->indexBuffer.get(), 0, this->numTriangles, this);
		/*
		if (_loc2_ > 0)
		{
			if ((_loc2_ & Debug.EDGES))
			{
				Debug.drawEdges(param1, null, 0xFFFFFF);
			}
			if ((_loc2_ & Debug.BOUNDS))
			{
				Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);
			}
		}
		*/
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
	std::shared_ptr<Face> face = this->prepareNode(this->root.get(), culling, param1);
	if (face != nullptr)
	{
		std::shared_ptr<Object3D> object = this->ptr();
		return VG::create(object, face, 3, 0, false);
	}

	return nullptr;
}

void BSP::destroy()
{
	if (this->vertexList != nullptr)
	{
		while (this->vertexList->next != nullptr)
		{
			std::shared_ptr<Vertex> temp = this->vertexList;
			this->vertexList = nullptr;
			this->vertexList = temp->next;
		}
		this->vertexList = nullptr;
	}
	if (this->faces != nullptr)
	{
		for (std::shared_ptr<Face> &f : *this->faces)
		{
			f->destroy();
		}
		this->faces->clear();
	}
}




static std::shared_ptr<BSP_Node> cloneNode(BSP_Node *param1, QHash<Face*, Face*> &param2)
{
	Face *_loc4_ = nullptr;

	std::shared_ptr<BSP_Node> _loc3_ = std::make_shared<BSP_Node>();

	std::shared_ptr<Face> _loc5_ = param1->faceList;

	while (_loc5_ != nullptr)
	{
		std::shared_ptr<Face> _loc6_;

		Face *_loc6_1 = param2.value(_loc5_.get());

		if (_loc6_1 == nullptr)
		{
			_loc6_ = Face::create();
			_loc6_->material = _loc5_->material;
			_loc6_->normalX = _loc5_->normalX;
			_loc6_->normalY = _loc5_->normalY;
			_loc6_->normalZ = _loc5_->normalZ;
			_loc6_->offset = _loc5_->offset;

			Wrapper *wrapper_temp = nullptr;
			Wrapper *wrapper_src = _loc5_->wrapper.get();

			while (wrapper_src != nullptr)
			{
				std::shared_ptr<Wrapper> wrapper = Wrapper::create(21);
				wrapper->vertex = wrapper_src->vertex->value;
				if (wrapper_temp != nullptr)
				{
					wrapper_temp->setNext(wrapper);
				}
				else
				{
					_loc6_->wrapper = wrapper;
				}
				wrapper_temp = wrapper.get();
				wrapper_src = wrapper_src->_next;
			}
		}
		else
		{
			_loc6_ = _loc6_1->ptr();
		}

		if (_loc3_->faceList != nullptr)
		{
			_loc4_->next = _loc6_;
		}
		else
		{
			_loc3_->faceList = _loc6_;
		}
		_loc4_ = _loc6_.get();
		_loc5_ = _loc5_->next;
	}

	_loc3_->normalX = param1->normalX;
	_loc3_->normalY = param1->normalY;
	_loc3_->normalZ = param1->normalZ;
	_loc3_->offset = param1->offset;

	if (param1->negative != nullptr)
		_loc3_->negative = cloneNode(param1->negative.get(), param2);

	if (param1->positive != nullptr)
		_loc3_->positive = cloneNode(param1->positive.get(), param2);

	return _loc3_;
}

std::shared_ptr<BSP> new_BSP()
{
	_function_name("new_BSP");

	return std::make_shared<BSP>(Object3D::TT(BSP::TYPE));
}

std::shared_ptr<BSP> BSP_clone(const std::shared_ptr<BSP> &in)
{
	_function_name("BSP_clone");

	if (in->isType(BSP::TYPE)) {
		auto out = new_BSP();
		out->clonePropertiesFromBSP(in.get());
		return out;
	}

	throw 41132421;
}





std::shared_ptr<Face> BSP::prepareNode(BSP_Node *param1, int param2, Camera3D *param3)
{
	std::shared_ptr<Face> _loc4_;

	if ((((this->matrix_i.md * param1->normalX) + (this->matrix_i.mh * param1->normalY)) + (this->matrix_i.ml * param1->normalZ)) > param1->offset)
	{
		_loc4_ = param1->faceList;
		Face *_loc8_ = _loc4_.get();
		while (_loc8_ != nullptr)
		{
			Wrapper *_loc5_ = _loc8_->wrapper.get();
			while (_loc5_ != nullptr)
			{
				Vertex *_loc9_ = _loc5_->vertex.get();
				if (_loc9_->transformId != transformId)
				{
					double _loc10_ = _loc9_->x;
					double _loc11_ = _loc9_->y;
					double _loc12_ = _loc9_->z;
					_loc9_->cameraX = ((((this->matrix.ma * _loc10_) + (this->matrix.mb * _loc11_)) + (this->matrix.mc * _loc12_)) + this->matrix.md);
					_loc9_->cameraY = ((((this->matrix.me * _loc10_) + (this->matrix.mf * _loc11_)) + (this->matrix.mg * _loc12_)) + this->matrix.mh);
					_loc9_->cameraZ = ((((this->matrix.mi * _loc10_) + (this->matrix.mj * _loc11_)) + (this->matrix.mk * _loc12_)) + this->matrix.ml);
					_loc9_->transformId = transformId;
					_loc9_->drawId = 0;
				}
				_loc5_ = _loc5_->_next;
			}
			_loc8_->processNext = _loc8_->next;
			_loc8_ = _loc8_->next.get();
		}

		if (param2 > 0)
		{
			if (this->clipping == 1)
			{
				_loc4_ = param3->cull(_loc4_, param2);
			}
			else
			{
				_loc4_ = param3->clip(_loc4_, param2);
			}
		}
	}

	std::shared_ptr<Face> _loc6_ = ((!(param1->negative == nullptr)) ? this->prepareNode(param1->negative.get(), param2, param3) : nullptr);
	std::shared_ptr<Face> _loc7_ = ((!(param1->positive == nullptr)) ? this->prepareNode(param1->positive.get(), param2, param3) : nullptr);

	if (((!(_loc4_ == nullptr)) || ((!(_loc6_ == nullptr)) && (!(_loc7_ == nullptr)))))
	{
		if (_loc4_ == nullptr)
		{
			_loc4_ = Face::create();
			param3->lastFace->next = _loc4_;
			param3->lastFace = _loc4_;
		}
		Wrapper *_loc5_ = param1->faceList->wrapper.get();
		Vertex *_loc13_ = _loc5_->vertex.get();
		_loc5_ = _loc5_->_next;
		Vertex *_loc14_ = _loc5_->vertex.get();
		_loc5_ = _loc5_->_next;
		Vertex *_loc15_ = _loc5_->vertex.get();
		if (_loc13_->transformId != transformId)
		{
			_loc13_->cameraX = ((((this->matrix.ma * _loc13_->x) + (this->matrix.mb * _loc13_->y)) + (this->matrix.mc * _loc13_->z)) + this->matrix.md);
			_loc13_->cameraY = ((((this->matrix.me * _loc13_->x) + (this->matrix.mf * _loc13_->y)) + (this->matrix.mg * _loc13_->z)) + this->matrix.mh);
			_loc13_->cameraZ = ((((this->matrix.mi * _loc13_->x) + (this->matrix.mj * _loc13_->y)) + (this->matrix.mk * _loc13_->z)) + this->matrix.ml);
			_loc13_->transformId = transformId;
			_loc13_->drawId = 0;
		}
		if (_loc14_->transformId != transformId)
		{
			_loc14_->cameraX = ((((this->matrix.ma * _loc14_->x) + (this->matrix.mb * _loc14_->y)) + (this->matrix.mc * _loc14_->z)) + this->matrix.md);
			_loc14_->cameraY = ((((this->matrix.me * _loc14_->x) + (this->matrix.mf * _loc14_->y)) + (this->matrix.mg * _loc14_->z)) + this->matrix.mh);
			_loc14_->cameraZ = ((((this->matrix.mi * _loc14_->x) + (this->matrix.mj * _loc14_->y)) + (this->matrix.mk * _loc14_->z)) + this->matrix.ml);
			_loc14_->transformId = transformId;
			_loc14_->drawId = 0;
		}
		if (_loc15_->transformId != transformId)
		{
			_loc15_->cameraX = ((((this->matrix.ma * _loc15_->x) + (this->matrix.mb * _loc15_->y)) + (this->matrix.mc * _loc15_->z)) + this->matrix.md);
			_loc15_->cameraY = ((((this->matrix.me * _loc15_->x) + (this->matrix.mf * _loc15_->y)) + (this->matrix.mg * _loc15_->z)) + this->matrix.mh);
			_loc15_->cameraZ = ((((this->matrix.mi * _loc15_->x) + (this->matrix.mj * _loc15_->y)) + (this->matrix.mk * _loc15_->z)) + this->matrix.ml);
			_loc15_->transformId = transformId;
			_loc15_->drawId = 0;
		}
		double _loc16_ = (_loc14_->cameraX - _loc13_->cameraX);
		double _loc17_ = (_loc14_->cameraY - _loc13_->cameraY);
		double _loc18_ = (_loc14_->cameraZ - _loc13_->cameraZ);
		double _loc19_ = (_loc15_->cameraX - _loc13_->cameraX);
		double _loc20_ = (_loc15_->cameraY - _loc13_->cameraY);
		double _loc21_ = (_loc15_->cameraZ - _loc13_->cameraZ);
		double _loc22_ = ((_loc21_ * _loc17_) - (_loc20_ * _loc18_));
		double _loc23_ = ((_loc19_ * _loc18_) - (_loc21_ * _loc16_));
		double _loc24_ = ((_loc20_ * _loc16_) - (_loc19_ * _loc17_));
		double _loc25_ = (((_loc22_ * _loc22_) + (_loc23_ * _loc23_)) + (_loc24_ * _loc24_));
		if (_loc25_ > 0)
		{
			_loc25_ = (1. / MyMath::sqrt(1)); //length
			_loc22_ = (_loc22_ * _loc25_);
			_loc23_ = (_loc23_ * _loc25_);
			_loc24_ = (_loc24_ * _loc25_);
		}
		_loc4_->normalX = _loc22_;
		_loc4_->normalY = _loc23_;
		_loc4_->normalZ = _loc24_;
		_loc4_->offset = (((_loc13_->cameraX * _loc22_) + (_loc13_->cameraY * _loc23_)) + (_loc13_->cameraZ * _loc24_));
		_loc4_->processNegative = _loc6_;
		_loc4_->processPositive = _loc7_;
	}
	else
	{
		_loc4_ = ((!(_loc6_ == nullptr)) ? _loc6_ : _loc7_);
	}
	return _loc4_;
}


static bool intersectRayNode(BSP *bsp, BSP_Node *node, double ox, double oy, double oz, double dx, double dy, double dz, RayIntersectionData *result)
{
	double offset = ((((node->normalX * ox) + (node->normalY * oy)) + (node->normalZ * oz)) - node->offset);
	if (offset > 0)
	{
		if (node->positive != nullptr)
		{
			RayIntersectionData data;
			if (intersectRayNode(bsp, node->positive.get(), ox, oy, oz, dx, dy, dz, &data))
			{
				result->copy(&data);
				return true;
			}
		}
		double dot = (((dx * node->normalX) + (dy * node->normalY)) + (dz * node->normalZ));
		if (dot < 0)
		{
			double time = (-(offset) / dot);
			double cx = (ox + (dx * time));
			double cy = (oy + (dy * time));
			double cz = (oz + (dz * time));
			Face *f = node->faceList.get();
			while (f != nullptr)
			{
				Wrapper *wrapper = f->wrapper.get();
				while (wrapper != nullptr)
				{
					Vertex *a = wrapper->vertex.get();
					Vertex *b = ((wrapper->_next != nullptr) ? wrapper->_next->vertex.get() : f->wrapper->vertex.get());
					double abx = (b->x - a->x);
					double aby = (b->y - a->y);
					double abz = (b->z - a->z);
					double acx = (cx - a->x);
					double acy = (cy - a->y);
					double acz = (cz - a->z);
					if ((((((acz * aby) - (acy * abz)) * node->normalX) + (((acx * abz) - (acz * abx)) * node->normalY)) + (((acy * abx) - (acx * aby)) * node->normalZ)) < 0) break;
					wrapper = wrapper->_next;
				}
				if (wrapper == nullptr)
				{
					result->object = bsp;
					result->face = f;
					result->point.reset(cx, cy, cz);
					result->time = time;
					result->uv.x = 0;
					result->uv.y = 0;
					return true;
				}
				f = f->next.get();
			}
			if (node->negative != nullptr)
			{
				return intersectRayNode(bsp, node->negative.get(), ox, oy, oz, dx, dy, dz, result);
			}
		}
	}
	else
	{
		if (node->negative != nullptr)
		{
			RayIntersectionData data;
			if (intersectRayNode(bsp, node->negative.get(), ox, oy, oz, dx, dy, dz, &data))
			{
				result->copy(&data);
				return true;
			}
		}
		if (((!(node->positive == nullptr)) && ((((dx * node->normalX) + (dy * node->normalY)) + (dz * node->normalZ)) > 0)))
		{
			return intersectRayNode(bsp, node->positive.get(), ox, oy, oz, dx, dy, dz, result);
		}
	}

	return false;
}

bool BSP::intersectRay(const Vector3 *origin, const Vector3 *direction, Object3DFilter *filter, Camera3D *camera, RayIntersectionData *result)
{
	(void)camera;

	if (filter != nullptr && filter->check(this))
	{
		return false;
	}

	if (!boundIntersectRay(origin, direction, bound.MinX, bound.MinY, bound.MinZ, bound.MaxX, bound.MaxY, bound.MaxZ))
	{
		return false;
	}

	if (this->root != nullptr)
	{
		return intersectRayNode(this, this->root.get(), origin->x, origin->y, origin->z, direction->x, direction->y, direction->z, result);
	}

	return false;
}


void BSP::updateBounds(Object3D *param1, Object3D *param2)
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
