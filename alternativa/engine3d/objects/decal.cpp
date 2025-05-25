#include "decal.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/gfx/core/vertexbufferresource.h"
#include "alternativa/gfx/core/indexbufferresource.h"
#include "../core/face.h"
#include "../core/wrapper.h"
#include "../core/camera3d.h"
#include "_global.h"
#include <QVector>

const Object3D::T *Decal::TYPE = new Object3D::T("Decal");



Decal::Decal(const TT &t) :
	Mesh(TT(t.type, this->TYPE)),
	__type_next(t.type_next)
{
	attenuation = 1000000;
	shadowMapAlphaThreshold = 100;
}


//not used
void Decal::clonePropertiesFromDecal(Decal *src)
{
	(void)src;
	/*
	super.clonePropertiesFrom(param1);
	var _loc2_:Decal = (param1 as Decal);
	this.attenuation = _loc2_.attenuation;
	*/
}

void Decal::draw(Camera3D *camera)
{
	_function_name("Decal::draw");

	if (faceList() == nullptr)
		return;

	if (clipping == 0)
	{
		if (culling & 0x01)
			return;
		culling = 0;
	}

	this->prepareResources();

	useDepth = true;

	if (faceList()->material != nullptr)
	{
		camera->addDecal(this);
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

	//int _loc2_ = ((!(!(param1.debug))) ? int(param1.checkInDebug(this)) : int(0));
	//if ((_loc2_ & Debug.BOUNDS))
		//Debug.drawBounds(param1, this, boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ);

	/*
	if ((_loc2_ & Debug.EDGES))
	{
		if (transformId > 0x1DCD6500)
		{
			transformId = 0;
			_loc4_ = vertexList;
			while (_loc4_ != null)
			{
				_loc4_.transformId = 0;
				_loc4_ = _loc4_.next;
			}
		}
		transformId++;
		calculateInverseMatrix();
		_loc3_ = prepareFaces(param1, faceList);
		if (_loc3_ == null)
		{
			return;
		}
		Debug.drawEdges(param1, _loc3_, 0xFFFFFF);
	}
	*/

}

object_ptr<VG> Decal::getVG(Camera3D *camera)
{
	_function_name("Decal::getVG");
	this->draw(camera);
	return nullptr;
}

void Decal::prepareResources()
{
	_function_name("Decal::prepareResources()");

	if (vertexBuffer == nullptr)
	{
		QVector<double> _loc1_;
		//int _loc2_ = 0;
		int _loc3_ = 0;
		Vertex *_loc4_ = vertexList.get();
		while (_loc4_ != nullptr)
		{
			_loc1_.append(_loc4_->x);
			_loc1_.append(_loc4_->y);
			_loc1_.append(_loc4_->z);
			_loc1_.append(_loc4_->u);
			_loc1_.append(_loc4_->v);
			_loc1_.append(_loc4_->normalX);
			_loc1_.append(_loc4_->normalY);
			_loc1_.append(_loc4_->normalZ);
			/*
			_loc1_[_loc2_] = _loc4_->x;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->y;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->z;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->u;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->v;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->normalX;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->normalY;
			_loc2_++;
			_loc1_[_loc2_] = _loc4_->normalZ;
			_loc2_++;
			*/
			_loc4_->index = _loc3_;
			_loc3_++;
			_loc4_ = _loc4_->next.get();
		}
		vertexBuffer = std::make_shared<VertexBufferResource>(_loc1_, 8);
		QVector<uint> _loc5_;
		//int _loc6_ = 0;
		numTriangles = 0;
		Face *_loc7_ = faceList().get();
		while (_loc7_ != nullptr)
		{
			Wrapper *_loc8_ = _loc7_->wrapper.get();
			uint32_t _loc9_ = _loc8_->vertex->index;
			_loc8_ = _loc8_->_next;
			uint32_t _loc10_ = _loc8_->vertex->index;
			_loc8_ = _loc8_->_next;
			while (_loc8_ != nullptr)
			{
				uint32_t _loc11_ = _loc8_->vertex->index;
				_loc5_.append(_loc9_);
				_loc5_.append(_loc10_);
				_loc5_.append(_loc11_);
				//_loc5_[_loc6_] = _loc9_;
				//_loc6_++;
				//_loc5_[_loc6_] = _loc10_;
				//_loc6_++;
				//_loc5_[_loc6_] = _loc11_;
				//_loc6_++;
				_loc10_ = _loc11_;
				numTriangles++;
				_loc8_ = _loc8_->_next;
			}
			_loc7_ = _loc7_->next.get();
		}
		indexBuffer = std::make_shared<IndexBufferResource>(_loc5_);
	}

}


std::shared_ptr<Decal> new_Decal()
{
	return std::make_shared<Decal>(Object3D::TT(Decal::TYPE));
}

