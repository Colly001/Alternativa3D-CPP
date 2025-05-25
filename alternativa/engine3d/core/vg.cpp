#include "vg.h"
#include "vertex.h"
#include "face.h"
#include "object3d.h"
#include "mymath.h"
#include "wrapper.h"
#include "face.h"
#include "camera3d.h"
#include "_global.h"

static object_ptr<VG> collector;

static Face* collectNode(Face *param1, Face *param2);


void VG::setFaceStruct(std::shared_ptr<Face> face, int id)
{
	_faceStruct = face;
	faceStruct_id = id;
}

VG::VG()
{
	_function_name("VG::VG()");

	boundVertexList = Vertex::createList(8);
	boundPlaneList = Vertex::createList(6);
	viewAligned = false;
	debug = 0;
	space = 0;
	sorting = 0;
	object = 0;
	setFaceStruct(nullptr, 0);
	next = nullptr;
	viewAligned = false;
}

VG::~VG()
{
	this->destroy();
}

object_ptr<VG> VG::create(std::shared_ptr<Object3D> &object, std::shared_ptr<Face> &faceStruct, int sorting, int debug, bool viewAligned)
{
	_function_name("VG::create()");

	object_ptr<VG> vg;
	if (collector != nullptr)
	{
		vg = collector;
		collector = collector->next;
		vg->next = nullptr;
	}
	else
	{
		vg = new_object<VG>();
	}

	vg->object = object;
	vg->setFaceStruct(faceStruct, 10);
	vg->sorting = sorting;
	vg->debug = debug;
	vg->viewAligned = viewAligned;
	return vg;
}

void VG::destroy()
{
	_function_name("VG::destroy()");

	if (this->faceStruct() != nullptr)
	{
		this->destroyFaceStruct(this->faceStruct());
		this->setFaceStruct(nullptr, 5);
	}
	this->object = nullptr;
	this->numOccluders = 0;
	this->debug = 0;
	this->space = 0;

	while (this->next != nullptr)
	{
		object_ptr<VG> temp = this->next;
		this->next = nullptr;
		this->next = temp->next;
	}


	//this->next = collector;
	//collector = this;
}

void VG::calculateAABB(const Matrix4 *matrix)
{
	_function_name("VG::calculateAABB()");

	this->boundMinX = 1E22;
	this->boundMinY = 1E22;
	this->boundMinZ = 1E22;
	this->boundMaxX = -1E22;
	this->boundMaxY = -1E22;
	this->boundMaxZ = -1E22;
	this->calculateAABBStruct(this->faceStruct().get(), ++this->object->transformId, matrix);
	this->space = 1;
}

void VG::calculateOOBB(Object3D *param1)
{
	_function_name("VG::calculateOOBB()");

	if (this->space == 1)
	{
		this->transformStruct(this->faceStruct().get(),
							  ++this->object->transformId,
							  &param1->matrix
							/*param1->matrix.ma,
							  param1->matrix.mb,
							  param1->matrix.mc,
							  param1->matrix.md,
							  param1->matrix.me,
							  param1->matrix.mf,
							  param1->matrix.mg,
							  param1->matrix.mh,
							  param1->matrix.mi,
							  param1->matrix.mj,
							  param1->matrix.mk,
							  param1->matrix.ml*/);
	}

	if ((!(this->viewAligned)))
	{
		this->boundMinX = 1E22;
		this->boundMinY = 1E22;
		this->boundMinZ = 1E22;
		this->boundMaxX = -1E22;
		this->boundMaxY = -1E22;
		this->boundMaxZ = -1E22;

		this->calculateOOBBStruct(this->faceStruct().get(), ++this->object->transformId);


		if ((this->boundMaxX - this->boundMinX) < 1)
		{
			this->boundMaxX = (this->boundMinX + 1);
		}
		if ((this->boundMaxY - this->boundMinY) < 1)
		{
			this->boundMaxY = (this->boundMinY + 1);
		}
		if ((this->boundMaxZ - this->boundMinZ) < 1)
		{
			this->boundMaxZ = (this->boundMinZ + 1);
		}
		Vertex *_loc2_ = this->boundVertexList.get();
		_loc2_->x = this->boundMinX;
		_loc2_->y = this->boundMinY;
		_loc2_->z = this->boundMinZ;
		Vertex *_loc3_ = _loc2_->next.get();
		_loc3_->x = this->boundMaxX;
		_loc3_->y = this->boundMinY;
		_loc3_->z = this->boundMinZ;
		Vertex *_loc4_ = _loc3_->next.get();
		_loc4_->x = this->boundMinX;
		_loc4_->y = this->boundMaxY;
		_loc4_->z = this->boundMinZ;
		Vertex *_loc5_ = _loc4_->next.get();
		_loc5_->x = this->boundMaxX;
		_loc5_->y = this->boundMaxY;
		_loc5_->z = this->boundMinZ;
		Vertex *_loc6_ = _loc5_->next.get();
		_loc6_->x = this->boundMinX;
		_loc6_->y = this->boundMinY;
		_loc6_->z = this->boundMaxZ;
		Vertex *_loc7_ = _loc6_->next.get();
		_loc7_->x = this->boundMaxX;
		_loc7_->y = this->boundMinY;
		_loc7_->z = this->boundMaxZ;
		Vertex *_loc8_ = _loc7_->next.get();
		_loc8_->x = this->boundMinX;
		_loc8_->y = this->boundMaxY;
		_loc8_->z = this->boundMaxZ;
		Vertex *_loc9_ = _loc8_->next.get();
		_loc9_->x = this->boundMaxX;
		_loc9_->y = this->boundMaxY;
		_loc9_->z = this->boundMaxZ;
		Vertex *_loc10_ = _loc2_;
		while (_loc10_ != nullptr)
		{
			_loc10_->cameraX = ((((this->object->matrix.ma * _loc10_->x) + (this->object->matrix.mb * _loc10_->y)) + (this->object->matrix.mc * _loc10_->z)) + this->object->matrix.md);
			_loc10_->cameraY = ((((this->object->matrix.me * _loc10_->x) + (this->object->matrix.mf * _loc10_->y)) + (this->object->matrix.mg * _loc10_->z)) + this->object->matrix.mh);
			_loc10_->cameraZ = ((((this->object->matrix.mi * _loc10_->x) + (this->object->matrix.mj * _loc10_->y)) + (this->object->matrix.mk * _loc10_->z)) + this->object->matrix.ml);
			_loc10_ = _loc10_->next.get();
		}
		Vertex *_loc11_ = this->boundPlaneList.get();
		Vertex *_loc12_ = _loc11_->next.get();
		double _loc13_ = _loc2_->cameraX;
		double _loc14_ = _loc2_->cameraY;
		double _loc15_ = _loc2_->cameraZ;
		double _loc16_ = (_loc3_->cameraX - _loc13_);
		double _loc17_ = (_loc3_->cameraY - _loc14_);
		double _loc18_ = (_loc3_->cameraZ - _loc15_);
		double _loc19_ = (_loc6_->cameraX - _loc13_);
		double _loc20_ = (_loc6_->cameraY - _loc14_);
		double _loc21_ = (_loc6_->cameraZ - _loc15_);
		double _loc22_ = ((_loc21_ * _loc17_) - (_loc20_ * _loc18_));
		double _loc23_ = ((_loc19_ * _loc18_) - (_loc21_ * _loc16_));
		double _loc24_ = ((_loc20_ * _loc16_) - (_loc19_ * _loc17_));
		double _loc25_ = (1. / MyMath::sqrt((((_loc22_ * _loc22_) + (_loc23_ * _loc23_)) + (_loc24_ * _loc24_))));
		_loc22_ = (_loc22_ * _loc25_);
		_loc23_ = (_loc23_ * _loc25_);
		_loc24_ = (_loc24_ * _loc25_);
		_loc11_->cameraX = _loc22_;
		_loc11_->cameraY = _loc23_;
		_loc11_->cameraZ = _loc24_;
		_loc11_->offset = (((_loc13_ * _loc22_) + (_loc14_ * _loc23_)) + (_loc15_ * _loc24_));
		_loc12_->cameraX = -(_loc22_);
		_loc12_->cameraY = -(_loc23_);
		_loc12_->cameraZ = -(_loc24_);
		_loc12_->offset = (((-(_loc4_->cameraX) * _loc22_) - (_loc4_->cameraY * _loc23_)) - (_loc4_->cameraZ * _loc24_));
		Vertex *_loc26_ = _loc12_->next.get();
		Vertex *_loc27_ = _loc26_->next.get();
		_loc13_ = _loc2_->cameraX;
		_loc14_ = _loc2_->cameraY;
		_loc15_ = _loc2_->cameraZ;
		_loc16_ = (_loc6_->cameraX - _loc13_);
		_loc17_ = (_loc6_->cameraY - _loc14_);
		_loc18_ = (_loc6_->cameraZ - _loc15_);
		_loc19_ = (_loc4_->cameraX - _loc13_);
		_loc20_ = (_loc4_->cameraY - _loc14_);
		_loc21_ = (_loc4_->cameraZ - _loc15_);
		_loc22_ = ((_loc21_ * _loc17_) - (_loc20_ * _loc18_));
		_loc23_ = ((_loc19_ * _loc18_) - (_loc21_ * _loc16_));
		_loc24_ = ((_loc20_ * _loc16_) - (_loc19_ * _loc17_));
		_loc25_ = (1. / MyMath::sqrt((((_loc22_ * _loc22_) + (_loc23_ * _loc23_)) + (_loc24_ * _loc24_))));
		_loc22_ = (_loc22_ * _loc25_);
		_loc23_ = (_loc23_ * _loc25_);
		_loc24_ = (_loc24_ * _loc25_);
		_loc26_->cameraX = _loc22_;
		_loc26_->cameraY = _loc23_;
		_loc26_->cameraZ = _loc24_;
		_loc26_->offset = (((_loc13_ * _loc22_) + (_loc14_ * _loc23_)) + (_loc15_ * _loc24_));
		_loc27_->cameraX = -(_loc22_);
		_loc27_->cameraY = -(_loc23_);
		_loc27_->cameraZ = -(_loc24_);
		_loc27_->offset = (((-(_loc3_->cameraX) * _loc22_) - (_loc3_->cameraY * _loc23_)) - (_loc3_->cameraZ * _loc24_));
		Vertex *_loc28_ = _loc27_->next.get();
		Vertex *_loc29_ = _loc28_->next.get();
		_loc13_ = _loc6_->cameraX;
		_loc14_ = _loc6_->cameraY;
		_loc15_ = _loc6_->cameraZ;
		_loc16_ = (_loc7_->cameraX - _loc13_);
		_loc17_ = (_loc7_->cameraY - _loc14_);
		_loc18_ = (_loc7_->cameraZ - _loc15_);
		_loc19_ = (_loc8_->cameraX - _loc13_);
		_loc20_ = (_loc8_->cameraY - _loc14_);
		_loc21_ = (_loc8_->cameraZ - _loc15_);
		_loc22_ = ((_loc21_ * _loc17_) - (_loc20_ * _loc18_));
		_loc23_ = ((_loc19_ * _loc18_) - (_loc21_ * _loc16_));
		_loc24_ = ((_loc20_ * _loc16_) - (_loc19_ * _loc17_));
		_loc25_ = (1. / MyMath::sqrt((((_loc22_ * _loc22_) + (_loc23_ * _loc23_)) + (_loc24_ * _loc24_))));
		_loc22_ = (_loc22_ * _loc25_);
		_loc23_ = (_loc23_ * _loc25_);
		_loc24_ = (_loc24_ * _loc25_);
		_loc28_->cameraX = _loc22_;
		_loc28_->cameraY = _loc23_;
		_loc28_->cameraZ = _loc24_;
		_loc28_->offset = (((_loc13_ * _loc22_) + (_loc14_ * _loc23_)) + (_loc15_ * _loc24_));
		_loc29_->cameraX = -(_loc22_);
		_loc29_->cameraY = -(_loc23_);
		_loc29_->cameraZ = -(_loc24_);
		_loc29_->offset = (((-(_loc2_->cameraX) * _loc22_) - (_loc2_->cameraY * _loc23_)) - (_loc2_->cameraZ * _loc24_));
		if (_loc11_->offset < -(_loc12_->offset))
		{
			_loc12_->cameraX = -(_loc12_->cameraX);
			_loc12_->cameraY = -(_loc12_->cameraY);
			_loc12_->cameraZ = -(_loc12_->cameraZ);
			_loc12_->offset = -(_loc12_->offset);
			_loc11_->cameraX = -(_loc11_->cameraX);
			_loc11_->cameraY = -(_loc11_->cameraY);
			_loc11_->cameraZ = -(_loc11_->cameraZ);
			_loc11_->offset = -(_loc11_->offset);
		}
		if (_loc26_->offset < -(_loc27_->offset))
		{
			_loc26_->cameraX = -(_loc26_->cameraX);
			_loc26_->cameraY = -(_loc26_->cameraY);
			_loc26_->cameraZ = -(_loc26_->cameraZ);
			_loc26_->offset = -(_loc26_->offset);
			_loc27_->cameraX = -(_loc27_->cameraX);
			_loc27_->cameraY = -(_loc27_->cameraY);
			_loc27_->cameraZ = -(_loc27_->cameraZ);
			_loc27_->offset = -(_loc27_->offset);
		}
		if (_loc29_->offset < -(_loc28_->offset))
		{
			_loc29_->cameraX = -(_loc29_->cameraX);
			_loc29_->cameraY = -(_loc29_->cameraY);
			_loc29_->cameraZ = -(_loc29_->cameraZ);
			_loc29_->offset = -(_loc29_->offset);
			_loc28_->cameraX = -(_loc28_->cameraX);
			_loc28_->cameraY = -(_loc28_->cameraY);
			_loc28_->cameraZ = -(_loc28_->cameraZ);
			_loc28_->offset = -(_loc28_->offset);
		}
	}
	this->space = 2;
}

//not used
void VG::destroyFaceStruct(std::shared_ptr<Face> face)
{
	_function_name("VG::destroyFaceStruct()");

	if (face->processNegative != nullptr)
	{
		this->destroyFaceStruct(face->processNegative);
		face->processNegative = nullptr;
	}
	if (face->processPositive != nullptr)
	{
		this->destroyFaceStruct(face->processPositive);
		face->processPositive = nullptr;
	}
	std::shared_ptr<Face> temp = face->processNext;
	while (temp != nullptr)
	{
		face->processNext = nullptr;
		face = temp;
		temp = face->processNext;
	}

}

void VG::calculateAABBStruct(Face *param1, int param2, const Matrix4 *matrix)
{
	_function_name("VG::calculateAABBStruct()");

	//.ma
	//.mb
	//.mc
	//.md
	//.me
	//.mf
	//.mg
	//.mh
	//.mi
	//.mj
	//.mk
	//.ml

	double param3 = matrix->ma;
	double param4 = matrix->mb;
	double param5 = matrix->mc;
	double param6 = matrix->md;
	double param7 = matrix->me;
	double param8 = matrix->mf;
	double param9 = matrix->mg;
	double param10 = matrix->mh;
	double param11 = matrix->mi;
	double param12 = matrix->mj;
	double param13 = matrix->mk;
	double param14 = matrix->ml;

	if (param1 == nullptr)
	{
		qDebug("error VG::calculateAABBStruct()");
		return;
	}


	Face *_loc15_ = param1;
	while (_loc15_ != nullptr)
	{
		Wrapper *_loc16_ = _loc15_->wrapper.get();
		while (_loc16_ != nullptr)
		{
			Vertex *_loc17_ = _loc16_->vertex.get();
			if (_loc17_ != nullptr && _loc17_->transformId != param2)
			{
				double _loc18_ = _loc17_->cameraX;
				double _loc19_ = _loc17_->cameraY;
				double _loc20_ = _loc17_->cameraZ;
				_loc17_->cameraX = ((((param3 * _loc18_) + (param4 * _loc19_)) + (param5 * _loc20_)) + param6);
				_loc17_->cameraY = ((((param7 * _loc18_) + (param8 * _loc19_)) + (param9 * _loc20_)) + param10);
				_loc17_->cameraZ = ((((param11 * _loc18_) + (param12 * _loc19_)) + (param13 * _loc20_)) + param14);
				if (_loc17_->cameraX < this->boundMinX)
				{
					this->boundMinX = _loc17_->cameraX;
				}
				if (_loc17_->cameraX > this->boundMaxX)
				{
					this->boundMaxX = _loc17_->cameraX;
				}
				if (_loc17_->cameraY < this->boundMinY)
				{
					this->boundMinY = _loc17_->cameraY;
				}
				if (_loc17_->cameraY > this->boundMaxY)
				{
					this->boundMaxY = _loc17_->cameraY;
				}
				if (_loc17_->cameraZ < this->boundMinZ)
				{
					this->boundMinZ = _loc17_->cameraZ;
				}
				if (_loc17_->cameraZ > this->boundMaxZ)
				{
					this->boundMaxZ = _loc17_->cameraZ;
				}
				_loc17_->transformId = param2;
			}
			_loc16_ = _loc16_->_next;
		}
		_loc15_ = _loc15_->processNext.get();
	}
	if (param1->processNegative != nullptr)
	{
		this->calculateAABBStruct(param1->processNegative.get(), param2, matrix);
	}
	if (param1->processPositive != nullptr)
	{
		this->calculateAABBStruct(param1->processPositive.get(), param2, matrix);
	}
}

void VG::calculateOOBBStruct(Face *param1, int param2)
{
	_function_name("VG::calculateOOBBStruct()");

	if (param1 == nullptr)
	{
		qDebug("error VG::calculateOOBBStruct");
		//???
		//return;
	}

	Face *_loc15_ = param1;
	while (_loc15_ != nullptr)
	{
		Wrapper *_loc16_ = _loc15_->wrapper.get();
		while (_loc16_ != nullptr)
		{
			Vertex *_loc17_ = _loc16_->vertex.get();
			if (_loc17_->transformId != param2)
			{
				if (_loc17_->x < this->boundMinX)
				{
					this->boundMinX = _loc17_->x;
				}
				if (_loc17_->x > this->boundMaxX)
				{
					this->boundMaxX = _loc17_->x;
				}
				if (_loc17_->y < this->boundMinY)
				{
					this->boundMinY = _loc17_->y;
				}
				if (_loc17_->y > this->boundMaxY)
				{
					this->boundMaxY = _loc17_->y;
				}
				if (_loc17_->z < this->boundMinZ)
				{
					this->boundMinZ = _loc17_->z;
				}
				if (_loc17_->z > this->boundMaxZ)
				{
					this->boundMaxZ = _loc17_->z;
				}
				_loc17_->transformId = param2;
			}
			_loc16_ = _loc16_->_next;
		}
		_loc15_ = _loc15_->processNext.get();
	}
	if (param1->processNegative != nullptr)
	{
		this->calculateOOBBStruct(param1->processNegative.get(), param2);
	}
	if (param1->processPositive != nullptr)
	{
		this->calculateOOBBStruct(param1->processPositive.get(), param2);
	}
}


//not used
void VG::updateAABBStruct(Face *param1, int param2)
{
	_function_name("VG::updateAABBStruct()");

	(void)param1;
	(void)param2;
	qDebug("used VG::updateAABBStruct");
#if 0
	var _loc4_:Wrapper;
	var _loc5_:Vertex;
	var _loc3_:Face = param1;
	while (_loc3_ != null)
	{
		_loc4_ = _loc3_->wrapper;
		while (_loc4_ != null)
		{
			_loc5_ = _loc4_->vertex;
			if (_loc5_->transformId != param2)
			{
				if (_loc5_->cameraX < this->boundMinX)
				{
					this->boundMinX = _loc5_->cameraX;
				}
				if (_loc5_->cameraX > this->boundMaxX)
				{
					this->boundMaxX = _loc5_->cameraX;
				}
				if (_loc5_->cameraY < this->boundMinY)
				{
					this->boundMinY = _loc5_->cameraY;
				}
				if (_loc5_->cameraY > this->boundMaxY)
				{
					this->boundMaxY = _loc5_->cameraY;
				}
				if (_loc5_->cameraZ < this->boundMinZ)
				{
					this->boundMinZ = _loc5_->cameraZ;
				}
				if (_loc5_->cameraZ > this->boundMaxZ)
				{
					this->boundMaxZ = _loc5_->cameraZ;
				}
				_loc5_->transformId = param2;
			}
			_loc4_ = _loc4_->next;
		}
		_loc3_ = _loc3_->processNext;
	}
	if (param1->processNegative != null)
	{
		this->updateAABBStruct(param1->processNegative, param2);
	}
	if (param1->processPositive != null)
	{
		this->updateAABBStruct(param1->processPositive, param2);
	}
#endif
}


//not used
void VG::splitFaceStruct(Camera3D *param1, Face *param2, Face *param3, double param4, double param5, double param6, double param7, double param8, double param9)
{
	_function_name("VG::splitFaceStruct()");

	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;
	(void)param7;
	(void)param8;
	(void)param9;
	qDebug("used VG::splitFaceStruct");

#if 0
	var _loc10_:Face;
	var _loc11_:Face;
	var _loc12_:Wrapper;
	var _loc13_:Vertex;
	var _loc14_:Vertex;
	var _loc15_:Face;
	var _loc16_:Face;
	var _loc17_:Face;
	var _loc18_:Face;
	var _loc19_:Face;
	var _loc20_:Face;
	var _loc21_:Face;
	var _loc22_:Face;
	var _loc23_:Face;
	var _loc24_:Face;
	var _loc25_:Wrapper;
	var _loc26_:Wrapper;
	var _loc27_:Wrapper;
	var _loc28_:Boolean;
	var _loc29_:Vertex;
	var _loc30_:Vertex;
	var _loc31_:Vertex;
	var _loc32_:Number = NaN;
	var _loc33_:Number = NaN;
	var _loc34_:Number = NaN;
	var _loc35_:Boolean;
	var _loc36_:Boolean;
	var _loc37_:Boolean;
	var _loc38_:Number = NaN;
	var _loc39_:Number = NaN;
	if (param2->processNegative != null)
	{
		this->splitFaceStruct(param1, param2->processNegative, param3, param4, param5, param6, param7, param8, param9);
		param2->processNegative = null;
		_loc15_ = param3.processNegative;
		_loc16_ = param3.processPositive;
	}
	if (param2->processPositive != null)
	{
		this->splitFaceStruct(param1, param2->processPositive, param3, param4, param5, param6, param7, param8, param9);
		param2->processPositive = null;
		_loc17_ = param3.processNegative;
		_loc18_ = param3.processPositive;
	}
	if (param2->wrapper != null)
	{
		_loc10_ = param2;
		while (_loc10_ != null)
		{
			_loc11_ = _loc10_->processNext;
			_loc12_ = _loc10_->wrapper;
			_loc29_ = _loc12_->vertex;
			_loc12_ = _loc12_->next;
			_loc30_ = _loc12_->vertex;
			_loc12_ = _loc12_->next;
			_loc31_ = _loc12_->vertex;
			_loc12_ = _loc12_->next;
			_loc32_ = (((_loc29_->cameraX * param4) + (_loc29_->cameraY * param5)) + (_loc29_->cameraZ * param6));
			_loc33_ = (((_loc30_->cameraX * param4) + (_loc30_->cameraY * param5)) + (_loc30_->cameraZ * param6));
			_loc34_ = (((_loc31_->cameraX * param4) + (_loc31_->cameraY * param5)) + (_loc31_->cameraZ * param6));
			_loc35_ = (((_loc32_ < param8) || (_loc33_ < param8)) || (_loc34_ < param8));
			_loc36_ = (((_loc32_ > param9) || (_loc33_ > param9)) || (_loc34_ > param9));
			_loc37_ = (((_loc32_ < param8) && (_loc33_ < param8)) && (_loc34_ < param8));
			while (_loc12_ != null)
			{
				_loc13_ = _loc12_->vertex;
				_loc38_ = (((_loc13_->cameraX * param4) + (_loc13_->cameraY * param5)) + (_loc13_->cameraZ * param6));
				if (_loc38_ < param8)
				{
					_loc35_ = true;
				}
				else
				{
					_loc37_ = false;
					if (_loc38_ > param9)
					{
						_loc36_ = true;
					}
				}
				_loc13_->offset = _loc38_;
				_loc12_ = _loc12_->next;
			}
			if ((!(_loc35_)))
			{
				if (_loc21_ != null)
				{
					_loc22_->processNext = _loc10_;
				}
				else
				{
					_loc21_ = _loc10_;
				}
				_loc22_ = _loc10_;
			}
			else
			{
				if ((!(_loc36_)))
				{
					if (_loc37_)
					{
						if (_loc19_ != null)
						{
							_loc20_->processNext = _loc10_;
						}
						else
						{
							_loc19_ = _loc10_;
						}
						_loc20_ = _loc10_;
					}
					else
					{
						_loc29_->offset = _loc32_;
						_loc30_->offset = _loc33_;
						_loc31_->offset = _loc34_;
						_loc23_ = _loc10_->create();
						_loc23_->material = _loc10_->material;
						param1->lastFace.next = _loc23_;
						param1->lastFace = _loc23_;
						_loc25_ = null;
						_loc28_ = ((!(_loc10_->material == null)) && (_loc10_->material.useVerticesNormals));
						_loc12_ = _loc10_->wrapper;
						while (_loc12_ != null)
						{
							_loc30_ = _loc12_->vertex;
							if (_loc30_->offset >= param8)
							{
								_loc14_ = _loc30_->create();
								param1->lastVertex.next = _loc14_;
								param1->lastVertex = _loc14_;
								_loc14_->x = _loc30_->x;
								_loc14_->y = _loc30_->y;
								_loc14_->z = _loc30_->z;
								_loc14_->u = _loc30_->u;
								_loc14_->v = _loc30_->v;
								_loc14_->cameraX = _loc30_->cameraX;
								_loc14_->cameraY = _loc30_->cameraY;
								_loc14_->cameraZ = _loc30_->cameraZ;
								if (_loc28_)
								{
									_loc14_->normalX = _loc30_->normalX;
									_loc14_->normalY = _loc30_->normalY;
									_loc14_->normalZ = _loc30_->normalZ;
								}
								_loc30_ = _loc14_;
							}
							_loc27_ = _loc12_->create();
							_loc27_->vertex = _loc30_;
							if (_loc25_ != null)
							{
								_loc25_->next = _loc27_;
							}
							else
							{
								_loc23_->wrapper = _loc27_;
							}
							_loc25_ = _loc27_;
							_loc12_ = _loc12_->next;
						}
						if (_loc19_ != null)
						{
							_loc20_->processNext = _loc23_;
						}
						else
						{
							_loc19_ = _loc23_;
						}
						_loc20_ = _loc23_;
						_loc10_->processNext = null;
					}
				}
				else
				{
					_loc29_->offset = _loc32_;
					_loc30_->offset = _loc33_;
					_loc31_->offset = _loc34_;
					_loc23_ = _loc10_->create();
					_loc23_->material = _loc10_->material;
					param1->lastFace.next = _loc23_;
					param1->lastFace = _loc23_;
					_loc24_ = _loc10_->create();
					_loc24_->material = _loc10_->material;
					param1->lastFace.next = _loc24_;
					param1->lastFace = _loc24_;
					_loc25_ = null;
					_loc26_ = null;
					_loc12_ = _loc10_->wrapper.next.next;
					while (_loc12_->next != null)
					{
						_loc12_ = _loc12_->next;
					}
					_loc29_ = _loc12_->vertex;
					_loc32_ = _loc29_->offset;
					_loc28_ = ((!(_loc10_->material == null)) && (_loc10_->material.useVerticesNormals));
					_loc12_ = _loc10_->wrapper;
					while (_loc12_ != null)
					{
						_loc30_ = _loc12_->vertex;
						_loc33_ = _loc30_->offset;
						if ((((_loc32_ < param8) && (_loc33_ > param9)) || ((_loc32_ > param9) && (_loc33_ < param8))))
						{
							_loc39_ = ((param7 - _loc32_) / (_loc33_ - _loc32_));
							_loc13_ = _loc30_->create();
							param1->lastVertex.next = _loc13_;
							param1->lastVertex = _loc13_;
							_loc13_->x = (_loc29_->x + ((_loc30_->x - _loc29_->x) * _loc39_));
							_loc13_->y = (_loc29_->y + ((_loc30_->y - _loc29_->y) * _loc39_));
							_loc13_->z = (_loc29_->z + ((_loc30_->z - _loc29_->z) * _loc39_));
							_loc13_->u = (_loc29_->u + ((_loc30_->u - _loc29_->u) * _loc39_));
							_loc13_->v = (_loc29_->v + ((_loc30_->v - _loc29_->v) * _loc39_));
							_loc13_->cameraX = (_loc29_->cameraX + ((_loc30_->cameraX - _loc29_->cameraX) * _loc39_));
							_loc13_->cameraY = (_loc29_->cameraY + ((_loc30_->cameraY - _loc29_->cameraY) * _loc39_));
							_loc13_->cameraZ = (_loc29_->cameraZ + ((_loc30_->cameraZ - _loc29_->cameraZ) * _loc39_));
							if (_loc28_)
							{
								_loc13_->normalX = (_loc29_->normalX + ((_loc30_->normalX - _loc29_->normalX) * _loc39_));
								_loc13_->normalY = (_loc29_->normalY + ((_loc30_->normalY - _loc29_->normalY) * _loc39_));
								_loc13_->normalZ = (_loc29_->normalZ + ((_loc30_->normalZ - _loc29_->normalZ) * _loc39_));
							}
							_loc27_ = _loc12_->create();
							_loc27_->vertex = _loc13_;
							if (_loc25_ != null)
							{
								_loc25_->next = _loc27_;
							}
							else
							{
								_loc23_->wrapper = _loc27_;
							}
							_loc25_ = _loc27_;
							_loc14_ = _loc30_->create();
							param1->lastVertex.next = _loc14_;
							param1->lastVertex = _loc14_;
							_loc14_->x = _loc13_->x;
							_loc14_->y = _loc13_->y;
							_loc14_->z = _loc13_->z;
							_loc14_->u = _loc13_->u;
							_loc14_->v = _loc13_->v;
							_loc14_->cameraX = _loc13_->cameraX;
							_loc14_->cameraY = _loc13_->cameraY;
							_loc14_->cameraZ = _loc13_->cameraZ;
							if (_loc28_)
							{
								_loc14_->normalX = _loc13_->normalX;
								_loc14_->normalY = _loc13_->normalY;
								_loc14_->normalZ = _loc13_->normalZ;
							}
							_loc27_ = _loc12_->create();
							_loc27_->vertex = _loc14_;
							if (_loc26_ != null)
							{
								_loc26_->next = _loc27_;
							}
							else
							{
								_loc24_->wrapper = _loc27_;
							}
							_loc26_ = _loc27_;
						}
						if (_loc30_->offset < param8)
						{
							_loc27_ = _loc12_->create();
							_loc27_->vertex = _loc30_;
							if (_loc25_ != null)
							{
								_loc25_->next = _loc27_;
							}
							else
							{
								_loc23_->wrapper = _loc27_;
							}
							_loc25_ = _loc27_;
						}
						else
						{
							if (_loc30_->offset > param9)
							{
								_loc27_ = _loc12_->create();
								_loc27_->vertex = _loc30_;
								if (_loc26_ != null)
								{
									_loc26_->next = _loc27_;
								}
								else
								{
									_loc24_->wrapper = _loc27_;
								}
								_loc26_ = _loc27_;
							}
							else
							{
								_loc27_ = _loc12_->create();
								_loc27_->vertex = _loc30_;
								if (_loc26_ != null)
								{
									_loc26_->next = _loc27_;
								}
								else
								{
									_loc24_->wrapper = _loc27_;
								}
								_loc26_ = _loc27_;
								_loc14_ = _loc30_->create();
								param1->lastVertex.next = _loc14_;
								param1->lastVertex = _loc14_;
								_loc14_->x = _loc30_->x;
								_loc14_->y = _loc30_->y;
								_loc14_->z = _loc30_->z;
								_loc14_->u = _loc30_->u;
								_loc14_->v = _loc30_->v;
								_loc14_->cameraX = _loc30_->cameraX;
								_loc14_->cameraY = _loc30_->cameraY;
								_loc14_->cameraZ = _loc30_->cameraZ;
								if (_loc28_)
								{
									_loc14_->normalX = _loc30_->normalX;
									_loc14_->normalY = _loc30_->normalY;
									_loc14_->normalZ = _loc30_->normalZ;
								}
								_loc27_ = _loc12_->create();
								_loc27_->vertex = _loc14_;
								if (_loc25_ != null)
								{
									_loc25_->next = _loc27_;
								}
								else
								{
									_loc23_->wrapper = _loc27_;
								}
								_loc25_ = _loc27_;
							}
						}
						_loc29_ = _loc30_;
						_loc32_ = _loc33_;
						_loc12_ = _loc12_->next;
					}
					if (_loc19_ != null)
					{
						_loc20_->processNext = _loc23_;
					}
					else
					{
						_loc19_ = _loc23_;
					}
					_loc20_ = _loc23_;
					if (_loc21_ != null)
					{
						_loc22_->processNext = _loc24_;
					}
					else
					{
						_loc21_ = _loc24_;
					}
					_loc22_ = _loc24_;
					_loc10_->processNext = null;
				}
			}
			_loc10_ = _loc11_;
		}
	}
	if (((!(_loc19_ == null)) || ((!(_loc15_ == null)) && (!(_loc17_ == null)))))
	{
		if (_loc19_ == null)
		{
			_loc19_ = param2->create();
			param1->lastFace.next = _loc19_;
			param1->lastFace = _loc19_;
		}
		else
		{
			_loc20_->processNext = null;
		}
		if (this->sorting == 3)
		{
			_loc19_->normalX = param2->normalX;
			_loc19_->normalY = param2->normalY;
			_loc19_->normalZ = param2->normalZ;
			_loc19_->offset = param2->offset;
		}
		_loc19_->processNegative = _loc15_;
		_loc19_->processPositive = _loc17_;
		param3.processNegative = _loc19_;
	}
	else
	{
		param3.processNegative = ((!(_loc15_ == null)) ? _loc15_ : _loc17_);
	}
	if (((!(_loc21_ == null)) || ((!(_loc16_ == null)) && (!(_loc18_ == null)))))
	{
		if (_loc21_ == null)
		{
			_loc21_ = param2->create();
			param1->lastFace.next = _loc21_;
			param1->lastFace = _loc21_;
		}
		else
		{
			_loc22_->processNext = null;
		}
		if (this->sorting == 3)
		{
			_loc21_->normalX = param2->normalX;
			_loc21_->normalY = param2->normalY;
			_loc21_->normalZ = param2->normalZ;
			_loc21_->offset = param2->offset;
		}
		_loc21_->processNegative = _loc16_;
		_loc21_->processPositive = _loc18_;
		param3.processPositive = _loc21_;
	}
	else
	{
		param3.processPositive = ((!(_loc16_ == null)) ? _loc16_ : _loc18_);
	}
#endif
}

//not used
Face *VG::cropFaceStruct(Camera3D *param1, Face *param2, double param3, double param4, double param5, double param6, double param7, double param8)
{
	_function_name("VG::cropFaceStruct()");

	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;
	(void)param7;
	(void)param8;

	qDebug("used VG::cropFaceStruct");

#if 0
	var _loc9_:Face;
	var _loc10_:Face;
	var _loc11_:Wrapper;
	var _loc12_:Vertex;
	var _loc13_:Face;
	var _loc14_:Face;
	var _loc15_:Face;
	var _loc16_:Face;
	var _loc17_:Vertex;
	var _loc18_:Vertex;
	var _loc19_:Vertex;
	var _loc20_:Number = NaN;
	var _loc21_:Number = NaN;
	var _loc22_:Number = NaN;
	var _loc23_:Boolean;
	var _loc24_:Boolean;
	var _loc25_:Number = NaN;
	var _loc26_:Face;
	var _loc27_:Wrapper;
	var _loc28_:Wrapper;
	var _loc29_:Boolean;
	var _loc30_:Number = NaN;
	if (param2->processNegative != null)
	{
		_loc13_ = this->cropFaceStruct(param1, param2->processNegative, param3, param4, param5, param6, param7, param8);
		param2->processNegative = null;
	}
	if (param2->processPositive != null)
	{
		_loc14_ = this->cropFaceStruct(param1, param2->processPositive, param3, param4, param5, param6, param7, param8);
		param2->processPositive = null;
	}
	if (param2->wrapper != null)
	{
		_loc9_ = param2;
		while (_loc9_ != null)
		{
			_loc10_ = _loc9_->processNext;
			_loc11_ = _loc9_->wrapper;
			_loc17_ = _loc11_->vertex;
			_loc11_ = _loc11_->next;
			_loc18_ = _loc11_->vertex;
			_loc11_ = _loc11_->next;
			_loc19_ = _loc11_->vertex;
			_loc11_ = _loc11_->next;
			_loc20_ = (((_loc17_->cameraX * param3) + (_loc17_->cameraY * param4)) + (_loc17_->cameraZ * param5));
			_loc21_ = (((_loc18_->cameraX * param3) + (_loc18_->cameraY * param4)) + (_loc18_->cameraZ * param5));
			_loc22_ = (((_loc19_->cameraX * param3) + (_loc19_->cameraY * param4)) + (_loc19_->cameraZ * param5));
			_loc23_ = (((_loc20_ < param7) || (_loc21_ < param7)) || (_loc22_ < param7));
			_loc24_ = (((_loc20_ > param8) || (_loc21_ > param8)) || (_loc22_ > param8));
			while (_loc11_ != null)
			{
				_loc12_ = _loc11_->vertex;
				_loc25_ = (((_loc12_->cameraX * param3) + (_loc12_->cameraY * param4)) + (_loc12_->cameraZ * param5));
				if (_loc25_ < param7)
				{
					_loc23_ = true;
				}
				else
				{
					if (_loc25_ > param8)
					{
						_loc24_ = true;
					}
				}
				_loc12_->offset = _loc25_;
				_loc11_ = _loc11_->next;
			}
			if ((!(_loc24_)))
			{
				_loc9_->processNext = null;
			}
			else
			{
				if ((!(_loc23_)))
				{
					if (_loc15_ != null)
					{
						_loc16_->processNext = _loc9_;
					}
					else
					{
						_loc15_ = _loc9_;
					}
					_loc16_ = _loc9_;
				}
				else
				{
					_loc17_->offset = _loc20_;
					_loc18_->offset = _loc21_;
					_loc19_->offset = _loc22_;
					_loc26_ = _loc9_->create();
					_loc26_->material = _loc9_->material;
					param1->lastFace.next = _loc26_;
					param1->lastFace = _loc26_;
					_loc27_ = null;
					_loc11_ = _loc9_->wrapper.next.next;
					while (_loc11_->next != null)
					{
						_loc11_ = _loc11_->next;
					}
					_loc17_ = _loc11_->vertex;
					_loc20_ = _loc17_->offset;
					_loc29_ = ((!(_loc9_->material == null)) && (_loc9_->material.useVerticesNormals));
					_loc11_ = _loc9_->wrapper;
					while (_loc11_ != null)
					{
						_loc18_ = _loc11_->vertex;
						_loc21_ = _loc18_->offset;
						if ((((_loc20_ < param7) && (_loc21_ > param8)) || ((_loc20_ > param8) && (_loc21_ < param7))))
						{
							_loc30_ = ((param6 - _loc20_) / (_loc21_ - _loc20_));
							_loc12_ = _loc18_->create();
							param1->lastVertex.next = _loc12_;
							param1->lastVertex = _loc12_;
							_loc12_->x = (_loc17_->x + ((_loc18_->x - _loc17_->x) * _loc30_));
							_loc12_->y = (_loc17_->y + ((_loc18_->y - _loc17_->y) * _loc30_));
							_loc12_->z = (_loc17_->z + ((_loc18_->z - _loc17_->z) * _loc30_));
							_loc12_->u = (_loc17_->u + ((_loc18_->u - _loc17_->u) * _loc30_));
							_loc12_->v = (_loc17_->v + ((_loc18_->v - _loc17_->v) * _loc30_));
							_loc12_->cameraX = (_loc17_->cameraX + ((_loc18_->cameraX - _loc17_->cameraX) * _loc30_));
							_loc12_->cameraY = (_loc17_->cameraY + ((_loc18_->cameraY - _loc17_->cameraY) * _loc30_));
							_loc12_->cameraZ = (_loc17_->cameraZ + ((_loc18_->cameraZ - _loc17_->cameraZ) * _loc30_));
							if (_loc29_)
							{
								_loc12_->normalX = (_loc17_->normalX + ((_loc18_->normalX - _loc17_->normalX) * _loc30_));
								_loc12_->normalY = (_loc17_->normalY + ((_loc18_->normalY - _loc17_->normalY) * _loc30_));
								_loc12_->normalZ = (_loc17_->normalZ + ((_loc18_->normalZ - _loc17_->normalZ) * _loc30_));
							}
							_loc28_ = _loc11_->create();
							_loc28_->vertex = _loc12_;
							if (_loc27_ != null)
							{
								_loc27_->next = _loc28_;
							}
							else
							{
								_loc26_->wrapper = _loc28_;
							}
							_loc27_ = _loc28_;
						}
						if (_loc21_ >= param7)
						{
							_loc28_ = _loc11_->create();
							_loc28_->vertex = _loc18_;
							if (_loc27_ != null)
							{
								_loc27_->next = _loc28_;
							}
							else
							{
								_loc26_->wrapper = _loc28_;
							}
							_loc27_ = _loc28_;
						}
						_loc17_ = _loc18_;
						_loc20_ = _loc21_;
						_loc11_ = _loc11_->next;
					}
					if (_loc15_ != null)
					{
						_loc16_->processNext = _loc26_;
					}
					else
					{
						_loc15_ = _loc26_;
					}
					_loc16_ = _loc26_;
					_loc9_->processNext = null;
				}
			}
			_loc9_ = _loc10_;
		}
	}
	if (((!(_loc15_ == null)) || ((!(_loc13_ == null)) && (!(_loc14_ == null)))))
	{
		if (_loc15_ == null)
		{
			_loc15_ = param2->create();
			param1->lastFace.next = _loc15_;
			param1->lastFace = _loc15_;
		}
		else
		{
			_loc16_->processNext = null;
		}
		if (this->sorting == 3)
		{
			_loc15_->normalX = param2->normalX;
			_loc15_->normalY = param2->normalY;
			_loc15_->normalZ = param2->normalZ;
			_loc15_->offset = param2->offset;
		}
		_loc15_->processNegative = _loc13_;
		_loc15_->processPositive = _loc14_;
		return (_loc15_);
	}
	return ((!(_loc13_ == null)) ? _loc13_ : _loc14_);
#endif
	return nullptr;
}

void VG::transformStruct(Face *param1, int param2, const Matrix4 *matrix)
{
	_function_name("VG::transformStruct()");


	double param3 = matrix->ma;
	double param4 = matrix->mb;
	double param5 = matrix->mc;
	double param6 = matrix->md;
	double param7 = matrix->me;
	double param8 = matrix->mf;
	double param9 = matrix->mg;
	double param10 = matrix->mh;
	double param11 = matrix->mi;
	double param12 = matrix->mj;
	double param13 = matrix->mk;
	double param14 = matrix->ml;


	Face *_loc15_ = param1;
	while (_loc15_ != nullptr)
	{
		Wrapper *_loc16_ = _loc15_->wrapper.get();
		while (_loc16_ != nullptr)
		{
			Vertex *_loc17_ = _loc16_->vertex.get();
			if (_loc17_->transformId != param2)
			{
				double _loc18_ = _loc17_->cameraX;
				double _loc19_ = _loc17_->cameraY;
				double _loc20_ = _loc17_->cameraZ;
				_loc17_->cameraX = ((((param3 * _loc18_) + (param4 * _loc19_)) + (param5 * _loc20_)) + param6);
				_loc17_->cameraY = ((((param7 * _loc18_) + (param8 * _loc19_)) + (param9 * _loc20_)) + param10);
				_loc17_->cameraZ = ((((param11 * _loc18_) + (param12 * _loc19_)) + (param13 * _loc20_)) + param14);
				_loc17_->transformId = param2;
			}
			_loc16_ = _loc16_->_next;
		}
		_loc15_ = _loc15_->processNext.get();
	}
	if (param1->processNegative != nullptr)
	{
		this->transformStruct(param1->processNegative.get(), param2, matrix);
	}
	if (param1->processPositive != nullptr)
	{
		this->transformStruct(param1->processPositive.get(), param2, matrix);
	}
}

void VG::draw(Camera3D *param1, double param2, Object3D *param3)
{
	_function_name("VG::draw()");

	if (this->faceStruct() == nullptr)
	{
		qDebug("error VG::draw");
		//return;
	}

	Face *_loc6_ = nullptr;


	if (this->space == 1)
	{
		this->transformStruct(this->faceStruct().get(), ++this->object->transformId, &param3->matrix
							 /*param3->matrix.ma,
							  param3->matrix.mb,
							  param3->matrix.mc,
							  param3->matrix.md,
							  param3->matrix.me,
							  param3->matrix.mf,
							  param3->matrix.mg,
							  param3->matrix.mh,
							  param3->matrix.mi,
							  param3->matrix.mj,
							  param3->matrix.mk,
							  param3->matrix.ml*/);
	}

	if (this->viewAligned)
	{
		std::shared_ptr<Face> faceStruct = this->faceStruct();
		/*
		if (this->debug > 0)
		{
			if ((this->debug & Debug.EDGES))
			{
				Debug.drawEdges(param1, _loc4_, ((!(this->space == 2)) ? int(0xFFFFFF) : int(0xFF9900)));
			}
			if ((this->debug & Debug.BOUNDS))
			{
				if (this->space == 1)
				{
					Debug.drawBounds(param1, param3, this->boundMinX, this->boundMinY, this->boundMinZ, this->boundMaxX, this->boundMaxY, this->boundMaxZ, 0x99FF00);
				}
			}
		}
		*/
		param1->addTransparent(faceStruct, this->object);
	}
	else
	{
		Face *_loc4_;

		switch (this->sorting)
		{
			case 0:
				_loc4_ = this->faceStruct().get();
				break;
			case 1:
				_loc4_ = ((!(this->faceStruct()->processNext == nullptr)) ? param1->sortByAverageZ(this->faceStruct().get()) : this->faceStruct().get());
				break;
			case 2:
				_loc4_ = ((!(this->faceStruct()->processNext == nullptr)) ? param1->sortByDynamicBSP(this->faceStruct().get(), param2) : this->faceStruct().get());
				break;
			case 3:
				_loc4_ = collectNode(this->faceStruct().get(), nullptr);
				break;
			default:
				_loc4_ = nullptr;
				break;
		}

		if (this->debug > 0)
		{
			/*
			if ((this->debug & Debug.EDGES))
			{
				Debug.drawEdges(param1, _loc4_, 0xFFFFFF);
			}
			if ((this->debug & Debug.BOUNDS))
			{
				if (this->space == 1)
				{
					Debug.drawBounds(param1, param3, this->boundMinX, this->boundMinY, this->boundMinZ, this->boundMaxX, this->boundMaxY, this->boundMaxZ, 0x99FF00);
				}
				else
				{
					if (this->space == 2)
					{
						Debug.drawBounds(param1, this->object, this->boundMinX, this->boundMinY, this->boundMinZ, this->boundMaxX, this->boundMaxY, this->boundMaxZ, 0xFF9900);
					}
				}
			}
			*/
		}

		Face *_loc7_ = _loc4_;
		while (_loc7_ != nullptr)
		{
			Face *_loc5_ = _loc7_->processNext.get();
			if (((_loc5_ == nullptr) || (!(_loc5_->material == _loc4_->material))))
			{
				_loc7_->processNext = nullptr;
				if (_loc4_->material != nullptr)
				{
					_loc4_->processNegative = Face::ptr_safe(_loc6_);
					_loc6_ = _loc4_;
				}
				else
				{
					while (_loc4_ != nullptr)
					{
						_loc7_ = _loc4_->processNext.get();
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
			Face *_loc5_ = _loc4_->processNegative.get();
			_loc4_->processNegative = nullptr;
			std::shared_ptr<Face> face = _loc4_->ptr();
			param1->addTransparent(face, this->object);
			_loc4_ = _loc5_;
		}
	}

	this->setFaceStruct(nullptr, 6);
}


//not used
static Face* collectNode(Face *param1, Face *param2)
{
	_function_name("VG::collectNode()");

	Face *_loc4_;
	Face *_loc5_;

	if (param1->offset < 0)
	{
		_loc4_ = param1->processNegative.get();
		_loc5_ = param1->processPositive.get();
	}
	else
	{
		_loc4_ = param1->processPositive.get();
		_loc5_ = param1->processNegative.get();
	}
	param1->processNegative = nullptr;
	param1->processPositive = nullptr;
	if (_loc5_ != nullptr)
	{
		param2 = collectNode(_loc5_, param2);
	}
	if (param1->wrapper != nullptr)
	{
		Face *_loc3_ = param1;
		while (_loc3_->processNext != nullptr)
		{
			_loc3_ = _loc3_->processNext.get();
		}
		_loc3_->processNext = Face::ptr_safe(param2);
		param2 = param1;
	}
	if (_loc4_ != nullptr)
	{
		param2 = collectNode(_loc4_, param2);
	}
	return param2;
}


//not used
void VG::split(Camera3D *param1, double param2, double param3, double param4, double param5, double param6)
{
	_function_name("VG::split()");

	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;

	qDebug("used VG::split");

#if 0
	var _loc8_:VG;
	var _loc7_:Face = this->faceStruct.create();
	this->splitFaceStruct(param1, this->faceStruct, _loc7_, param2, param3, param4, param5, (param5 - param6), (param5 + param6));
	if (_loc7_->processNegative != null)
	{
		if (collector != null)
		{
			_loc8_ = collector;
			collector = collector.next;
			_loc8_->next = null;
		}
		else
		{
			_loc8_ = new VG();
		}
		this->next = _loc8_;
		_loc8_->faceStruct = _loc7_->processNegative;
		_loc7_->processNegative = null;
		_loc8_->object = this->object;
		_loc8_->sorting = this->sorting;
		_loc8_->debug = this->debug;
		_loc8_->space = this->space;
		_loc8_->viewAligned = this->viewAligned;
		_loc8_->boundMinX = 1E22;
		_loc8_->boundMinY = 1E22;
		_loc8_->boundMinZ = 1E22;
		_loc8_->boundMaxX = -1E22;
		_loc8_->boundMaxY = -1E22;
		_loc8_->boundMaxZ = -1E22;
		_loc8_->updateAABBStruct(_loc8_->faceStruct, ++this->object.transformId);
	}
	else
	{
		this->next = null;
	}
	if (_loc7_->processPositive != null)
	{
		this->faceStruct = _loc7_->processPositive;
		_loc7_->processPositive = null;
		this->boundMinX = 1E22;
		this->boundMinY = 1E22;
		this->boundMinZ = 1E22;
		this->boundMaxX = -1E22;
		this->boundMaxY = -1E22;
		this->boundMaxZ = -1E22;
		this->updateAABBStruct(this->faceStruct, ++this->object.transformId);
	}
	else
	{
		this->faceStruct = null;
	}
	_loc7_->next = Face.collector;
	Face.collector = _loc7_;
#endif
}

//not used
void VG::crop(Camera3D *param1, double param2, double param3, double param4, double param5, double param6)
{
	_function_name("VG::crop()");

	(void)param1;
	(void)param2;
	(void)param3;
	(void)param4;
	(void)param5;
	(void)param6;

	qDebug("used VG::crop");

#if 0
	this->faceStruct = this->cropFaceStruct(param1, this->faceStruct, param2, param3, param4, param5, (param5 - param6), (param5 + param6));
	if (this->faceStruct != null)
	{
		this->boundMinX = 1E22;
		this->boundMinY = 1E22;
		this->boundMinZ = 1E22;
		this->boundMaxX = -1E22;
		this->boundMaxY = -1E22;
		this->boundMaxZ = -1E22;
		this->updateAABBStruct(this->faceStruct, ++this->object.transformId);
	}
#endif
}
