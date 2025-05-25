#include "kdnode.h"
#include "alternativa/engine3d/objects/mesh.h"
#include "alternativa/engine3d/objects/bsp.h"
#include "alternativa/engine3d/materials/texturematerial.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/engine3d/core/shadow.h"
#include "alternativa/engine3d/objects/mesh.h"
#include "alternativa/engine3d/objects/decal.h"


static void clip(Decal *param1, double param2, double param3, Object3D *param4);


static QVector<std::shared_ptr<Face>> getFaces(Mesh *mesh)
{
	QVector<std::shared_ptr<Face>> faces;

	std::shared_ptr<Face> face = mesh->faceList();

	while (face != nullptr)
	{
		faces.append(face);
		face = face->next;
	}

	return faces;
}



KDNode::KDNode()
{
	negative = nullptr;
	positive = nullptr;
}

void KDNode::createReceivers(QVector<std::shared_ptr<QVector<double>>> *param1, QVector<std::shared_ptr<QVector<uint32_t>>> *param2)
{
	std::shared_ptr<Receiver> _loc3_;
	Vertex *_loc7_ = nullptr;
	QVector<std::shared_ptr<Face>> _loc8_;

	std::shared_ptr<QVector<double>> _loc13_;
	std::shared_ptr<QVector<uint32_t>> _loc14_;

	this->receiverList = nullptr;

	Object3D *_loc4_ = this->objectList.get();

	while (_loc4_ != nullptr)
	{
		_loc4_->composeMatrix();
		std::shared_ptr<Receiver> _loc5_ = std::make_shared<Receiver>();
		if (_loc3_ != nullptr)
		{
			_loc3_->next = _loc5_;
		}
		else
		{
			this->receiverList = _loc5_;
		}
		_loc3_ = _loc5_;
		if (_loc4_->isType(Mesh::TYPE))
		{
			Mesh *mesh = (Mesh*)_loc4_;
			_loc7_ = mesh->vertexList.get();
			_loc8_ = getFaces(mesh);
		}
		else
		{
			if (_loc4_->isType(BSP::TYPE))
			{
				BSP *bsp = (BSP*)_loc4_;
				_loc7_ = bsp->vertexList.get();
				_loc8_ = *bsp->faces;
			}
		}

		int _loc9_ = _loc8_.length();

		std::shared_ptr<Material> m = _loc8_.at(0)->material;

		TextureMaterial *_loc10_;

		if (m->isType(TextureMaterial::TYPE))
		{
			_loc10_ = (TextureMaterial*)m.get();
		}
		else
		{
			_loc10_ = nullptr;
		}


		if (((_loc9_ > 0) && (!(_loc10_ == nullptr))))
		{
			int _loc11_ = 0;
			Vertex *_loc6_ = _loc7_;
			while (_loc6_ != nullptr)
			{
				_loc11_++;
				_loc6_ = _loc6_->next.get();
			}

			int _loc12_ = param1->length() - 1;
			//_loc13_ = param1[_loc12_];
			_loc13_ = param1->last();

			if (((_loc13_->length() / 3) + _loc11_) > 0xFFFF)
			{
				//leakage
				param1->append(std::make_shared<QVector<double>>());
				param2->append(std::make_shared<QVector<uint32_t>>());
				_loc12_++;
				//param1[_loc12_] = new Vector.<Number>();
				//param2[_loc12_] = new Vector.<uint>();
				//_loc13_ = param1->at(_loc12_);
				_loc13_ = param1->last();
			}

			//_loc14_ = param2[_loc12_];
			_loc14_ = param2->last();

			int _loc15_ = _loc13_->length();
			int _loc16_ = _loc15_ / 3;
			int _loc17_ = _loc14_->length();
			_loc5_->buffer = _loc12_;
			_loc5_->firstIndex = _loc17_;
			_loc5_->transparent = _loc10_->transparent();
			_loc6_ = _loc7_;
			while (_loc6_ != nullptr)
			{
				_loc13_->append((((_loc6_->x * _loc4_->matrix.ma) + (_loc6_->y * _loc4_->matrix.mb)) + (_loc6_->z * _loc4_->matrix.mc)) + _loc4_->matrix.md);
				_loc15_++;
				_loc13_->append((((_loc6_->x * _loc4_->matrix.me) + (_loc6_->y * _loc4_->matrix.mf)) + (_loc6_->z * _loc4_->matrix.mg)) + _loc4_->matrix.mh);
				_loc15_++;
				_loc13_->append((((_loc6_->x * _loc4_->matrix.mi) + (_loc6_->y * _loc4_->matrix.mj)) + (_loc6_->z * _loc4_->matrix.mk)) + _loc4_->matrix.ml);
				_loc15_++;
				_loc6_->index = _loc16_;
				_loc16_++;
				_loc6_ = _loc6_->next.get();
			}

			int _loc18_ = 0;
			while (_loc18_ < _loc9_)
			{
				Face *_loc19_ = _loc8_.at(_loc18_).get();
				if ((((_loc19_->normalX * _loc4_->matrix.mi) + (_loc19_->normalY * _loc4_->matrix.mj)) + (_loc19_->normalZ * _loc4_->matrix.mk)) >= -0.5)
				{
					Wrapper *_loc20_ = _loc19_->wrapper.get();
					uint _loc21_ = _loc20_->vertex->index;
					_loc20_ = _loc20_->_next;
					uint _loc22_ = _loc20_->vertex->index;
					_loc20_ = _loc20_->_next;
					while (_loc20_ != nullptr)
					{
						uint _loc23_ = _loc20_->vertex->index;
						_loc14_->append(_loc21_);
						_loc17_++;
						_loc14_->append(_loc22_);
						_loc17_++;
						_loc14_->append(_loc23_);
						_loc17_++;
						_loc5_->numTriangles++;
						_loc22_ = _loc23_;
						_loc20_ = _loc20_->_next;
					}
				}
				_loc18_++;
			}
		}
		_loc4_ = _loc4_->next.get();
	}
	if (this->negative != nullptr)
	{
		this->negative->createReceivers(param1, param2);
	}
	if (this->positive != nullptr)
	{
		this->positive->createReceivers(param1, param2);
	}
}


void KDNode::collectReceivers(Shadow *param1)
{
	if (this->negative != nullptr)
	{
		bool _loc5_ = (this->axis == 0);
		bool _loc6_ = (this->axis == 1);
		double _loc7_ = ((!(!(_loc5_))) ? param1->bound.MinX : ((!(!(_loc6_))) ? param1->bound.MinY : param1->bound.MinZ));
		double _loc8_ = ((!(!(_loc5_))) ? param1->bound.MaxX : ((!(!(_loc6_))) ? param1->bound.MaxY : param1->bound.MaxZ));
		if (_loc8_ <= this->maxCoord)
		{
			this->negative->collectReceivers(param1);
		}
		else
		{
			if (_loc7_ >= this->minCoord)
			{
				this->positive->collectReceivers(param1);
			}
			else
			{
				if (_loc5_)
				{
					Object3D *_loc3_ = this->objectBoundList.get();
					Object3D *_loc2_ = this->objectList.get();
					Receiver *_loc4_ = this->receiverList.get();
					while (_loc3_ != nullptr)
					{
						if ((((((_loc4_->numTriangles > 0) && (param1->bound.MinY < _loc3_->bound.MaxY)) && (param1->bound.MaxY > _loc3_->bound.MinY)) && (param1->bound.MinZ < _loc3_->bound.MaxZ)) && (param1->bound.MaxZ > _loc3_->bound.MinZ)))
						{
							if (!_loc4_->transparent)
							{
								if (param1->receiversBuffers.length() < (param1->receiversCount+1))
									param1->receiversBuffers.resize(param1->receiversCount+1);

								if (param1->receiversFirstIndexes.length() < (param1->receiversCount+1))
									param1->receiversFirstIndexes.resize(param1->receiversCount+1);

								if (param1->receiversNumsTriangles.length() < (param1->receiversCount+1))
									param1->receiversNumsTriangles.resize(param1->receiversCount+1);


								param1->receiversBuffers[param1->receiversCount] = _loc4_->buffer;
								param1->receiversFirstIndexes[param1->receiversCount] = _loc4_->firstIndex;
								param1->receiversNumsTriangles[param1->receiversCount] = _loc4_->numTriangles;
								param1->receiversCount++;
							}
						}
						_loc3_ = _loc3_->next.get();
						_loc2_ = _loc2_->next.get();
						_loc4_ = _loc4_->next.get();
					}
				}
				else
				{
					if (_loc6_)
					{
						Object3D *_loc3_ = this->objectBoundList.get();
						Object3D *_loc2_ = this->objectList.get();
						Receiver *_loc4_ = this->receiverList.get();
						while (_loc3_ != nullptr)
						{
							if ((((((_loc4_->numTriangles > 0) && (param1->bound.MinX < _loc3_->bound.MaxX)) && (param1->bound.MaxX > _loc3_->bound.MinX)) && (param1->bound.MinZ < _loc3_->bound.MaxZ)) && (param1->bound.MaxZ > _loc3_->bound.MinZ)))
							{
								if (!_loc4_->transparent)
								{
									if (param1->receiversBuffers.length() < (param1->receiversCount+1))
										param1->receiversBuffers.resize(param1->receiversCount+1);

									if (param1->receiversFirstIndexes.length() < (param1->receiversCount+1))
										param1->receiversFirstIndexes.resize(param1->receiversCount+1);

									if (param1->receiversNumsTriangles.length() < (param1->receiversCount+1))
										param1->receiversNumsTriangles.resize(param1->receiversCount+1);

									param1->receiversBuffers[param1->receiversCount] = _loc4_->buffer;
									param1->receiversFirstIndexes[param1->receiversCount] = _loc4_->firstIndex;
									param1->receiversNumsTriangles[param1->receiversCount] = _loc4_->numTriangles;
									param1->receiversCount++;
								}
							}
							_loc3_ = _loc3_->next.get();
							_loc2_ = _loc2_->next.get();
							_loc4_ = _loc4_->next.get();
						}
					}
					else
					{
						Object3D *_loc3_ = this->objectBoundList.get();
						Object3D *_loc2_ = this->objectList.get();
						Receiver *_loc4_ = this->receiverList.get();
						while (_loc3_ != nullptr)
						{
							if ((((((_loc4_->numTriangles > 0) && (param1->bound.MinX < _loc3_->bound.MaxX)) && (param1->bound.MaxX > _loc3_->bound.MinX)) && (param1->bound.MinY < _loc3_->bound.MaxY)) && (param1->bound.MaxY > _loc3_->bound.MinY)))
							{
								if (!_loc4_->transparent)
								{
									if (param1->receiversBuffers.length() < (param1->receiversCount+1))
										param1->receiversBuffers.resize(param1->receiversCount+1);

									if (param1->receiversFirstIndexes.length() < (param1->receiversCount+1))
										param1->receiversFirstIndexes.resize(param1->receiversCount+1);

									if (param1->receiversNumsTriangles.length() < (param1->receiversCount+1))
										param1->receiversNumsTriangles.resize(param1->receiversCount+1);

									param1->receiversBuffers[param1->receiversCount] = _loc4_->buffer;
									param1->receiversFirstIndexes[param1->receiversCount] = _loc4_->firstIndex;
									param1->receiversNumsTriangles[param1->receiversCount] = _loc4_->numTriangles;
									param1->receiversCount++;
								}
							}
							_loc3_ = _loc3_->next.get();
							_loc2_ = _loc2_->next.get();
							_loc4_ = _loc4_->next.get();
						}
					}
				}
				this->negative->collectReceivers(param1);
				this->positive->collectReceivers(param1);
			}
		}
	}
	else
	{
		Object3D *_loc2_ = this->objectList.get();
		Receiver *_loc4_ = this->receiverList.get();
		while (_loc4_ != nullptr)
		{
			if (_loc4_->numTriangles > 0)
			{
				if (!_loc4_->transparent)
				{
					if (param1->receiversBuffers.length() < (param1->receiversCount+1))
						param1->receiversBuffers.resize(param1->receiversCount+1);

					if (param1->receiversFirstIndexes.length() < (param1->receiversCount+1))
						param1->receiversFirstIndexes.resize(param1->receiversCount+1);

					if (param1->receiversNumsTriangles.length() < (param1->receiversCount+1))
						param1->receiversNumsTriangles.resize(param1->receiversCount+1);


					param1->receiversBuffers[param1->receiversCount] = _loc4_->buffer;
					param1->receiversFirstIndexes[param1->receiversCount] = _loc4_->firstIndex;
					param1->receiversNumsTriangles[param1->receiversCount] = _loc4_->numTriangles;
					param1->receiversCount++;
				}
			}
			_loc2_ = _loc2_->next.get();
			_loc4_ = _loc4_->next.get();
		}
	}
}

void KDNode::collectPolygons(Decal *param1, double param2, double param3, double param4, double param5, double param6, double param7, double param8, double param9)
{
	if (this->negative != nullptr)
	{
		bool _loc12_ = (this->axis == 0);
		bool _loc13_ = (this->axis == 1);
		double _loc14_ = ((!(!(_loc12_))) ? param4 : ((!(!(_loc13_))) ? param6 : param8));
		double _loc15_ = ((!(!(_loc12_))) ? param5 : ((!(!(_loc13_))) ? param7 : param9));
		if (_loc15_ <= this->maxCoord)
		{
			this->negative->collectPolygons(param1, param2, param3, param4, param5, param6, param7, param8, param9);
		}
		else
		{
			if (_loc14_ >= this->minCoord)
			{
				this->positive->collectPolygons(param1, param2, param3, param4, param5, param6, param7, param8, param9);
			}
			else
			{
				Object3D *_loc11_ = this->objectBoundList.get();
				Object3D *_loc10_ = this->objectList.get();
				while (_loc11_ != nullptr)
				{
					if (_loc12_)
					{
						if (((((param6 < _loc11_->bound.MaxY) && (param7 > _loc11_->bound.MinY)) && (param8 < _loc11_->bound.MaxZ)) && (param9 > _loc11_->bound.MinZ)))
						{
							::clip(param1, param2, param3, _loc10_);
						}
					}
					else
					{
						if (_loc13_)
						{
							if (((((param4 < _loc11_->bound.MaxX) && (param5 > _loc11_->bound.MinX)) && (param8 < _loc11_->bound.MaxZ)) && (param9 > _loc11_->bound.MinZ)))
							{
								::clip(param1, param2, param3, _loc10_);
							}
						}
						else
						{
							if (((((param4 < _loc11_->bound.MaxX) && (param5 > _loc11_->bound.MinX)) && (param6 < _loc11_->bound.MaxY)) && (param7 > _loc11_->bound.MinY)))
							{
								::clip(param1, param2, param3, _loc10_);
							}
						}
					}
					_loc11_ = _loc11_->next.get();
					_loc10_ = _loc10_->next.get();
				}
				this->negative->collectPolygons(param1, param2, param3, param4, param5, param6, param7, param8, param9);
				this->positive->collectPolygons(param1, param2, param3, param4, param5, param6, param7, param8, param9);
			}
		}
	}
	else
	{
		Object3D *_loc10_ = this->objectList.get();
		while (_loc10_ != nullptr)
		{
			::clip(param1, param2, param3, _loc10_);
			_loc10_ = _loc10_->next.get();
		}
	}
}


static void clip(Decal *param1, double param2, double param3, Object3D *param4)
{
	QVector<std::shared_ptr<Face>> _loc9_;


	std::shared_ptr<Vertex> _loc8_;

	std::shared_ptr<Vertex> _loc14_;
	std::shared_ptr<Vertex> _loc15_;


	if (param4->isType(Mesh::TYPE))
	{
		Mesh *mesh = (Mesh*)param4;

		_loc8_ = mesh->vertexList;
		Face *face = mesh->faceList().get();
		if (((face->material == nullptr) || (face->material->transparent())))
		{
			return;
		}
		_loc9_ = getFaces(mesh);
	}
	else
	{
		if (param4->isType(BSP::TYPE))
		{
			BSP *bsp = (BSP*)param4;

			_loc8_ = bsp->vertexList;
			_loc9_ = *bsp->faces;
			Face *face = _loc9_.at(0).get();
			if (face->material == nullptr || face->material->transparent())
			{
				return;
			}
		}
	}

	param4->composeAndAppend(param1);
	param4->calculateInverseMatrix();
	param4->transformId++;
	int _loc10_ = _loc9_.length();
	int _loc11_ = 0;
	while (_loc11_ < _loc10_)
	{
		Face *_loc5_ = _loc9_.at(_loc11_).get();
		if ((((-(_loc5_->normalX) * param4->matrix_i.mc) - (_loc5_->normalY * param4->matrix_i.mg)) - (_loc5_->normalZ * param4->matrix_i.mk)) >= param3)
		{
			double _loc12_ = (((_loc5_->normalX * param4->matrix_i.md) + (_loc5_->normalY * param4->matrix_i.mh)) + (_loc5_->normalZ * param4->matrix_i.ml));
			if ((!((_loc12_ <= (_loc5_->offset - param2)) || (_loc12_ >= (_loc5_->offset + param2)))))
			{
				Wrapper *_loc7_ = _loc5_->wrapper.get();
				while (_loc7_ != nullptr)
				{
					Vertex *_loc6_ = _loc7_->vertex.get();
					if (_loc6_->transformId != param4->transformId)
					{
						_loc6_->cameraX = ((((param4->matrix.ma * _loc6_->x) + (param4->matrix.mb * _loc6_->y)) + (param4->matrix.mc * _loc6_->z)) + param4->matrix.md);
						_loc6_->cameraY = ((((param4->matrix.me * _loc6_->x) + (param4->matrix.mf * _loc6_->y)) + (param4->matrix.mg * _loc6_->z)) + param4->matrix.mh);
						_loc6_->cameraZ = ((((param4->matrix.mi * _loc6_->x) + (param4->matrix.mj * _loc6_->y)) + (param4->matrix.mk * _loc6_->z)) + param4->matrix.ml);
						_loc6_->transformId = param4->transformId;
					}
					_loc7_ = _loc7_->_next;
				}
				_loc7_ = _loc5_->wrapper.get();
				while (_loc7_ != nullptr)
				{
					if (_loc7_->vertex->cameraX > param1->bound.MinX)
					{
						break;
					}
					_loc7_ = _loc7_->_next;
				}
				if (_loc7_ != nullptr)
				{
					_loc7_ = _loc5_->wrapper.get();
					while (_loc7_ != nullptr)
					{
						if (_loc7_->vertex->cameraX < param1->bound.MaxX)
						{
							break;
						}
						_loc7_ = _loc7_->_next;
					}
					if (_loc7_ != nullptr)
					{
						_loc7_ = _loc5_->wrapper.get();
						while (_loc7_ != nullptr)
						{
							if (_loc7_->vertex->cameraY > param1->bound.MinY)
							{
								break;
							}
							_loc7_ = _loc7_->_next;
						}
						if (_loc7_ != nullptr)
						{
							_loc7_ = _loc5_->wrapper.get();
							while (_loc7_ != nullptr)
							{
								if (_loc7_->vertex->cameraY < param1->bound.MaxY)
								{
									break;
								}
								_loc7_ = _loc7_->_next;
							}
							if (_loc7_ != nullptr)
							{
								_loc7_ = _loc5_->wrapper.get();
								while (_loc7_ != nullptr)
								{
									if (_loc7_->vertex->cameraZ > param1->bound.MinZ)
									{
										break;
									}
									_loc7_ = _loc7_->_next;
								}
								if (_loc7_ != nullptr)
								{
									_loc7_ = _loc5_->wrapper.get();
									while (_loc7_ != nullptr)
									{
										if (_loc7_->vertex->cameraZ < param1->bound.MaxZ)
										{
											break;
										}
										_loc7_ = _loc7_->_next;
									}
									if (_loc7_ != nullptr)
									{
										std::shared_ptr<Vertex> _loc18_;
										std::shared_ptr<Vertex> _loc19_;
										_loc7_ = _loc5_->wrapper.get();
										while (_loc7_ != nullptr)
										{
											Vertex *_loc6_ = _loc7_->vertex.get();
											std::shared_ptr<Vertex> _loc16_ = Vertex::create();
											_loc16_->x = _loc6_->cameraX;
											_loc16_->y = _loc6_->cameraY;
											_loc16_->z = _loc6_->cameraZ;
											_loc16_->normalX = (((param4->matrix.ma * _loc6_->normalX) + (param4->matrix.mb * _loc6_->normalY)) + (param4->matrix.mc * _loc6_->normalZ));
											_loc16_->normalY = (((param4->matrix.me * _loc6_->normalX) + (param4->matrix.mf * _loc6_->normalY)) + (param4->matrix.mg * _loc6_->normalZ));
											_loc16_->normalZ = (((param4->matrix.mi * _loc6_->normalX) + (param4->matrix.mj * _loc6_->normalY)) + (param4->matrix.mk * _loc6_->normalZ));
											if (_loc19_ != nullptr)
											{
												_loc19_->next = _loc16_;
											}
											else
											{
												_loc18_ = _loc16_;
											}
											_loc19_ = _loc16_;
											_loc7_ = _loc7_->_next;
										}
										_loc14_ = _loc19_;
										_loc15_ = _loc18_;
										_loc18_ = nullptr;
										_loc19_ = nullptr;
										while (_loc15_ != nullptr)
										{
											std::shared_ptr<Vertex> _loc17_ = _loc15_->next;
											_loc15_->next = nullptr;
											if ((((_loc15_->z > param1->bound.MinZ) && (_loc14_->z <= param1->bound.MinZ)) || ((_loc15_->z <= param1->bound.MinZ) && (_loc14_->z > param1->bound.MinZ))))
											{
												double _loc13_ = ((param1->bound.MinZ - _loc14_->z) / (_loc15_->z - _loc14_->z));
												std::shared_ptr<Vertex> _loc16_ = Vertex::create();
												_loc16_->x = (_loc14_->x + ((_loc15_->x - _loc14_->x) * _loc13_));
												_loc16_->y = (_loc14_->y + ((_loc15_->y - _loc14_->y) * _loc13_));
												_loc16_->z = (_loc14_->z + ((_loc15_->z - _loc14_->z) * _loc13_));
												_loc16_->normalX = (_loc14_->normalX + ((_loc15_->normalX - _loc14_->normalX) * _loc13_));
												_loc16_->normalY = (_loc14_->normalY + ((_loc15_->normalY - _loc14_->normalY) * _loc13_));
												_loc16_->normalZ = (_loc14_->normalZ + ((_loc15_->normalZ - _loc14_->normalZ) * _loc13_));
												if (_loc19_ != nullptr)
												{
													_loc19_->next = _loc16_;
												}
												else
												{
													_loc18_ = _loc16_;
												}
												_loc19_ = _loc16_;
											}
											if (_loc15_->z > param1->bound.MinZ)
											{
												if (_loc19_ != nullptr)
												{
													_loc19_->next = _loc15_;
												}
												else
												{
													_loc18_ = _loc15_;
												}
												_loc19_ = _loc15_;
											}
											_loc14_ = _loc15_;
											_loc15_ = _loc17_;
										}
										if (_loc18_ != nullptr)
										{
											_loc14_ = _loc19_;
											_loc15_ = _loc18_;
											_loc18_ = nullptr;
											_loc19_ = nullptr;
											while (_loc15_ != nullptr)
											{
												std::shared_ptr<Vertex> _loc17_ = _loc15_->next;
												_loc15_->next = nullptr;
												if ((((_loc15_->z < param1->bound.MaxZ) && (_loc14_->z >= param1->bound.MaxZ)) || ((_loc15_->z >= param1->bound.MaxZ) && (_loc14_->z < param1->bound.MaxZ))))
												{
													double _loc13_ = ((param1->bound.MaxZ - _loc14_->z) / (_loc15_->z - _loc14_->z));
													std::shared_ptr<Vertex> _loc16_ = Vertex::create();
													_loc16_->x = (_loc14_->x + ((_loc15_->x - _loc14_->x) * _loc13_));
													_loc16_->y = (_loc14_->y + ((_loc15_->y - _loc14_->y) * _loc13_));
													_loc16_->z = (_loc14_->z + ((_loc15_->z - _loc14_->z) * _loc13_));
													_loc16_->normalX = (_loc14_->normalX + ((_loc15_->normalX - _loc14_->normalX) * _loc13_));
													_loc16_->normalY = (_loc14_->normalY + ((_loc15_->normalY - _loc14_->normalY) * _loc13_));
													_loc16_->normalZ = (_loc14_->normalZ + ((_loc15_->normalZ - _loc14_->normalZ) * _loc13_));
													if (_loc19_ != nullptr)
													{
														_loc19_->next = _loc16_;
													}
													else
													{
														_loc18_ = _loc16_;
													}
													_loc19_ = _loc16_;
												}
												if (_loc15_->z < param1->bound.MaxZ)
												{
													if (_loc19_ != nullptr)
													{
														_loc19_->next = _loc15_;
													}
													else
													{
														_loc18_ = _loc15_;
													}
													_loc19_ = _loc15_;
												}
												_loc14_ = _loc15_;
												_loc15_ = _loc17_;
											}
											if (_loc18_ != nullptr)
											{
												_loc14_ = _loc19_;
												_loc15_ = _loc18_;
												_loc18_ = nullptr;
												_loc19_ = nullptr;
												while (_loc15_ != nullptr)
												{
													std::shared_ptr<Vertex> _loc17_ = _loc15_->next;
													_loc15_->next = nullptr;
													if ((((_loc15_->x > param1->bound.MinX) && (_loc14_->x <= param1->bound.MinX)) || ((_loc15_->x <= param1->bound.MinX) && (_loc14_->x > param1->bound.MinX))))
													{
														double _loc13_ = ((param1->bound.MinX - _loc14_->x) / (_loc15_->x - _loc14_->x));
														std::shared_ptr<Vertex> _loc16_ = Vertex::create();
														_loc16_->x = (_loc14_->x + ((_loc15_->x - _loc14_->x) * _loc13_));
														_loc16_->y = (_loc14_->y + ((_loc15_->y - _loc14_->y) * _loc13_));
														_loc16_->z = (_loc14_->z + ((_loc15_->z - _loc14_->z) * _loc13_));
														_loc16_->normalX = (_loc14_->normalX + ((_loc15_->normalX - _loc14_->normalX) * _loc13_));
														_loc16_->normalY = (_loc14_->normalY + ((_loc15_->normalY - _loc14_->normalY) * _loc13_));
														_loc16_->normalZ = (_loc14_->normalZ + ((_loc15_->normalZ - _loc14_->normalZ) * _loc13_));
														if (_loc19_ != nullptr)
														{
															_loc19_->next = _loc16_;
														}
														else
														{
															_loc18_ = _loc16_;
														}
														_loc19_ = _loc16_;
													}
													if (_loc15_->x > param1->bound.MinX)
													{
														if (_loc19_ != nullptr)
														{
															_loc19_->next = _loc15_;
														}
														else
														{
															_loc18_ = _loc15_;
														}
														_loc19_ = _loc15_;
													}
													_loc14_ = _loc15_;
													_loc15_ = _loc17_;
												}
												if (_loc18_ != nullptr)
												{
													_loc14_ = _loc19_;
													_loc15_ = _loc18_;
													_loc18_ = nullptr;
													_loc19_ = nullptr;
													while (_loc15_ != nullptr)
													{
														std::shared_ptr<Vertex> _loc17_ = _loc15_->next;
														_loc15_->next = nullptr;
														if ((((_loc15_->x < param1->bound.MaxX) && (_loc14_->x >= param1->bound.MaxX)) || ((_loc15_->x >= param1->bound.MaxX) && (_loc14_->x < param1->bound.MaxX))))
														{
															double _loc13_ = ((param1->bound.MaxX - _loc14_->x) / (_loc15_->x - _loc14_->x));
															std::shared_ptr<Vertex> _loc16_ = Vertex::create();
															_loc16_->x = (_loc14_->x + ((_loc15_->x - _loc14_->x) * _loc13_));
															_loc16_->y = (_loc14_->y + ((_loc15_->y - _loc14_->y) * _loc13_));
															_loc16_->z = (_loc14_->z + ((_loc15_->z - _loc14_->z) * _loc13_));
															_loc16_->normalX = (_loc14_->normalX + ((_loc15_->normalX - _loc14_->normalX) * _loc13_));
															_loc16_->normalY = (_loc14_->normalY + ((_loc15_->normalY - _loc14_->normalY) * _loc13_));
															_loc16_->normalZ = (_loc14_->normalZ + ((_loc15_->normalZ - _loc14_->normalZ) * _loc13_));
															if (_loc19_ != nullptr)
															{
																_loc19_->next = _loc16_;
															}
															else
															{
																_loc18_ = _loc16_;
															}
															_loc19_ = _loc16_;
														}
														if (_loc15_->x < param1->bound.MaxX)
														{
															if (_loc19_ != nullptr)
															{
																_loc19_->next = _loc15_;
															}
															else
															{
																_loc18_ = _loc15_;
															}
															_loc19_ = _loc15_;
														}
														_loc14_ = _loc15_;
														_loc15_ = _loc17_;
													}
													if (_loc18_ != nullptr)
													{
														_loc14_ = _loc19_;
														_loc15_ = _loc18_;
														_loc18_ = nullptr;
														_loc19_ = nullptr;
														while (_loc15_ != nullptr)
														{
															std::shared_ptr<Vertex> _loc17_ = _loc15_->next;
															_loc15_->next = nullptr;
															if ((((_loc15_->y > param1->bound.MinY) && (_loc14_->y <= param1->bound.MinY)) || ((_loc15_->y <= param1->bound.MinY) && (_loc14_->y > param1->bound.MinY))))
															{
																double _loc13_ = ((param1->bound.MinY - _loc14_->y) / (_loc15_->y - _loc14_->y));
																std::shared_ptr<Vertex> _loc16_ = Vertex::create();
																_loc16_->x = (_loc14_->x + ((_loc15_->x - _loc14_->x) * _loc13_));
																_loc16_->y = (_loc14_->y + ((_loc15_->y - _loc14_->y) * _loc13_));
																_loc16_->z = (_loc14_->z + ((_loc15_->z - _loc14_->z) * _loc13_));
																_loc16_->normalX = (_loc14_->normalX + ((_loc15_->normalX - _loc14_->normalX) * _loc13_));
																_loc16_->normalY = (_loc14_->normalY + ((_loc15_->normalY - _loc14_->normalY) * _loc13_));
																_loc16_->normalZ = (_loc14_->normalZ + ((_loc15_->normalZ - _loc14_->normalZ) * _loc13_));
																if (_loc19_ != nullptr)
																{
																	_loc19_->next = _loc16_;
																}
																else
																{
																	_loc18_ = _loc16_;
																}
																_loc19_ = _loc16_;
															}
															if (_loc15_->y > param1->bound.MinY)
															{
																if (_loc19_ != nullptr)
																{
																	_loc19_->next = _loc15_;
																}
																else
																{
																	_loc18_ = _loc15_;
																}
																_loc19_ = _loc15_;
															}
															_loc14_ = _loc15_;
															_loc15_ = _loc17_;
														}
														if (_loc18_ != nullptr)
														{
															_loc14_ = _loc19_;
															_loc15_ = _loc18_;
															_loc18_ = nullptr;
															_loc19_ = nullptr;
															while (_loc15_ != nullptr)
															{
																std::shared_ptr<Vertex> _loc17_ = _loc15_->next;
																_loc15_->next = nullptr;
																if ((((_loc15_->y < param1->bound.MaxY) && (_loc14_->y >= param1->bound.MaxY)) || ((_loc15_->y >= param1->bound.MaxY) && (_loc14_->y < param1->bound.MaxY))))
																{
																	double _loc13_ = ((param1->bound.MaxY - _loc14_->y) / (_loc15_->y - _loc14_->y));
																	std::shared_ptr<Vertex> _loc16_ = Vertex::create();
																	_loc16_->x = (_loc14_->x + ((_loc15_->x - _loc14_->x) * _loc13_));
																	_loc16_->y = (_loc14_->y + ((_loc15_->y - _loc14_->y) * _loc13_));
																	_loc16_->z = (_loc14_->z + ((_loc15_->z - _loc14_->z) * _loc13_));
																	_loc16_->normalX = (_loc14_->normalX + ((_loc15_->normalX - _loc14_->normalX) * _loc13_));
																	_loc16_->normalY = (_loc14_->normalY + ((_loc15_->normalY - _loc14_->normalY) * _loc13_));
																	_loc16_->normalZ = (_loc14_->normalZ + ((_loc15_->normalZ - _loc14_->normalZ) * _loc13_));
																	if (_loc19_ != nullptr)
																	{
																		_loc19_->next = _loc16_;
																	}
																	else
																	{
																		_loc18_ = _loc16_;
																	}
																	_loc19_ = _loc16_;
																}
																if (_loc15_->y < param1->bound.MaxY)
																{
																	if (_loc19_ != nullptr)
																	{
																		_loc19_->next = _loc15_;
																	}
																	else
																	{
																		_loc18_ = _loc15_;
																	}
																	_loc19_ = _loc15_;
																}
																_loc14_ = _loc15_;
																_loc15_ = _loc17_;
															}
															if (_loc18_ != nullptr)
															{
																std::shared_ptr<Face> _loc5_ = Face::create();
																Wrapper *_loc20_ = nullptr;
																std::shared_ptr<Vertex> _loc6_ = _loc18_;
																while (_loc6_ != nullptr)
																{
																	std::shared_ptr<Vertex> _loc17_ = _loc6_->next;
																	_loc6_->next = param1->vertexList;
																	param1->vertexList = _loc6_;
																	_loc6_->u = ((_loc6_->x - param1->bound.MinX) / (param1->bound.MaxX - param1->bound.MinX));
																	_loc6_->v = ((_loc6_->y - param1->bound.MinY) / (param1->bound.MaxY - param1->bound.MinY));

																	if (_loc20_ != nullptr)
																	{
																		_loc20_->setNext(Wrapper::create(40));
																		_loc20_ = _loc20_->_next;
																	}
																	else
																	{
																		_loc5_->wrapper = Wrapper::create(41);
																		_loc20_ = _loc5_->wrapper.get();
																	}
																	_loc20_->vertex = _loc6_;
																	_loc6_ = _loc17_;
																}
																_loc5_->calculateBestSequenceAndNormal();
																_loc5_->next = param1->faceList();
																param1->setFaceList(_loc5_, 20);
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
		_loc11_++;
	}
}

