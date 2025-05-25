#include "parser3ds.h"
#include "alternativa/engine3d/core/object3d.h"
#include "alternativa/engine3d/materials/material.h"
#include "alternativa/engine3d/materials/texturematerial.h"
#include "alternativa/engine3d/materials/fillmaterial.h"
#include "alternativa/engine3d/objects/mesh.h"
#include "mymath.h"
#include "alternativa/engine3d/core/face.h"
#include "alternativa/engine3d/core/vertex.h"
#include "alternativa/engine3d/core/wrapper.h"
#include "alternativa/math/vector3.h"
#include "_global.h"
#include <QMapIterator>
#include <QMap>
#include <QDebug>


#if 0
MaterialData
MapData
ObjectData
AnimationData
ChunkInfo
#endif



struct ChunkInfo
{
	int id;
	int size;
	int dataSize;
	int dataPosition;
	int nextChunkPosition;
};

struct ObjectData
{
	QString name;
	std::shared_ptr<QVector<double>> vertices;
	std::shared_ptr<QVector<double>> uvs;
	std::shared_ptr<QVector<int>> faces;
	std::shared_ptr<QVector<uint>> smoothingGroups;
	std::shared_ptr<QMap<QString, std::shared_ptr<QVector<int>>>> surfaces;
	double a;
	double b;
	double c;
	double d;
	double e;
	double f;
	double g;
	double h;
	double i;
	double j;
	double k;
	double l;

	~ObjectData()
	{

	}
};

struct AnimationData
{
	QString objectName;
	std::shared_ptr<Object3D> object;
	std::shared_ptr<Vector3> pivot;
	std::shared_ptr<Vector3> position;
	std::shared_ptr<Vector3> rotation;
	std::shared_ptr<Vector3> scale;
	int parentIndex;
	bool isInstance;

	AnimationData()
	{
		parentIndex = 0;
		isInstance = false;
	}
};


struct MapData
{
	QString filename;
	double scaleU = 1;
	double scaleV = 1;
	double offsetU = 0;
	double offsetV = 0;
	double rotation = 0;
};


struct MaterialData
{
	QString name;
	int color;
	int specular;
	int glossiness;
	int transparency;
	std::shared_ptr<MapData> diffuseMap;
	std::shared_ptr<MapData> opacityMap;
	std::shared_ptr<Matrix> matrix;
	std::shared_ptr<Material> material;
};

class Parser3DS_StreamRead
{
	const uint8_t *data;
	const uint8_t *data_end;
	uint32_t length;
	uint32_t _position;

public:

	Parser3DS_StreamRead(const uint8_t *data, uint32_t length)
	{
		this->data = data;
		this->data_end = data + length;
		this->length = length;
		this->_position = 0;
	}

	const uint8_t* getData()
	{
		return this->data + this->_position;
	}

	void setPosition(uint32_t position)
	{
		if (position > length)
			throw 121;

		_position = position;
	}

	uint32_t getPosition()
	{
		return _position;
	}

	uint32_t bytesAvailable()
	{
		return this->length - this->_position;
	}

	uint32_t size()
	{
		return this->length;
	}

	uint8_t readUInt8()
	{
		if (_position + sizeof(uint8_t) > length)
			throw 124;

		uint8_t v = *(uint8_t*)getData();
		_position += sizeof(uint8_t);
		return v;
	}

	uint16_t readUInt16()
	{
		if (_position + sizeof(uint16_t) > length)
			throw 122;

		uint16_t v = *(uint16_t*)getData();
		_position += sizeof(uint16_t);
		return v;
	}

	uint32_t readUInt32()
	{
		if (_position + sizeof(uint32_t) > length)
			throw 123;

		uint32_t v = *(uint32_t*)getData();
		_position += sizeof(uint32_t);
		return v;
	}

	float readFloat()
	{
		if (_position + sizeof(float) > length)
			throw 125;

		float v = *(float*)getData();
		_position += sizeof(float);
		return v;
	}
};



enum
{
	CHUNK_MAIN = 19789,
	CHUNK_VERSION = 2,
	CHUNK_SCENE = 15677,
	CHUNK_ANIMATION = 45056,
	CHUNK_OBJECT = 16384,
	CHUNK_TRIMESH = 16640,
	CHUNK_VERTICES = 16656,
	CHUNK_FACES = 16672,
	CHUNK_FACESMATERIAL = 16688,
	CHUNK_FACESSMOOTH = 16720,
	CHUNK_MAPPINGCOORDS = 16704,
	CHUNK_TRANSFORMATION = 16736,
	CHUNK_MATERIAL = 45055,
};



Parser3DS::Parser3DS()
{
	_function_name("Parser3DS::Parser3DS");
}

Parser3DS::~Parser3DS()
{
	_function_name("Parser3DS::~Parser3DS");
}

bool Parser3DS::parse(const QByteArray &_data, const QString &texturesBaseURL, double scale)
{
	_function_name("Parser3DS::parse");

	Parser3DS_StreamRead data((const uint8_t*)_data.constData(), _data.size());


	if (data.size() < 6)
		return false;

	try
	{
		this->data = &data;
		this->parse3DSChunk(data.getPosition(), data.bytesAvailable());
		this->buildContent(texturesBaseURL, scale);
		this->animationDatas = nullptr;
		this->materialDatas = nullptr;
	}
	catch (int e)
	{
		qDebug() << "error Parser3DS::parse" << e;
		return false;
	}

	return true;
}

void Parser3DS::parse3DSChunk(int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parse3DSChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo); //error 3
	this->data->setPosition(dataPosition);

	switch (chunkinfo.id)
	{
		case CHUNK_MAIN:
			this->parseMainChunk(chunkinfo.dataPosition, chunkinfo.dataSize);
			break;
	}

	this->parse3DSChunk(chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size); //error 2
}

void Parser3DS::readChunkInfo(int dataPosition, ChunkInfo *chunkinfo)
{
	_function_name("Parser3DS::readChunkInfo");
	this->data->setPosition(dataPosition);
	chunkinfo->id = this->data->readUInt16();
	chunkinfo->size = this->data->readUInt32();
	chunkinfo->dataSize = chunkinfo->size - 6;
	chunkinfo->dataPosition = this->data->getPosition();
	chunkinfo->nextChunkPosition = dataPosition + chunkinfo->size;
}

void Parser3DS::buildContent(const QString &texturesBaseURL, double scale)
{
	_function_name("Parser3DS::buildContent");

	QMapIterator<QString, std::shared_ptr<MaterialData>> iter(*this->materialDatas);
	while (iter.hasNext())
	{
		iter.next();
		QString materialName = iter.key();

		MaterialData *materialData = iter.value().get();
		MapData *mapData = materialData->diffuseMap.get();
		if (mapData != nullptr)
		{
			std::shared_ptr<Matrix> materialMatrix = std::make_shared<Matrix>();

			double rot = (mapData->rotation * MyMath::PI) / 180.;
			materialMatrix->translate(-mapData->offsetU, mapData->offsetV);
			materialMatrix->translate(-0.5, -0.5);
			materialMatrix->rotate(-rot);
			materialMatrix->scale(mapData->scaleU, mapData->scaleV);
			materialMatrix->translate(0.5, 0.5);
			materialData->matrix = materialMatrix;

			std::shared_ptr<TextureMaterial> textureMaterial = new_TextureMaterial();
			textureMaterial->init();
			textureMaterial->setName(materialName);
			textureMaterial->diffuseMapURL = texturesBaseURL + mapData->filename;
			textureMaterial->opacityMapURL = (materialData->opacityMap != nullptr) ? (texturesBaseURL + materialData->opacityMap->filename) : nullptr;
			materialData->material = textureMaterial;
			textureMaterial->setName(materialData->name);
			this->textureMaterials.append(textureMaterial);
		}
		else
		{
			std::shared_ptr<FillMaterial> fillMaterial = new_FillMaterial(materialData->color);
			materialData->material = fillMaterial;
			fillMaterial->setName(materialData->name);
		}
		this->materials.append(materialData->material);
	}

	if (this->animationDatas != nullptr)
	{
		if (this->objectDatas != nullptr)
		{
			int length = this->animationDatas->length();

			for (int i = 0; i < length; i++)
			{
				std::shared_ptr<AnimationData> animationData = this->animationDatas->at(i);
				QString objectName = animationData->objectName;
				std::shared_ptr<ObjectData> objectData = this->objectDatas->value(objectName);
				if (objectData != nullptr)
				{
					int nameCounter = 1;
					for (int j = i + 1; j < length; j++)
					{
						AnimationData *animationData2 = this->animationDatas->at(j).get();
						if (((!(animationData2->isInstance)) && (objectName == animationData2->objectName)))
						{
							std::shared_ptr<ObjectData> newObjectData = std::make_shared<ObjectData>();
							QString newName = objectName + QString::number(nameCounter++);
							newObjectData->name = newName;
							this->objectDatas->insert(newName, newObjectData);
							animationData2->objectName = newName;
							newObjectData->vertices = objectData->vertices;
							newObjectData->uvs = objectData->uvs;
							newObjectData->faces = objectData->faces;
							newObjectData->surfaces = objectData->surfaces;
							newObjectData->a = objectData->a;
							newObjectData->b = objectData->b;
							newObjectData->c = objectData->c;
							newObjectData->d = objectData->d;
							newObjectData->e = objectData->e;
							newObjectData->f = objectData->f;
							newObjectData->g = objectData->g;
							newObjectData->h = objectData->h;
							newObjectData->i = objectData->i;
							newObjectData->j = objectData->j;
							newObjectData->k = objectData->k;
							newObjectData->l = objectData->l;
						}
					}
				}

				std::shared_ptr<Object3D> object;

				if (((!(objectData == nullptr)) && (!(objectData->vertices == nullptr))))
				{
					object = new_Mesh();
					this->buildMesh((Mesh*)object.get(), objectData.get(), animationData.get(), scale);
				}
				else
				{
					object = new_Object3D();
				}

				object->setName(objectName);
				animationData->object = object;

				if (animationData->position != nullptr)
				{
					object->position.x = (animationData->position->x * scale);
					object->position.y = (animationData->position->y * scale);
					object->position.z = (animationData->position->z * scale);
				}

				if (animationData->rotation != nullptr)
				{
					object->rotation.x = animationData->rotation->x;
					object->rotation.y = animationData->rotation->y;
					object->rotation.z = animationData->rotation->z;
				}

				if (animationData->scale != nullptr)
				{
					object->scale.x = animationData->scale->x;
					object->scale.y = animationData->scale->y;
					object->scale.z = animationData->scale->z;
				}
			}

			for (int i = 0; i < length; i++)
			{
				std::shared_ptr<AnimationData> animationData = this->animationDatas->at(i);
				this->objects.append(animationData->object);
				this->parents.append((animationData->parentIndex == 0xFFFF) ? nullptr : this->animationDatas->at(animationData->parentIndex)->object);
			}
		}
	}
	else
	{
		QMapIterator<QString, std::shared_ptr<ObjectData>> iter(*objectDatas);
		while (iter.hasNext())
		{
			iter.next();
			std::shared_ptr<ObjectData> objectData = iter.value();
			if (objectData->vertices != nullptr)
			{
				std::shared_ptr<Mesh> mesh = new_Mesh();
				mesh->setName(iter.key());
				this->buildMesh(mesh.get(), objectData.get(), nullptr, scale);
				this->objects.append(mesh);
				this->parents.append(nullptr);
			}
		}
	}
}


void Parser3DS::parseMainChunk(int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseMainChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo);
	switch (chunkinfo.id)
	{
		case CHUNK_VERSION:
			break;
		case CHUNK_SCENE:
			this->parse3DChunk(chunkinfo.dataPosition, chunkinfo.dataSize);
			break;
		case CHUNK_ANIMATION:
			this->parseAnimationChunk(chunkinfo.dataPosition, chunkinfo.dataSize);
			break;
	}

	this->parseMainChunk(chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size);
}



void Parser3DS::buildMesh(Mesh *mesh, ObjectData *objectData, AnimationData *animationData, double scale)
{
	_function_name("Parser3DS::buildMesh");

	bool correct = false;

	if (animationData != nullptr)
	{
		double a = objectData->a;
		double b = objectData->b;
		double c = objectData->c;
		double d = objectData->d;
		double e = objectData->e;
		double f = objectData->f;
		double g = objectData->g;
		double h = objectData->h;
		double i = objectData->i;
		double j = objectData->j;
		double k = objectData->k;
		double l = objectData->l;
		double det = (1. / (((((((-(c) * f) * i) + ((b * g) * i)) + ((c * e) * j)) - ((a * g) * j)) - ((b * e) * k)) + ((a * f) * k)));
		objectData->a = (((-(g) * j) + (f * k)) * det);
		objectData->b = (((c * j) - (b * k)) * det);
		objectData->c = (((-(c) * f) + (b * g)) * det);
		objectData->d = ((((((((d * g) * j) - ((c * h) * j)) - ((d * f) * k)) + ((b * h) * k)) + ((c * f) * l)) - ((b * g) * l)) * det);
		objectData->e = (((g * i) - (e * k)) * det);
		objectData->f = (((-(c) * i) + (a * k)) * det);
		objectData->g = (((c * e) - (a * g)) * det);
		objectData->h = ((((((((c * h) * i) - ((d * g) * i)) + ((d * e) * k)) - ((a * h) * k)) - ((c * e) * l)) + ((a * g) * l)) * det);
		objectData->i = (((-(f) * i) + (e * j)) * det);
		objectData->j = (((b * i) - (a * j)) * det);
		objectData->k = (((-(b) * e) + (a * f)) * det);
		objectData->l = ((((((((d * f) * i) - ((b * h) * i)) - ((d * e) * j)) + ((a * h) * j)) + ((b * e) * l)) - ((a * f) * l)) * det);
		if (animationData->pivot != nullptr)
		{
			objectData->d = (objectData->d - animationData->pivot->x);
			objectData->h = (objectData->h - animationData->pivot->y);
			objectData->l = (objectData->l - animationData->pivot->z);
		}
		correct = true;
	}

	QVector<std::shared_ptr<Vertex>> vertices;
	bool uv = ((!(objectData->uvs == nullptr)) && (objectData->uvs->length() > 0));
	int m = 0;
	for (int n = 0; n < objectData->vertices->length();)
	{
		std::shared_ptr<Vertex> vertex = Vertex::create();
		if (correct)
		{
			double x = objectData->vertices->at(n++);
			double y = objectData->vertices->at(n++);
			double z = objectData->vertices->at(n++);
			vertex->x = ((((objectData->a * x) + (objectData->b * y)) + (objectData->c * z)) + objectData->d);
			vertex->y = ((((objectData->e * x) + (objectData->f * y)) + (objectData->g * z)) + objectData->h);
			vertex->z = ((((objectData->i * x) + (objectData->j * y)) + (objectData->k * z)) + objectData->l);
		}
		else
		{
			vertex->x = objectData->vertices->at(n++);
			vertex->y = objectData->vertices->at(n++);
			vertex->z = objectData->vertices->at(n++);
		}
		vertex->x = vertex->x * scale;
		vertex->y = vertex->y * scale;
		vertex->z = vertex->z * scale;

		if (uv)
		{
			vertex->u = objectData->uvs->at(m++);
			vertex->v = 1. - objectData->uvs->at(m++);
		}

		vertex->transformId = -1;
		vertices.append(vertex);
		vertex->next = mesh->vertexList;
		mesh->vertexList = vertex;
	}

	std::shared_ptr<Face> last;
	QVector<std::shared_ptr<Face>> faces;

	for (int n = 0; n < objectData->faces->length();)
	{
		std::shared_ptr<Face> face = Face::create();
		face->wrapper = Wrapper::create(1);
		face->wrapper->setNext(Wrapper::create(2));
		face->wrapper->_next->setNext(Wrapper::create(3));
		face->wrapper->vertex = vertices.at(objectData->faces->at(n++));
		face->wrapper->_next->vertex = vertices.at(objectData->faces->at(n++));
		face->wrapper->_next->_next->vertex = vertices.at(objectData->faces->at(n++));
		faces.append(face);
		if (last != nullptr)
		{
			last->next = face;
		}
		else
		{
			mesh->setFaceList(face, 1);
		}
		last = face;
	}

	if (objectData->surfaces != nullptr)
	{
		QMapIterator<QString, std::shared_ptr<QVector<int>>> iter(*objectData->surfaces);
		while (iter.hasNext())
		{
			iter.next();

			std::shared_ptr<QVector<int>> surface = iter.value();
			std::shared_ptr<MaterialData> materialData = this->materialDatas->value(iter.key());
			std::shared_ptr<Material> material = materialData->material;
			for (int n = 0; n < surface->length(); n++)
			{
				std::shared_ptr<Face> face = faces.at(surface->at(n));
				face->material = material;
				if (materialData->matrix != nullptr)
				{
					Wrapper *w = face->wrapper.get();
					while (w != nullptr)
					{
						Vertex *vertex = w->vertex.get();
						if (vertex->transformId < 0)
						{
							double u = vertex->u;
							double v = vertex->v;
							vertex->u = (((materialData->matrix->a * u) + (materialData->matrix->b * v)) + materialData->matrix->tx);
							vertex->v = (((materialData->matrix->c * u) + (materialData->matrix->d * v)) + materialData->matrix->ty);
							vertex->transformId = 0;
						}
						w = w->_next;
					}
				}
			}
		}
	}

	auto defaultMaterial = new_FillMaterial(0x7F7F7F);
	defaultMaterial->setName("default");
	std::shared_ptr<Face> face = mesh->faceList();
	while (face != nullptr)
	{
		if (face->material == nullptr)
		{
			face->material = defaultMaterial;
		}
		face = face->next;
	}

	mesh->calculateFacesNormals(true);
	mesh->calculateBounds();
}



void Parser3DS::parse3DChunk(int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parse3DChunk");

	while (bytesAvailable >= 6)
	{
		ChunkInfo chunkinfo;
		this->readChunkInfo(dataPosition, &chunkinfo);
		switch (chunkinfo.id)
		{
			case CHUNK_MATERIAL:
			{
				std::shared_ptr<MaterialData> material = std::make_shared<MaterialData>();
				this->parseMaterialChunk(material, chunkinfo.dataPosition, chunkinfo.dataSize);
				break;
			}
			case CHUNK_OBJECT:
				this->parseObject(&chunkinfo);
				break;
		}

		dataPosition = chunkinfo.nextChunkPosition;
		bytesAvailable = bytesAvailable - chunkinfo.size;
	}
}



void Parser3DS::parseMaterialChunk(std::shared_ptr<MaterialData> &material, int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseMaterialChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo);
	switch (chunkinfo.id)
	{
		case 0xa000:
			this->parseMaterialName(material);
			break;
		case 0xa010:
			break;
		case 0xa020:
			this->data->setPosition(chunkinfo.dataPosition + 6);
			material->color = (this->data->readUInt8() << 16) + (this->data->readUInt8() << 8) + this->data->readUInt8();
			break;
		case 0xa030:
			break;
		case 0xa040:
			this->data->setPosition(chunkinfo.dataPosition + 6);
			material->glossiness = this->data->readUInt16();
			break;
		case 41025:
			this->data->setPosition(chunkinfo.dataPosition + 6);
			material->specular = this->data->readUInt16();
			break;
		case 0xa050:
			this->data->setPosition(chunkinfo.dataPosition + 6);
			material->transparency = this->data->readUInt16();
			break;
		case 41472:
			material->diffuseMap = std::make_shared<MapData>();
			this->parseMapChunk(material->name, material->diffuseMap.get(), chunkinfo.dataPosition, chunkinfo.dataSize);
			break;
		case 41786:
			break;
		case 41488:
			material->opacityMap = std::make_shared<MapData>();
			this->parseMapChunk(material->name, material->opacityMap.get(), chunkinfo.dataPosition, chunkinfo.dataSize);
			break;
		case 692 * 60:
			break;
		case 41788:
			break;
		case 41476:
			break;
		case 41789:
			break;
		case 41504:
			break;
	}
	this->parseMaterialChunk(material, chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size);
}




void Parser3DS::parseMapChunk(const QString &materialName, MapData *map, int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseMapChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo);
	switch (chunkinfo.id)
	{
		case 41728:
			map->filename = this->getString(chunkinfo.dataPosition).toLower();
			break;
		case 41809:
			break;
		case 41812:
			map->scaleU = this->data->readFloat();
			break;
		case 41814:
			map->scaleV = this->data->readFloat();
			break;
		case 41816:
			map->offsetU = this->data->readFloat();
			break;
		case 41818:
			map->offsetV = this->data->readFloat();
			break;
		case 697 * 60:
			map->rotation = this->data->readFloat();
			break;
	}
	this->parseMapChunk(materialName, map, chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size);
}



QString Parser3DS::getString(int pos)
{
	_function_name("Parser3DS::getString");

	this->data->setPosition(pos);
	QString res = QString::fromLocal8Bit((char*)this->data->getData());
	this->data->setPosition(this->data->getPosition() + res.length() + 1);
	return res;
}


void Parser3DS::parseMaterialName(std::shared_ptr<MaterialData> &material)
{
	_function_name("Parser3DS::parseMaterialName");

	if (this->materialDatas == nullptr)
		this->materialDatas = std::make_shared<QMap<QString, std::shared_ptr<MaterialData>>>();

	material->name = this->getString(this->data->getPosition());
	this->materialDatas->insert(material->name, material);
}



void Parser3DS::parseObject(ChunkInfo *chunkInfo)
{
	_function_name("Parser3DS::parseObject");

	if (this->objectDatas == nullptr)
		this->objectDatas = std::make_shared<QMap<QString, std::shared_ptr<ObjectData>>>();

	std::shared_ptr<ObjectData> objectdata = std::make_shared<ObjectData>();
	objectdata->name = this->getString(chunkInfo->dataPosition);
	this->objectDatas->insert(objectdata->name, objectdata);
	int offset = objectdata->name.length() + 1;
	this->parseObjectChunk(objectdata, chunkInfo->dataPosition + offset, chunkInfo->dataSize - offset);
}

void Parser3DS::parseObjectChunk(std::shared_ptr<ObjectData> &object, int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseObjectChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo);
	switch (chunkinfo.id)
	{
		case CHUNK_TRIMESH:
			this->parseMeshChunk(object, chunkinfo.dataPosition, chunkinfo.dataSize);
			break;
		case 17920:
			break;
		case 18176:
			break;
	}
	this->parseObjectChunk(object, chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size);
}



void Parser3DS::parseMeshChunk(std::shared_ptr<ObjectData> &object, int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseObjectChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo);
	switch (chunkinfo.id)
	{
		case CHUNK_VERTICES:
			this->parseVertices(object);
			break;
		case CHUNK_MAPPINGCOORDS:
			this->parseUVs(object);
			break;
		case CHUNK_TRANSFORMATION:
			this->parseMatrix(object);
			break;
		case CHUNK_FACES:
			this->parseFaces(object, &chunkinfo);
			break;
	}

	this->parseMeshChunk(object, chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size);
}



void Parser3DS::parseVertices(std::shared_ptr<ObjectData> &object)
{
	_function_name("Parser3DS::parseVertices");

	int num = this->data->readUInt16();
	object->vertices = std::make_shared<QVector<double>>();
	for (int i = 0; i < num; i++)
	{
		object->vertices->append(this->data->readFloat());
		object->vertices->append(this->data->readFloat());
		object->vertices->append(this->data->readFloat());
	}
}



void Parser3DS::parseUVs(std::shared_ptr<ObjectData> &object)
{
	_function_name("Parser3DS::parseUVs");

	int num = this->data->readUInt16();
	object->uvs = std::make_shared<QVector<double>>();
	for (int i = 0; i < num; i++)
	{
		object->uvs->append(this->data->readFloat());
		object->uvs->append(this->data->readFloat());
	}
}



void Parser3DS::parseMatrix(std::shared_ptr<ObjectData> &object)
{
	_function_name("Parser3DS::parseMatrix");

	object->a = this->data->readFloat();
	object->e = this->data->readFloat();
	object->i = this->data->readFloat();
	object->b = this->data->readFloat();
	object->f = this->data->readFloat();
	object->j = this->data->readFloat();
	object->c = this->data->readFloat();
	object->g = this->data->readFloat();
	object->k = this->data->readFloat();
	object->d = this->data->readFloat();
	object->h = this->data->readFloat();
	object->l = this->data->readFloat();
}




void Parser3DS::parseFaces(std::shared_ptr<ObjectData> &object, ChunkInfo *chunkInfo)
{
	_function_name("Parser3DS::parseFaces");

	int num = this->data->readUInt16();
	object->faces = std::make_shared<QVector<int>>();
	object->smoothingGroups = std::make_shared<QVector<uint>>();

	for (int i = 0; i < num; i++)
	{
		object->faces->append(this->data->readUInt16());
		object->faces->append(this->data->readUInt16());
		object->faces->append(this->data->readUInt16());
		this->data->setPosition(this->data->getPosition() + 2);
	}

	int offset = 2 + (8 * num);
	this->parseFacesChunk(object, (chunkInfo->dataPosition + offset), (chunkInfo->dataSize - offset));
}



void Parser3DS::parseFacesChunk(std::shared_ptr<ObjectData> &ObjectData, int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseFacesChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkInfo;
	this->readChunkInfo(dataPosition, &chunkInfo);
	switch (chunkInfo.id)
	{
		case CHUNK_FACESMATERIAL:
			this->parseSurface(ObjectData);
			break;
		case CHUNK_FACESSMOOTH:
			this->parseSmoothingGroups(ObjectData);
			break;
	}
	this->parseFacesChunk(ObjectData, chunkInfo.nextChunkPosition, bytesAvailable - chunkInfo.size);
}



void Parser3DS::parseSurface(std::shared_ptr<ObjectData> &object)
{
	_function_name("Parser3DS::parseSurface");

	if (object->surfaces == nullptr)
		object->surfaces = std::make_shared<QMap<QString, std::shared_ptr<QVector<int>>>>();

	std::shared_ptr<QVector<int>> surface = std::make_shared<QVector<int>>();
	QString key = this->getString(this->data->getPosition());
	object->surfaces->insert(key, surface);

	int num = this->data->readUInt16();
	for (int i = 0; i < num; i++)
		surface->append(this->data->readUInt16());
}




void Parser3DS::parseSmoothingGroups(std::shared_ptr<ObjectData> &object)
{
	_function_name("Parser3DS::parseSmoothingGroups");

	int num = object->faces->count() / 3;
	for (int i = 0; i < num; i++)
		object->smoothingGroups->append(this->data->readUInt32());
}



void Parser3DS::parseAnimationChunk(int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseAnimationChunk");

	while (bytesAvailable >= 6)
	{
		ChunkInfo chunkinfo;
		this->readChunkInfo(dataPosition, &chunkinfo);
		switch (chunkinfo.id)
		{
			case 45057:
			case 45058:
			case 45059:
			case 45060:
			case 45061:
			case 45062:
			case 45063:
			{
				if (this->animationDatas == nullptr)
					this->animationDatas = std::make_shared<QVector<std::shared_ptr<AnimationData>>>();

				std::shared_ptr<AnimationData> animation = std::make_shared<AnimationData>();
				this->animationDatas->append(animation);
				this->parseObjectAnimationChunk(animation, chunkinfo.dataPosition, chunkinfo.dataSize);
				break;
			}
			case 45064:
				break;
		}
		dataPosition = chunkinfo.nextChunkPosition;
		bytesAvailable = bytesAvailable - chunkinfo.size;
	}
}



void Parser3DS::parseObjectAnimationChunk(std::shared_ptr<AnimationData> &animation, int dataPosition, int bytesAvailable)
{
	_function_name("Parser3DS::parseObjectAnimationChunk");

	if (bytesAvailable < 6)
		return;

	ChunkInfo chunkinfo;
	this->readChunkInfo(dataPosition, &chunkinfo);
	switch (chunkinfo.id)
	{
		case 0xb010:
			animation->objectName = this->getString(this->data->getPosition());
			this->data->setPosition(this->data->getPosition() + 4);
			animation->parentIndex = this->data->readUInt16();
			break;
		case 45073:
			animation->objectName = this->getString(this->data->getPosition());
			break;
		case 45075:
		{
			double x = this->data->readFloat();
			double y = this->data->readFloat();
			double z = this->data->readFloat();
			animation->pivot = std::make_shared<Vector3>(x, y, z);
			break;
		}
		case 0xb020:
		{
			this->data->setPosition(this->data->getPosition() + 20);
			double x = this->data->readFloat();
			double y = this->data->readFloat();
			double z = this->data->readFloat();
			animation->position = std::make_shared<Vector3>(x, y, z);
			break;
		}
		case 45089:
		{
			this->data->setPosition(this->data->getPosition() + 20);
			double angle = this->data->readFloat();
			double x = this->data->readFloat();
			double z = this->data->readFloat();
			double y = this->data->readFloat();
			animation->rotation = this->getRotationFrom3DSAngleAxis(angle, x, z, y);
			break;
		}
		case 45090:
		{
			this->data->setPosition(this->data->getPosition() + 20);
			double x = this->data->readFloat();
			double y = this->data->readFloat();
			double z = this->data->readFloat();
			animation->scale = std::make_shared<Vector3>(x, y, z);
			break;
		}
	}
	this->parseObjectAnimationChunk(animation, chunkinfo.nextChunkPosition, bytesAvailable - chunkinfo.size);
}


std::shared_ptr<Vector3> Parser3DS::getRotationFrom3DSAngleAxis(double angle, double x, double z, double y)
{
	_function_name("Parser3DS::getRotationFrom3DSAngleAxis");

	std::shared_ptr<Vector3> res = std::make_shared<Vector3>();
	double s = MyMath::sin(angle);
	double c = MyMath::cos(angle);
	double t = 1 - c;
	double k = x * y * t + z * s;

	if (k >= 1)
	{
		double half = angle / 2.;
		res->z = -2. * MyMath::atan2(x * MyMath::sin(half), MyMath::cos(half));
		res->y = -MyMath::PI / 2.;
		res->x = 0.;
		return res;
	}

	if (k <= -1)
	{
		double half = angle / 2.;
		res->z = 2. * MyMath::atan2(x * MyMath::sin(half), MyMath::cos(half));
		res->y = MyMath::PI / 2.;
		res->x = 0.;
		return res;
	}

	res->x = -(MyMath::atan2(((y * s) - ((x * z) * t)), (1 - (((y * y) + (z * z)) * t))));
	res->y = -(MyMath::asin((((x * y) * t) + (z * s))));
	res->z = -(MyMath::atan2(((x * s) - ((y * z) * t)), (1 - (((x * x) + (z * z)) * t))));

	return res;
}
